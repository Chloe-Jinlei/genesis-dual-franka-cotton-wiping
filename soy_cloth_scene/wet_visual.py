"""Local soy staining of the existing deforming cotton visual mesh.

This is a display adapter, not a liquid solver. ``saturation`` comes from the
absorption model. The adapter keeps the original woven texture and the native
PBD visual-vertex skinning, and multiplies its color by local wetness. It uses
the installed Genesis rasterizer's real ``vertex`` buffer and viewer lock; no
decal, fixed world-space stain, extra cloth surface, or engine edit is needed.

The small private-API bridge below is specific to the installed Genesis
``RasterizerContext`` / ``pyrender.Primitive`` implementation. Its buffer layout
is checked explicitly so an incompatible future engine fails visibly.
"""

from __future__ import annotations

import numpy as np


def _vertex_buffer(primitive):
    """Match Primitive._add_to_context's interleaved ``vertex`` attributes."""
    if not primitive.stream_positions:
        raise RuntimeError("Wet cloth requires Genesis streamed PBD positions.")
    if primitive.joints_0 is not None or primitive.weights_0 is not None:
        raise RuntimeError("Unexpected skeletal attributes on the PBD cloth.")
    arrays = [getattr(primitive, name) for name in
              ("tangents", "texcoord_0", "texcoord_1", "color_0")]
    arrays = [array for array in arrays if array is not None]
    if not arrays:
        raise RuntimeError("Wet cloth has no vertex-color buffer.")
    return np.ascontiguousarray(np.concatenate(arrays, axis=1), dtype=np.float32)


class WetClothVisual:
    """Tint one built PBD cloth from per-simulation-particle saturation.

    Instantiate after ``scene.build()``. Call ``update(saturation)`` after an
    absorption step and before rendering, and ``reset()`` with scene reset.
    The public camera renderer and the native viewer share these exact nodes.
    This adapter supports the scene's rasterizer, not ray tracing or the batch
    renderer. Color intensity is illustrative, not optically calibrated.
    """

    def __init__(self, scene, cloth):
        from genesis.ext import pyrender
        from genesis.utils.misc import qd_to_numpy

        if cloth.surface.vis_mode != "visual":
            raise ValueError("WetClothVisual requires the cloth's visual mesh.")
        visualizer = scene.visualizer
        if visualizer._renderer is not visualizer._rasterizer:
            raise ValueError("WetClothVisual currently supports the rasterizer.")
        self.scene = scene
        self.cloth = cloth
        self._context = visualizer._context
        self._lock = visualizer.viewer_lock
        self._nodes = []
        self._saturation = np.zeros(cloth.n_particles, dtype=np.float32)
        self.visual_saturation = np.zeros(cloth.n_vverts, dtype=np.float32)

        # These are the exact material-space supports used by Genesis to move
        # this visual mesh with its simulation particles. Negative affine
        # weights can occur at a boundary; nonnegative normalized weights avoid
        # inventing negative wetness or saturation above the input maximum.
        sl = slice(cloth.vvert_start, cloth.vvert_end)
        info = cloth.solver.vverts_info
        self._support_idx = np.asarray(
            qd_to_numpy(info.support_idxs)[sl] - cloth.particle_start,
            dtype=np.int64)
        self._support_weights = np.maximum(
            qd_to_numpy(info.support_weights)[sl], 0.0).astype(np.float32)
        if (np.any(self._support_idx < 0)
                or np.any(self._support_idx >= cloth.n_particles)):
            raise RuntimeError("Cloth visual supports reference another entity.")
        weight_sum = self._support_weights.sum(axis=1, keepdims=True)
        if np.any(weight_sum <= 0):
            raise RuntimeError("Cloth visual support weights are empty.")
        self._support_weights /= weight_sum

        context = self._context
        # Install a color attribute before each replacement primitive gets a
        # GL buffer. Adding an attribute to an already uploaded primitive would
        # leave its VAO/layout stale. Replacing only this rendering node also
        # works when the native viewer has already rendered scene.build().
        with self._lock:
            render_positions = qd_to_numpy(cloth.solver.vverts_render.pos)[sl]
            for env_idx in context.rendered_envs_idx:
                key = (env_idx, cloth.uid)
                old_node = context.static_nodes[key]
                if len(old_node.mesh.primitives) != 1:
                    raise RuntimeError("Expected one visual primitive for cloth.")
                old = old_node.mesh.primitives[0]
                positions = context._scene.reorder_vertices(
                    old_node, render_positions[:, env_idx])
                dry_colors = (np.ones((len(positions), 4), dtype=np.float32)
                              if old.color_0 is None else old.color_0.copy())
                kwargs = {name: getattr(old, name) for name in (
                    "normals", "tangents", "texcoord_0", "texcoord_1",
                    "joints_0", "weights_0", "indices", "material", "mode",
                    "targets", "poses", "stream_positions", "stream_poses",
                    "vertex_mapping", "double_sided", "is_floor", "envs")}
                primitive = pyrender.Primitive(
                    positions=positions, color_0=dry_colors.copy(), **kwargs)
                _vertex_buffer(primitive)  # Validate layout before replacing.
                mesh = pyrender.Mesh(
                    [primitive], name=old_node.mesh.name,
                    weights=old_node.mesh.weights,
                    is_visible=old_node.mesh.is_visible,
                    is_marker=old_node.mesh.is_marker)
                pose = old_node.matrix.copy()
                # Drop queued uploads for the replaced primitive. Otherwise
                # the next flush could upload to its retired GL buffers.
                for buffer_key in tuple(context.jit._buffer):
                    if buffer_key[0] is old:
                        del context.jit._buffer[buffer_key]
                context.remove_node_seg(old_node)
                context.remove_node(old_node)
                context.add_static_node(cloth, mesh, i_b=env_idx, pose=pose)
                node = context.static_nodes[key]
                self._nodes.append((node, dry_colors))
            # Let native PBD position/normal updating run on the next render.
            context._t = -1

    def update(self, saturation):
        """Display an array of ``cloth.n_particles`` values in [0, 1]."""
        if hasattr(saturation, "detach"):
            saturation = saturation.detach().cpu().numpy()
        saturation = np.asarray(saturation, dtype=np.float32)
        if saturation.shape != (self.cloth.n_particles,):
            raise ValueError("Expected one saturation value per cloth particle.")
        if not np.isfinite(saturation).all():
            raise ValueError("Cloth saturation must be finite.")
        saturation = np.clip(saturation, 0.0, 1.0)
        if np.array_equal(saturation, self._saturation):
            return
        self._saturation = saturation.copy()
        self.visual_saturation = np.sum(
            self._support_weights * saturation[self._support_idx], axis=1)
        # Beer-Lambert-like color attenuation retains the woven cotton texture.
        # Brown darkens continuously with measured local liquid saturation.
        attenuation = np.exp(
            -self.visual_saturation[:, None] * np.array((2.0, 3.1, 4.6),
                                                        dtype=np.float32))
        with self._lock:
            for node, dry_colors in self._nodes:
                primitive = node.mesh.primitives[0]
                tint = self._context._scene.reorder_vertices(node, attenuation)
                colors = dry_colors.copy()
                colors[:, :3] *= tint
                primitive.color_0 = colors
                self._context.jit.update_buffer(
                    node, "vertex", _vertex_buffer(primitive))

    def reset(self):
        """Return to the original dry woven cotton appearance."""
        self.update(np.zeros(self.cloth.n_particles, dtype=np.float32))
