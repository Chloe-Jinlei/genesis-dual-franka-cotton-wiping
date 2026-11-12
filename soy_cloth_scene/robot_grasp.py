"""Contact-gated local cloth constraints for real Franka hand/finger links.

Install before ``scene.build()``. After the arms physically approach and close,
``attach_grasp`` checks the current TCP, current finger-pad geometry and only the
explicitly requested cloth particles. It preserves their current local offsets;
it never moves cloth into a gripper. All other cloth particles stay simulated.

This is an ideal no-slip attachment after a geometric contact gate. It does not
prove force closure and its constraint load is not fed back into the robot.
Call ``detach(cloth)`` before resetting a scene: Genesis 1.4.3 does not clear its
particle attachment table on reset. Coupled checkpoints need that table too if
they are captured while a grasp is active.
"""
from importlib.metadata import version
import numpy as np


TCP_LOCAL = np.array((0.0, 0.0, 0.1029), dtype=float)


def cloth_contact_radius(cloth):
    """Collision shell thickness is independent of mesh vertex spacing."""
    return float(getattr(cloth, "collision_radius_m", cloth.particle_size/2.0))
# Exact box centres and half sizes from the installed Panda MJCF, in each
# authored finger frame. Both fingers use the same boxes; right finger rotates
# by pi about its Z axis, which the actual link transform already contains.
PAD_CENTRES = np.array(((0, .0055, .0445), (.0055, .002, .05),
    (-.0055, .002, .05), (.0055, .002, .0395), (-.0055, .002, .0395)))
PAD_HALF_SIZES = np.array(((.0085, .004, .0085), (.003, .002, .003),
    (.003, .002, .003), (.003, .002, .0035), (.003, .002, .0035)))


def _pad_gap_geometry(particle_radius):
    """Relate central-pad gap to the narrower authored distal pad gap.

    The mirrored finger frames add twice the inner-face offset difference.
    In this Panda MJCF the central inner face is at y=1.5 mm while the four
    smaller pads reach y=0, so central gap = distal gap + 3 mm.
    """
    inner_faces = PAD_CENTRES[:, 1]-PAD_HALF_SIZES[:, 1]
    offset = float(2*(inner_faces[0]-inner_faces.min()))
    diameter = 2*float(particle_radius)
    return {"central_to_distal_pad_gap_offset_m": offset,
            "collision_diameter_m": diameter,
            "maximum_distal_pad_gap_m": 3*diameter,
            "maximum_central_pad_gap_m": offset+3*diameter,
            "derivation": "2*(central inner-face y - minimum pad inner-face y) + 3*collision diameter",
            "geometry_source": "authored Panda PAD_CENTRES/PAD_HALF_SIZES in actual mirrored finger frames"}


def _array(value):
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value, dtype=float)


def _rotation(quaternion):
    q = _array(quaternion).reshape(4)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid robot link quaternion")
    w, x, y, z = q/norm
    return np.array(((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
        (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
        (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y))))


def _link_pose(link):
    # Authored link frames match MJCF pad coordinates and TCP_LOCAL. Native
    # fix_particles_to_link itself stores offsets in the solver's link frame.
    return _array(link.get_pos()).reshape(3), _rotation(link.get_quat())


def _pad_box_distances(points, finger):
    pos, rotation = _link_pose(finger)
    local = (points-pos) @ rotation
    delta = np.maximum(np.abs(local[:, None]-PAD_CENTRES)-PAD_HALF_SIZES, 0.0)
    return np.linalg.norm(delta, axis=2)


def _pad_distance(points, finger):
    return _pad_box_distances(points, finger).min(axis=1)


def tcp_position(robot):
    """Current grasp-centre world position, from the actual hand transform."""
    pos, rotation = _link_pose(robot.get_link("hand"))
    return pos+rotation @ TCP_LOCAL


def pad_contact_report(robot, cloth):
    """Measure all cloth nodes against the current physical fingertip pad boxes.

    A contact candidate is at most cloth-particle radius + 0.3 mm from a pad's
    box surface. This tighter monitor does not use the grasp gate's 1.5 mm
    capture tolerance. It reports geometric collision proximity, not a measured
    pressure or a solver contact force. No state is changed.
    """
    if robot._scene is not cloth._scene or cloth._scene.n_envs != 0:
        raise ValueError("Robot and cloth must share one unbatched scene")
    points = _array(cloth.get_particles_pos())
    if points.shape != (cloth.n_particles, 3) or not np.isfinite(points).all():
        raise ValueError("Invalid cloth positions for pad contact measurement")
    particle_radius = cloth_contact_radius(cloth)
    tolerance = 0.0003
    threshold = particle_radius+tolerance
    fingers = {}
    union = np.zeros(cloth.n_particles, dtype=bool)
    for name in ("left_finger", "right_finger"):
        distances = _pad_box_distances(points, robot.get_link(name))
        minima = distances.min(axis=1)
        contact = minima <= threshold
        union |= contact
        pads = [{"pad_index": index+1,
                 "minimum_distance_m": float(distances[:, index].min()),
                 "contact_particle_indices": np.flatnonzero(distances[:, index] <= threshold).tolist(),
                 "contact_count": int(np.count_nonzero(distances[:, index] <= threshold))}
                for index in range(len(PAD_CENTRES))]
        fingers[name] = {
            "minimum_pad_distance_m": float(minima.min()),
            "contact_particle_indices": np.flatnonzero(contact).tolist(),
            "contact_count": int(contact.sum()), "pads": pads,
        }
    contact_counts = [finger["contact_count"] for finger in fingers.values()]
    return {
        "robot_name": robot.name,
        "simulation_step": int(cloth._scene.sim.cur_step_global),
        "particle_radius_m": particle_radius, "tolerance_m": tolerance,
        "contact_distance_threshold_m": threshold,
        "minimum_pad_distance_m": min(f["minimum_pad_distance_m"] for f in fingers.values()),
        "contact_particle_indices": np.flatnonzero(union).tolist(),
        "contact_count": int(union.sum()),
        "any_finger_contact": bool(any(contact_counts)),
        "both_fingers_contact": bool(all(contact_counts)),
        "fingers": fingers,
        "evidence_type": "actual finger-pad box geometry against PBD particle spheres",
        "pressure_or_force_measured": False,
    }


def edge_patch_indices(cloth, side, *, center_xy=(0.0, -0.23),
                       x_offset=0.13, half_width=0.0105,
                       half_length=0.0105, reference_positions=None):
    """Explicit small patch at the centre of a left/right short cloth edge.

    Select before grasping, preferably after initial settling, and retain these
    material indices through the motion. The default centre is the dual-arm
    wiping scene's authored (0, -0.23) m; pass another centre for other scenes.
    """
    if side not in ("left", "right"):
        raise ValueError("side must be 'left' or 'right'")
    points = (_array(cloth.get_particles_pos()) if reference_positions is None
              else _array(reference_positions))
    if points.shape != (cloth.n_particles, 3):
        raise ValueError("Expected one reference position per cloth particle")
    target = np.asarray(center_xy, dtype=float).copy()
    target[0] += -x_offset if side == "left" else x_offset
    inside = ((np.abs(points[:, 0]-target[0]) <= half_width)
              & (np.abs(points[:, 1]-target[1]) <= half_length))
    return np.flatnonzero(inside)


def install_rigid_pbd_friction(scene, *, preserve_integrated_velocity=False):
    """Enable the existing local Coulomb adapter before exact-link attachment.

    Copied from ``bear_arms_rigid_scene/pbd_adapter.py``. This adds position-level
    Coulomb friction to the installed legacy solver's normal-only PBD contacts.
    Install this FIRST, then ``install_grasp_adapter``, both before scene build.
    The smaller cloth/rigid coefficient sets the actual friction limit.
    """
    import quadrants as qd
    import genesis as gs
    import genesis.utils.sdf as sdf
    from genesis.engine.couplers.legacy_coupler import LegacyCoupler
    from genesis.utils import array_class

    if version("genesis-world") != "1.4.3" or scene.is_built:
        raise RuntimeError("Install the Genesis 1.4.3 friction adapter before build")
    if getattr(scene, "_contact_grasp_adapter", None) is not None:
        raise RuntimeError("Install rigid/PBD friction BEFORE install_grasp_adapter")
    if getattr(scene, "_cloth_friction_adapter", None) is not None:
        return dict(scene._cloth_friction_adapter)
    original = scene.sim.coupler
    if type(original) is not LegacyCoupler:
        raise RuntimeError("Rigid/PBD friction expects the unmodified legacy coupler")

    @qd.data_oriented
    class FrictionCoupler(LegacyCoupler):
        @qd.func
        def _func_pbd_collide_with_rigid_geom(
            self, i, pos_world, vel, mass, normal_prev, geom_idx, batch_idx,
            geoms_state: array_class.GeomsState,
            geoms_info: array_class.GeomsInfo,
            links_state: array_class.LinksState,
            sdf_info: array_class.SDFInfo,
            rigid_info: array_class.RigidInfo,
            collider_static_config: qd.template(),
        ):
            distance = sdf.sdf_func_world(
                geom_idx, batch_idx, pos_world, geoms_state, geoms_info, sdf_info)
            normal = sdf.sdf_func_normal_world(
                geom_idx, batch_idx, pos_world, geoms_state, geoms_info,
                rigid_info, sdf_info, collider_static_config)
            new_pos, new_vel = pos_world, vel
            depth = self.pbd_solver.particle_size / 2 - distance
            if depth > 0:
                new_pos = pos_world + normal * depth
                previous = self.pbd_solver.particles_reordered[i, batch_idx].ipos
                if self.pbd_solver.particles_reordered[i, batch_idx].free:
                    rigid_velocity = self.rigid_solver._func_vel_at_point(
                        pos_world=pos_world, link_idx=geoms_info.link_idx[geom_idx],
                        i_b=batch_idx, links_state=links_state)
                    relative_delta = new_pos-previous-rigid_velocity*self.pbd_solver._substep_dt
                    tangent = relative_delta-normal*relative_delta.dot(normal)
                    tangent_length = tangent.norm()
                    mu_s = qd.min(self.pbd_solver.particles_info_reordered[i, batch_idx].mu_s,
                                  geoms_info.coup_friction[geom_idx])
                    mu_k = qd.min(self.pbd_solver.particles_info_reordered[i, batch_idx].mu_k,
                                  geoms_info.coup_friction[geom_idx])
                    if tangent_length > 1e-12:
                        fraction = 1.0
                        if tangent_length > mu_s*depth:
                            fraction = qd.min(1.0, mu_k*depth/tangent_length)
                        new_pos -= fraction*tangent
                if qd.static(preserve_integrated_velocity):
                    # Preserve the integrated velocity; add only this contact's
                    # position-projection impulse. Rebuilding from world ipos
                    # would discard small damping updates again in float32.
                    new_vel = vel+(new_pos-pos_world)/self.pbd_solver._substep_dt
                else:
                    new_vel = (new_pos-previous)/self.pbd_solver._substep_dt
                # Preserve the native reciprocal rigid force, including friction.
                force = -mass*(new_vel-vel)/self.rigid_solver._substep_dt
                self.rigid_solver._func_apply_coupling_force(
                    geoms_info.link_idx[geom_idx], batch_idx, pos_world, force, links_state)
            return new_pos, new_vel, normal

    scene.sim._coupler = FrictionCoupler(scene.sim, original.options)
    report = {
        "adapter": "scene-local Genesis 1.4.3 legacy Coulomb friction",
        "source": "bear_arms_rigid_scene/pbd_adapter.py:install_rigid_pbd_friction",
        "method": "position-level static/kinetic friction limited by normal SDF projection",
        "coefficient_combination": "minimum(particle coefficient, rigid coup_friction)",
        "applies_to": "free PBD particles contacting coupled rigid geometry",
        "normal_projection_unchanged": True, "reciprocal_rigid_force": True,
        "world_position_pinning": False, "installed_genesis_modified": False,
    }
    if preserve_integrated_velocity:
        report["velocity_update"] = "incoming integrated velocity plus this contact projection displacement / substep_dt"
    scene._cloth_friction_adapter = report
    return dict(report)


def install_grasp_adapter(scene):
    """Install the existing exact-link kernel locally before scene build.

    The kernel is the reviewed ``install_exact_link_attachments`` algorithm in
    ``bear_arms_rigid_scene/pbd_adapter.py``. No Genesis source is modified.
    """
    import genesis as gs
    import quadrants as qd
    from genesis.utils import array_class
    from genesis.utils.geom import qd_transform_by_trans_quat

    if version("genesis-world") != "1.4.3" or scene.is_built:
        raise RuntimeError("Install the Genesis 1.4.3 grasp adapter before build")
    if getattr(scene, "_contact_grasp_adapter", None) is not None:
        return dict(scene._contact_grasp_adapter)

    @qd.data_oriented
    class ExactLinkAttachments:
        def __init__(self):
            self.coupler = scene.sim.coupler
            self.pbd = scene.sim.pbd_solver

        @qd.kernel
        def solve(self, clamped_inv_dt: gs.qd_float,
                  links_state: array_class.LinksState):
            for i_p, i_env in qd.ndrange(self.pbd._n_particles, self.pbd._B):
                link = self.coupler.particle_attach_info[i_p, i_env].link_idx
                if link >= 0:
                    local = self.coupler.particle_attach_info[i_p, i_env].local_pos
                    target = qd_transform_by_trans_quat(
                        local, links_state.pos[link, i_env], links_state.quat[link, i_env])
                    i_rp = self.pbd.particles_ng[i_p, i_env].reordered_idx
                    previous = self.pbd.particles_reordered[i_rp, i_env].ipos
                    self.pbd.particles_reordered[i_rp, i_env].pos = target
                    self.pbd.particles_reordered[i_rp, i_env].vel = (
                        target-previous)/self.pbd.substep_dt

    adapter = ExactLinkAttachments()
    scene.sim.coupler.kernel_pbd_rigid_solve_animate_particles_by_link = adapter.solve
    report = {
        "model": "contact-gated exact local no-slip constraint on configured actual Franka hand or finger links",
        "kernel_source": "bear_arms_rigid_scene/pbd_adapter.py:install_exact_link_attachments",
        "geometry_source": "Genesis assets/xml/franka_emika_panda/panda.xml fingertip pad boxes",
        "tcp_local_m": TCP_LOCAL.tolist(), "genesis_version": version("genesis-world"),
        "free_particles_repositioned": False, "actual_robot_link_transform": True,
        "grasp_load_transferred_to_arm": False, "force_closure_validated": False,
        "native_genesis_source_modified": False,
    }
    scene._contact_grasp_adapter = report
    return dict(report)


def _indices(cloth, indices):
    ids = np.asarray(indices)
    if ids.ndim != 1 or (len(ids) and not np.issubdtype(ids.dtype, np.integer)):
        raise ValueError("indices must be a 1-D integer array of local cloth particles")
    ids = np.unique(ids.astype(np.int64))
    if np.any(ids < 0) or np.any(ids >= cloth.n_particles):
        raise ValueError("Cloth particle index out of range")
    return ids


def _connected_contact_patch(points, faces, tcp, left_ids, right_ids, available,
                             radius=0.025, minimum_each=2, max_path_length=None):
    """One topology-connected patch with contacts on both jaws, plus a path."""
    import heapq
    local = (np.linalg.norm(points-tcp, axis=1) <= radius) & available
    adjacency = {int(i): set() for i in np.flatnonzero(local)}
    faces = np.asarray(faces, dtype=np.int64)
    for a, b, c in faces:
        for u, v in ((int(a), int(b)), (int(b), int(c)), (int(c), int(a))):
            if local[u] and local[v]:
                adjacency[u].add(v)
                adjacency[v].add(u)
    left, right = set(map(int, left_ids)), set(map(int, right_ids))
    visited, candidates = set(), []
    for start in sorted(adjacency):
        if start in visited:
            continue
        component, stack = set(), [start]
        visited.add(start)
        while stack:
            current = stack.pop()
            component.add(current)
            for neighbor in sorted(adjacency[current]):
                if neighbor not in visited:
                    visited.add(neighbor)
                    stack.append(neighbor)
        lc, rc = left & component, right & component
        if len(lc) >= minimum_each and len(rc) >= minimum_each:
            candidates.append((component, lc, rc))
    evidence = {"connectivity_radius_m": radius,
                "minimum_contact_nodes_per_finger": minimum_each,
                "connected_bilateral_patch_found": False,
                "left_contact_indices": [], "right_contact_indices": [],
                "minimum_connection_path_indices": [],
                "minimum_connection_path_length_m": None}
    if not candidates:
        return np.empty(0, dtype=np.int64), evidence
    component, lc, rc = max(candidates,
        key=lambda item: (min(len(item[1]), len(item[2])), len(item[1] | item[2]), -min(item[0])))
    # Dijkstra over actual mesh edges gives a concrete shortest local surface
    # path joining the two contact sets. No spatial-neighbor shortcuts are used.
    distances = {i: 0.0 for i in lc}
    previous = {i: None for i in lc}
    queue = [(0.0, i) for i in sorted(lc)]
    heapq.heapify(queue)
    endpoint = None
    while queue:
        distance, node = heapq.heappop(queue)
        if distance != distances[node]:
            continue
        if node in rc:
            endpoint = node
            break
        for neighbor in sorted(adjacency[node]):
            candidate = distance+float(np.linalg.norm(points[node]-points[neighbor]))
            if candidate < distances.get(neighbor, float("inf")):
                distances[neighbor], previous[neighbor] = candidate, node
                heapq.heappush(queue, (candidate, neighbor))
    path = []
    node = endpoint
    while node is not None:
        path.append(node)
        node = previous[node]
    path.reverse()
    evidence.update(connected_bilateral_patch_found=True,
        connected_component_particle_count=len(component),
        left_contact_indices=sorted(lc), right_contact_indices=sorted(rc),
        minimum_connection_path_indices=path,
        minimum_connection_path_length_m=float(distances[endpoint]),
        minimum_connection_path_edges=len(path)-1,
        path_maximum_tcp_distance_m=float(np.linalg.norm(points[path]-tcp, axis=1).max()))
    if max_path_length is not None:
        evidence["maximum_connection_path_length_m"] = max_path_length
        if distances[endpoint] > max_path_length:
            evidence.update(connected_bilateral_patch_found=False,
                            rejection="Shortest local mesh path exceeds the surface-length limit")
            return np.empty(0, dtype=np.int64), evidence
    return np.array(sorted(lc | rc), dtype=np.int64), evidence


def _minimum_reference_safe_subset(points, reference, faces, tcp, left_ids,
                                   right_ids, available, graph_radius,
                                   max_path_length, maximum_extension,
                                   minimum_bound_per_finger=2,
                                   preserve_contact_geometry=False,
                                   collision_diameter_m=None,
                                   require_disjoint_representatives=False):
    """Choose minimal bound representatives from a verified two-per-jaw patch.

    Pairwise current distances cannot exceed the corresponding material-space
    distances by more than the requested limit. All fixed pairs are checked,
    including pairs which are not mesh neighbors. Strict contact geometry also
    limits compression of direct material edges and overlapping selected
    collision spheres; non-neighbor folding remains allowed above that floor.
    Strict candidates need all incident mesh edges within 5% absolute strain
    before binding. This rejects already distorted neighborhoods, but does not
    supply the edge/face collision detection missing from native PBD.
    One representative per jaw still requires two supporting contacts per jaw
    and at least two distinct bound nodes. It never weakens the contact gate.
    """
    from itertools import combinations
    from scipy.sparse import csr_matrix
    from scipy.sparse.csgraph import dijkstra

    if (not isinstance(minimum_bound_per_finger, (int, np.integer))
            or isinstance(minimum_bound_per_finger, (bool, np.bool_))
            or minimum_bound_per_finger not in (1, 2)):
        raise ValueError("minimum_bound_per_finger must be 1 or 2")
    if preserve_contact_geometry and (collision_diameter_m is None
            or not np.isfinite(collision_diameter_m) or collision_diameter_m <= 0):
        raise ValueError("Strict contact geometry requires a positive collision diameter")
    penetration_tolerance = 0.00005
    left, right = np.unique(np.asarray(left_ids, np.int64)), np.unique(np.asarray(right_ids, np.int64))
    contact_ids = np.union1d(left, right)
    evidence = {"maximum_allowed_fixed_pair_extension": maximum_extension,
                "minimum_contact_subset_found": False,
                "contact_nodes_before_subset": len(contact_ids),
                "minimum_supporting_contact_nodes_per_finger": 2,
                "supporting_left_contact_indices": left.tolist(),
                "supporting_right_contact_indices": right.tolist(),
                "minimum_bound_per_finger": minimum_bound_per_finger,
                "fixed_point_pair_max_extension": None,
                "fixed_point_pair_max_length_ratio": None,
                "fixed_edge_max_compression": None,
                "preserve_contact_geometry": bool(preserve_contact_geometry),
                "maximum_allowed_fixed_edge_compression": maximum_extension if preserve_contact_geometry else None,
                "selected_pair_collision_diameter_m": collision_diameter_m if preserve_contact_geometry else None,
                "collision_penetration_tolerance_m": penetration_tolerance if preserve_contact_geometry else None,
                "maximum_allowed_incident_edge_absolute_strain": .05 if preserve_contact_geometry else None,
                "require_disjoint_representatives": bool(require_disjoint_representatives)}
    if min(len(left), len(right)) < 2:
        return np.empty(0, dtype=np.int64), evidence
    index = {int(p): i for i, p in enumerate(contact_ids)}
    ref = reference[contact_ids]
    cur = points[contact_ids]
    ref_distance = np.linalg.norm(ref[:, None]-ref[None], axis=2)
    current_distance = np.linalg.norm(cur[:, None]-cur[None], axis=2)
    ratio = np.divide(current_distance, ref_distance,
        out=np.full_like(ref_distance, np.inf), where=ref_distance > 1e-12)
    np.fill_diagonal(ratio, 1.0)

    # Test shortest mesh paths for candidate pairs once, rather than rebuilding
    # topology for every combination. The final chosen subset is also passed
    # through the original topology gate to produce its concrete path evidence.
    local = available & (np.linalg.norm(points-tcp, axis=1) <= graph_radius)
    faces = np.asarray(faces, dtype=np.int64)
    edges = np.unique(np.sort(np.concatenate((faces[:, [0, 1]], faces[:, [1, 2]],
                                             faces[:, [2, 0]])), axis=1), axis=0)
    all_edges = edges
    reference_edge_length = np.linalg.norm(reference[edges[:, 0]]-reference[edges[:, 1]], axis=1)
    current_edge_length = np.linalg.norm(points[edges[:, 0]]-points[edges[:, 1]], axis=1)
    edge_strain = np.divide(current_edge_length, reference_edge_length,
        out=np.full_like(reference_edge_length, np.inf), where=reference_edge_length > 1e-12)-1.
    incident_absolute_strain = np.zeros(len(points))
    np.maximum.at(incident_absolute_strain, edges.reshape(-1), np.repeat(np.abs(edge_strain), 2))
    evidence["contact_candidate_incident_max_absolute_strain"] = {
        str(int(i)): float(incident_absolute_strain[i]) for i in contact_ids}
    evidence["contact_candidates_rejected_by_incident_strain"] = (
        contact_ids[incident_absolute_strain[contact_ids] > .05+1e-8].tolist()
        if preserve_contact_geometry else [])
    contact_edges = np.zeros((len(contact_ids), len(contact_ids)), dtype=bool)
    for u, v in edges:
        if int(u) in index and int(v) in index:
            contact_edges[index[int(u)], index[int(v)]] = True
            contact_edges[index[int(v)], index[int(u)]] = True
    edges = edges[local[edges].all(axis=1)]
    lengths = np.linalg.norm(points[edges[:, 0]]-points[edges[:, 1]], axis=1)
    graph = csr_matrix((np.r_[lengths, lengths],
        (np.r_[edges[:, 0], edges[:, 1]], np.r_[edges[:, 1], edges[:, 0]])),
        shape=(len(points), len(points)))
    paths = dijkstra(graph, directed=False, indices=left)[:, right]
    left_index = {int(p): i for i, p in enumerate(left)}
    right_index = {int(p): i for i, p in enumerate(right)}
    best = None
    viable = 0
    for lp in combinations(map(int, left), minimum_bound_per_finger):
        for rp in combinations(map(int, right), minimum_bound_per_finger):
            if require_disjoint_representatives and set(lp) & set(rp):
                continue
            chosen = np.array(sorted(set(lp+rp)), dtype=np.int64)
            if len(chosen) < 2:
                continue
            rows = np.array([index[int(p)] for p in chosen])
            ratios = ratio[np.ix_(rows, rows)]
            pair_rows, pair_cols = np.triu_indices(len(chosen), 1)
            maximum_ratio = float(ratios[pair_rows, pair_cols].max())
            if maximum_ratio > 1.0+maximum_extension+1e-8:
                continue
            direct_edges = contact_edges[np.ix_(rows, rows)][pair_rows, pair_cols]
            if preserve_contact_geometry:
                if np.any(incident_absolute_strain[chosen] > .05+1e-8):
                    continue
                if np.any(ratios[pair_rows, pair_cols][direct_edges] < 1.0-maximum_extension-1e-8):
                    continue
                pair_reference = ref_distance[np.ix_(rows, rows)][pair_rows, pair_cols]
                pair_current = current_distance[np.ix_(rows, rows)][pair_rows, pair_cols]
                # Native self-collision activates when rest_dist > diameter.
                # Include equality conservatively rather than freezing a pair
                # that roundoff could place on the native active side.
                if np.any((pair_reference >= collision_diameter_m)
                          & (pair_current < collision_diameter_m-penetration_tolerance)):
                    continue
            shortest = paths[np.ix_([left_index[p] for p in lp],
                                    [right_index[p] for p in rp])].min()
            if shortest > max_path_length:
                continue
            viable += 1
            material_diameter = float(ref_distance[np.ix_(rows, rows)].max())
            # After all hard gates, prefer a minimally constrained, less
            # distorted neighborhood before compactness in strict wet mode.
            # Legacy ordering is unchanged for the already validated dry mode.
            if preserve_contact_geometry:
                score = (len(chosen), float(incident_absolute_strain[chosen].max()),
                         material_diameter, maximum_ratio, tuple(chosen))
            else:
                score = (len(chosen), material_diameter, maximum_ratio, tuple(chosen))
            if best is None or score < best[0]:
                best = (score, chosen, rows, ratios, lp, rp)
    evidence["strain_and_topology_safe_subset_count"] = viable
    if best is None:
        return np.empty(0, dtype=np.int64), evidence
    _, chosen, rows, ratios, bound_left, bound_right = best
    pair_rows, pair_cols = np.triu_indices(len(chosen), 1)
    worst = int(np.argmax(ratios[pair_rows, pair_cols]))
    a, b = int(pair_rows[worst]), int(pair_cols[worst])
    maximum_ratio = float(ratios[a, b])
    direct_edge_ratios = ratios[pair_rows, pair_cols][
        contact_edges[np.ix_(rows, rows)][pair_rows, pair_cols]]
    incident_edge_mask = np.isin(all_edges, chosen).any(axis=1)
    incident_edges = all_edges[incident_edge_mask]
    incident_strain = edge_strain[incident_edge_mask]
    worst_incident = int(np.argmax(np.abs(incident_strain)))
    evidence.update(minimum_contact_subset_found=True,
        contact_nodes_after_subset=len(chosen),
        selected_left_representative_indices=list(bound_left),
        selected_right_representative_indices=list(bound_right),
        selected_left_contact_indices=np.intersect1d(chosen, left).tolist(),
        selected_right_contact_indices=np.intersect1d(chosen, right).tolist(),
        supporting_contact_nodes_left_free=np.setdiff1d(contact_ids, chosen).tolist(),
        fixed_point_pair_max_extension=max(0.0, maximum_ratio-1.0),
        fixed_point_pair_max_length_ratio=maximum_ratio,
        fixed_edge_count=int(len(direct_edge_ratios)),
        fixed_edge_max_compression=(float(max(0.0, 1.0-direct_edge_ratios.min()))
                                    if len(direct_edge_ratios) else 0.0),
        fixed_edge_min_length_ratio=(float(direct_edge_ratios.min())
                                    if len(direct_edge_ratios) else None),
        fixed_pair_min_distance_m=float(current_distance[np.ix_(rows, rows)][pair_rows, pair_cols].min()),
        selected_incident_edge_count=int(len(incident_edges)),
        selected_incident_edge_max_absolute_strain=float(np.abs(incident_strain).max()),
        selected_incident_edge_max_extension=float(max(0., incident_strain.max())),
        selected_incident_edge_max_compression=float(max(0., -incident_strain.min())),
        selected_incident_worst_edge_indices=incident_edges[worst_incident].tolist(),
        selected_incident_worst_edge_signed_strain=float(incident_strain[worst_incident]),
        selected_incident_edge_indices=incident_edges.tolist(),
        selected_incident_edge_signed_strains=incident_strain.tolist(),
        selection_priority=("node count, incident absolute strain, material diameter, fixed pair extension"
                            if preserve_contact_geometry else "node count, material diameter, fixed pair extension"),
        worst_fixed_point_pair_indices=[int(chosen[a]), int(chosen[b])],
        worst_fixed_point_pair_reference_distance_m=float(ref_distance[rows[a], rows[b]]),
        worst_fixed_point_pair_current_distance_m=float(current_distance[rows[a], rows[b]]),
        attached_reference_material_patch_bbox_m=[reference[chosen].min(axis=0).tolist(),
                                                 reference[chosen].max(axis=0).tolist()],
        attached_reference_material_diameter_m=float(ref_distance[np.ix_(rows, rows)].max()))
    return chosen, evidence


def attach_grasp(robot, cloth, indices, *, max_tcp_distance=0.012,
                 max_jaw_gap=None, contact_tolerance=0.0015, min_particles=1,
                 gate_mode="same_particle", commanded_finger_opening=None,
                 reference_positions=None, max_fixed_pair_extension=0.02,
                 minimum_bound_per_finger=2, attachment_mode="hand",
                 preserve_contact_geometry=False):
    """Attach only requested nodes currently between both nearly closed pads.

    Returns an evidence report; a failed contact gate leaves the cloth untouched
    and returns ``attached=False``. ``same_particle`` requires both pads near
    every accepted node. ``bilateral_patch`` handles a folded cloth: at least two
    nodes touch each jaw within radius + 0.3 mm, all accepted nodes are within
    12 mm of the actual TCP and between the jaws, and one mesh-connected patch
    intersecting a 25 mm neighbourhood links both sides. Its graph includes a
    particle centre when its sphere intersects that region (25 mm plus particle
    radius), and the shortest mesh path cannot exceed that expanded diameter.
    Only contacting nodes are attached; the connecting interior remains free.
    The legacy central gap is at most three PBD diameters. In explicit strict
    contact-geometry mode this allowance applies to the narrower distal pads;
    the measured central gap additionally includes the authored 3 mm offset.
    Pass the currently commanded finger opening (normally 0) in this mode; that
    command is an additional check and never substitutes for measured geometry.
    With initial ``reference_positions``, only a minimal subset is bound,
    normally two nodes per jaw. ``minimum_bound_per_finger=1`` binds one distinct
    representative per jaw but preserves the two-per-jaw supporting contact
    gate. Other contacting nodes remain free under native contact and friction.
    Every chosen pair must stay within ``max_fixed_pair_extension`` of
    its initial material distance (at most 2%). The caller supplies candidate
    indices from the intended initial material corner patch; this function does
    not replace them with nearby particles from another part of the cloth.
    ``attachment_mode='finger_pads'`` requires the safe bilateral material
    subset and attaches its distinct left/right representatives to the actual
    corresponding finger links at their current offsets. This follows later
    finger motion instead of fixing nodes in the hand frame. Native contact,
    path and reference-length gates are unchanged. With
    ``preserve_contact_geometry=True``, all candidate spheres must clear every
    pad box within 50 micrometres, selected native collision pairs must not
    overlap beyond that tolerance, and direct selected mesh edges may not be
    locked in compression beyond the same material strain limit. Every mesh
    edge incident on a selected node must have at most 5% absolute strain;
    selection prefers less strained neighborhoods before material compactness.
    This does not detect edge/face intersection with pads or guarantee later
    contact convergence. This opt-in mode requires the reference mesh; False
    preserves the earlier dry grasp.
    """
    scene = cloth._scene
    if scene.n_envs != 0 or not scene.is_built:
        raise ValueError("Grasp requires one built unbatched scene")
    if getattr(scene, "_contact_grasp_adapter", None) is None:
        raise RuntimeError("Call install_grasp_adapter(scene) before scene.build()")
    if robot._scene is not scene:
        raise ValueError("Robot and cloth must belong to the same scene")
    if gate_mode not in ("same_particle", "bilateral_patch"):
        raise ValueError("Unknown grasp gate mode")
    if attachment_mode not in ("hand", "finger_pads"):
        raise ValueError("attachment_mode must be 'hand' or 'finger_pads'")
    if attachment_mode == "finger_pads" and (gate_mode != "bilateral_patch" or reference_positions is None):
        raise ValueError("Finger-pad attachment requires a bilateral patch and initial reference_positions")
    if not isinstance(preserve_contact_geometry, (bool, np.bool_)):
        raise ValueError("preserve_contact_geometry must be a boolean")
    if preserve_contact_geometry and (gate_mode != "bilateral_patch" or reference_positions is None):
        raise ValueError("Strict contact geometry requires a bilateral patch and initial reference_positions")
    if (not isinstance(minimum_bound_per_finger, (int, np.integer))
            or isinstance(minimum_bound_per_finger, (bool, np.bool_))
            or minimum_bound_per_finger not in (1, 2)):
        raise ValueError("minimum_bound_per_finger must be 1 or 2")
    if minimum_bound_per_finger == 1 and reference_positions is None:
        raise ValueError("One-per-finger binding requires initial reference_positions")
    particle_radius = cloth_contact_radius(cloth)
    gap_geometry = _pad_gap_geometry(particle_radius)
    bilateral_gap_limit = (gap_geometry["maximum_central_pad_gap_m"]
                           if preserve_contact_geometry else 6*particle_radius)
    if max_jaw_gap is None:
        max_jaw_gap = bilateral_gap_limit if gate_mode == "bilateral_patch" else 0.012
    if gate_mode == "bilateral_patch":
        contact_tolerance = 0.0003
        if max_tcp_distance > 0.012 or max_jaw_gap > bilateral_gap_limit+1e-9:
            raise ValueError("Bilateral patch gate exceeds the 12 mm TCP or geometry-derived gap limit")
    limits = np.array((max_tcp_distance, max_jaw_gap, contact_tolerance), dtype=float)
    if not np.isfinite(limits).all() or np.any(limits < 0) or min_particles < 1:
        raise ValueError("Invalid grasp-gate limits")
    ids = _indices(cloth, indices)
    points = _array(cloth.get_particles_pos())
    if not np.isfinite(points).all():
        raise ValueError("Cloth particle positions must be finite")
    reference = None if reference_positions is None else _array(reference_positions)
    if reference is not None:
        if reference.shape != points.shape or not np.isfinite(reference).all():
            raise ValueError("reference_positions must contain finite initial positions of all cloth particles")
        if not np.isfinite(max_fixed_pair_extension) or not 0 <= max_fixed_pair_extension <= 0.02:
            raise ValueError("The fixed-point material extension limit must be between zero and 2%")
    hand = robot.get_link("hand")
    fingers = (robot.get_link("left_finger"), robot.get_link("right_finger"))
    tcp = tcp_position(robot)
    pos_hand, rotation_hand = _link_pose(hand)
    inner = []
    for finger in fingers:
        pos, rotation = _link_pose(finger)
        inner.append(pos+rotation @ np.array((0.0, 0.0015, 0.0445)))
    jaw_gap = float(np.linalg.norm(inner[0]-inner[1]))
    tcp_distance = np.linalg.norm(points[ids]-tcp, axis=1)
    left_distance = _pad_distance(points[ids], fingers[0])
    right_distance = _pad_distance(points[ids], fingers[1])
    shell_nonpenetrating = ((left_distance >= particle_radius-0.00005)
                           & (right_distance >= particle_radius-0.00005))
    padding = cloth_contact_radius(cloth)+contact_tolerance
    local = (points[ids]-pos_hand) @ rotation_hand
    inner_local = (np.asarray(inner)-pos_hand) @ rotation_hand
    between = ((local[:, 1] >= inner_local[:, 1].min()-padding)
               & (local[:, 1] <= inner_local[:, 1].max()+padding))
    links = scene.sim.coupler.particle_attach_info.link_idx.to_numpy()
    attached_links = links[cloth.particle_start+ids, 0]
    q_indices = [robot.get_joint(f"finger_joint{i}").dofs_idx_local[0] for i in (1, 2)]
    actual_fingers = _array(robot.get_dofs_position(q_indices))
    patch_evidence = None
    subset_evidence = None
    bound_path_evidence = None
    command = None
    if gate_mode == "same_particle":
        eligible = ((tcp_distance <= max_tcp_distance) & between
                    & (left_distance <= padding) & (right_distance <= padding)
                    & (attached_links < 0) & (jaw_gap <= max_jaw_gap))
        selected = ids[eligible]
    else:
        # Sphere centres must lie inside the actual central inner jaw planes;
        # only the 0.3 mm numerical contact tolerance extends that interval.
        between = ((local[:, 1] >= inner_local[:, 1].min()-contact_tolerance)
                   & (local[:, 1] <= inner_local[:, 1].max()+contact_tolerance))
        local_candidate = (tcp_distance <= max_tcp_distance) & between & (attached_links < 0)
        if preserve_contact_geometry:
            local_candidate &= shell_nonpenetrating
        left_ids = ids[local_candidate & (left_distance <= padding)]
        right_ids = ids[local_candidate & (right_distance <= padding)]
        available = links[cloth.particle_start:cloth.particle_start+cloth.n_particles, 0] < 0
        region_radius = 0.025
        particle_radius = cloth_contact_radius(cloth)
        # Connectivity is between finite PBD spheres, not zero-size vertices:
        # a sphere whose centre is just beyond 25 mm still intersects the local
        # region. This also avoids severing an otherwise local folded bridge
        # solely because a sampled vertex lies < one radius over its boundary.
        graph_radius = region_radius+particle_radius
        selected, patch_evidence = _connected_contact_patch(
            points, cloth.mesh.faces, tcp, left_ids, right_ids, available,
            radius=graph_radius, max_path_length=2*graph_radius)
        patch_evidence.update(left_near_pad_count=len(left_ids), right_near_pad_count=len(right_ids),
                              attached_bridge_nodes=False,
                              connectivity_region_radius_m=region_radius,
                              connectivity_particle_sphere_radius_m=particle_radius)
        if commanded_finger_opening is not None:
            command = np.broadcast_to(np.asarray(commanded_finger_opening, dtype=float), (2,)).copy()
    if reference is not None and len(selected):
        if gate_mode == "bilateral_patch":
            safe_left = np.array(patch_evidence["left_contact_indices"], dtype=np.int64)
            safe_right = np.array(patch_evidence["right_contact_indices"], dtype=np.int64)
        else:
            safe_left = safe_right = selected
            graph_radius = 0.025+cloth_contact_radius(cloth)
            available = links[cloth.particle_start:cloth.particle_start+cloth.n_particles, 0] < 0
        selected, subset_evidence = _minimum_reference_safe_subset(
            points, reference, cloth.mesh.faces, tcp, safe_left, safe_right,
            available, graph_radius, 2*graph_radius, max_fixed_pair_extension,
            minimum_bound_per_finger=minimum_bound_per_finger,
            preserve_contact_geometry=preserve_contact_geometry,
            collision_diameter_m=2*particle_radius,
            require_disjoint_representatives=attachment_mode == "finger_pads")
        if len(selected) and gate_mode == "bilateral_patch":
            selected, bound_path_evidence = _connected_contact_patch(
                points, cloth.mesh.faces, tcp,
                subset_evidence["selected_left_representative_indices"],
                subset_evidence["selected_right_representative_indices"],
                available, radius=graph_radius, minimum_each=minimum_bound_per_finger,
                max_path_length=2*graph_radius)
            # Keep the original >=2-per-jaw contact proof separate from the
            # smaller bound subset: reducing constraints never changes support.
    reasons = []
    if not len(ids):
        reasons.append("empty candidate patch")
    if jaw_gap > max_jaw_gap:
        reasons.append("actual finger-pad gap exceeds the closure threshold")
    if gate_mode == "bilateral_patch":
        if (command is None or not np.isfinite(command).all() or np.any(command < 0)
                or np.any(command > 0.0015) or np.any(command > actual_fingers+0.0003)):
            reasons.append("bilateral patch grasp requires an explicit near-closed finger command")
        if not patch_evidence["connected_bilateral_patch_found"]:
            reasons.append("no single local cloth component has at least two contact nodes at each finger")
    if len(selected) < min_particles:
        reasons.append("too few requested cloth particles satisfy the actual contact gate")
    if subset_evidence is not None and not subset_evidence["minimum_contact_subset_found"]:
        reasons.append("no requested per-jaw bound subset can be fixed without exceeding the reference material strain limit")
    attachment_plan = [("hand", hand, selected)]
    if attachment_mode == "finger_pads":
        attachment_plan = []
        valid = subset_evidence is not None and subset_evidence.get("minimum_contact_subset_found", False)
        left_bound = np.asarray((subset_evidence or {}).get("selected_left_representative_indices", []), np.int64)
        right_bound = np.asarray((subset_evidence or {}).get("selected_right_representative_indices", []), np.int64)
        valid = (valid and len(left_bound) == minimum_bound_per_finger
                 and len(right_bound) == minimum_bound_per_finger
                 and len(np.unique(left_bound)) == len(left_bound)
                 and len(np.unique(right_bound)) == len(right_bound)
                 and not len(np.intersect1d(left_bound, right_bound))
                 and np.array_equal(np.union1d(left_bound, right_bound), np.sort(selected))
                 and np.isin(left_bound, patch_evidence["left_contact_indices"]).all()
                 and np.isin(right_bound, patch_evidence["right_contact_indices"]).all()
                 and bound_path_evidence is not None
                 and bound_path_evidence["connected_bilateral_patch_found"])
        if not valid:
            reasons.append("finger-pad attachment needs distinct, explicit left/right representatives from a safe bilateral subset")
        elif int(fingers[0].idx) == int(fingers[1].idx):
            reasons.append("left and right finger links must be distinct")
        else:
            attachment_plan = [("left_finger", fingers[0], left_bound),
                               ("right_finger", fingers[1], right_bound)]
    report = {
        "attached": False, "robot_name": robot.name, "hand_link_idx": int(hand.idx),
        "gate_mode": gate_mode, "attachment_mode": attachment_mode,
        "preserve_contact_geometry": bool(preserve_contact_geometry),
        "attachment_links": [{"name": name, "link_idx": int(link.idx),
                              "particle_indices": assigned.tolist()}
                             for name, link, assigned in attachment_plan],
        "particle_link_owners": {},
        "minimum_bound_per_finger": minimum_bound_per_finger,
        "candidate_indices": ids.tolist(), "eligible_indices": selected.tolist(),
        "attached_indices": [], "reasons": reasons,
        "tcp_world_m": tcp.tolist(), "tcp_local_m": TCP_LOCAL.tolist(),
        "actual_finger_joint_positions_m": actual_fingers.tolist(),
        "actual_central_pad_gap_m": jaw_gap, "max_jaw_gap_m": max_jaw_gap,
        "pad_gap_geometry": gap_geometry,
        "gap_limit_includes_authored_pad_offset": bool(preserve_contact_geometry),
        "candidate_shell_nonpenetrating": shell_nonpenetrating.tolist(),
        "shell_penetration_tolerance_m": 0.00005 if preserve_contact_geometry else None,
        "max_tcp_distance_m": max_tcp_distance,
        "particle_radius_plus_contact_tolerance_m": padding,
        "candidate_tcp_distances_m": tcp_distance.tolist(),
        "candidate_left_pad_distances_m": left_distance.tolist(),
        "candidate_right_pad_distances_m": right_distance.tolist(),
        "simulation_step": int(scene.sim.cur_step_global),
        "position_setter_used": False, "force_closure_validated": False,
        "grasp_load_transferred_to_arm": False,
    }
    if gate_mode == "bilateral_patch":
        report.update(bilateral_patch=patch_evidence,
            bound_subset_connectivity=bound_path_evidence,
            commanded_finger_opening_m=None if command is None else command.tolist(),
            maximum_gap_particle_diameters=3,
            finger_motor_control_force_N=_array(robot.get_dofs_control_force(q_indices)).tolist(),
            motor_force_is_not_measured_contact_force=True)
    if reference is not None:
        report.update(reference_material_patch_bbox_m=(
            [reference[ids].min(axis=0).tolist(), reference[ids].max(axis=0).tolist()] if len(ids) else None),
            reference_material_candidate_count=len(ids),
            max_fixed_pair_extension=max_fixed_pair_extension,
            fixed_subset_selection=subset_evidence)
    if reasons:
        return report
    # This API stores the particles' CURRENT offsets and does not reposition
    # them. Each subsequent substep follows the measured rigid link transform.
    for _, link, assigned in attachment_plan:
        cloth.fix_particles_to_link(link.idx, particles_idx_local=assigned)
    after = _array(cloth.get_particles_pos())
    displacement = float(np.abs(after-points).max(initial=0.0))
    ownership = getattr(cloth, "_contact_grasp_nodes", {})
    attached_owners = {int(index): int(link.idx) for _, link, assigned in attachment_plan
                       for index in assigned}
    ownership.update(attached_owners)
    cloth._contact_grasp_nodes = ownership
    report.update(attached=True, attached_indices=selected.tolist(),
                  attached_count=len(selected),
                  particle_link_owners={str(index): link for index, link in attached_owners.items()},
                  remaining_unconstrained_count=cloth.n_particles-len(ownership),
                  immediate_max_position_change_m=displacement)
    return report


def detach(cloth, indices=None, *, robot=None):
    """Release this adapter's explicitly tracked nodes, preserving velocities."""
    ownership = getattr(cloth, "_contact_grasp_nodes", {})
    requested = np.array(sorted(ownership), dtype=np.int64) if indices is None else _indices(cloth, indices)
    wanted_links = None if robot is None else {
        int(robot.get_link(name).idx) for name in ("hand", "left_finger", "right_finger")}
    links = cloth._scene.sim.coupler.particle_attach_info.link_idx.to_numpy()
    released = [int(i) for i in requested if int(i) in ownership
                and (wanted_links is None or ownership[int(i)] in wanted_links)
                and links[cloth.particle_start+int(i), 0] == ownership[int(i)]]
    if released:
        cloth.release_particle(particles_idx_local=np.array(released, dtype=np.int64))
    for index in requested:
        if wanted_links is None or ownership.get(int(index)) in wanted_links:
            ownership.pop(int(index), None)
    cloth._contact_grasp_nodes = ownership
    return {"detached_indices": released, "detached_count": len(released),
            "position_setter_used": False, "velocity_setter_used": False}


def capture_grasp_state(cloth):
    """Return JSON-serializable attachment state for this cloth only.

    Save alongside the matching native scene state (including PBD ``free``
    flags) and absorption state. Link indices belong to the same scene topology.
    This captures original link-local offsets, so restoring never re-derives a
    grasp from the robot's current pose or moves the cloth to a new offset.
    """
    scene = cloth._scene
    if not scene.is_built or scene.n_envs != 0:
        raise ValueError("Grasp checkpoint requires one built unbatched scene")
    sl = slice(cloth.particle_start, cloth.particle_start+cloth.n_particles)
    info = scene.sim.coupler.particle_attach_info
    links = info.link_idx.to_numpy()[sl].copy()
    offsets = info.local_pos.to_numpy()[sl].copy()
    ownership = getattr(cloth, "_contact_grasp_nodes", {})
    # Ignore stale Python ownership after an external native detach. The actual
    # engine attachment fields are authoritative.
    ownership = {str(int(i)): int(link) for i, link in ownership.items()
                 if 0 <= int(i) < cloth.n_particles and links[int(i), 0] == int(link)}
    return {"format_version": 1, "n_particles": int(cloth.n_particles),
            "link_idx": links.tolist(), "local_pos": offsets.tolist(),
            "ownership": ownership}


def restore_grasp_state(cloth, state):
    """Restore a captured attachment slice after matching native physics state.

    Only this cloth's rows are changed. The full ``from_numpy`` writes preserve
    all other PBD entities, as required by the installed Quadrants field API.
    No particle position or velocity setter is used. Native PBD free flags must
    already match the native scene checkpoint being restored.
    """
    scene = cloth._scene
    if not scene.is_built or scene.n_envs != 0:
        raise ValueError("Grasp restore requires one built unbatched scene")
    expected = {"format_version", "n_particles", "link_idx", "local_pos", "ownership"}
    if not isinstance(state, dict) or set(state) != expected:
        raise ValueError("Invalid grasp checkpoint fields")
    if state["format_version"] != 1 or state["n_particles"] != cloth.n_particles:
        raise ValueError("Grasp checkpoint version or cloth particle count mismatch")
    links = np.asarray(state["link_idx"])
    offsets = np.asarray(state["local_pos"], dtype=float)
    if (links.shape != (cloth.n_particles, 1)
            or not np.issubdtype(links.dtype, np.integer)
            or offsets.shape != (cloth.n_particles, 1, 3)):
        raise ValueError("Invalid grasp checkpoint attachment array shape/type")
    if (np.any(links < -1) or np.any(links >= scene.sim.rigid_solver.n_links)
            or not np.isfinite(offsets).all()):
        raise ValueError("Invalid grasp checkpoint link index or local offset")
    if not isinstance(state["ownership"], dict):
        raise ValueError("Invalid grasp checkpoint ownership")
    ownership = {}
    for key, value in state["ownership"].items():
        try:
            index, link = int(key), int(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("Invalid grasp checkpoint ownership entry") from exc
        if (str(index) != str(key) or link != value or not 0 <= index < cloth.n_particles
                or link < 0 or links[index, 0] != link):
            raise ValueError("Grasp ownership does not match native attachment data")
        ownership[index] = link
    sl = slice(cloth.particle_start, cloth.particle_start+cloth.n_particles)
    free = cloth.solver.particles.free.to_numpy()[sl].reshape(cloth.n_particles, 1)
    if np.any((links >= 0) & free.astype(bool)):
        raise ValueError("Restore matching native PBD state before grasp state: attached nodes are still free")
    info = scene.sim.coupler.particle_attach_info
    all_links = info.link_idx.to_numpy()
    all_offsets = info.local_pos.to_numpy()
    all_links[sl] = links.astype(all_links.dtype, copy=False)
    all_offsets[sl] = offsets.astype(all_offsets.dtype, copy=False)
    with scene.visualizer.viewer_lock:
        info.link_idx.from_numpy(np.ascontiguousarray(all_links))
        info.local_pos.from_numpy(np.ascontiguousarray(all_offsets))
        cloth._contact_grasp_nodes = ownership
    return {"restored_attachment_count": int(np.count_nonzero(links >= 0)),
            "restored_owned_count": len(ownership),
            "position_setter_used": False, "velocity_setter_used": False}
