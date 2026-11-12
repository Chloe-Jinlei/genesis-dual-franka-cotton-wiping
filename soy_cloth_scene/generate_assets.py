"""Deterministic meshes and woven cotton texture for the soy-and-cloth scene.

``ensure_assets()`` returns absolute ``Path`` values under this directory's
``assets/`` directory:

* ``cloth_mesh``: 0.28 m × 0.18 m rectangular triangle grid, 41 × 27 vertices,
  centered on local X/Y = 0, with every vertex at local Z = 0. Positive face
  normals point toward +Z. UVs span [0, 1] in both directions. This is a
  single open sheet intended for ``gs.materials.PBD.Cloth``.
* ``cloth_texture``: warm ivory cotton weave, with a subtle stitched hem.
* ``soy_mesh``: a closed irregular puddle, exactly 0.20 m × 0.14 m in X/Y,
  bounds centered on local X/Y = 0, bottom Z = 0, maximum top Z = 0.002 m.
  Its rounded meniscus is a visual representation of spilled liquid.

All dimensions are meters; no Genesis imports or simulation builds are needed.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from PIL import Image


ASSETS_DIR = Path(__file__).resolve().parent / "assets"


def _write_text(path: Path, contents: str) -> None:
    if not path.exists() or path.read_text(encoding="utf-8") != contents:
        path.write_text(contents, encoding="utf-8")


def _cloth_obj() -> str:
    nx, ny = 41, 27
    lines = ["# Cotton sheet: 0.28 x 0.18 m; local XY plane; +Z normals", "o cotton_cloth"]
    for j in range(ny):
        for i in range(nx):
            lines.append(f"v {-0.14 + 0.28 * i / (nx - 1):.9f} {-0.09 + 0.18 * j / (ny - 1):.9f} 0.000000000")
    for j in range(ny):
        for i in range(nx):
            lines.append(f"vt {i / (nx - 1):.9f} {j / (ny - 1):.9f}")
    lines.append("s 1")
    for j in range(ny - 1):
        for i in range(nx - 1):
            a = j * nx + i + 1
            b, c, d = a + 1, a + nx + 1, a + nx
            # Alternating diagonals avoid giving the entire sheet one preferred
            # diagonal direction while preserving one UV for each vertex.
            tris = ((a, b, c), (a, c, d)) if (i + j) % 2 == 0 else ((a, b, d), (b, c, d))
            for tri in tris:
                lines.append("f " + " ".join(f"{v}/{v}" for v in tri))
    return "\n".join(lines) + "\n"


def _cotton_texture(path: Path) -> None:
    width, height = 1120, 720
    yy, xx = np.mgrid[:height, :width]
    rng = np.random.default_rng(46027)
    # Each 8-pixel thread is ~2 mm in the full-scale cloth. Alternating
    # warp/weft shading gives the weave a restrained over-under structure.
    warp = np.cos(2 * math.pi * xx / 8)
    weft = np.cos(2 * math.pi * yy / 8)
    alternating = ((xx // 8 + yy // 8) % 2) * 2 - 1
    weave = 1.6 * (warp + weft) + 0.9 * alternating * (warp - weft)
    long_variation = 1.2 * np.sin(xx * 0.031) + 0.8 * np.cos(yy * 0.027)
    fibers = rng.normal(0.0, 0.7, size=(height, width))
    base = np.array((229.0, 226.0, 214.0))
    rgb = base + (weave + long_variation + fibers)[..., None]
    border = np.minimum(np.minimum(xx, width - 1 - xx), np.minimum(yy, height - 1 - yy))
    rgb[border < 12] -= np.array((5.0, 5.0, 4.0))
    # Narrow seam line with small pale stitch segments, never a dark border.
    seam = (border >= 19) & (border <= 20)
    rgb[seam] -= 7.0
    horizontal = ((yy == 17) | (yy == height - 18)) & ((xx % 16) < 7) & (xx > 16) & (xx < width - 16)
    vertical = ((xx == 17) | (xx == width - 18)) & ((yy % 16) < 7) & (yy > 16) & (yy < height - 16)
    rgb[horizontal | vertical] += 8.0
    image = Image.fromarray(np.clip(rgb, 0, 255).astype(np.uint8))
    image.save(path, optimize=False)


def _soy_obj() -> str:
    count = 160
    theta = np.arange(count) * (2 * math.pi / count)
    radius = 1 + 0.067 * np.sin(3 * theta + 0.4) + 0.041 * np.cos(5 * theta - 0.3) + 0.018 * np.sin(7 * theta)
    x, y = radius * np.cos(theta), radius * np.sin(theta)
    x = (x - 0.5 * (x.max() + x.min())) * (0.20 / (x.max() - x.min()))
    y = (y - 0.5 * (y.max() + y.min())) * (0.14 / (y.max() - y.min()))
    vertices = [(0.0, 0.0, 0.002)]
    # Cross section travels from the flat glossy center over the meniscus
    # and back under the puddle. Adjacent rings are always distinct.
    profiles = ((0.35, 0.002), (0.75, 0.00196), (0.94, 0.00165),
                (1.0, 0.00065), (0.999, 0.00020), (0.985, 0.0))
    for scale, z in profiles:
        vertices.extend((float(px * scale), float(py * scale), z) for px, py in zip(x, y))
    vertices.append((0.0, 0.0, 0.0))
    faces = []
    for i in range(count):
        faces.append((1, 2 + i, 2 + (i + 1) % count))
    for ring in range(len(profiles) - 1):
        a0, b0 = 2 + ring * count, 2 + (ring + 1) * count
        for i in range(count):
            nxt = (i + 1) % count
            faces.extend(((a0 + i, b0 + i, b0 + nxt), (a0 + i, b0 + nxt, a0 + nxt)))
    bottom_center = len(vertices)
    bottom_start = 2 + (len(profiles) - 1) * count
    for i in range(count):
        faces.append((bottom_center, bottom_start + (i + 1) % count, bottom_start + i))
    lines = ["# Soy puddle: closed mesh; XY bounds 0.20 x 0.14 m; Z 0..0.002 m", "o soy_sauce_puddle"]
    lines.extend(f"v {xv:.9f} {yv:.9f} {zv:.9f}" for xv, yv, zv in vertices)
    lines.append("s 1")
    lines.extend("f " + " ".join(str(v) for v in tri) for tri in faces)
    return "\n".join(lines) + "\n"


def ensure_assets() -> dict[str, Path]:
    """Create deterministic local assets and return their absolute paths."""
    ASSETS_DIR.mkdir(parents=True, exist_ok=True)
    paths = {
        "cloth_mesh": ASSETS_DIR / "cotton_cloth.obj",
        "cloth_texture": ASSETS_DIR / "cotton_weave.png",
        "soy_mesh": ASSETS_DIR / "soy_puddle.obj",
    }
    _write_text(paths["cloth_mesh"], _cloth_obj())
    _cotton_texture(paths["cloth_texture"])
    _write_text(paths["soy_mesh"], _soy_obj())
    return paths


if __name__ == "__main__":
    for name, path in ensure_assets().items():
        print(f"{name}: {path}")
