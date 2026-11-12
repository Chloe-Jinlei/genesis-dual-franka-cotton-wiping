"""CPU diagnostic of the bending projection's small-amplitude stability.

Compare the previous simultaneous Jacobi sum with disjoint four-node hinge
colors using identical rigidity, mass, time step and relaxation. This isolates
the solver schedule: it includes no contact, robot, gravity, or GPU execution.
The NumPy projection mirrors the signed-angle XPBD equations in cotton_bending.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from cotton_bending import build_hinges, disjoint_hinge_colors
from cotton_mesh import rectangular_cotton_mesh


def _angles_and_gradients(points, hinges):
    p = points[hinges]
    edge = p[:, 1] - p[:, 0]
    edge_squared = np.einsum("ij,ij->i", edge, edge)
    length = np.sqrt(edge_squared)
    normal1 = np.cross(edge, p[:, 2] - p[:, 0])
    normal2 = np.cross(p[:, 3] - p[:, 0], edge)
    n1_squared = np.einsum("ij,ij->i", normal1, normal1)
    n2_squared = np.einsum("ij,ij->i", normal2, normal2)
    if np.any(edge_squared <= 1e-20) or np.any(np.minimum(n1_squared, n2_squared) <= 1e-24):
        raise ValueError("Diagnostic encountered a collapsed hinge.")
    a, b = normal1 / np.sqrt(n1_squared[:, None]), normal2 / np.sqrt(n2_squared[:, None])
    theta = np.arctan2(np.einsum("ij,ij->i", edge / length[:, None], np.cross(a, b)),
                       np.einsum("ij,ij->i", a, b))
    g3 = -length[:, None] * normal1 / n1_squared[:, None]
    g4 = -length[:, None] * normal2 / n2_squared[:, None]
    t3 = np.einsum("ij,ij->i", p[:, 2] - p[:, 0], edge) / edge_squared
    t4 = np.einsum("ij,ij->i", p[:, 3] - p[:, 0], edge) / edge_squared
    g2 = -t3[:, None] * g3 - t4[:, None] * g4
    gradients = np.stack((-g2 - g3 - g4, g2, g3, g4), axis=1)
    return theta, gradients


def bending_energy(points, hinges, rest_angles, compliance):
    theta, _ = _angles_and_gradients(points, hinges)
    error = theta - rest_angles
    error = np.arctan2(np.sin(error), np.cos(error))
    return float(0.5 * np.sum(error**2 / compliance))


def project_bending_numpy(initial, hinges, rest_angles, compliance, masses,
                          *, dt=1e-4, relaxation=.5, sweeps=4, colors=None):
    """Run one substep's bending only; return positions and per-sweep metrics.

    ``colors=None`` reproduces the previous simultaneous Jacobi update. A pair
    ``(order, bounds)`` from the production coloring helper uses disjoint groups.
    Multipliers start at zero and persist across this substep's four sweeps.
    """
    points = np.asarray(initial, dtype=np.float64).copy()
    masses = np.asarray(masses, dtype=np.float64)
    inverse_mass = 1.0 / masses
    multipliers = np.zeros(len(hinges), dtype=np.float64)
    alpha = np.asarray(compliance, dtype=np.float64) / dt**2
    if colors is None:
        groups = [np.arange(len(hinges))]
    else:
        order, bounds = colors
        groups = [order[lo:hi] for lo, hi in zip(bounds[:-1], bounds[1:])]
    history = []
    for _ in range(sweeps):
        maximum_update = 0.
        for group in groups:
            ids = hinges[group]
            theta, gradients = _angles_and_gradients(points, ids)
            error = theta - rest_angles[group]
            error = np.arctan2(np.sin(error), np.cos(error))
            weights = inverse_mass[ids]
            denominator = np.sum(weights * np.sum(gradients**2, axis=2), axis=1)
            dlambda = relaxation * (-error - alpha[group] * multipliers[group]) / (denominator + alpha[group])
            multipliers[group] += dlambda
            correction = weights[:, :, None] * dlambda[:, None, None] * gradients
            update = np.zeros_like(points)
            np.add.at(update, ids.ravel(), correction.reshape(-1, 3))
            maximum_update = max(maximum_update, float(np.linalg.norm(update, axis=1).max()))
            points += update
        history.append({
            "bending_energy_J": bending_energy(points, hinges, rest_angles, compliance),
            "z_standard_deviation_m": float(points[:, 2].std()),
            "maximum_group_position_update_m": maximum_update,
        })
    return points, history


def run_diagnostic():
    points, faces = rectangular_cotton_mesh(center=(0., -.23, .766))
    # Native float32 rest locations, then float64 arithmetic to isolate schedule
    # instability from the separate issue of float32 position quantization.
    points = points.astype(np.float32).astype(np.float64)
    hinges, angles, compliance = build_hinges(points, faces, 5e-5)
    order, bounds = disjoint_hinge_colors(hinges, len(points))
    for lo, hi in zip(bounds[:-1], bounds[1:]):
        nodes = hinges[order[lo:hi]].ravel()
        if len(np.unique(nodes)) != len(nodes):
            raise AssertionError("Production hinge colors contain shared nodes.")
    face_area = np.linalg.norm(np.cross(points[faces[:, 1]] - points[faces[:, 0]],
                                       points[faces[:, 2]] - points[faces[:, 0]]), axis=1).sum() / 2
    mass = .18 * face_area / len(points)
    masses = np.full(len(points), mass)
    nx, ny = len(np.unique(points[:, 0])), len(np.unique(points[:, 1]))
    initial = points.copy()
    initial[:, 2] += 1e-6 * (-1.)**np.indices((ny, nx)).sum(axis=0).ravel()
    initial_std = float(initial[:, 2].std())
    initial_energy = bending_energy(initial, hinges, angles, compliance)
    old, old_history = project_bending_numpy(initial, hinges, angles, compliance, masses)
    colored, color_history = project_bending_numpy(
        initial, hinges, angles, compliance, masses, colors=(order, bounds))
    source_root = Path(__file__).resolve().parent
    return {
        "scope": "Independent CPU float64 bending-only schedule diagnostic; not a GPU/native/contact task validation",
        "input": {
            "physical_mesh": "95 by 61 alternating-diagonal rectangular mesh, native float32 rest coordinates",
            "particles": len(points), "hinges": len(hinges), "colors": len(bounds) - 1,
            "areal_density_kg_m2": .18, "native_equal_particle_mass_kg": mass,
            "bending_rigidity_N_m": 5e-5, "substep_dt_s": 1e-4,
            "relaxation": .5, "sweeps": 4, "checkerboard_z_amplitude_m": 1e-6,
            "gravity_enabled": False, "contact_enabled": False, "robot_enabled": False,
        },
        "initial": {"bending_energy_J": initial_energy, "z_standard_deviation_m": initial_std},
        "previous_simultaneous_jacobi": {
            "sweeps": old_history,
            "z_amplitude_gain": float(old[:, 2].std() / initial_std),
            "bending_energy_ratio": old_history[-1]["bending_energy_J"] / initial_energy,
        },
        "disjoint_hinge_colors": {
            "sweeps": color_history,
            "z_amplitude_gain": float(colored[:, 2].std() / initial_std),
            "bending_energy_ratio": color_history[-1]["bending_energy_J"] / initial_energy,
            "all_color_nodes_disjoint": True,
        },
        "checks": {
            "previous_schedule_amplifies_checkerboard": old_history[-1]["bending_energy_J"] > initial_energy,
            "colored_schedule_reduces_bending_energy": color_history[-1]["bending_energy_J"] < initial_energy,
        },
        "source_sha256": {name: hashlib.sha256((source_root / name).read_bytes()).hexdigest()
                          for name in ("cotton_bending.py", "cotton_mesh.py", Path(__file__).name)},
        "limits": [
            "One small-amplitude disturbance and one substep do not prove nonlinear/global solver stability.",
            "No forces, native contact, membrane solve, air drag, or robot motion are included.",
            "The NumPy equation reproduction does not verify GPU compiler execution or float32 roundoff.",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "output/pdf_cotton_bending_diagnostic.json")
    args = parser.parse_args()
    report = run_diagnostic()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "checks": report["checks"],
                      "previous_gain": report["previous_simultaneous_jacobi"]["z_amplitude_gain"],
                      "colored_gain": report["disjoint_hinge_colors"]["z_amplitude_gain"]}, indent=2))
    if not all(report["checks"].values()):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
