"""Trusted local checkpoints for the mixed rigid/SPH/PBD wiping scene.

Genesis 1.4.3 cannot export PBD/SPH entities or checkpoint those solvers through
Scene.__getstate__: their descriptions/data protocols are not implemented.
This helper therefore preserves the native rigid SolverCheckpoint and copies
every allocated SPH/PBD/coupler field, plus clocks and local adapter ledgers.
It restores into an ALREADY BUILT matching scene, preserving cameras, callbacks,
the porous-sheet step wrapper and exact-grasp kernel installed on that instance.

Pickle files must be ones created by this task and trusted by the caller. Never
load a downloaded or otherwise untrusted pickle. No live scene/entity/callback
objects are put into the payload; tensor data is moved to CPU before saving.
"""
from __future__ import annotations

import copy
import dataclasses
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import pickle
import tempfile
from typing import Any

import numpy as np

FORMAT = "soy-cloth-robot-checkpoint"
FORMAT_VERSION = 2


def _host(value):
    """Copy nested checkpoint records without retaining device buffers."""
    if hasattr(value, "detach") and hasattr(value, "cpu"):
        return value.detach().cpu().numpy().copy()
    if isinstance(value, np.ndarray):
        return value.copy()
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return dataclasses.replace(value, **{f.name: _host(getattr(value, f.name))
                                             for f in dataclasses.fields(value)})
    if isinstance(value, dict):
        return {key: _host(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_host(item) for item in value]
    if isinstance(value, tuple):
        return tuple(_host(item) for item in value)
    return copy.deepcopy(value)


def _metadata(value, path="task_state"):
    """Allow task data while rejecting accidental scene or callback references."""
    if value is None or isinstance(value, (str, bool, int, float, np.generic)):
        return value.item() if isinstance(value, np.generic) else value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.ndarray) or (hasattr(value, "detach") and hasattr(value, "cpu")):
        result = _host(value)
        if result.dtype.hasobject:
            raise TypeError(f"{path}: object arrays cannot be checkpoint metadata")
        return result
    if isinstance(value, dict):
        if any(not isinstance(k, (str, int)) for k in value):
            raise TypeError(f"{path}: metadata dictionary keys must be strings or integers")
        return {k: _metadata(v, f"{path}.{k}") for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        result = [_metadata(v, f"{path}[{i}]") for i, v in enumerate(value)]
        return tuple(result) if isinstance(value, tuple) else result
    raise TypeError(f"{path}: save plain task values, not {type(value).__name__} objects")


def _fields(owner):
    """Only allocated array fields, never engine references or callables."""
    return {name: value for name, value in vars(owner).items()
            if callable(getattr(value, "to_numpy", None))
            and callable(getattr(value, "from_numpy", None))}


def _field_owners(scene):
    sim = scene.sim
    return {"pbd": sim.pbd_solver, "pbd_hash": sim.pbd_solver.sh,
            "sph": sim.sph_solver, "sph_hash": sim.sph_solver.sh,
            "legacy_coupler": sim.coupler}


def _array_digest(value):
    array = np.ascontiguousarray(value)
    return {"shape": list(array.shape), "dtype": array.dtype.str,
            "sha256": hashlib.sha256(array.tobytes()).hexdigest()}


def _material_profile_fingerprint(profile):
    """Fingerprint scene-local material physics absent from native model_dump.

    The scene supplies handles['material_profile'] with cloth_config and the
    installed material adapter's description. None identifies the legacy
    native material. Only finite, JSON-compatible descriptive data is allowed;
    no callbacks or device arrays are silently serialized as live physics.
    """
    if profile is None:
        return None
    if not isinstance(profile, dict) or not {"cloth_config", "model"}.issubset(profile):
        raise ValueError("material_profile requires cloth_config and model descriptions")
    def plain(value):
        if isinstance(value, np.ndarray):
            return plain(value.tolist())
        if isinstance(value, dict):
            if any(not isinstance(k, str) for k in value):
                raise ValueError("Material profile keys must be strings")
            return {k: plain(v) for k, v in value.items()}
        if isinstance(value, (tuple, list)):
            return [plain(v) for v in value]
        return value
    description = plain(_metadata(profile, "material_profile"))
    encoded = json.dumps(description, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return {"sha256": hashlib.sha256(encoded).hexdigest(), "description": description}


def _validate_saved_signature(record, scene, handles):
    """Run before any native mutation, including the simulator restart."""
    if not isinstance(record, dict) or record.get("format") != FORMAT \
            or record.get("format_version") != FORMAT_VERSION:
        raise ValueError("Unsupported checkpoint schema; rebuild this material profile from its dry initial state")
    if record.get("signature") != _signature(scene, handles):
        raise ValueError("Checkpoint physics, particle topology, material profile or coupling profile differs")


def _signature(scene, handles):
    coupling = handles["coupling"]
    # Native digest covers rigid authored geometry and every physics option;
    # the particle initial geometry and the local porous model are added here.
    data = {
        "native_rigid_physics_digest": scene._desc_digest,
        "layout": dataclasses.asdict(scene._layout),
        "entities": [{"name": e.name, "class": type(e).__name__}
                     for e in scene.entities],
        "cloth_particles": int(handles["cloth"].n_particles),
        "cloth_collision_radius_m": float(handles["cloth"].solver.particle_radius),
        "soy_particles": int(handles["soy"].n_particles),
        "cloth_initial": _array_digest(coupling._initial_cloth_pos),
        "soy_initial": _array_digest(coupling._initial_soy_pos),
        "cloth_topology": _array_digest(coupling.faces),
        "liquid_mass": _array_digest(coupling.liquid_particle_mass),
        "dry_cloth_mass": _array_digest(coupling.dry_vertex_mass),
        "porous_options": _metadata(coupling.config, "coupling.config"),
        "pbd_material": _metadata(handles["cloth"].material.model_dump()),
        "sph_material": _metadata(handles["soy"].material.model_dump()),
        "material_profile": _material_profile_fingerprint(handles.get("material_profile")),
    }
    encoded = json.dumps(data, sort_keys=True, separators=(",", ":")).encode()
    return {"sha256": hashlib.sha256(encoded).hexdigest(), "description": data}


def _require_scene(scene, handles):
    from genesis.engine.couplers.legacy_coupler import LegacyCoupler

    if not scene.is_built or scene.n_envs != 0:
        raise ValueError("Checkpoint requires the built single-environment wiping scene")
    if version("genesis-world") != "1.4.3":
        raise RuntimeError("This mixed-field checkpoint adapter targets Genesis 1.4.3")
    if not isinstance(scene.sim.coupler, LegacyCoupler):
        raise ValueError("Checkpoint requires a LegacyCoupler or its installed friction subclass")
    if set(type(s).__name__ for s in scene.active_solvers) != {"RigidSolver", "PBDSolver", "SPHSolver"}:
        raise ValueError("Checkpoint supports exactly this scene's rigid/PBD/SPH solvers")
    for key in ("cloth", "soy", "coupling", "wet_visual"):
        if key not in handles:
            raise ValueError(f"Missing checkpoint handle: {key}")


def _same_array_tree(saved, live, label):
    if isinstance(live, dict):
        if not isinstance(saved, dict) or set(saved) != set(live):
            raise ValueError(f"Checkpoint fields differ: {label}")
        for key in live:
            _same_array_tree(saved[key], live[key], label + "." + key)
    elif isinstance(live, np.ndarray):
        if not isinstance(saved, np.ndarray) or saved.shape != live.shape or saved.dtype != live.dtype:
            raise ValueError(f"Checkpoint array shape/dtype differs: {label}")
    else:
        raise TypeError(f"Unsupported field representation at {label}: {type(live).__name__}")


def save_checkpoint(path, scene, handles, task_state=None):
    """Best-effort auxiliary save; return its resolved path or None on failure.

    A diagnostic snapshot must never interrupt the physical robot sequence.
    Failures are explicitly printed; no partial file is installed. Callers must
    check the return value before advertising or resuming this checkpoint.
    """
    try:
        return _save_checkpoint(path, scene, handles, task_state)
    except Exception as exc:
        print(f"[robot checkpoint] Save skipped: {type(exc).__name__}: {exc}", flush=True)
        return None


def _save_checkpoint(path, scene, handles, task_state=None):
    """Atomically save physics, absorption, grasp ownership and plain task data.

    Call between outer scene.step calls. Return the resolved saved path. The
    native whole-Scene export guard is respected, never patched or bypassed.
    """
    from robot_grasp import capture_grasp_state

    _require_scene(scene, handles)
    task_state = _metadata({} if task_state is None else task_state)
    owners = _field_owners(scene)
    record = {
        "format": FORMAT, "format_version": FORMAT_VERSION,
        "genesis_version": version("genesis-world"),
        "mode": "native-rigid-plus-explicit-particle-fields",
        "signature": _signature(scene, handles),
        "rigid": _host(scene.sim.rigid_solver.__getstate__()),
        "particle_fields": {owner_name: {name: _host(field.to_numpy())
                            for name, field in _fields(owner).items()}
                            for owner_name, owner in owners.items()},
        "steps": _host(scene.sim._steps),
        "cur_substep_global": int(scene.sim._cur_substep_global),
        "absorption": handles["coupling"].capture_state(),
        "grasp": capture_grasp_state(handles["cloth"]),
        "task_state": task_state,
    }
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="wb", dir=path.parent,
                                         prefix=path.name+".", suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            pickle.dump(record, stream, protocol=pickle.HIGHEST_PROTOCOL)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    return path


def load_checkpoint(path, scene, handles):
    """Restore our trusted file into this built instance; return task_state.

    Rebuild with the same physics/assets/coupling profile first. Viewer and
    camera choices may change. Config, topology and array shapes are checked
    before any mutation. Installed step callbacks and grasp kernels are kept.
    """
    from genesis.utils.array_class import check_data
    from robot_grasp import restore_grasp_state
    import torch

    _require_scene(scene, handles)
    with Path(path).expanduser().open("rb") as stream:
        record = pickle.load(stream)  # Trusted files generated by this task only.
    _validate_saved_signature(record, scene, handles)
    if record["genesis_version"] != version("genesis-world"):
        raise ValueError("Checkpoint Genesis version differs")
    rigid = record["rigid"]
    check_data((item for item in scene.sim.rigid_solver.data if item.kind in rigid.kinds), rigid.arrays)
    owners = _field_owners(scene)
    if set(record["particle_fields"]) != set(owners):
        raise ValueError("Checkpoint particle solver groups differ")
    for owner_name, owner in owners.items():
        live_fields = _fields(owner)
        saved_fields = record["particle_fields"][owner_name]
        if set(saved_fields) != set(live_fields):
            raise ValueError(f"Checkpoint allocated fields differ: {owner_name}")
        for name, field in live_fields.items():
            _same_array_tree(saved_fields[name], field.to_numpy(), owner_name+"."+name)
    current_absorption = handles["coupling"].capture_state()
    if set(record["absorption"]) != set(current_absorption):
        raise ValueError("Checkpoint absorption ledger fields differ")
    for name, live in current_absorption.items():
        if isinstance(live, np.ndarray):
            _same_array_tree(record["absorption"][name], live, "absorption."+name)
    if record["grasp"].get("n_particles") != handles["cloth"].n_particles:
        raise ValueError("Checkpoint grasp particle count differs")
    if np.shape(record["steps"]) != tuple(scene.sim._steps.shape):
        raise ValueError("Checkpoint step-clock shape differs")
    task_state = _metadata(record["task_state"])

    # Match the native restore lifecycle, without constructing or replacing the
    # already configured scene. Restart before filling fields because native
    # restart clears coupler caches and gradient buffers.
    scene._recorder_manager.record(scene.sim.cur_step_global)
    scene._recorder_manager.reset()
    scene.sim._restart()
    scene.sim.rigid_solver.__setstate__(rigid)
    for owner_name, owner in owners.items():
        for name, field in _fields(owner).items():
            field.from_numpy(record["particle_fields"][owner_name][name])
    scene.sim._steps[:] = torch.as_tensor(record["steps"], device=scene.sim._steps.device,
                                          dtype=scene.sim._steps.dtype)
    scene.sim._cur_substep_global = record["cur_substep_global"]
    scene._restart()
    handles["coupling"].restore_state(record["absorption"])
    restore_grasp_state(handles["cloth"], record["grasp"])
    handles["wet_visual"].update(handles["coupling"].vertex_saturation)
    return task_state
