"""Preserve integrated cotton velocity through PBD position projection.

Genesis 1.4.3 reconstructs velocity from two absolute world positions after
every substep. At float32, world height around one metre and h=100 us, this
can discard small acceleration/damping updates even when gravity is resolvable.

Cache the position and velocity AFTER native force integration. Then use
``v_new = v_pred + (x_projected - x_pred) / h``. In exact arithmetic this equals
the original PBD formula; without constraints it preserves the integrated
velocity exactly. Constraint corrections still contribute their usual impulse.

This instance-local adapter does not change material parameters, position
integration, constraints, collision, fixed/attached state, or installed Genesis.
It does not fix world-position rounding, contact kernels that subsequently
reconstruct velocity, or errors in the constraint solve. Caches are overwritten
each substep and contain no history required by a scene checkpoint.
"""
from importlib.metadata import version


def install_cotton_velocity_update(scene, cloth):
    """Install before build; return an adapter with ``description`` and ``close``.

    The existing force/integration hook runs first, including gravity and air
    drag; a capture kernel then saves predictions in original particle order.
    The replacement velocity kernel maps original indices to the current
    reordered storage after stretch/bending/self-contact. Rigid contact and
    exact-link attachments run later and remain responsible for their own
    velocity changes. Install any force-hook wrappers before this adapter, or
    ensure later wrappers still call the wrapped method normally.
    """
    import genesis as gs
    import quadrants as qd
    import torch

    if version("genesis-world") != "1.4.3" or scene.is_built:
        raise RuntimeError("Install the Genesis 1.4.3 cotton velocity adapter before scene.build().")
    solver = cloth.solver
    if solver is not scene.pbd_solver:
        raise ValueError("Cotton must belong to this scene's PBD solver.")
    if len(solver.entities) != 1 or solver.entities[0] is not cloth:
        raise ValueError("Cotton velocity update requires this cloth as the only PBD entity.")
    if not isinstance(cloth.material, gs.materials.PBD.Cloth):
        raise TypeError("Cotton velocity update requires PBD.Cloth.")
    if getattr(solver, "_cotton_velocity_adapter", None) is not None:
        raise RuntimeError("Cotton velocity update is already installed.")

    @qd.data_oriented
    class CottonVelocityUpdate:
        def __init__(self):
            self.solver = solver
            self.previous_force = solver._kernel_apply_external_force
            self.previous_velocity = solver._kernel_compute_velocity
            self.predicted_pos = None
            self.predicted_vel = None
            self.description = {
                "model": "integrated prediction plus constraint-displacement velocity update",
                "formula": "v_projected = v_predicted + (x_projected - x_predicted) / substep_dt",
                "exact_arithmetic_equivalent_to_native": True,
                "capture": "after existing native external-force and position-integration kernel each substep",
                "particle_order": "cache original indices; read projected positions via particles_ng.reordered_idx",
                "active_particles_only": True,
                "fixed_particles": "no special velocity clamp; preserve native predicted motion and any existing projection",
                "cache_reset": "overwritten each substep, including first build warmup and after scene restore",
                "persistent_checkpoint_state": False,
                "material_parameters_modified": False,
                "position_integration_modified": False,
                "constraint_projection_modified": False,
                "rigid_contact_and_attachment_modified": False,
                "installed_genesis_modified": False,
                "limitations": [
                    "Does not correct rounding of absolute positions during integration or constraint projection.",
                    "Later rigid-contact or attachment kernels may independently reconstruct velocity from world positions.",
                    "Does not guarantee energy monotonicity, eliminate physical motion, or validate all constraint/contact errors.",
                ],
            }

        def apply_external_force(self, f, t):
            self.previous_force(f, t)
            if self.predicted_pos is None:
                shape = (self.solver._n_particles, self.solver._B, 3)
                self.predicted_pos = torch.empty(shape, dtype=gs.tc_float, device=gs.device)
                self.predicted_vel = torch.empty(shape, dtype=gs.tc_float, device=gs.device)
            self.capture(self.predicted_pos, self.predicted_vel)

        def compute_velocity(self, f):
            if self.predicted_pos is None:
                raise RuntimeError("Cotton velocity reconstruction requires this substep's integrated predictions.")
            self.update(self.predicted_pos, self.predicted_vel)

        @qd.kernel
        def capture(self, positions: qd.types.ndarray(), velocities: qd.types.ndarray()):
            for original, batch in qd.ndrange(self.solver._n_particles, self.solver._B):
                if self.solver.particles_ng[original, batch].active:
                    for axis in qd.static(range(3)):
                        positions[original, batch, axis] = self.solver.particles[original, batch].pos[axis]
                        velocities[original, batch, axis] = self.solver.particles[original, batch].vel[axis]

        @qd.kernel
        def update(self, positions: qd.types.ndarray(), velocities: qd.types.ndarray()):
            for original, batch in qd.ndrange(self.solver._n_particles, self.solver._B):
                if self.solver.particles_ng[original, batch].active:
                    reordered = self.solver.particles_ng[original, batch].reordered_idx
                    for axis in qd.static(range(3)):
                        correction = (self.solver.particles_reordered[reordered, batch].pos[axis]
                                      - positions[original, batch, axis]) / self.solver._substep_dt
                        self.solver.particles_reordered[reordered, batch].vel[axis] = (
                            velocities[original, batch, axis] + correction)

        def close(self):
            """Restore only hooks still owned by this adapter."""
            if self.solver._kernel_apply_external_force == self.apply_external_force:
                self.solver._kernel_apply_external_force = self.previous_force
            if self.solver._kernel_compute_velocity == self.compute_velocity:
                self.solver._kernel_compute_velocity = self.previous_velocity
            if getattr(self.solver, "_cotton_velocity_adapter", None) is self:
                del self.solver._cotton_velocity_adapter

    adapter = CottonVelocityUpdate()
    solver._kernel_apply_external_force = adapter.apply_external_force
    solver._kernel_compute_velocity = adapter.compute_velocity
    solver._cotton_velocity_adapter = adapter
    return adapter
