"""Optional nonlocal, momentum-preserving damping of free-cloth deformation.

At fixed geometry, fit the mass-weighted rigid velocity of FREE, ACTIVE nodes:
    r_i=x_i-x_COM, I=sum m_i (|r_i|^2 Id-r_i r_i^T),
    omega=I^+ sum r_i cross m_i(v_i-v_COM),
    v_rigid_i=v_COM+omega cross r_i.
Then v_i <- v_rigid_i + exp(-rate*h)*(v_i-v_rigid_i).

The free set's linear/angular momenta are preserved and its nonrigid kinetic
energy decays by exp(-2*rate*h), in exact arithmetic. Prescribed velocities are
untouched and excluded from the fit; this operator supplies no anchor impulse
or work. Thus a rigid swinging/rotating free set is deliberately NOT damped.

This is a whole-cloth nonlocal engineering approximation of internal damping,
not a calibrated Kelvin-Voigt cloth model and not a local viscosity. A 4/s rate
may be chosen to interpret the PDF's scalar internal-damping setting, but the
PDF does not identify this nonlocal constitutive model. Replace the edge-axial
operator when comparing models; do not silently stack both 4/s operators.

Only velocities change. Current native wet masses/free flags are used every
substep. This does not fix constraint/contact energy injection, float32 position
rounding, or later velocity reconstruction that erases integrated damping.
The small inertia pseudoinverse discards eigenvalues <= 1e-12 of its maximum;
linear/planar/point degeneracy is safe, with conservation up to this relative
rank tolerance and native velocity rounding. Native reductions and the 3x3
solve use float64 GPU buffers, local position offsets, and no CPU sync.
"""
from importlib.metadata import version

import numpy as np


INERTIA_RCOND = 1e-12


def _validate(points, velocities, masses, free, rate_per_s, dt):
    p, v, m = (np.asarray(x, dtype=np.float64) for x in (points, velocities, masses))
    flags = np.asarray(free)
    if p.ndim != 2 or p.shape[1:] != (3,) or v.shape != p.shape or m.shape != (len(p),) or flags.shape != (len(p),):
        raise ValueError('Need positions/velocities (N,3), masses/free (N,)')
    if flags.dtype != np.dtype(bool):
        raise ValueError('Free flags must be booleans')
    if not all(np.isfinite(x).all() for x in (p, v, m)) or np.any(m <= 0):
        raise ValueError('Positions/velocities must be finite and masses strictly positive')
    if not np.isfinite(rate_per_s) or rate_per_s < 0 or not np.isfinite(dt) or dt <= 0:
        raise ValueError('Need finite nonnegative damping rate and positive finite dt')
    return p, v, m, flags


def damp_deformation_reference(points, velocities, masses, free, *, rate_per_s=0., dt):
    """Return copied velocities and independently measured CPU diagnostics.

    ``free`` should already exclude inactive nodes. Inputs are never mutated.
    Uses the CURRENT supplied masses; no dry-mass or geometry cache exists.
    The fit is of the free set alone, including when prescribed vertices move.
    """
    p, v, mass, flags = _validate(points, velocities, masses, free, rate_per_s, dt)
    changed = v.copy()
    stats = {'enabled': bool(rate_per_s > 0), 'free_count': int(flags.sum()),
             'free_mass_kg': float(mass[flags].sum()), 'inertia_rank': 0,
             'kinetic_energy_before_J': 0., 'kinetic_energy_after_J': 0.,
             'deformation_energy_before_J': 0., 'deformation_energy_after_J': 0.,
             'linear_momentum_change_kg_m_s': np.zeros(3),
             'angular_momentum_change_about_com_kg_m2_s': np.zeros(3),
             'center_of_mass_m': np.zeros(3), 'rigid_velocity_m_s': np.zeros(3),
             'rigid_angular_velocity_rad_s': np.zeros(3),
             'velocity_decay_factor': float(np.exp(-rate_per_s*dt)),
             'anchor_work_J': 0., 'support_impulse_kg_m_s': np.zeros(3)}
    if not flags.any():
        return changed, stats
    x, old, m = p[flags], v[flags], mass[flags]
    total = m.sum()
    # Center offsets from an existing point, not squared absolute world coords.
    offset = x-x[0]
    center_offset = np.sum(m[:, None]*offset, axis=0)/total
    r = offset-center_offset
    center_velocity = np.sum(m[:, None]*old, axis=0)/total
    relative = old-center_velocity
    inertia = np.eye(3)*np.sum(m*np.einsum('ij,ij->i', r, r)) - (r*m[:, None]).T@r
    angular = np.sum(np.cross(r, m[:, None]*relative), axis=0)
    eigenvalues, axes = np.linalg.eigh(inertia)
    keep = eigenvalues > max(float(eigenvalues[-1]), 0.)*INERTIA_RCOND
    omega = np.zeros(3)
    if keep.any():
        omega = axes[:, keep]@((axes[:, keep].T@angular)/eigenvalues[keep])
    rigid = center_velocity+np.cross(omega, r)
    residual = old-rigid
    decay = stats['velocity_decay_factor']
    if rate_per_s > 0:
        changed[flags] = old+(decay-1.)*residual
    new = changed[flags]
    stats.update(inertia_rank=int(keep.sum()), center_of_mass_m=x[0]+center_offset,
        rigid_velocity_m_s=center_velocity, rigid_angular_velocity_rad_s=omega,
        kinetic_energy_before_J=float(.5*np.sum(m[:, None]*old**2)),
        kinetic_energy_after_J=float(.5*np.sum(m[:, None]*new**2)),
        deformation_energy_before_J=float(.5*np.sum(m[:, None]*residual**2)),
        deformation_energy_after_J=float(.5*np.sum(m[:, None]*(new-rigid)**2)),
        linear_momentum_change_kg_m_s=np.sum(m[:, None]*(new-old), axis=0),
        angular_momentum_change_about_com_kg_m2_s=np.sum(np.cross(r, m[:, None]*(new-old)), axis=0))
    return changed, stats


def install_cotton_deformation_damping(scene, cloth, *, rate_per_s=0.):
    """Create a before-build optional adapter, WITHOUT installing solver hooks.

    Call ``apply()`` (alias ``start_substep()``) exactly once after store_initial
    and before native force/integration each native substep. ``description`` is
    JSON-safe; ``diagnostics()`` explicitly synchronizes when requested and
    measures the last rounded native update. ``apply`` performs no CPU reads.
    Default 0/s is disabled. There is no physical state to reset/checkpoint;
    buffers are overwritten every pass. Use with prediction-preserving velocity
    reconstruction, so native absolute-position rounding does not erase damping.
    """
    import genesis as gs
    import quadrants as qd
    import torch

    if not np.isfinite(rate_per_s) or rate_per_s < 0:
        raise ValueError('Damping rate must be finite and nonnegative')
    if version('genesis-world') != '1.4.3' or scene.is_built:
        raise RuntimeError('Create the Genesis 1.4.3 deformation damping adapter before scene.build()')
    solver = cloth.solver
    if solver is not scene.pbd_solver or len(solver.entities) != 1 or solver.entities[0] is not cloth:
        raise ValueError('Deformation damping requires this cloth as the sole PBD entity')
    if not isinstance(cloth.material, gs.materials.PBD.Cloth) or cloth.n_particles < 1:
        raise ValueError('Deformation damping requires a nonempty PBD cloth')

    @qd.data_oriented
    class CottonDeformationDamping:
        def __init__(self):
            self.solver = solver
            self.start, self.stop = cloth.particle_start, cloth.particle_start+cloth.n_particles
            self.rate = float(rate_per_s)
            self.enabled = self.rate > 0.
            # center: mass, weighted local position(3), momentum(3), origin(3)
            self.center = torch.zeros(10, dtype=torch.float64, device=gs.device)
            # angular: symmetric inertia(9), angular momentum(3), omega(3), rank
            self.angular = torch.zeros(16, dtype=torch.float64, device=gs.device)
            # energy pre/post, deformation energy pre/post, delta P(3), delta L(3), count
            self.stats = torch.zeros(11, dtype=torch.float64, device=gs.device)
            self.description = {
                'model': 'mass-weighted best-fit rigid-motion residual velocity damping',
                'enabled': self.enabled, 'rate_per_s': self.rate,
                'integration': 'v = v_rigid + exp(-rate*substep_dt)*(v-v_rigid), once per native substep',
                'fit_scope': 'current free and active particles only; fixed velocities untouched',
                'current_native_wet_masses_read_each_call': True,
                'linear_and_angular_momentum_preserved': 'free set, within inertia rank tolerance and native velocity rounding',
                'inertia_pseudoinverse_relative_tolerance': INERTIA_RCOND,
                'degenerate_geometry': 'symmetric eigensolve pseudoinverse; point/line configurations supported',
                'gpu_reductions_and_inertia_precision': 'float64, centered local positions, no CPU sync in apply',
                'rigid_motion_preserved': True, 'positions_changed': False,
                'prescribed_velocities_changed': False, 'anchor_work_or_support_impulse': False,
                'nonlocal_engineering_approximation': True, 'experimentally_calibrated': False,
                'pdf_model_equivalence_claimed': False,
                'physical_state_persisted_between_substeps': False,
                'diagnostics': 'last-pass free kinetic energies and momenta measured before/after rounded native velocity writes',
                'limitations': [
                    'Does not damp a rigid swing or rotation of the free set.',
                    'Does not prevent energy injected by constraints, contact, attachments or position rounding.',
                    'Use as an alternative to, not an undisclosed addition to, the edge-axial internal damping model.',
                    'Requires integrated velocity preservation through subsequent position-based reconstruction.',
                ],
            }

        def apply(self):
            if not self.enabled:
                return
            if not scene.is_built or self.solver._B != 1:
                raise RuntimeError('Deformation damping requires built single-environment PBD fields')
            self.project(self.center, self.angular, self.stats)

        def start_substep(self):
            self.apply()

        def diagnostics(self):
            s = self.stats.detach().cpu().numpy()
            c = self.center.detach().cpu().numpy()
            a = self.angular.detach().cpu().numpy()
            center = c[7:10].copy()
            velocity = np.zeros(3)
            if c[0] > 0:
                center += c[1:4]/c[0]
                velocity = c[4:7]/c[0]
            return {'enabled': self.enabled, 'free_count': int(round(s[10])),
                    'free_mass_kg': float(c[0]), 'inertia_rank': int(round(a[15])),
                    'kinetic_energy_before_J': float(s[0]), 'kinetic_energy_after_J': float(s[1]),
                    'deformation_energy_before_J': float(s[2]), 'deformation_energy_after_J': float(s[3]),
                    'linear_momentum_change_kg_m_s': s[4:7].tolist(),
                    'angular_momentum_change_about_com_kg_m2_s': s[7:10].tolist(),
                    'center_of_mass_m': center.tolist(), 'rigid_velocity_m_s': velocity.tolist(),
                    'rigid_angular_velocity_rad_s': a[12:15].tolist(),
                    'anchor_work_J': 0., 'support_impulse_kg_m_s': [0., 0., 0.]}

        @qd.kernel
        def project(self, center: qd.types.ndarray(), angular: qd.types.ndarray(), stats: qd.types.ndarray()):
            # Each top-level loop is a separate GPU offload, giving global
            # barriers between reductions/fit/application without CPU copies.
            for j in range(16):
                angular[j] = 0.
                if j < 10:
                    center[j] = 0.
                if j < 11:
                    stats[j] = 0.
            for axis in range(3):
                center[7+axis] = qd.cast(self.solver.particles[self.start, 0].pos[axis], qd.f64)
            for node in range(self.start, self.stop):
                if self.solver.particles_ng[node, 0].active and self.solver.particles[node, 0].free:
                    mass = qd.cast(self.solver.particles_info[node].mass, qd.f64)
                    center[0] += mass
                    for axis in qd.static(range(3)):
                        center[1+axis] += mass*(qd.cast(self.solver.particles[node, 0].pos[axis], qd.f64)-center[7+axis])
                        center[4+axis] += mass*qd.cast(self.solver.particles[node, 0].vel[axis], qd.f64)
            for node in range(self.start, self.stop):
                if self.solver.particles_ng[node, 0].active and self.solver.particles[node, 0].free and center[0] > 0.:
                    mass = qd.cast(self.solver.particles_info[node].mass, qd.f64)
                    r, relative = qd.Vector.zero(qd.f64, 3), qd.Vector.zero(qd.f64, 3)
                    for axis in qd.static(range(3)):
                        r[axis] = qd.cast(self.solver.particles[node, 0].pos[axis], qd.f64)-center[7+axis]-center[1+axis]/center[0]
                        relative[axis] = qd.cast(self.solver.particles[node, 0].vel[axis], qd.f64)-center[4+axis]/center[0]
                    l = mass*r.cross(relative)
                    for i, j in qd.static(qd.ndrange(3, 3)):
                        term = -r[i]*r[j]
                        if qd.static(i == j):
                            term += r.norm_sqr()
                        angular[3*i+j] += mass*term
                    for axis in qd.static(range(3)):
                        angular[9+axis] += l[axis]
            for _ in range(1):
                inertia = qd.Matrix.zero(qd.f64, 3, 3)
                l = qd.Vector.zero(qd.f64, 3)
                for i, j in qd.static(qd.ndrange(3, 3)):
                    inertia[i, j] = angular[3*i+j]
                for axis in qd.static(range(3)):
                    l[axis] = angular[9+axis]
                values, axes = qd.sym_eig(inertia, qd.f64)
                maximum = qd.max(values[0], qd.max(values[1], values[2]))
                omega = qd.Vector.zero(qd.f64, 3)
                for axis in qd.static(range(3)):
                    if values[axis] > qd.max(maximum, 0.)*INERTIA_RCOND:
                        direction = axes[:, axis]
                        omega += direction*(direction.dot(l)/values[axis])
                        angular[15] += 1.
                for axis in qd.static(range(3)):
                    angular[12+axis] = omega[axis]
            for node in range(self.start, self.stop):
                if self.solver.particles_ng[node, 0].active and self.solver.particles[node, 0].free and center[0] > 0.:
                    mass = qd.cast(self.solver.particles_info[node].mass, qd.f64)
                    r, old = qd.Vector.zero(qd.f64, 3), qd.Vector.zero(qd.f64, 3)
                    velocity, omega = qd.Vector.zero(qd.f64, 3), qd.Vector.zero(qd.f64, 3)
                    for axis in qd.static(range(3)):
                        r[axis] = qd.cast(self.solver.particles[node, 0].pos[axis], qd.f64)-center[7+axis]-center[1+axis]/center[0]
                        old[axis] = qd.cast(self.solver.particles[node, 0].vel[axis], qd.f64)
                        velocity[axis] = center[4+axis]/center[0]
                        omega[axis] = angular[12+axis]
                    rigid = velocity+omega.cross(r)
                    residual = old-rigid
                    factor = qd.exp(-qd.cast(self.rate*self.solver._substep_dt, qd.f64))
                    update = old+(factor-1.)*residual
                    for axis in qd.static(range(3)):
                        self.solver.particles[node, 0].vel[axis] = qd.cast(update[axis], gs.qd_float)
                    # Read back rounded native velocities for diagnostics.
                    new = qd.Vector.zero(qd.f64, 3)
                    for axis in qd.static(range(3)):
                        new[axis] = qd.cast(self.solver.particles[node, 0].vel[axis], qd.f64)
                    stats[0] += .5*mass*old.norm_sqr()
                    stats[1] += .5*mass*new.norm_sqr()
                    stats[2] += .5*mass*residual.norm_sqr()
                    stats[3] += .5*mass*(new-rigid).norm_sqr()
                    impulse = mass*(new-old)
                    torque = r.cross(impulse)
                    for axis in qd.static(range(3)):
                        stats[4+axis] += impulse[axis]
                        stats[7+axis] += torque[axis]
                    stats[10] += 1.

    return CottonDeformationDamping()
