"""Optional passive angular-velocity damping for this cloth's mesh hinges.

This is a frozen-geometry, backward-Euler dashpot split, not a replacement for
the elastic bending solve. With g_i = d(theta)/dx_i and w_i = free_i/m_i,

    speed = sum(g_i dot v_i), W = sum(w_i |g_i|^2), k = 3 B l_rest^2/(A1+A2),
    c = 2 zeta sqrt(k/W), J = -h c speed/(1+h c W), dv_i = w_i J g_i.

The dimensionless zeta is a LOCAL hinge damping ratio. It is not a calibrated
cotton property or a guarantee of critical damping of global cloth modes.
Four-node-disjoint colors run sequentially. Current native wet masses and free
flags are read on every call. Positions, elastic parameters and masses are not
changed; rigid translation/rotation have zero hinge rate and are preserved.

For prescribed vertices, their supplied velocities are included in speed but
never changed. The work supplied by moving anchors is -J sum_fixed(g_i dot v_i).
Free kinetic-energy change minus this work is nonpositive. Required support
impulses/work are reported, not applied back to the robot. Callers must supply
physically meaningful prescribed velocities; zeroing them would change this
energy boundary. Native exact-link attachments provide the previous substep's
kinematic velocity, so accelerating anchors retain that explicit splitting lag.

The elastic energy scale follows cotton_bending.py / Discrete Shells. Implicit
constraint dissipation background: Macklin et al., XPBD (2016),
https://matthias-research.github.io/pages/publications/XPBD.pdf . The velocity-only
operator and its work accounting here are derived explicitly above; this is not
the paper's combined elastic/damping multiplier update. No global convergence,
continuous contact, or complete bouncing suppression is promised by this split.
"""
from importlib.metadata import version

import numpy as np

from cotton_bending import build_hinges, disjoint_hinge_colors, hinge_angle_gradient


def _parameters(stiffness, damping_ratio, dt):
    values = np.asarray([stiffness, damping_ratio, dt], dtype=float)
    if not np.isfinite(values).all() or stiffness <= 0 or damping_ratio < 0 or dt <= 0:
        raise ValueError("Need positive finite angular stiffness/time step and nonnegative finite damping ratio")


def _state(points, velocities, masses, free, count):
    p, v, m = (np.asarray(x, dtype=np.float64) for x in (points, velocities, masses))
    raw_free = np.asarray(free)
    if p.shape != (count, 3) or v.shape != p.shape or m.shape != (count,) or raw_free.shape != (count,):
        raise ValueError("Damping state shapes must be (N,3), (N,3), (N,), (N,)")
    if not all(np.isfinite(x).all() for x in (p, v, m)) or np.any(m <= 0):
        raise ValueError("Damping positions/velocities must be finite and masses positive")
    if raw_free.dtype != np.dtype(bool):
        raise ValueError("Free flags must be booleans, not arbitrary inverse-mass weights")
    return p, v, m, raw_free


def damp_hinge_reference(points, velocities, masses, free, *, stiffness,
                         damping_ratio=0., dt):
    """Pure NumPy reference: return new velocities and local work diagnostics.

    Inputs are not mutated. Collapsed hinges and hinges with no free particles
    are skipped. For moving anchors free kinetic energy can increase, but only
    within the explicitly reported anchor work. Elastic energy is unchanged
    because this operator does not move positions.
    """
    _parameters(stiffness, damping_ratio, dt)
    p, v, mass, flags = _state(points, velocities, masses, free, 4)
    changed = v.copy()
    stats = {"applied": False, "skip_reason": None, "angular_speed_before": 0.,
             "angular_speed_after": 0., "angular_inverse_mass": 0.,
             "angular_mode_kinetic_energy_before_J": 0., "angular_mode_kinetic_energy_after_J": 0.,
             "kinetic_energy_change_free_J": 0., "anchor_work_J": 0.,
             "dissipated_energy_J": 0., "passivity_balance_error_J": 0.,
             "support_impulse_on_free_kg_m_s": np.zeros(3),
             "support_angular_impulse_on_free_kg_m2_s": np.zeros(3),
             "angular_impulse_kg_m2_s": 0., "fraction_removed": 0.}
    if damping_ratio == 0 or not flags.any():
        stats["skip_reason"] = "disabled" if damping_ratio == 0 else "all_prescribed"
        return changed, stats
    try:
        _, gradient = hinge_angle_gradient(p)
    except ValueError:
        stats["skip_reason"] = "collapsed_hinge"
        return changed, stats
    inverse = flags/mass
    W = float(np.sum(inverse[:, None]*gradient**2))
    if W <= 0 or not np.isfinite(W):
        stats["skip_reason"] = "unresolvable_angular_inverse_mass"
        return changed, stats
    speed = float(np.sum(gradient*v))
    fixed_speed = float(np.sum(gradient[~flags]*v[~flags]))
    # a=h*c*W=2*zeta*h*sqrt(k*W). Evaluate the fraction in a form that
    # saturates at one for very strong damping instead of producing inf/inf.
    with np.errstate(over="ignore"):
        a = 2.*float(damping_ratio)*float(dt)*np.sqrt(float(stiffness))*np.sqrt(W)
    fraction = a/(1.+a) if a <= 1 else 1./(1.+1./a)
    impulse = -fraction*speed/W
    changed += inverse[:, None]*impulse*gradient
    after_speed = float(np.sum(gradient*changed))
    energy_change = float(.5*np.sum(mass[flags, None]*(changed[flags]**2-v[flags]**2)))
    anchor_work = -impulse*fixed_speed
    dissipated = .5*fraction*(2.-fraction)*speed**2/W
    # Support is the equal/opposite of the impulse the frozen vertices would
    # have received. The opposite reaction is owed to the prescribed robot.
    support = -impulse*np.sum(gradient[~flags], axis=0)
    support_torque = -impulse*np.sum(np.cross(p[~flags], gradient[~flags]), axis=0)
    stats.update(applied=True, angular_speed_before=speed, angular_speed_after=after_speed,
                 angular_inverse_mass=W, angular_impulse_kg_m2_s=impulse, fraction_removed=float(fraction),
                 angular_mode_kinetic_energy_before_J=.5*speed**2/W,
                 angular_mode_kinetic_energy_after_J=.5*after_speed**2/W,
                 kinetic_energy_change_free_J=energy_change, anchor_work_J=anchor_work,
                 dissipated_energy_J=float(dissipated),
                 passivity_balance_error_J=float(energy_change-anchor_work+dissipated),
                 support_impulse_on_free_kg_m_s=support,
                 support_angular_impulse_on_free_kg_m2_s=support_torque)
    return changed, stats


class BendingDampingReference:
    """CPU reference for a complete colored mesh pass; no Genesis dependency."""
    def __init__(self, points, faces, *, rigidity=5e-5, damping_ratio=0.):
        _parameters(1., damping_ratio, 1.)
        hinges, _, compliance = build_hinges(points, faces, rigidity)
        order, bounds = disjoint_hinge_colors(hinges, len(points))
        self.hinges, self.stiffness = hinges[order], 1./compliance[order]
        self.color_bounds, self.n_particles = bounds, len(points)
        self.damping_ratio = float(damping_ratio)

    def apply(self, points, velocities, masses, free, dt):
        p, v, mass, flags = _state(points, velocities, masses, free, self.n_particles)
        _parameters(1., self.damping_ratio, dt)
        changed = v.copy()
        records = []
        for ids, stiffness in zip(self.hinges, self.stiffness):
            changed[ids], stats = damp_hinge_reference(p[ids], changed[ids], mass[ids], flags[ids],
                stiffness=stiffness, damping_ratio=self.damping_ratio, dt=dt)
            records.append(stats)
        energy_change = float(.5*np.sum(mass[flags, None]*(changed[flags]**2-v[flags]**2)))
        work = sum(s["anchor_work_J"] for s in records)
        loss = sum(s["dissipated_energy_J"] for s in records)
        return changed, {
            "applied_hinges": sum(s["applied"] for s in records),
            "kinetic_energy_change_free_J": energy_change, "anchor_work_J": work,
            "dissipated_energy_J": loss, "passivity_balance_error_J": energy_change-work+loss,
            "support_impulse_on_free_kg_m_s": sum((s["support_impulse_on_free_kg_m_s"] for s in records), np.zeros(3)),
            "support_angular_impulse_on_free_kg_m2_s": sum((s["support_angular_impulse_on_free_kg_m2_s"] for s in records), np.zeros(3)),
        }


def install_cotton_bending_damping(scene, cloth, points, faces, *, rigidity=5e-5,
                                   damping_ratio=0.):
    """Create an optional native adapter before build; does not install a hook.

    Call ``adapter.apply()`` ONCE at each material substep start, after native
    store_initial and before integration. ``start_substep`` is an alias. Do not
    call once per bending iteration: that would multiply the dissipative time.
    The default ratio zero does not launch a kernel or change native state.
    ``diagnostics()`` reads the most recent pass, not accumulated history.
    There is no physical history to reset/capture/restore; buffers are diagnostic.
    """
    import genesis as gs
    import quadrants as qd
    import torch

    if version("genesis-world") != "1.4.3" or scene.is_built:
        raise RuntimeError("Create the Genesis 1.4.3 damping adapter before scene.build()")
    solver = cloth.solver
    if solver is not scene.pbd_solver or len(solver.entities) != 1 or solver.entities[0] is not cloth:
        raise ValueError("Bending damping requires this cloth as the scene's sole PBD entity")
    prepared = BendingDampingReference(points, faces, rigidity=rigidity, damping_ratio=damping_ratio)
    if len(points) != cloth.n_particles or len(prepared.hinges) != cloth.n_inner_edges:
        raise ValueError("Bending damping topology must match the native cloth")

    @qd.data_oriented
    class CottonBendingDamping:
        def __init__(self):
            self.solver = solver
            self.ratio = float(damping_ratio)
            self.enabled = self.ratio > 0 and len(prepared.hinges) > 0
            self.bounds = prepared.color_bounds
            self.colors = len(self.bounds)-1
            self.hinges = torch.as_tensor(np.ascontiguousarray(prepared.hinges+cloth.particle_start),
                                          dtype=gs.tc_int, device=gs.device)
            self.stiffness = torch.as_tensor(np.ascontiguousarray(prepared.stiffness), dtype=gs.tc_float, device=gs.device)
            # Per-hinge: free energy change, anchor work, loss, support impulse
            # xyz, support angular impulse xyz, applied flag. Overwritten each
            # pass, never used as an input to the physical update.
            self.stats = torch.zeros((len(prepared.hinges), 10), dtype=gs.tc_float, device=gs.device)
            self.description = {
                "model": "passive implicit local signed-dihedral velocity dashpots",
                "enabled": self.enabled, "local_damping_ratio": self.ratio,
                "bending_rigidity_N_m": float(rigidity),
                "hinges": len(prepared.hinges), "disjoint_hinge_colors": self.colors,
                "angular_stiffness": "k=3*B*rest_edge_length^2/sum_adjacent_rest_areas",
                "angular_inverse_mass": "W=sum(free_i/current_native_mass_i * |dtheta/dx_i|^2)",
                "angular_viscosity": "c=2*zeta*sqrt(k/W)",
                "angular_impulse": "J=-h*c*theta_dot/(1+h*c*W)",
                "integration": "one sequential colored implicit velocity pass per native substep",
                "rigid_motion_preserved": True, "positions_changed": False,
                "native_wet_masses_read_each_call": True, "prescribed_velocities_changed": False,
                "moving_anchor_energy_boundary": "delta_free_kinetic - anchor_work = -dissipation <= 0",
                "support_reaction_returned_to_robot": False,
                "physical_state_persisted_between_substeps": False,
                "diagnostics": "last-pass algebraic impulse/work ledger, not an independent measurement of rounded native kinetic energy",
                "local_ratio_is_global_modal_critical_damping": False,
                "experimentally_calibrated": False,
            }

        def apply(self):
            if not self.enabled:
                return
            if not scene.is_built or solver._B != 1:
                raise RuntimeError("Bending damping requires built single-environment PBD fields")
            self.project(self.hinges, self.stiffness, self.stats)

        def start_substep(self):
            self.apply()

        def diagnostics(self):
            total = self.stats.detach().cpu().numpy().astype(np.float64).sum(axis=0)
            return {"enabled": self.enabled, "applied_hinges": int(round(total[9])),
                    "kinetic_energy_change_free_J": float(total[0]), "anchor_work_J": float(total[1]),
                    "dissipated_energy_J": float(total[2]),
                    "passivity_balance_error_J": float(total[0]-total[1]+total[2]),
                    "support_impulse_on_free_kg_m_s": total[3:6].tolist(),
                    "support_angular_impulse_on_free_kg_m2_s": total[6:9].tolist()}

        @qd.kernel
        def project(self, ids: qd.types.ndarray(), stiffness: qd.types.ndarray(), stats: qd.types.ndarray()):
            for color in qd.static(range(self.colors)):
                for k in range(self.bounds[color], self.bounds[color+1]):
                    for j in qd.static(range(10)):
                        stats[k, j] = 0.
                    p1 = self.solver.particles[ids[k, 0], 0].pos
                    p2 = self.solver.particles[ids[k, 1], 0].pos
                    p3 = self.solver.particles[ids[k, 2], 0].pos
                    p4 = self.solver.particles[ids[k, 3], 0].pos
                    positions = qd.Matrix.rows([p1, p2, p3, p4])
                    inverse = qd.Vector.zero(gs.qd_float, 4)
                    for j in qd.static(range(4)):
                        node = ids[k, j]
                        inverse[j] = self.solver.particles[node, 0].free/self.solver.particles_info[node].mass
                    edge = positions[1, :]-positions[0, :]
                    edge2 = edge.norm_sqr()
                    n1 = edge.cross(positions[2, :]-positions[0, :])
                    n2 = (positions[3, :]-positions[0, :]).cross(edge)
                    n12, n22 = n1.norm_sqr(), n2.norm_sqr()
                    if edge2 > 1e-24 and n12 > 1e-24 and n22 > 1e-24:
                        length = qd.sqrt(edge2)
                        g3, g4 = -length*n1/n12, -length*n2/n22
                        g2 = (-(positions[2, :]-positions[0, :]).dot(edge)/edge2*g3
                              -(positions[3, :]-positions[0, :]).dot(edge)/edge2*g4)
                        g1 = -g2-g3-g4
                        gradient = qd.Matrix.rows([g1, g2, g3, g4])
                        W, speed, fixed_speed = gs.qd_float(0.), gs.qd_float(0.), gs.qd_float(0.)
                        fixed_gradient = qd.Vector.zero(gs.qd_float, 3)
                        fixed_torque = qd.Vector.zero(gs.qd_float, 3)
                        for j in qd.static(range(4)):
                            W += inverse[j]*gradient[j, :].norm_sqr()
                            rate = gradient[j, :].dot(self.solver.particles[ids[k, j], 0].vel)
                            speed += rate
                            if not self.solver.particles[ids[k, j], 0].free:
                                fixed_speed += rate
                                fixed_gradient += gradient[j, :]
                                fixed_torque += positions[j, :].cross(gradient[j, :])
                        if W > 0.:
                            a = 2.*self.ratio*self.solver._substep_dt*qd.sqrt(stiffness[k])*qd.sqrt(W)
                            fraction = a/(1.+a)
                            if a > 1.:
                                fraction = 1./(1.+1./a)
                            impulse = -fraction*speed/W
                            for j in qd.static(range(4)):
                                self.solver.particles[ids[k, j], 0].vel += inverse[j]*impulse*gradient[j, :]
                            work = -impulse*fixed_speed
                            loss = .5*fraction*(2.-fraction)*speed*speed/W
                            stats[k, 0], stats[k, 1], stats[k, 2] = work-loss, work, loss
                            for j in qd.static(range(3)):
                                stats[k, 3+j] = -impulse*fixed_gradient[j]
                                stats[k, 6+j] = -impulse*fixed_torque[j]
                            stats[k, 9] = 1.

    return CottonBendingDamping()
