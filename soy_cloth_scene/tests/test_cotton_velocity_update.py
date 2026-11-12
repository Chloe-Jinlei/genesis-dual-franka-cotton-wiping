"""CPU lifecycle/indexing checks of the actual local velocity adapter.

Decorator/field stubs execute its kernel Python bodies without compiling native
code. Numerical tests exercise world-coordinate rounding separately. No GPU or
scene build occurs; native compiler/contact checks remain integration work.
"""
import copy
from itertools import product
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cotton_velocity_update import install_cotton_velocity_update


class Field:
    def __init__(self, rows):
        self.rows = rows

    def __getitem__(self, key):
        return self.rows[key[0]][key[1]]


class ClothMaterial:
    pass


class CottonVelocityUpdateTests(unittest.TestCase):
    def setUp(self):
        self.count, self.batches, self.h = 4, 2, .0001
        def particle(i, b):
            return SimpleNamespace(pos=np.array([.8+i*.01, -.2+b*.1, .95], np.float32),
                                   vel=np.array([.01+i*.02, -.02+b*.01, .03], np.float32),
                                   ipos=np.zeros(3, np.float32), free=i != 2)
        self.original = Field([[particle(i, b) for b in range(self.batches)] for i in range(self.count)])
        self.reordered = Field([[particle(i, b) for b in range(self.batches)] for i in range(self.count)])
        # Different nontrivial permutations for the two environments detect
        # both original/reordered and batch index mistakes.
        permutations = [[2, 0, 3, 1], [1, 3, 0, 2]]
        self.ng = Field([[SimpleNamespace(active=True, reordered_idx=permutations[b][i])
                          for b in range(self.batches)] for i in range(self.count)])
        self.calls = []

        def native_force(f, t):
            self.calls.append(("force", f, t))
            for i, b in product(range(self.count), range(self.batches)):
                p = self.original[i, b]
                p.ipos = p.pos.copy()
                p.pos = (p.pos + p.vel*np.float32(self.h)).astype(np.float32)

        def native_velocity(f):
            self.calls.append(("velocity", f))

        self.solver = SimpleNamespace(_n_particles=self.count, _B=self.batches,
            _substep_dt=self.h, particles=self.original, particles_reordered=self.reordered,
            particles_ng=self.ng, _kernel_apply_external_force=native_force,
            _kernel_compute_velocity=native_velocity)
        self.cloth = SimpleNamespace(solver=self.solver, material=ClothMaterial())
        self.solver.entities = [self.cloth]
        self.scene = SimpleNamespace(is_built=False, pbd_solver=self.solver)
        fake_gs = SimpleNamespace(tc_float=np.float32, device="cpu",
            materials=SimpleNamespace(PBD=SimpleNamespace(Cloth=ClothMaterial)))
        fake_qd = SimpleNamespace(data_oriented=lambda x:x, kernel=lambda x:x,
            static=lambda x:x, ndrange=lambda *dims: product(*(range(n) for n in dims)),
            types=SimpleNamespace(ndarray=lambda:object))
        fake_torch = SimpleNamespace(empty=lambda shape,dtype,device: np.empty(shape,dtype=dtype))
        modules = patch.dict(sys.modules, {"genesis":fake_gs,"quadrants":fake_qd,"torch":fake_torch})
        modules.start();self.addCleanup(modules.stop)
        versions = patch("cotton_velocity_update.version",return_value="1.4.3")
        versions.start();self.addCleanup(versions.stop)

    def reorder(self):
        for i,b in product(range(self.count),range(self.batches)):
            if self.ng[i,b].active:
                self.reordered.rows[self.ng[i,b].reordered_idx][b] = copy.deepcopy(self.original[i,b])

    def test_unconstrained_predictions_survive_reorder_without_rounding_snap(self):
        adapter = install_cotton_velocity_update(self.scene,self.cloth)
        self.solver._kernel_apply_external_force(7,.003)
        self.reorder()
        expected = adapter.predicted_vel.copy()
        native_velocity = (adapter.predicted_pos[0,0]-self.original[0,0].ipos)/self.h
        self.assertGreater(np.linalg.norm(native_velocity-expected[0,0]), 1e-5)
        self.solver._kernel_compute_velocity(7)
        for i,b in product(range(self.count),range(self.batches)):
            np.testing.assert_array_equal(self.reordered[self.ng[i,b].reordered_idx,b].vel,expected[i,b])
        self.assertEqual(self.calls,[("force",7,.003)])

    def test_constraint_displacement_contributes_impulse_with_correct_mapping(self):
        adapter = install_cotton_velocity_update(self.scene,self.cloth)
        self.solver._kernel_apply_external_force(0,0.)
        self.reorder()
        expected={}
        for i,b in product(range(self.count),range(self.batches)):
            j=self.ng[i,b].reordered_idx;p=self.reordered[j,b]
            # An actual projected position, not a velocity command.
            p.pos += np.array([i+1,-b-1,(i-b)*.5],np.float32)*1e-5
            expected[i,b]=adapter.predicted_vel[i,b]+(p.pos-adapter.predicted_pos[i,b])/self.h
        self.solver._kernel_compute_velocity(0)
        for i,b in expected:
            np.testing.assert_allclose(self.reordered[self.ng[i,b].reordered_idx,b].vel,expected[i,b],rtol=2e-7)

    def test_fixed_particles_are_not_frozen_and_inactive_slots_are_untouched(self):
        self.ng[3,1].active=False
        unused=self.ng[3,1].reordered_idx
        sentinel=np.array([98.,99.,100.],np.float32)
        self.reordered[unused,1].vel=sentinel.copy()
        adapter=install_cotton_velocity_update(self.scene,self.cloth)
        self.solver._kernel_apply_external_force(0,0.);self.reorder()
        self.solver._kernel_compute_velocity(0)
        np.testing.assert_array_equal(self.reordered[unused,1].vel,sentinel)
        for b in range(self.batches):
            self.assertFalse(self.original[2,b].free)
            np.testing.assert_array_equal(self.reordered[self.ng[2,b].reordered_idx,b].vel,adapter.predicted_vel[2,b])
            self.assertGreater(np.linalg.norm(adapter.predicted_vel[2,b]),0.)

    def test_every_substep_overwrites_predictions_after_a_restored_state(self):
        adapter=install_cotton_velocity_update(self.scene,self.cloth)
        with self.assertRaisesRegex(RuntimeError,"integrated predictions"):
            self.solver._kernel_compute_velocity(0)
        self.solver._kernel_apply_external_force(0,0.)
        cache_id=id(adapter.predicted_pos)
        for i,b in product(range(self.count),range(self.batches)):
            self.original[i,b].pos[:]=[2.,3.,4.]
            self.original[i,b].vel[:]=[.31,.32,.33]
        self.solver._kernel_apply_external_force(0,10.)
        self.reorder();self.solver._kernel_compute_velocity(0)
        self.assertEqual(id(adapter.predicted_pos),cache_id)
        np.testing.assert_allclose(adapter.predicted_vel,np.broadcast_to([.31,.32,.33],adapter.predicted_vel.shape),rtol=1e-7)
        self.assertFalse(adapter.description["persistent_checkpoint_state"])

    def test_install_guard_and_restore_do_not_clobber_later_wrappers(self):
        old_force=self.solver._kernel_apply_external_force;old_velocity=self.solver._kernel_compute_velocity
        adapter=install_cotton_velocity_update(self.scene,self.cloth)
        with self.assertRaisesRegex(RuntimeError,"already installed"):
            install_cotton_velocity_update(self.scene,self.cloth)
        adapter.close()
        self.assertIs(self.solver._kernel_apply_external_force,old_force)
        self.assertIs(self.solver._kernel_compute_velocity,old_velocity)
        self.assertFalse(hasattr(self.solver,"_cotton_velocity_adapter"))
        adapter=install_cotton_velocity_update(self.scene,self.cloth)
        replacement=lambda f:None
        self.solver._kernel_compute_velocity=replacement
        adapter.close()
        self.assertIs(self.solver._kernel_compute_velocity,replacement)
        self.scene.is_built=True
        with self.assertRaisesRegex(RuntimeError,"before scene.build"):
            install_cotton_velocity_update(self.scene,self.cloth)


class VelocityArithmeticTests(unittest.TestCase):
    def test_equivalent_to_native_in_exact_precision_with_constraint_corrections(self):
        rng=np.random.default_rng(7);h=.0001
        x=rng.uniform(-1.,1.,(50,3));v=rng.normal(size=(50,3))
        predicted=x+v*h;projected=predicted+rng.normal(scale=1e-6,size=(50,3))
        np.testing.assert_allclose(v+(projected-predicted)/h,(projected-x)/h,atol=6e-13,rtol=1e-10)

    def test_float32_increment_preserves_drag_that_world_reconstruction_loses(self):
        from probe_pbd_velocity_resolution import simulate_free_decay
        native=simulate_free_decay("native",np.float32)
        moved=simulate_free_decay("post_drag",np.float32)
        incremental=simulate_free_decay("incremental",np.float32)
        reference=simulate_free_decay("native",np.float64)
        expected=.1*np.exp(-1.)
        self.assertGreater(native["final_velocity_m_s"],.09)
        self.assertGreater(moved["final_velocity_m_s"],.09)
        self.assertLess(abs(incremental["final_velocity_m_s"]-expected),1e-5)
        self.assertLess(abs(reference["final_velocity_m_s"]-expected),1e-9)


if __name__=="__main__":
    unittest.main()
