"""CPU regressions for corner material selection and locked-in cloth stretch.

No Genesis runtime or scene is created. Inputs represent already measured pad
contact sets; the production contact and topology gates remain independent.
"""
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import robot_grasp
from robot_grasp import _minimum_reference_safe_subset


class MaterialGraspTests(unittest.TestCase):
    @staticmethod
    def choose(points, reference, faces, left, right, *, minimum=2,
               path_limit=.056, strict=False, diameter=.0015):
        return _minimum_reference_safe_subset(
            np.asarray(points, dtype=float), np.asarray(reference, dtype=float),
            np.asarray(faces, dtype=np.int64), np.zeros(3),
            np.asarray(left, dtype=np.int64), np.asarray(right, dtype=np.int64),
            np.ones(len(points), dtype=bool), .028, path_limit, .02,
            minimum_bound_per_finger=minimum,
            preserve_contact_geometry=strict, collision_diameter_m=diameter)

    def test_strict_geometry_rejects_compressed_direct_material_edges(self):
        rest = np.array([[0, 0, 0], [.004, 0, 0], [.002, .003, 0]])
        current = rest*.97
        # Legacy extension-only behavior remains available for the validated
        # dry prefix. Wet strict mode must not freeze a 3% compressed edge,
        # even though this is inside the separate 5% incident-neighborhood cap.
        chosen, evidence = self.choose(current, rest, [[0, 1, 2]],
                                       [0, 1], [1, 2], minimum=1)
        self.assertEqual(len(chosen), 2)
        self.assertAlmostEqual(evidence["fixed_edge_max_compression"], .03)
        chosen, evidence = self.choose(current, rest, [[0, 1, 2]],
                                       [0, 1], [1, 2], minimum=1, strict=True)
        self.assertEqual(len(chosen), 0)
        self.assertFalse(evidence["minimum_contact_subset_found"])

    def test_strict_geometry_allows_folded_nonadjacent_material_endpoints(self):
        rest = np.array([[0, -.002, 0], [.004, -.002, 0], [.008, -.002, 0],
                         [0, .002, 0], [.004, .002, 0], [.008, .002, 0]])
        current = rest.copy()
        # Rotate the last panel around the material fold line, preserving its
        # edge lengths rather than stretching it to obtain the 1.6 mm height.
        current[[2, 5], 0] = .004-np.sqrt(.004**2-.0016**2)
        current[[2, 5], 2] = .0016
        faces = [[0, 1, 3], [1, 4, 3], [1, 2, 4], [2, 5, 4]]
        chosen, evidence = self.choose(current, rest, faces, [0, 3], [2, 5],
                                       minimum=1, strict=True)
        np.testing.assert_array_equal(chosen, [0, 2])
        self.assertEqual(evidence["fixed_edge_count"], 0)
        self.assertGreater(evidence["fixed_pair_min_distance_m"], .0015)
        self.assertLess(evidence["selected_incident_edge_max_absolute_strain"], 1e-10)
        self.assertEqual(evidence["fixed_point_pair_max_extension"], 0.)
        # A real 8+ mm folded surface path exists; a spatial shortcut is not
        # allowed to meet a shorter 2 mm topology gate.
        chosen, _ = self.choose(current, rest, faces, [0, 3], [2, 5],
                                 minimum=1, strict=True, path_limit=.002)
        self.assertEqual(len(chosen), 0)

    def test_strict_geometry_rejects_selected_native_selfcollision_pairs(self):
        # Nonadjacent endpoints may fold, but may not be fixed with collision
        # spheres interpenetrating. Distinct support nodes do not waive this.
        rest = np.array([[0, -.0001, 0], [.004, -.0001, 0], [.008, -.0001, 0],
                         [0, .0001, 0], [.004, .0001, 0], [.008, .0001, 0]])
        current = rest.copy()
        current[[2, 5], 0] = .004-np.sqrt(.004**2-.0013**2)
        current[[2, 5], 2] = .0013
        faces = [[0, 1, 3], [1, 4, 3], [1, 2, 4], [2, 5, 4]]
        chosen, _ = self.choose(current, rest, faces, [0, 3], [2, 5], minimum=1)
        self.assertEqual(len(chosen), 2)
        chosen, _ = self.choose(current, rest, faces, [0, 3], [2, 5],
                                 minimum=1, strict=True)
        self.assertEqual(len(chosen), 0)

    def test_incident_gate_rejects_strained_free_neighbors_with_safe_fixed_pair(self):
        rest = np.array([[-.002, -.004, 0], [.002, -.004, 0],
                         [-.002, .004, 0], [.002, .004, 0], [0, 0, 0]])
        current = rest.copy()
        current[4, 2] = .003  # Every possible bound node has a strained free neighbor.
        faces = [[0, 1, 4], [1, 3, 4], [3, 2, 4], [2, 0, 4]]
        chosen, legacy = self.choose(current, rest, faces, [0, 1], [2, 3], minimum=1)
        self.assertEqual(len(chosen), 2)
        self.assertEqual(legacy["fixed_point_pair_max_extension"], 0.)
        chosen, evidence = self.choose(current, rest, faces, [0, 1], [2, 3], minimum=1, strict=True)
        self.assertEqual(len(chosen), 0)
        self.assertEqual(evidence["contact_candidates_rejected_by_incident_strain"], [0, 1, 2, 3])
        self.assertTrue(all(v > .05 for v in evidence["contact_candidate_incident_max_absolute_strain"].values()))

    def test_strict_selection_prioritizes_lower_incident_strain_before_compactness(self):
        rest = np.array([[0, -.002, 0], [.004, -.004, 0], [0, .002, 0], [.004, .004, 0],
                         [-.003, 0, 0]])
        current = rest.copy()
        current[4, 0] -= .0001  # 2.32% strain only at the more compact 0/2 pair.
        faces = [[0, 1, 2], [1, 3, 2], [0, 2, 4]]
        chosen, _ = self.choose(current, rest, faces, [0, 1], [2, 3], minimum=1)
        np.testing.assert_array_equal(chosen, [0, 2])
        chosen, evidence = self.choose(current, rest, faces, [0, 1], [2, 3], minimum=1, strict=True)
        np.testing.assert_array_equal(chosen, [1, 3])
        self.assertEqual(evidence["selected_incident_edge_max_absolute_strain"], 0.)
        self.assertAlmostEqual(evidence["attached_reference_material_diameter_m"], .008)
        self.assertGreater(evidence["contact_candidate_incident_max_absolute_strain"]["0"], .02)

    def test_incident_gate_uses_only_mesh_edges_at_selected_nodes(self):
        rest = np.array([[0, -.002, 0], [.004, -.004, 0], [0, .002, 0], [.004, .004, 0],
                         [-.003, 0, 0], [-.006, 0, 0]])
        current = rest.copy()
        current[5, 2] = .005  # Distorted distant edge, not incident to selected 1/3.
        faces = [[0, 1, 2], [1, 3, 2], [0, 2, 4], [0, 4, 5]]
        chosen, evidence = self.choose(current, rest, faces, [0, 1], [2, 3], minimum=1, strict=True)
        np.testing.assert_array_equal(chosen, [1, 2])
        self.assertEqual(evidence["selected_incident_edge_max_absolute_strain"], 0.)
        self.assertEqual(evidence["contact_candidates_rejected_by_incident_strain"], [0])

    def test_rejects_already_stretched_fixed_pair(self):
        # Both nodes on each jaw would be fixed. Once the 3% stretch was locked
        # into their offsets, increasing cloth stiffness could never remove it.
        rest = np.array([[-.002, -.004, 0], [.002, -.004, 0],
                         [-.002, .004, 0], [.002, .004, 0]])
        current = rest.copy()
        current[:, 0] *= 1.03
        chosen, evidence = self.choose(current, rest, [[0, 1, 2], [1, 3, 2]],
                                       [0, 1], [2, 3])
        self.assertEqual(len(chosen), 0)
        self.assertFalse(evidence["minimum_contact_subset_found"])

    def test_selects_minimal_two_per_jaw_low_strain_subset(self):
        rest = np.array([[-.004, -.004, 0], [0, -.004, 0], [.004, -.004, 0],
                         [-.004, .004, 0], [0, .004, 0], [.004, .004, 0]])
        current = rest.copy()
        current[2, 0] += .004  # This extra contact is already overstretched.
        chosen, evidence = self.choose(current, rest,
            [[0, 1, 3], [1, 4, 3], [1, 2, 4], [2, 5, 4]],
            [0, 1, 2], [3, 4, 5])
        np.testing.assert_array_equal(chosen, [0, 1, 3, 4])
        self.assertEqual(evidence["contact_nodes_after_subset"], 4)
        self.assertEqual(evidence["fixed_point_pair_max_extension"], 0.0)
        self.assertTrue(evidence["minimum_contact_subset_found"])

    def test_never_recruits_nodes_outside_supplied_material_candidates(self):
        # Nodes 4/5 are closer to the TCP and belong to the same local connected
        # cloth, but are outside the material corner candidates the caller gave.
        rest = np.array([[-.004, -.004, 0], [.004, -.004, 0],
                         [-.004, .004, 0], [.004, .004, 0],
                         [0, -.003, 0], [0, .003, 0]])
        faces = [[0, 4, 2], [4, 5, 2], [4, 1, 5], [1, 3, 5]]
        chosen, evidence = self.choose(rest, rest, faces, [0, 1], [2, 3])
        np.testing.assert_array_equal(chosen, [0, 1, 2, 3])
        self.assertTrue(evidence["minimum_contact_subset_found"])
        # Nor may it borrow the nearby outside node to fill a missing second
        # contact on one jaw when the intended material patch has only one.
        chosen, evidence = self.choose(rest, rest, faces, [0], [2, 3])
        self.assertEqual(len(chosen), 0)
        self.assertFalse(evidence["minimum_contact_subset_found"])

    def test_one_per_jaw_keeps_support_gate_and_avoids_overconstraint(self):
        rest = np.array([[-.002, -.004, 0], [.002, -.004, 0],
                         [-.002, .004, 0], [.002, .004, 0]])
        current = rest.copy()
        current[:, 0] *= 1.03
        faces = [[0, 1, 2], [1, 3, 2]]
        chosen, evidence = self.choose(current, rest, faces, [0, 1], [2, 3], minimum=1)
        np.testing.assert_array_equal(chosen, [0, 2])
        self.assertEqual(evidence["selected_left_representative_indices"], [0])
        self.assertEqual(evidence["selected_right_representative_indices"], [2])
        self.assertEqual(evidence["supporting_contact_nodes_left_free"], [1, 3])
        self.assertEqual(evidence["fixed_point_pair_max_extension"], 0.0)
        # Smaller binding must not fill a missing supporting contact from any
        # other material or treat one real pad contact as sufficient evidence.
        chosen, evidence = self.choose(current, rest, faces, [0], [2, 3], minimum=1)
        self.assertEqual(len(chosen), 0)

    def test_one_per_jaw_still_rejects_stretched_or_disconnected_pairs(self):
        rest = np.array([[-.002, -.004, 0], [.002, -.004, 0],
                         [-.002, .004, 0], [.002, .004, 0]])
        faces = [[0, 1, 2], [1, 3, 2]]
        chosen, _ = self.choose(rest*1.03, rest, faces, [0, 1], [2, 3], minimum=1)
        self.assertEqual(len(chosen), 0)
        chosen, _ = self.choose(rest, rest, faces, [0, 1], [2, 3],
                                minimum=1, path_limit=.007)
        self.assertEqual(len(chosen), 0)

    def test_shared_contacts_need_two_distinct_representatives_and_real_path(self):
        rest = np.array([[0, 0, 0], [.004, 0, 0], [.002, .003, 0]])
        chosen, evidence = self.choose(rest, rest, [[0, 1, 2]], [0, 1], [0, 1], minimum=1)
        np.testing.assert_array_equal(chosen, [0, 1])
        self.assertNotEqual(evidence["selected_left_representative_indices"],
                            evidence["selected_right_representative_indices"])
        # A shared contact must not supply a zero-length path between two
        # physically different bound representatives.
        chosen, _ = self.choose(rest, rest, [[0, 1, 2]], [0, 1], [0, 1],
                                minimum=1, path_limit=.003)
        self.assertEqual(len(chosen), 0)

    def test_attach_keeps_two_per_jaw_proof_separate_from_bound_subset(self):
        reference = np.array([[-.002, -.001, 0], [.002, -.001, 0],
                              [-.002, .001, 0], [.002, .001, 0]])
        points = reference.copy()
        points[:, 0] *= 1.03
        links = np.full((4, 1), -1)
        field = SimpleNamespace(to_numpy=lambda: links.copy())
        scene = SimpleNamespace(n_envs=0, is_built=True, _contact_grasp_adapter={},
            sim=SimpleNamespace(cur_step_global=10,
                coupler=SimpleNamespace(particle_attach_info=SimpleNamespace(link_idx=field))))
        hand = SimpleNamespace(idx=7, pos=np.array([0, 0, -.1029]), rotation=np.eye(3))
        left = SimpleNamespace(pos=np.array([0, -.0035, -.0445]), rotation=np.eye(3))
        right = SimpleNamespace(pos=np.array([0, .0035, -.0445]), rotation=np.diag([-1, -1, 1]))
        named_links = {"hand": hand, "left_finger": left, "right_finger": right}
        robot = SimpleNamespace(_scene=scene, name="mock_franka", get_link=named_links.__getitem__,
            get_joint=lambda name: SimpleNamespace(dofs_idx_local=[int(name[-1])-1]),
            get_dofs_position=lambda ids: np.zeros(2),
            get_dofs_control_force=lambda ids: np.ones(2))
        bound = []
        cloth = SimpleNamespace(_scene=scene, n_particles=4, particle_start=0,
            particle_size=.003, collision_radius_m=.00075,
            mesh=SimpleNamespace(faces=np.array([[0, 1, 2], [1, 3, 2]])),
            get_particles_pos=lambda: points.copy(),
            fix_particles_to_link=lambda link, particles_idx_local: bound.extend(particles_idx_local))
        def distance(values, finger):
            contact = values[:, 1] < 0 if finger is left else values[:, 1] > 0
            return np.where(contact, 0.0, .01)
        with patch.object(robot_grasp, "_link_pose", side_effect=lambda link: (link.pos, link.rotation)), \
                patch.object(robot_grasp, "_pad_distance", side_effect=distance):
            report = robot_grasp.attach_grasp(robot, cloth, np.arange(4),
                gate_mode="bilateral_patch", commanded_finger_opening=0.,
                reference_positions=reference, minimum_bound_per_finger=1)
        self.assertTrue(report["attached"], report["reasons"])
        self.assertEqual(report["bilateral_patch"]["minimum_contact_nodes_per_finger"], 2)
        self.assertEqual(report["bilateral_patch"]["left_contact_indices"], [0, 1])
        self.assertEqual(report["bilateral_patch"]["right_contact_indices"], [2, 3])
        self.assertEqual(report["bound_subset_connectivity"]["minimum_contact_nodes_per_finger"], 1)
        self.assertGreater(report["bound_subset_connectivity"]["minimum_connection_path_length_m"], 0)
        self.assertEqual(bound, [0, 2])
        self.assertEqual(report["immediate_max_position_change_m"], 0.)


class LinkAttachmentTests(unittest.TestCase):
    def setUp(self):
        self.reference = np.array([[-.002, -.001, 0], [.002, -.001, 0],
                                   [-.002, .001, 0], [.002, .001, 0]])
        self.points = self.reference.copy()
        self.points[:, 0] *= 1.03
        self.links = np.full((4, 1), -1)
        field = SimpleNamespace(to_numpy=lambda: self.links.copy())
        scene = SimpleNamespace(n_envs=0, is_built=True, _contact_grasp_adapter={},
            sim=SimpleNamespace(cur_step_global=10,
                coupler=SimpleNamespace(particle_attach_info=SimpleNamespace(link_idx=field))))
        self.hand = SimpleNamespace(idx=7, pos=np.array([0., 0., -.1029]), rotation=np.eye(3))
        self.left = SimpleNamespace(idx=8, pos=np.array([0., -.0035, -.0445]), rotation=np.eye(3))
        self.right = SimpleNamespace(idx=9, pos=np.array([0., .0035, -.0445]), rotation=np.diag([-1, -1, 1]))
        named = {"hand": self.hand, "left_finger": self.left, "right_finger": self.right}
        self.robot = SimpleNamespace(_scene=scene, name="mock_franka", get_link=named.__getitem__,
            get_joint=lambda name: SimpleNamespace(dofs_idx_local=[int(name[-1])-1]),
            get_dofs_position=lambda ids: np.zeros(2),
            get_dofs_control_force=lambda ids: np.ones(2))
        self.calls, self.released = [], []
        def fix(link_id, particles_idx_local):
            link = next(link for link in named.values() if link.idx == link_id)
            ids = np.asarray(particles_idx_local, np.int64)
            self.calls.append((link_id, ids.copy(), (self.points[ids]-link.pos)@link.rotation))
            self.links[ids, 0] = link_id
        def release(particles_idx_local):
            self.released.extend(particles_idx_local.tolist())
            self.links[particles_idx_local, 0] = -1
        # No position/velocity setter exists: attachment must use current data.
        self.cloth = SimpleNamespace(_scene=scene, n_particles=4, particle_start=0,
            particle_size=.003, collision_radius_m=.00075,
            mesh=SimpleNamespace(faces=np.array([[0, 1, 2], [1, 3, 2]])),
            get_particles_pos=lambda: self.points.copy(),
            fix_particles_to_link=fix, release_particle=release)
        self.contact_distance = .00075
        self.shared_contacts = False
        def distance(values, finger):
            contact = (np.ones(len(values), bool) if self.shared_contacts else
                       values[:, 1] < 0 if finger is self.left else values[:, 1] > 0)
            return np.where(contact, self.contact_distance, .01)
        self.pose_patch = patch.object(robot_grasp, "_link_pose", side_effect=lambda link: (link.pos, link.rotation))
        self.distance_patch = patch.object(robot_grasp, "_pad_distance", side_effect=distance)
        self.pose_patch.start()
        self.distance_patch.start()
        self.addCleanup(self.pose_patch.stop)
        self.addCleanup(self.distance_patch.stop)

    def attach(self, **kwargs):
        options = dict(gate_mode="bilateral_patch", commanded_finger_opening=0.,
                       reference_positions=self.reference, minimum_bound_per_finger=1,
                       attachment_mode="finger_pads", preserve_contact_geometry=True)
        options.update(kwargs)
        return robot_grasp.attach_grasp(self.robot, self.cloth, np.arange(4), **options)

    def test_finger_links_bind_two_current_offsets_without_moving_particles(self):
        before = self.points.copy()
        report = self.attach()
        self.assertTrue(report["attached"], report["reasons"])
        self.assertEqual([call[0] for call in self.calls], [8, 9])
        np.testing.assert_array_equal(self.calls[0][1], [0])
        np.testing.assert_array_equal(self.calls[1][1], [2])
        for call, link in zip(self.calls, (self.left, self.right)):
            np.testing.assert_array_equal(call[2], (before[call[1]]-link.pos)@link.rotation)
        np.testing.assert_array_equal(self.points, before)
        self.assertEqual(report["immediate_max_position_change_m"], 0.)
        self.assertEqual(report["particle_link_owners"], {"0": 8, "2": 9})
        self.assertEqual(self.cloth._contact_grasp_nodes, {0: 8, 2: 9})
        self.assertEqual(report["bilateral_patch"]["minimum_contact_nodes_per_finger"], 2)

    def test_robot_detach_releases_its_hand_and_fingers_only(self):
        self.assertTrue(self.attach()["attached"])
        self.cloth._contact_grasp_nodes.update({1: 7, 3: 100})
        self.links[1, 0], self.links[3, 0] = 7, 100
        report = robot_grasp.detach(self.cloth, robot=self.robot)
        self.assertEqual(report["detached_indices"], [0, 1, 2])
        self.assertEqual(self.released, [0, 1, 2])
        self.assertEqual(self.cloth._contact_grasp_nodes, {3: 100})
        self.assertEqual(self.links[3, 0], 100)

    def test_finger_mode_rejects_missing_reference_or_ambiguous_shared_subset(self):
        with self.assertRaises(ValueError):
            self.attach(reference_positions=None)
        self.shared_contacts = True
        # Only two safe nodes remain: both would be claimed by both jaws if
        # each side bound two representatives. This assignment is ambiguous.
        self.points[1] = [.1, 0, 0]
        self.points[3] = [.1, .1, 0]
        report = self.attach(minimum_bound_per_finger=2)
        self.assertFalse(report["attached"])
        self.assertEqual(self.calls, [])
        self.assertTrue(any("distinct" in reason for reason in report["reasons"]))

    def test_strict_mode_excludes_pad_sphere_penetration_but_legacy_unchanged(self):
        self.contact_distance = .0006  # 0.15 mm penetration, outside 0.05 mm allowance.
        report = self.attach()
        self.assertFalse(report["attached"])
        self.assertFalse(any(report["candidate_shell_nonpenetrating"]))
        self.assertEqual(self.calls, [])
        report = self.attach(preserve_contact_geometry=False, attachment_mode="hand")
        self.assertTrue(report["attached"], report["reasons"])
        self.assertEqual([call[0] for call in self.calls], [7])

    def test_strict_gap_adds_only_authored_central_to_distal_offset(self):
        geometry = robot_grasp._pad_gap_geometry(.00075)
        self.assertAlmostEqual(geometry["central_to_distal_pad_gap_offset_m"], .003)
        self.assertAlmostEqual(geometry["maximum_distal_pad_gap_m"], .0045)
        self.assertAlmostEqual(geometry["maximum_central_pad_gap_m"], .0075)
        # Actual central gap is 5.2 mm; narrower authored distal gap is 2.2 mm.
        self.left.pos[1], self.right.pos[1] = -.0041, .0041
        report = self.attach(preserve_contact_geometry=False, attachment_mode="hand")
        self.assertFalse(report["attached"])
        self.assertAlmostEqual(report["actual_central_pad_gap_m"], .0052)
        self.assertAlmostEqual(report["max_jaw_gap_m"], .0045)
        report = self.attach(attachment_mode="hand")
        self.assertTrue(report["attached"], report["reasons"])
        self.assertAlmostEqual(report["max_jaw_gap_m"], .0075)
        self.assertTrue(report["gap_limit_includes_authored_pad_offset"])
        with self.assertRaises(ValueError):
            self.attach(max_jaw_gap=.0076)

    def test_pad_clearance_uses_all_authored_boxes(self):
        # This node lies on a smaller pad, farther from the central pad. Taking
        # only the central-pad distance would miss the actual penetration.
        self.distance_patch.stop()
        p = np.array([[.007, .0001, .053]])
        finger = SimpleNamespace(pos=np.zeros(3), rotation=np.eye(3))
        boxes = robot_grasp._pad_box_distances(p, finger)
        self.assertGreater(boxes[0, 0], .00075)
        self.assertAlmostEqual(robot_grasp._pad_distance(p, finger)[0], 0.)


if __name__ == "__main__":
    unittest.main()
