"""Analytic checks: these catch scale, frame, occlusion, and invalid-count mistakes."""
import unittest

import numpy as np

from evaluation_geometry import cast_boxes, pose_delta, quaternion_matrix, reference_raster, region_metrics, rpy_matrix


class GeometryTest(unittest.TestCase):
    def test_optical_axes_and_depth_are_not_euclidean_range(self):
        rotation = rpy_matrix([-np.pi/2, 0, -np.pi/2])
        np.testing.assert_allclose(rotation @ [0, 0, 1], [1, 0, 0], atol=1e-12)
        np.testing.assert_allclose(rotation @ [1, 0, 0], [0, -1, 0], atol=1e-12)
        wall = dict(position=np.array([2.3, 0, 0]), rotation=np.eye(3), size=np.array([.1, 10, 10]))
        directions = np.array([[0, 0, 1], [.5, .3, 1]]) @ rotation.T
        depth, _ = cast_boxes(np.array([.25, 0, 0]), directions, [wall])
        np.testing.assert_allclose(depth, [2, 2])

    def test_ray_misses_parallel_slab_and_hits_nearest_surface(self):
        boxes = [dict(position=np.array([0, 0, z]), rotation=np.eye(3), size=np.array([1, 1, .2]))
                 for z in (4.1, 2.1)]
        depth, labels = cast_boxes(np.zeros(3), np.array([[0, 0, 1], [1, 0, 1], [0, 0, -1]]), boxes)
        np.testing.assert_allclose(depth, [2, np.inf, np.inf])
        np.testing.assert_array_equal(labels, [1, -1, -1])

    def test_body_rotation_changes_plane_intersection(self):
        # A 30 degree camera yaw: expected optical Z is plane distance / cos(yaw).
        r = rpy_matrix([0, np.pi/6, 0])
        box = dict(position=np.array([0, 0, 2.05]), rotation=np.eye(3), size=np.array([20, 20, .1]))
        depth, _ = cast_boxes(np.zeros(3), np.array([[0, 0, 1]]) @ r.T, [box])
        np.testing.assert_allclose(depth, [2/np.cos(np.pi/6)])

    def test_quaternion_and_pose_motion(self):
        r = quaternion_matrix([-.5, .5, -.5, .5])
        np.testing.assert_allclose(r, rpy_matrix([-np.pi/2, 0, -np.pi/2]), atol=1e-12)
        p, a = pose_delta((np.zeros(3), np.eye(3)), (np.array([.1, 0, 0]), rpy_matrix([0, 0, .2])))
        self.assertAlmostEqual(p, .1)
        self.assertAlmostEqual(a, .2)
        with self.assertRaises(ValueError):
            quaternion_matrix([0, 0, 0, 0])

    def test_invalids_count_toward_coverage_but_not_error(self):
        observed = np.array([[[2.1, np.nan]], [[1.9, np.nan]]])
        result = region_metrics(observed, np.array([[2., 2.]]), np.ones((1, 2), dtype=bool))
        self.assertEqual(result['valid_fraction'], .5)
        self.assertAlmostEqual(result['p95_absolute_error_m'], .1)

    def test_right_occlusion_mask_uses_other_view_and_excludes_out_of_fov(self):
        calibration = dict(image_size=[80, 40], fx=100., fy=100., cx=40., cy=20.,
                           baseline_m=.12, left_pose_body=[0]*6,
                           right_pose_body=[.12, 0, 0, 0, 0, 0])
        specifications = [('textured_wall', [0, 0, 4.05], [20, 20, .1]),
                          ('occluding_edge', [0, 0, 2.05], [.1, 20, .1]),
                          ('ground', [0, -100, 0], [20, 1, 20]),
                          ('low_texture_patch', [100, 0, 3], [1, 1, 1])]
        boxes = [dict(name=n, position=np.array(p), size=np.array(s), rotation=np.eye(3))
                 for n, p, s in specifications]
        criteria = dict(low_texture_interior_erosion_px=1, textured_roi_xywh=[50, 5, 10, 10],
                        edge_band_radius_px=1)
        depth, _, masks, _, _ = reference_raster((np.zeros(3), np.eye(3)), calibration, boxes, criteria)
        self.assertAlmostEqual(depth[20, 36], 4.)
        self.assertTrue(masks['right_occluded'][20, 36])
        self.assertAlmostEqual(depth[20, 40], 2.)
        self.assertFalse(masks['right_occluded'][20, 40])
        self.assertFalse(masks['right_occluded'][20, 50])
        self.assertTrue(masks['outside_right_fov'][20, 0])
        self.assertFalse(masks['right_occluded'][20, 0])


if __name__ == '__main__':
    unittest.main()
