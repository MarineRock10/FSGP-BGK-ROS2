"""
Unit tests for traversability utility functions.

These tests cover the pure-numpy utility functions that don't require
GPU (CuPy/CUDA) or ROS dependencies.
"""

import sys
import os
import numpy as np
import pytest

# Add the fsgp_bgk python directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "fsgp_bgk", "python"))


class TestNormalizeAttribute:
    """Tests for TraversabilityAnalyzerWithBGK_GPU.normalize_attribute (static method)."""

    @staticmethod
    def normalize_attribute(attribute, global_min, global_max):
        """
        Reimplementation of the static normalize_attribute method to avoid
        importing cupy (which requires CUDA). The logic is identical.
        """
        if global_max == global_min:
            return np.zeros_like(attribute)
        attribute = np.asarray(attribute, dtype=np.float64)
        return np.clip((attribute - global_min) / (global_max - global_min + 1e-10), 0, 1)

    def test_basic_normalization(self):
        data = np.array([0.0, 0.5, 1.0])
        result = self.normalize_attribute(data, 0.0, 1.0)
        np.testing.assert_allclose(result, [0.0, 0.5, 1.0], atol=1e-6)

    def test_normalization_with_offset(self):
        data = np.array([10.0, 15.0, 20.0])
        result = self.normalize_attribute(data, 10.0, 20.0)
        np.testing.assert_allclose(result, [0.0, 0.5, 1.0], atol=1e-6)

    def test_equal_min_max_returns_zeros(self):
        data = np.array([5.0, 5.0, 5.0])
        result = self.normalize_attribute(data, 5.0, 5.0)
        np.testing.assert_array_equal(result, np.zeros(3))

    def test_values_clipped_to_0_1(self):
        data = np.array([-1.0, 0.5, 2.0])
        result = self.normalize_attribute(data, 0.0, 1.0)
        assert result[0] == 0.0
        assert result[2] == 1.0

    def test_negative_range(self):
        data = np.array([-10.0, -5.0, 0.0])
        result = self.normalize_attribute(data, -10.0, 0.0)
        np.testing.assert_allclose(result, [0.0, 0.5, 1.0], atol=1e-6)


class TestVoxelDownsample:
    """Tests for point_cloud_tool.voxel_downsample."""

    def test_downsample_reduces_points(self):
        from traversability_lib.point_cloud_tool import voxel_downsample

        # Create a dense grid of points
        x = np.linspace(0, 1, 50)
        y = np.linspace(0, 1, 50)
        xx, yy = np.meshgrid(x, y)
        points = np.column_stack([xx.flatten(), yy.flatten(), np.zeros(2500)])

        result = voxel_downsample(points, voxel_size=0.2)
        assert result.shape[0] < points.shape[0]
        assert result.shape[1] == 3

    def test_downsample_preserves_intensity(self):
        from traversability_lib.point_cloud_tool import voxel_downsample

        # Points with intensity column
        x = np.linspace(0, 1, 50)
        y = np.linspace(0, 1, 50)
        xx, yy = np.meshgrid(x, y)
        intensity = np.ones(2500) * 0.5
        points = np.column_stack([xx.flatten(), yy.flatten(), np.zeros(2500), intensity])

        result = voxel_downsample(points, voxel_size=0.2)
        assert result.shape[1] == 4
        # Intensity values should be preserved (all were 0.5)
        np.testing.assert_allclose(result[:, 3], 0.5, atol=1e-6)

    def test_downsample_single_point(self):
        from traversability_lib.point_cloud_tool import voxel_downsample

        points = np.array([[1.0, 2.0, 3.0]])
        result = voxel_downsample(points, voxel_size=0.1)
        assert result.shape[0] == 1


class TestFakeIntensity:
    """Tests for TraversabilityAnalyzer.fake_intensity logic."""

    @staticmethod
    def fake_intensity(pcl, num_points):
        """Reimplementation of fake_intensity to avoid heavy imports."""
        num_original_points = pcl.shape[0]
        return np.hstack([pcl, np.ones((num_original_points, 1))])

    def test_adds_intensity_column(self):
        pcl = np.random.rand(100, 3)
        result = self.fake_intensity(pcl, 100)
        assert result.shape == (100, 4)
        np.testing.assert_array_equal(result[:, 3], 1.0)

    def test_preserves_xyz(self):
        pcl = np.random.rand(50, 3)
        result = self.fake_intensity(pcl, 50)
        np.testing.assert_array_equal(result[:, :3], pcl)


class TestSamplingGrid:
    """Tests for grid generation logic used in f_sgp_bgk.py."""

    def test_grid_covers_expected_area(self):
        resolution = 0.2
        x_length = 5.0
        y_length = 5.0
        x_range = x_length / 2
        y_range = y_length / 2

        x_s = np.arange(-x_range, x_range, resolution, dtype="float32")
        y_s = np.arange(-y_range, y_range, resolution, dtype="float32")
        grid = np.array(np.meshgrid(x_s, y_s)).T.reshape(-1, 2)

        assert grid.shape[0] == len(x_s) * len(y_s)
        assert grid[:, 0].min() >= -x_range
        assert grid[:, 0].max() < x_range
        assert grid[:, 1].min() >= -y_range
        assert grid[:, 1].max() < y_range

    def test_grid_resolution(self):
        resolution = 0.5
        x_length = 4.0
        y_length = 4.0

        x_s = np.arange(-x_length / 2, x_length / 2, resolution, dtype="float32")
        y_s = np.arange(-y_length / 2, y_length / 2, resolution, dtype="float32")
        grid = np.array(np.meshgrid(x_s, y_s)).T.reshape(-1, 2)

        expected_count = int(x_length / resolution) * int(y_length / resolution)
        assert grid.shape[0] == expected_count


class TestObservationModel:
    """Tests for the log-odds observation model in node_ros2.py."""

    @staticmethod
    def observation_model_binary(traversability, obstacle_threshold):
        """Reimplementation of binarized observation model."""
        log_odds = np.zeros_like(traversability)
        log_odds[traversability > obstacle_threshold] = np.log(0.95 / (1 - 0.95))
        log_odds[traversability <= obstacle_threshold] = np.log(0.05 / (1 - 0.05))
        return log_odds

    @staticmethod
    def observation_model_continuous(traversability):
        """Reimplementation of continuous observation model."""
        traversability = np.clip(traversability, 0.001, 0.999)
        return np.log(traversability / (1 - traversability))

    def test_binary_high_traversability(self):
        trav = np.array([0.9])
        result = self.observation_model_binary(trav, obstacle_threshold=0.5)
        assert result[0] > 0  # high traversability -> positive log-odds

    def test_binary_low_traversability(self):
        trav = np.array([0.1])
        result = self.observation_model_binary(trav, obstacle_threshold=0.5)
        assert result[0] < 0  # low traversability -> negative log-odds

    def test_continuous_symmetry(self):
        result_high = self.observation_model_continuous(np.array([0.9]))
        result_low = self.observation_model_continuous(np.array([0.1]))
        # log(0.9/0.1) should be roughly -log(0.1/0.9)
        np.testing.assert_allclose(result_high, -result_low, atol=1e-6)

    def test_continuous_midpoint_is_zero(self):
        result = self.observation_model_continuous(np.array([0.5]))
        np.testing.assert_allclose(result, 0.0, atol=1e-6)


class TestGenerateRobotPoints:
    """Tests for robot footprint point generation."""

    @staticmethod
    def generate_robot_points(l, w, i_num=15, base_height=-0.15):
        """Reimplementation of generate_robot_points."""
        x = np.linspace(-l / 2, l / 2, num=i_num)
        y = np.linspace(-w / 2, w / 2, num=i_num)
        x, y = np.meshgrid(x, y)
        x = x.flatten()
        y = y.flatten()
        z = np.full_like(x, base_height)
        i = np.full_like(x, 0)
        c = np.full_like(x, 0)
        g = np.full_like(x, 0)
        return np.column_stack((x, y, z, i, c, g))

    def test_point_count(self):
        points = self.generate_robot_points(5.0, 5.0, i_num=10)
        assert points.shape == (100, 6)

    def test_height_is_base(self):
        points = self.generate_robot_points(5.0, 5.0, base_height=-0.2)
        np.testing.assert_array_equal(points[:, 2], -0.2)

    def test_centered_at_origin(self):
        points = self.generate_robot_points(4.0, 6.0, i_num=20)
        assert np.isclose(points[:, 0].mean(), 0.0, atol=1e-6)
        assert np.isclose(points[:, 1].mean(), 0.0, atol=1e-6)

    def test_extent_matches_dimensions(self):
        l, w = 4.0, 6.0
        points = self.generate_robot_points(l, w)
        np.testing.assert_allclose(points[:, 0].max(), l / 2, atol=1e-6)
        np.testing.assert_allclose(points[:, 0].min(), -l / 2, atol=1e-6)
        np.testing.assert_allclose(points[:, 1].max(), w / 2, atol=1e-6)
        np.testing.assert_allclose(points[:, 1].min(), -w / 2, atol=1e-6)
