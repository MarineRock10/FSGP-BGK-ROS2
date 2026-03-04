# FSGP-BGK Architecture

This document describes the system architecture of FSGP-BGK, a real-time spatial-temporal traversability assessment system built on Sparse Gaussian Processes with Bayesian Generalized Kernel inference.

---

## High-Level Overview

The system takes 3D LiDAR point clouds and robot odometry as input, and produces a local traversability map as output. The pipeline has three main stages:

1. **Feature Extraction** -- select informative inducing points from raw point clouds
2. **Sparse GP Regression** -- predict terrain elevation and uncertainty over a 2D grid
3. **Traversability Estimation** -- fuse slope, flatness, step height, and uncertainty into a traversability score using BGK temporal fusion

```
LiDAR PointCloud2 + Odometry
        |
        v
  [Voxel Downsample]
        |
        v
  [Feature Extraction (curvature + gradient classification)]
        |
        v
  [Sparse Gaussian Process (inducing point kernel)]
        |
        v
  [Grid Prediction: mean, variance, gradient]
        |
        v
  [Traversability Metrics: slope, flatness, step height, uncertainty]
        |
        v
  [BGK Spatial-Temporal Fusion]
        |
        v
  [Global Map Update (log-odds grid)]
        |
        v
  PointCloud2 (traversability_pcl, traversability_pcl_ldd)
```

---

## Package Structure

| Package | Language | Description |
|---------|----------|-------------|
| `fsgp_bgk` | Python | Core algorithm -- feature extraction, SGP model, traversability analysis |
| `simulation_env_ros1` | C++ | ROS1 Noetic simulation environment with simulated LiDAR |
| `simulation_env_ros2` | C++ | ROS2 Humble simulation environment with simulated LiDAR |

### fsgp_bgk (Python)

```
src/fsgp_bgk/
  config/
    params.yaml              # All algorithm parameters
  python/
    f_sgp_bgk.py             # TraversabilityAnalyzer -- main algorithm orchestrator
    node_ros1.py              # ROS1 node wrapper (FSGP_BGK_Node)
    node_ros2.py              # ROS2 node wrapper (FSGP_BGK_Node)
    traversability_lib/
      choose_point.py         # GPU-accelerated feature point extraction
      point_cloud_tool.py     # PointCloud2 parsing and voxel downsampling
      sgp_model.py            # Sparse GP model (gpytorch)
      traversability.py       # Traversability metrics + BGK fusion (cupy GPU)
```

### simulation_env_ros2 (C++)

```
src/simulation_env_ros2/
  src/
    simulation_node.cpp       # Robot simulator (kinematics, terrain following, odometry)
    simulated_lidar.cpp       # Simulated 3D LiDAR sensor
  config/params.yaml          # Simulation parameters (PCD path, robot config)
  launch/simulation.launch.py # Launch file for sim + LiDAR + RViz
  PCD/                        # Pre-built terrain point clouds (.pcd)
  meshes/robot.dae            # Robot 3D model for visualization
```

---

## Core Algorithm Details

### 1. Feature Extraction (`choose_point.py`)

The system selects informative inducing points rather than using the full point cloud:

- **GPU KNN search** (`torch.cdist`) finds k=20 nearest neighbors for each point
- **Curvature** is computed from PCA eigenvalues of local neighborhoods
- **Gradient** is the mean absolute height difference to neighbors
- Points exceeding curvature or gradient thresholds are classified as **feature points**
- Non-feature points are voxel-downsampled to reduce density
- Both sets are merged and randomly subsampled to `inducing_points` count (default: 500)

### 2. Sparse Gaussian Process (`sgp_model.py`)

Uses gpytorch's `InducingPointKernel` for efficient GP regression:

- **Kernel**: `ScaleKernel(RQKernel)` -- Rational Quadratic kernel with configurable lengthscale and alpha
- **Input features**: (x, y, curvature, gradient) per point, optionally PCA-reduced to 4 components
- **Output**: terrain elevation (z)
- **Training**: single-step AdamW optimization of exact marginal log-likelihood
- **Prediction**: mean elevation, variance, and gradient (via autograd) on a regular 2D grid

### 3. Traversability Metrics (`traversability.py`)

Four metrics are computed from the GP predictions:

| Metric | Source | Meaning |
|--------|--------|---------|
| **Slope** | norm of elevation gradient | Steepness of terrain |
| **Flatness** | curvature values on grid | Local surface roughness |
| **Step Height** | gradient values on grid | Sudden elevation changes |
| **Uncertainty** | GP variance (information gain) | Prediction confidence |

Each metric is Gaussian-filtered and min-max normalized to [0, 1].

### 4. BGK Temporal Fusion (`traversability.py`)

The Bayesian Generalized Kernel provides spatial-temporal consistency:

- Historical observations are stored per grid cell with timestamps
- A time-decaying exponential weight prioritizes recent measurements
- Spatial kernel (`exp(-d^2 / 2l^2)`) fuses nearby observations
- Fusion only activates when terrain variance exceeds `bgk_threshold`, avoiding unnecessary computation on flat ground
- Old observations are pruned based on `time_window` (default: 600s)

### 5. Global Map Update (`node_ros2.py`)

The ROS node maintains a global occupancy-style grid:

- Local traversability maps are transformed to global frame using odometry
- **Log-odds update** accumulates evidence over time (with optional binarization)
- Spatial smoothing via uniform filter
- High-resolution interpolation for published point cloud output
- Two outputs: raw traversability (`traversability_pcl`) and log-odds filtered (`traversability_pcl_ldd`)

---

## ROS2 Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/simulated_lidar_local` | PointCloud2 | Input | Local LiDAR scan in robot frame |
| `/odom` | Odometry | Input | Robot pose (position + orientation) |
| `/traversability_pcl` | PointCloud2 | Output | Raw traversability map |
| `/traversability_pcl_ldd` | PointCloud2 | Output | Log-odds filtered traversability map |
| `/cmd_vel` | Twist | Input (sim) | Keyboard velocity commands |
| `/local_cloud` | PointCloud2 | Internal | Extracted local point cloud from world |
| `/robot_model` | Marker | Output (sim) | Robot mesh visualization |

---

## Key Configuration Parameters

All parameters live in `src/fsgp_bgk/config/params.yaml`:

- `inducing_points` (500): Number of sparse GP inducing points. Directly controls computation cost.
- `lengthscale` (0.1): GP kernel lengthscale. Smaller = more local, larger = smoother predictions.
- `alpha` (10): RQ kernel shape parameter. Controls how quickly correlation decays.
- `resolution` (0.2m): Grid cell size for traversability map.
- `x_length` / `y_length` (5m): Local map dimensions.
- `w_slope` / `w_flatness` / `w_step_height` (0.3/0.3/0.2): Traversability metric weights.
- `bgk_threshold` (0.8): Variance threshold to activate BGK fusion.
- `time_window` (600s): How long historical observations are kept.

---

## Dependencies

| Library | Purpose |
|---------|---------|
| PyTorch | GPU tensor operations, autograd for gradient computation |
| gpytorch | Sparse Gaussian Process implementation |
| CuPy | GPU-accelerated array operations for BGK fusion |
| Open3D | Point cloud voxel downsampling |
| scikit-learn | KNN search, PCA |
| PCL (C++) | Point cloud I/O, KD-tree, voxel filtering in simulation |
| Eigen (C++) | Linear algebra for pose computation in simulation |
