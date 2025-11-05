# Adaptive NDT Controller

## Overview

The **Adaptive NDT Controller** is a research package that implements real-time parameter adaptation for the NDT (Normal Distributions Transform) scan matcher based on EKF (Extended Kalman Filter) localization uncertainty estimates.

### Purpose

This package dynamically adjusts NDT scan matching parameters in response to localization uncertainty, aiming to:
- Improve localization accuracy during high-uncertainty situations
- Reduce computational overhead during low-uncertainty situations
- Enable robust localization in challenging environments (GPS-denied areas, dynamic obstacles, etc.)

### Research Application

This package is designed for research on **EKF uncertainty-based adaptive NDT scan matching** for autonomous vehicle localization.

---

## Architecture

```
┌─────────────────────────┐
│   EKF Localizer         │
│   (ekf_localizer)       │
└───────────┬─────────────┘
            │ PoseWithCovarianceStamped
            │ (with covariance matrix)
            ▼
┌─────────────────────────┐
│ Pose Uncertainty        │
│ Monitor                 │
└───────────┬─────────────┘
            │ PoseUncertaintyVector
            │ (std_x, std_y, std_z, std_yaw)
            ▼
┌─────────────────────────┐
│ Adaptive NDT Controller │◄─── This Package
│ (This Node)             │
└───────────┬─────────────┘
            │ Parameter Updates
            │ (step_size, max_iterations)
            ▼
┌─────────────────────────┐
│   NDT Scan Matcher      │
│   (ndt_scan_matcher)    │
└─────────────────────────┘
```

---

## Features

### 1. Real-time Uncertainty Monitoring
- Subscribes to uncertainty information from `pose_uncertainty_monitor`
- Processes standard deviations for x, y, z, and yaw

### 2. Adaptive Parameter Control
The controller adapts the following NDT parameters:

#### a) **Step Size** (Newton line search step length)
- **Formula**: `step_size = base_step_size + gain_step_size * position_uncertainty`
- **Logic**: Higher position uncertainty → Larger step size → Wider search range
- **Range**: [0.05, 1.0]

#### b) **Max Iterations** (Number of optimization iterations)
- **Formula**: `max_iterations = base_max_iterations + gain_max_iterations * std_yaw`
- **Logic**: Higher yaw uncertainty → More iterations → More thorough rotation search
- **Range**: [10, 100]

#### c) **Resolution** (Voxel grid resolution) - *Optional*
- **Formula**: `resolution = base_resolution - gain_resolution * position_uncertainty`
- **Logic**: Higher position uncertainty → Finer resolution → More precise matching
- **Range**: [0.5, 5.0]
- **Note**: Disabled by default due to computational overhead

### 3. Rate Limiting
- Configurable minimum update interval to prevent excessive parameter changes
- Default: 0.5 seconds between updates

### 4. Robustness Features
- Parameter clamping to prevent extreme values
- Service availability checking
- Timeout handling for parameter updates
- Detailed logging for debugging and analysis

---

## Installation

### Prerequisites
- ROS 2 (tested on Humble/Galactic)
- Autoware.universe
- `autoware_localization_msgs` package (with custom message types)

### Build Instructions

```bash
cd /path/to/autoware
colcon build --packages-select adaptive_ndt_controller
```

---

## Usage

### Launch the Node

```bash
ros2 launch adaptive_ndt_controller adaptive_ndt_controller.launch.xml
```

### Verify Operation

Check that the node is running:
```bash
ros2 node list | grep adaptive_ndt_controller
```

Monitor parameter updates:
```bash
ros2 topic echo /rosout | grep "Adaptive"
```

---

## Configuration

### Parameters

All parameters are defined in `config/adaptive_ndt_controller.param.yaml`.

#### Target Node
- `ndt_node_name` (string): Name of the NDT node to control
  - Default: `"ndt_scan_matcher"`

#### Input Topics
- `input_uncertainty_vector_topic` (string): Uncertainty vector topic
  - Default: `"/localization/diagnostics/uncertainty_vector"`
- `input_uncertainty_score_topic` (string): Uncertainty score topic (for logging)
  - Default: `"/localization/diagnostics/uncertainty_score"`

#### Control Gains
- `gain_step_size` (double): Gain for step size adaptation
  - Default: `0.5`
- `gain_max_iterations` (double): Gain for max iterations adaptation
  - Default: `10.0`
- `gain_resolution` (double): Gain for resolution adaptation
  - Default: `0.2`

#### Base Parameters
- `base_step_size` (double): Baseline step size
  - Default: `0.1`
- `base_max_iterations` (int): Baseline max iterations
  - Default: `30`
- `base_resolution` (double): Baseline voxel resolution
  - Default: `2.0`

#### Thresholds
- `uncertainty_threshold_high` (double): High uncertainty threshold [m or rad]
  - Default: `0.5`
- `uncertainty_threshold_low` (double): Low uncertainty threshold [m or rad]
  - Default: `0.1`

#### Enable Flags
- `enable_step_size_control` (bool): Enable adaptive step size
  - Default: `true`
- `enable_max_iterations_control` (bool): Enable adaptive max iterations
  - Default: `true`
- `enable_resolution_control` (bool): Enable adaptive resolution
  - Default: `false`

#### Rate Limiting
- `min_param_update_interval_sec` (double): Minimum update interval [sec]
  - Default: `0.5`

---

## Research Methodology

### Experimental Setup

1. **Baseline Experiment**: Run NDT with fixed parameters
2. **Adaptive Experiment**: Enable adaptive controller
3. **Comparison Metrics**:
   - Localization accuracy (RMSE)
   - Convergence rate
   - Computational time
   - Robustness in challenging scenarios

### Tuning Guidelines

1. **Gain Tuning**:
   - Start with default gains
   - Increase `gain_step_size` if localization fails in high-uncertainty regions
   - Increase `gain_max_iterations` if rotation alignment is poor

2. **Threshold Tuning**:
   - Set `uncertainty_threshold_high` based on your sensor characteristics
   - Analyze uncertainty distributions from logged data

3. **Rate Limiting**:
   - Adjust `min_param_update_interval_sec` based on your computational resources
   - Shorter intervals = more responsive, but higher overhead

---

## Troubleshooting

### Node Cannot Find NDT Parameter Service

**Symptom**: Warning message "NDT parameter service not available"

**Solution**:
- Ensure `ndt_scan_matcher` node is running
- Check `ndt_node_name` parameter matches actual node name
- Verify NDT node allows dynamic parameter updates

### Parameters Not Updating

**Symptom**: Logged uncertainty values but no parameter changes

**Solution**:
- Check `enable_*_control` flags are set to `true`
- Verify `min_param_update_interval_sec` is not too large
- Ensure uncertainty values exceed `uncertainty_threshold_low`

### Excessive Parameter Changes

**Symptom**: Parameters changing too frequently

**Solution**:
- Increase `min_param_update_interval_sec`
- Adjust gains to reduce sensitivity
- Review uncertainty estimation from EKF

---

## Code Reference

### Main Implementation Files

- **Node Header**: `include/adaptive_ndt_controller/adaptive_ndt_controller_node.hpp`
- **Node Implementation**: `src/adaptive_ndt_controller_node.cpp`
  - Uncertainty callback: Line 100-118
  - Adaptive control logic: Line 130-191
  - Parameter update: Line 193-224

### Key Functions

- `on_uncertainty_vector()`: Processes incoming uncertainty data
- `update_ndt_parameters()`: Implements adaptive control algorithm
- `set_ndt_parameters()`: Sends parameters to NDT node
- `compute_position_uncertainty_norm()`: Calculates 2D position uncertainty

---

## Publications

If you use this package in your research, please cite:

```bibtex
@article{adaptive_ndt_2025,
  title={EKF Uncertainty-Based Adaptive NDT Scan Matching for Robust Localization},
  author={Your Name},
  journal={Your Journal/Conference},
  year={2025}
}
```

---

## License

Apache License 2.0

---

## Contact

For research inquiries or collaboration:
- Email: research@example.com
- GitHub Issues: [Report issues here]

---

## Acknowledgments

This package is developed as part of research on adaptive localization for autonomous vehicles using Autoware.
