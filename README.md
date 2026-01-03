# robot_localization

robot_localization is a package of nonlinear state estimation nodes. The package was developed by Charles River Analytics, Inc.

Please see documentation here: http://wiki.ros.org/robot_localization

# Extended robot_localization

This meta-repository contains extensions around the upstream `robot_localization` package to expose
internal filter state for supervision and fault detection.

Currently it provides two ROS 2 packages:

- [robot_localization_msg](./docs/robot_localization_msg.md)
- [robot_localization_ft](./docs/robot_localization_ft.md)

## robot_localization_msg

This is a pure message package that defines the shared interfaces used by the fault-tolerant
filters and monitoring components.

## robot_localization_ft

This package provides fault-tolerant wrapper nodes around the upstream `robot_localization` EKF and
UKF filters. It **does not** modify the upstream library; instead it derives from the existing
filter and node types.

## Nodes

- `ekf_ft_node` and `ukf_ft_node` mirror the upstream EKF/UKF nodes but instantiate `RosEkfFT` and `RosUkfFT` instead. The UKF node also exposes the usual `alpha`, `kappa`, and `beta` parameters and forwards them into `UkfFT::setConstants`.

- These nodes can be dropped into existing launch files in place of the standard `ekf_node` /
`ukf_node` to obtain innovation and fault information with minimal changes to the rest of the
system.
