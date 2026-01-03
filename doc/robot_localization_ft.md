# robot_localization_ft

This package provides fault-tolerant wrapper nodes around the upstream `robot_localization` EKF and
UKF filters. It **does not** modify the upstream library; instead it derives from the existing
filter and node types.

### Filter extensions

- `EkfFT` derive from `robot_localization::Ekf`
- `UkfFT` derive from `robot_localization::Ukf`

They override `correct(const Measurement &)` to:

- Compute the innovation vector and innovation covariance (`S`) for each processed measurement.
- Store these as full-state `Eigen::VectorXd` / `Eigen::MatrixXd` members.
- Compute the Mahalanobis distance of the innovation using the same `checkMahalanobisThreshold` logic as upstream.
- Track per-measurement metadata:
	- whether the Mahalanobis gate passed (`last_fault_` = !passed),
	- the origin/topic of the measurement,
	- the Mahalanobis distance and threshold used.

The actual filtering behavior (prediction, correction, gating) remains identical to upstream `robot_localization`; we only add bookkeeping and getters.

### ROS wrapper

- `RosFilterFT<T>` derives from `robot_localization::RosFilter<T>` and is instantiated as `RosEkfFT` or `RosUkfFT`.

In `initialize()` it calls the base initialize and then creates a publisher for 	`robot_localization_msg::msg::InternalState` on the `internal_state` topic. A timer periodically calls `publishInnovation()` which:

- Reads innovation and innovation covariance from the underlying `EkfFT` / `UkfFT` instance.
- Fills the `InternalState` message with:
	- the current innovation vector and covariance matrix,
	- the last Mahalanobis fault flag,
	- the last measurement origin/topic,
	- the Mahalanobis distance and threshold.
