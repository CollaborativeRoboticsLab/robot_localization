# robot_localization_msg

This is a pure message package that defines the shared interfaces used by the fault-tolerant
filters and monitoring components.

## Messages

- `InternalState` (`robot_localization_msg/msg/InternalState.msg`)

	This message exposes the internal innovation statistics of a `robot_localization` filter and the
	result of the Mahalanobis outlier gate for the most recent measurement:

	| Field                  | Type                          | Description                                                                                  |
	|------------------------|-------------------------------|----------------------------------------------------------------------------------------------|
	| `header`               | `std_msgs/Header`             | Standard ROS header (timestamp, frame) for the internal state snapshot.                     |
	| `innovation`           | `std_msgs/Float64MultiArray`  | Full-state innovation vector for the last measurement.                                      |
	| `innovation_covariance` | `std_msgs/Float64MultiArray` | Full-state innovation covariance matrix `S` for the last measurement.                       |
	| `fault`                | `bool`                        | `true` if the last processed measurement failed the Mahalanobis outlier gate.               |
	| `origin`               | `string`                      | Origin of the last processed measurement (typically the sensor topic name).                 |
	| `mahalanobis_distance` | `float64`                     | Mahalanobis distance (in N-sigmas) for the last measurement’s innovation.                   |
	| `mahalanobis_threshold`| `float64`                     | Configured Mahalanobis threshold (N-sigmas) that was applied to the last measurement.       |

The intention is that supervisors subscribe to this message to identify which sensor stream is causing outliers and by how much.