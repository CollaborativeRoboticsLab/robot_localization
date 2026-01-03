#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <robot_localization/fault/ros_filter_types.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.arguments({"ukf_ft_node"});

  auto filter = std::make_shared<robot_localization::RosUkfFT>(options);

  // UKF-specific unscented transform parameters
  double alpha = filter->declare_parameter("alpha", 0.001);
  double kappa = filter->declare_parameter("kappa", 0.0);
  double beta = filter->declare_parameter("beta", 2.0);
  filter->getFilter().setConstants(alpha, kappa, beta);

  filter->initialize();

  rclcpp::spin(filter->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
