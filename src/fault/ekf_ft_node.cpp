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
  options.arguments({"ekf_ft_node"});

  auto filter = std::make_shared<robot_localization::RosEkfFT>(options);

  filter->initialize();

  rclcpp::spin(filter->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
