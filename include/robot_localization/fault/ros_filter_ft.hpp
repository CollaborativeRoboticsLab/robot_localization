#pragma once

#include <Eigen/Dense>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robot_localization/ros_filter.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "robot_localization/msg/internal_state.hpp"

namespace robot_localization
{

/**
 * @brief Extension of robot_localization::RosFilter that publishes innovation and innovation covariance
 * from a derived filter type T.
 */
template <class T>
class RosFilterFT : public RosFilter<T>
{
public:
  explicit RosFilterFT(const rclcpp::NodeOptions& options) : RosFilter<T>(options)
  {
  }

  ~RosFilterFT()
  {
  }

  /**
   * @brief Initialize base filter and set up innovation publishers.
   *
   * This should be called instead of RosFilter<T>::initialize() from
   * the corresponding *_ft_node main functions.
   */
  void initialize()
  {
    // Initialize the underlying RosFilter
    RosFilter<T>::initialize();

    // Create publishers for innovation vector and covariance
    rclcpp::PublisherOptions publisher_options;
    publisher_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

    internal_state_pub_ = this->template create_publisher<robot_localization::msg::InternalState>(
      "internal_state", rclcpp::QoS(10), publisher_options);

    // Timer for periodically publishing innovation data using the
    // same frequency parameter as the base filter.
    const std::chrono::duration<double> timespan{ 1.0 / this->frequency_ };
    innovation_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(timespan),
                                                std::bind(&RosFilterFT<T>::publishInnovation, this));
  }

private:
  void publishInnovation()
  {
    if (!this->filter_.getInitializedStatus())
      return;

    // Expect T to provide innovation getters
    const Eigen::VectorXd& innovation = this->filter_.getInnovation();
    const Eigen::MatrixXd& innovation_covar = this->filter_.getInnovationCovariance();

    if (internal_state_pub_)
    {
      robot_localization::msg::InternalState state_msg;
      state_msg.header.stamp = this->now();

      // Reuse the same layouts for innovation and covariance
      state_msg.innovation.layout = std_msgs::msg::MultiArrayLayout{};
      state_msg.innovation.layout.dim.resize(1);
      state_msg.innovation.layout.dim[0].label = "state";
      state_msg.innovation.layout.dim[0].size = static_cast<size_t>(innovation.size());
      state_msg.innovation.layout.dim[0].stride = static_cast<size_t>(innovation.size());
      state_msg.innovation.layout.data_offset = 0;
      state_msg.innovation.data.resize(static_cast<size_t>(innovation.size()));
      for (size_t i = 0; i < static_cast<size_t>(innovation.size()); ++i)
      {
        state_msg.innovation.data[i] = innovation(static_cast<Eigen::Index>(i));
      }

      const auto rows = static_cast<size_t>(innovation_covar.rows());
      const auto cols = static_cast<size_t>(innovation_covar.cols());

      state_msg.innovation_covariance.layout = std_msgs::msg::MultiArrayLayout{};
      state_msg.innovation_covariance.layout.dim.resize(2);
      state_msg.innovation_covariance.layout.dim[0].label = "rows";
      state_msg.innovation_covariance.layout.dim[0].size = rows;
      state_msg.innovation_covariance.layout.dim[0].stride = rows * cols;
      state_msg.innovation_covariance.layout.dim[1].label = "cols";
      state_msg.innovation_covariance.layout.dim[1].size = cols;
      state_msg.innovation_covariance.layout.dim[1].stride = cols;
      state_msg.innovation_covariance.layout.data_offset = 0;
      state_msg.innovation_covariance.data.resize(rows * cols);

      for (size_t r = 0; r < rows; ++r)
      {
        for (size_t c = 0; c < cols; ++c)
        {
          state_msg.innovation_covariance.data[r * cols + c] =
              innovation_covar(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c));
        }
      }

      // Fault metadata from the underlying FT filter
      state_msg.fault = this->filter_.getLastFault();
      state_msg.origin = this->filter_.getLastOrigin();
      state_msg.mahalanobis_distance = this->filter_.getLastMahalanobisDistance();
      state_msg.mahalanobis_threshold = this->filter_.getLastMahalanobisThreshold();

      internal_state_pub_->publish(state_msg);
    }
  }

  rclcpp::Publisher<robot_localization::msg::InternalState>::SharedPtr internal_state_pub_;
  rclcpp::TimerBase::SharedPtr innovation_timer_;
};

}  // namespace robot_localization
