#pragma once

#include <Eigen/Dense>
#include <robot_localization/ekf.hpp>
#include <robot_localization/filter_common.hpp>

namespace robot_localization
{

/**
 * @brief Extended EKF that exposes full-state innovation and innovation covariance.
 *
 * This class derives from the upstream robot_localization::Ekf and overrides
 * the correct() step to record the measurement innovation vector r and the
 * corresponding innovation covariance S = H P H' + R, stored as full
 * STATE_SIZE-sized structures. Entries corresponding to state variables that
 * were not updated by the last measurement are zero.
 */
class EkfFT : public Ekf
{
public:
  EkfFT()
    : Ekf(), innovation_(STATE_SIZE), innovation_covariance_(STATE_SIZE, STATE_SIZE), last_fault_(false),
      last_origin_(), last_mahalanobis_distance_(0.0), last_mahalanobis_threshold_(0.0)
  {
    innovation_.setZero();
    innovation_covariance_.setZero();
  }

  ~EkfFT()
  {
  }

  const Eigen::VectorXd& getInnovation() const
  {
    return innovation_;
  }

  const Eigen::MatrixXd& getInnovationCovariance() const
  {
    return innovation_covariance_;
  }

  bool getLastFault() const
  {
    return last_fault_;
  }

  const std::string& getLastOrigin() const
  {
    return last_origin_;
  }

  double getLastMahalanobisDistance() const
  {
    return last_mahalanobis_distance_;
  }

  double getLastMahalanobisThreshold() const
  {
    return last_mahalanobis_threshold_;
  }

  void correct(const Measurement& measurement) override
  {
    FB_DEBUG("---------------------- EkfFT::correct ----------------------\n"
             << "State is:\n"
             << state_
             << "\n"
                "Topic is:\n"
             << measurement.topic_name_
             << "\n"
                "Measurement is:\n"
             << measurement.measurement_
             << "\n"
                "Measurement topic name is:\n"
             << measurement.topic_name_
             << "\n\n"
                "Measurement covariance is:\n"
             << measurement.covariance_ << "\n");

    // We don't want to update everything, so we need to build matrices that only
    // update the measured parts of our state vector. Throughout prediction and
    // correction, we attempt to maximize efficiency in Eigen.

    // First, determine how many state vector values we're updating
    std::vector<size_t> update_indices;
    update_indices.reserve(measurement.update_vector_.size());

    for (size_t i = 0; i < measurement.update_vector_.size(); ++i)
    {
      if (measurement.update_vector_[i])
      {
        // Handle nan and inf values in measurements
        if (std::isnan(measurement.measurement_(i)))
        {
          FB_DEBUG("Value at index " << i << " was nan. Excluding from update.\n");
        }
        else if (std::isinf(measurement.measurement_(i)))
        {
          FB_DEBUG("Value at index " << i << " was inf. Excluding from update.\n");
        }
        else
        {
          update_indices.push_back(i);
        }
      }
    }

    FB_DEBUG("Update indices are:\n" << update_indices << "\n");

    const size_t update_size = update_indices.size();
    if (update_size == 0) return;

    // Now set up the relevant matrices
    Eigen::VectorXd state_subset(update_size);                                // x (in most literature)
    Eigen::VectorXd measurement_subset(update_size);                          // z
    Eigen::MatrixXd measurement_covariance_subset(update_size, update_size);  // R
    Eigen::MatrixXd state_to_measurement_subset(update_size, state_.rows());  // H
    Eigen::MatrixXd kalman_gain_subset(state_.rows(), update_size);           // K
    Eigen::VectorXd innovation_subset(update_size);                           // z - Hx

    state_subset.setZero();
    measurement_subset.setZero();
    measurement_covariance_subset.setZero();
    state_to_measurement_subset.setZero();
    kalman_gain_subset.setZero();
    innovation_subset.setZero();

    // Now build the sub-matrices from the full-sized matrices
    for (size_t i = 0; i < update_size; ++i)
    {
      measurement_subset(i) = measurement.measurement_(update_indices[i]);
      state_subset(i) = state_(update_indices[i]);

      for (size_t j = 0; j < update_size; ++j)
      {
        measurement_covariance_subset(i, j) = measurement.covariance_(update_indices[i], update_indices[j]);
      }

      // Handle negative (read: bad) covariances in the measurement. Rather
      // than exclude the measurement or make up a covariance, just take
      // the absolute value.
      if (measurement_covariance_subset(i, i) < 0)
      {
        FB_DEBUG("WARNING: Negative covariance for index " << i << " of measurement (value is"
                                                           << measurement_covariance_subset(i, i)
                                                           << "). Using absolute value...\n");

        measurement_covariance_subset(i, i) = ::fabs(measurement_covariance_subset(i, i));
      }

      // If the measurement variance for a given variable is very
      // near 0 (as in e-50 or so) and the variance for that
      // variable in the covariance matrix is also near zero, then
      // the Kalman gain computation will blow up. Really, no
      // measurement can be completely without error, so add a small
      // amount in that case.
      if (measurement_covariance_subset(i, i) < 1e-9)
      {
        FB_DEBUG("WARNING: measurement had very small error covariance for index "
                 << update_indices[i] << ". Adding some noise to maintain filter stability.\n");

        measurement_covariance_subset(i, i) = 1e-9;
      }
    }

    // The state-to-measurement function, h, will now be a measurement_size x
    // full_state_size matrix, with ones in the (i, i) locations of the values to
    // be updated
    for (size_t i = 0; i < update_size; ++i)
    {
      state_to_measurement_subset(i, update_indices[i]) = 1;
    }

    FB_DEBUG("Current state subset is:\n"
             << state_subset << "\nMeasurement subset is:\n"
             << measurement_subset << "\nMeasurement covariance subset is:\n"
             << measurement_covariance_subset << "\nState-to-measurement subset is:\n"
             << state_to_measurement_subset << "\n");

    // (1) Compute the Kalman gain: K = (PH') / (HPH' + R)
    Eigen::MatrixXd pht = estimate_error_covariance_ * state_to_measurement_subset.transpose();
    Eigen::MatrixXd innovation_covariance_subset = state_to_measurement_subset * pht + measurement_covariance_subset;
    Eigen::MatrixXd hphr_inverse = innovation_covariance_subset.inverse();
    kalman_gain_subset.noalias() = pht * hphr_inverse;

    innovation_subset = (measurement_subset - state_subset);

    // Wrap angles in the innovation
    for (size_t i = 0; i < update_size; ++i)
    {
      if (update_indices[i] == StateMemberRoll || update_indices[i] == StateMemberPitch ||
          update_indices[i] == StateMemberYaw)
      {
        innovation_subset(i) = ::angles::normalize_angle(innovation_subset(i));
      }
    }

    // (2) Compute Mahalanobis distance for this measurement and check gate.
    const double squared_mahalanobis = innovation_subset.dot(hphr_inverse * innovation_subset);
    last_mahalanobis_distance_ = std::sqrt(std::max(0.0, squared_mahalanobis));
    last_mahalanobis_threshold_ = measurement.mahalanobis_thresh_;
    last_origin_ = measurement.topic_name_;

    const bool passed_mahalanobis =
      checkMahalanobisThreshold(innovation_subset, hphr_inverse, measurement.mahalanobis_thresh_);
    last_fault_ = !passed_mahalanobis;

    if (passed_mahalanobis)
    {
      // Store full-state innovation and innovation covariance (S)
      innovation_.setZero();
      innovation_covariance_.setZero();

      for (size_t i = 0; i < update_size; ++i)
      {
        const size_t row_index = update_indices[i];
        innovation_(static_cast<Eigen::Index>(row_index)) = innovation_subset(i);

        for (size_t j = 0; j < update_size; ++j)
        {
          const size_t col_index = update_indices[j];
          innovation_covariance_(static_cast<Eigen::Index>(row_index), static_cast<Eigen::Index>(col_index)) =
              innovation_covariance_subset(i, j);
        }
      }

      // (3) Apply the gain to the difference between the state and measurement: x
      // = x + K(z - Hx)
      state_.noalias() += kalman_gain_subset * innovation_subset;

      // (4) Update the estimate error covariance using the Joseph form: (I -
      // KH)P(I - KH)' + KRK'
      Eigen::MatrixXd gain_residual = identity_;
      gain_residual.noalias() -= kalman_gain_subset * state_to_measurement_subset;
      estimate_error_covariance_ = gain_residual * estimate_error_covariance_ * gain_residual.transpose();
      estimate_error_covariance_.noalias() +=
          kalman_gain_subset * measurement_covariance_subset * kalman_gain_subset.transpose();

      // Handle wrapping of angles
      wrapStateAngles();

      FB_DEBUG("Kalman gain subset is:\n"
               << kalman_gain_subset << "\nInnovation is:\n"
               << innovation_subset << "\nCorrected full state is:\n"
               << state_ << "\nCorrected full estimate error covariance is:\n"
               << estimate_error_covariance_ << "\n\n---------------------- /EkfFT::correct ----------------------\n");
    }
  }

protected:
  Eigen::VectorXd innovation_;
  Eigen::MatrixXd innovation_covariance_;
  bool last_fault_;
  std::string last_origin_;
  double last_mahalanobis_distance_;
  double last_mahalanobis_threshold_;
};

}  // namespace robot_localization
