#pragma once

#include <angles/angles.h>

#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <cmath>
#include <robot_localization/filter_common.hpp>
#include <robot_localization/ukf.hpp>
#include <vector>

namespace robot_localization
{

/**
 * @brief Extended UKF that exposes full-state innovation and innovation covariance.
 *
 * This class derives from the upstream robot_localization::Ukf and overrides
 * the correct() step to record the measurement innovation vector r and the
 * corresponding innovation covariance S = H P H' + R, stored as full
 * STATE_SIZE-sized structures. Entries corresponding to state variables that
 * were not updated by the last measurement are zero.
 */
class UkfFT : public Ukf
{
public:
  UkfFT()
    : Ukf(), innovation_(STATE_SIZE), innovation_covariance_(STATE_SIZE, STATE_SIZE), last_fault_(false),
      last_origin_(), last_mahalanobis_distance_(0.0), last_mahalanobis_threshold_(0.0)
  {
    innovation_.setZero();
    innovation_covariance_.setZero();
  }

  ~UkfFT()
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
    FB_DEBUG("---------------------- UkfFT::correct ----------------------\n"
             << "State is:\n"
             << state_ << "\nMeasurement is:\n"
             << measurement.measurement_ << "\nMeasurement covariance is:\n"
             << measurement.covariance_ << "\n");

    // In our implementation, it may be that after we call predict once, we call
    // correct several times in succession (multiple measurements with different
    // time stamps). In that event, the sigma points need to be updated to reflect
    // the current state. Throughout prediction and correction, we attempt to
    // maximize efficiency in Eigen.
    if (!uncorrected_)
    {
      // Take the square root of a small fraction of the
      // estimate_error_covariance_ using LL' decomposition
      weighted_covar_sqrt_ = ((STATE_SIZE + lambda_) * estimate_error_covariance_).llt().matrixL();

      // Compute sigma points

      // First sigma point is the current state
      sigma_points_[0] = state_;

      // Next STATE_SIZE sigma points are state + weighted_covar_sqrt_[ith column]
      // STATE_SIZE sigma points after that are state - weighted_covar_sqrt_[ith
      // column]
      for (size_t sigma_ind = 0; sigma_ind < STATE_SIZE; ++sigma_ind)
      {
        sigma_points_[sigma_ind + 1] = state_ + weighted_covar_sqrt_.col(sigma_ind);
        sigma_points_[sigma_ind + 1 + STATE_SIZE] = state_ - weighted_covar_sqrt_.col(sigma_ind);
      }
    }

    // We don't want to update everything, so we need to build matrices that only
    // update the measured parts of our state vector

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
    Eigen::MatrixXd state_to_measurement_subset(update_size, STATE_SIZE);     // H
    Eigen::MatrixXd kalman_gain_subset(STATE_SIZE, update_size);              // K
    Eigen::VectorXd innovation_subset(update_size);                           // z - Hx
    Eigen::VectorXd predicted_measurement(update_size);
    Eigen::VectorXd sigma_diff(update_size);
    Eigen::MatrixXd predicted_meas_covar(update_size, update_size);
    Eigen::MatrixXd cross_covar(STATE_SIZE, update_size);

    std::vector<Eigen::VectorXd> sigma_point_measurements(sigma_points_.size(), Eigen::VectorXd(update_size));

    state_subset.setZero();
    measurement_subset.setZero();
    measurement_covariance_subset.setZero();
    state_to_measurement_subset.setZero();
    kalman_gain_subset.setZero();
    innovation_subset.setZero();
    predicted_measurement.setZero();
    predicted_meas_covar.setZero();
    cross_covar.setZero();

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
        measurement_covariance_subset(i, i) = 1e-9;

        FB_DEBUG("WARNING: measurement had very small error covariance for index "
                 << update_indices[i] << ". Adding some noise to maintain filter stability.\n");
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

    // (1) Generate sigma points, use them to generate a predicted measurement
    for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind)
    {
      sigma_point_measurements[sigma_ind] = state_to_measurement_subset * sigma_points_[sigma_ind];
      predicted_measurement.noalias() += state_weights_[sigma_ind] * sigma_point_measurements[sigma_ind];
    }

    // (2) Use the sigma point measurements and predicted measurement to compute a
    // predicted measurement covariance matrix P_zz and a state/measurement
    // cross-covariance matrix P_xz.
    for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind)
    {
      sigma_diff = sigma_point_measurements[sigma_ind] - predicted_measurement;
      predicted_meas_covar.noalias() += covar_weights_[sigma_ind] * (sigma_diff * sigma_diff.transpose());
      cross_covar.noalias() +=
          covar_weights_[sigma_ind] * ((sigma_points_[sigma_ind] - state_) * sigma_diff.transpose());
    }

    // (3) Compute the Kalman gain, making sure to use the actual measurement
    // covariance: K = P_xz * (P_zz + R)^-1
    Eigen::MatrixXd innovation_covariance_subset = predicted_meas_covar + measurement_covariance_subset;
    Eigen::MatrixXd inv_innov_cov = innovation_covariance_subset.inverse();
    kalman_gain_subset = cross_covar * inv_innov_cov;

    // (4) Apply the gain to the difference between the actual and predicted
    // measurements: x = x + K(z - z_hat)
    innovation_subset = (measurement_subset - predicted_measurement);

    // Wrap angles in the innovation
    for (size_t i = 0; i < update_size; ++i)
    {
      if (update_indices[i] == StateMemberRoll || update_indices[i] == StateMemberPitch ||
          update_indices[i] == StateMemberYaw)
      {
        innovation_subset(i) = ::angles::normalize_angle(innovation_subset(i));
      }
    }

    // (5) Compute Mahalanobis distance of innovation and check gate
    const double squared_mahalanobis = innovation_subset.dot(inv_innov_cov * innovation_subset);
    last_mahalanobis_distance_ = std::sqrt(std::max(0.0, squared_mahalanobis));
    last_mahalanobis_threshold_ = measurement.mahalanobis_thresh_;
    last_origin_ = measurement.topic_name_;

    const bool passed_mahalanobis =
      checkMahalanobisThreshold(innovation_subset, inv_innov_cov, measurement.mahalanobis_thresh_);
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

      state_.noalias() += kalman_gain_subset * innovation_subset;

      // (6) Compute the new estimate error covariance P = P - (K * P_zz * K')
      estimate_error_covariance_.noalias() -=
          (kalman_gain_subset * predicted_meas_covar * kalman_gain_subset.transpose());

      wrapStateAngles();

      // Mark that we need to re-compute sigma points for successive corrections
      uncorrected_ = false;

      FB_DEBUG("Predicated measurement covariance is:\n"
               << predicted_meas_covar << "\nCross covariance is:\n"
               << cross_covar << "\nKalman gain subset is:\n"
               << kalman_gain_subset << "\nInnovation:\n"
               << innovation_subset << "\nCorrected full state is:\n"
               << state_ << "\nCorrected full estimate error covariance is:\n"
               << estimate_error_covariance_ << "\n\n---------------------- /UkfFT::correct ----------------------\n");
    }
  }

private:
  Eigen::VectorXd innovation_;
  Eigen::MatrixXd innovation_covariance_;
  bool last_fault_;
  std::string last_origin_;
  double last_mahalanobis_distance_;
  double last_mahalanobis_threshold_;
};

}  // namespace robot_localization
