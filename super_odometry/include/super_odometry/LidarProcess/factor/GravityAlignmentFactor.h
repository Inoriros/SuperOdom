#pragma once

#include <algorithm>
#include <cmath>
#include <ceres/ceres.h>
#include <Eigen/Geometry>

// Constrains roll and pitch by comparing the world-up direction expressed in
// the sensor frame. Rotation around world Z (yaw) leaves this residual
// unchanged, so LiDAR registration remains free to estimate yaw.
class GravityAlignmentFactor final : public ceres::SizedCostFunction<3, 7> {
public:
  GravityAlignmentFactor(const Eigen::Quaterniond &q_w_sensor,
                         double weight)
      : measured_up_sensor_(
            q_w_sensor.normalized().conjugate() * Eigen::Vector3d::UnitZ()),
        sqrt_weight_(std::sqrt(std::max(0.0, weight))) {}

  bool Evaluate(double const *const *parameters,
                double *residuals,
                double **jacobians) const override {
    Eigen::Map<const Eigen::Quaterniond> q_raw(parameters[0] + 3);
    const Eigen::Quaterniond q_w_sensor = q_raw.normalized();
    const Eigen::Vector3d predicted_up_sensor =
        q_w_sensor.conjugate() * Eigen::Vector3d::UnitZ();

    Eigen::Map<Eigen::Vector3d> residual(residuals);
    residual = sqrt_weight_ *
        (predicted_up_sensor - measured_up_sensor_);

    if (jacobians != nullptr && jacobians[0] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian(
          jacobians[0]);
      jacobian.setZero();

      // PoseLocalParameterization applies a right perturbation q' = q*dq.
      // Its legacy ambient Jacobian maps local rotation directly through
      // columns 3..5 and drops column 6. Encode the true local derivative
      // here so this factor remains yaw-invariant under that parameterization.
      Eigen::Matrix3d skew_up;
      skew_up << 0.0, -predicted_up_sensor.z(), predicted_up_sensor.y(),
                 predicted_up_sensor.z(), 0.0, -predicted_up_sensor.x(),
                -predicted_up_sensor.y(), predicted_up_sensor.x(), 0.0;
      jacobian.block<3, 3>(0, 3) = sqrt_weight_ * skew_up;
    }
    return true;
  }

private:
  Eigen::Vector3d measured_up_sensor_;
  double sqrt_weight_;
};
