//
// Created by ubuntu on 2020/6/29.
//

#ifndef IMU_DATA_H
#define IMU_DATA_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include "super_odometry/container/MapRingBuffer.h"
#include <sensor_msgs/msg/imu.hpp>
#include "super_odometry/config/parameter.h"
#include "super_odometry/utils/Twist.h"

#define Gravity_Norm (9.81)
struct Imu {

public:
  typedef std::shared_ptr<Imu> Ptr;
  typedef std::shared_ptr<const Imu> ConstPtr;

public:
  Imu() : Imu(0.0, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)) {}
  Imu(double time, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
      : time(time), acc(acc), gyr(gyr) 
      {
        initialize();
      }
  ~Imu() {}
  
  void initialize()
  {
    acc_mean<<0.0,0.0,-1.0;
    gyr_mean<<0.0,0.0,0.0;
    acc_cov<<0.1,0.1,0.1;
    gyr_cov<<0.1,0.1,0.1;
    gravity<<0.0,0.0,0.0;
    gyr_bias<<0.0,0.0,0.0;
    acc_bias<<0.0,0.0,0.0;
    q_w_i.setIdentity();
    Roll_Pitch_Gravity_Matrix.setIdentity();
    imu_laser_R_Gravity.setIdentity();
    pitch_offset_gravity = 0.0;
    roll_offset_gravity = 0.0;
  }

  // Function to calculate pitch and roll and construct the combined rotation matrix
  Eigen::Matrix3d calculatePitchRollMatrix(double ax, double ay, double az) {
      // Calculate the pitch angle (theta)
      double theta = std::atan2(ax, std::sqrt(ay * ay + az * az));

      // Calculate the roll angle (phi)
      double phi = std::atan2(-ay, az);

      // Construct the pitch rotation matrix
      Eigen::Matrix3d R_y;
      R_y << std::cos(theta), 0, std::sin(theta),
            0, 1, 0,
            -std::sin(theta), 0, std::cos(theta);

      // Construct the roll rotation matrix
      Eigen::Matrix3d R_x;
      R_x << 1, 0, 0,
            0, std::cos(phi), -std::sin(phi),
            0, std::sin(phi), std::cos(phi);
      
      // Combine the rotation matrices
      Eigen::Matrix3d R = R_x * R_y;
      pitch_offset_gravity=theta;
      roll_offset_gravity=phi;
      return R;
  }

  void imuInit(MapRingBuffer<Imu::Ptr> imuBuf) {
    
    if (first_imu==false){
        return;
    }
    if (imuBuf.empty()) {
        return;
    }

    const std::size_t count = imuBuf.measMap_.size();
    acc_mean.setZero();
    gyr_mean.setZero();
    for (const auto &entry : imuBuf.measMap_) {
        acc_mean += entry.second->acc;
        gyr_mean += entry.second->gyr;
    }
    acc_mean /= static_cast<double>(count);
    gyr_mean /= static_cast<double>(count);

    acc_cov.setZero();
    gyr_cov.setZero();
    if (count > 1) {
        for (const auto &entry : imuBuf.measMap_) {
            const Eigen::Vector3d acc_error = entry.second->acc - acc_mean;
            const Eigen::Vector3d gyr_error = entry.second->gyr - gyr_mean;
            acc_cov += acc_error.cwiseProduct(acc_error);
            gyr_cov += gyr_error.cwiseProduct(gyr_error);
        }
        acc_cov /= static_cast<double>(count - 1);
        gyr_cov /= static_cast<double>(count - 1);

        const double duration =
            imuBuf.measMap_.rbegin()->first - imuBuf.measMap_.begin()->first;
        if (duration > 0.0) {
            imu_frequency = static_cast<double>(count - 1) / duration;
        }
    }

    time = imuBuf.measMap_.begin()->first;
    
    //TODO: double check the gravity direction
    gravity= - acc_mean / acc_mean.norm() *Gravity_Norm;
    gyr_bias = gyr_mean;
    acc_bias = acc_mean;
    first_imu = false;

    // //Align with Gravity if the IMU is rotated at the beginning. 
    Roll_Pitch_Gravity_Matrix=calculatePitchRollMatrix(acc_mean.x(), 
    acc_mean.y(), acc_mean.z());


    std::cout<<"imu_laser_R: "<<imu_laser_R<<std::endl;
    imu_laser_R_Gravity=Roll_Pitch_Gravity_Matrix.inverse()*imu_laser_R;
    Transformd imu_laser_transform_gravity_(imu_laser_R_Gravity, imu_laser_T); 
    imu_laser_gravity_Transform=imu_laser_transform_gravity_;
    std::cout<<"imu_laser_extrinsic_gravity: "<<imu_laser_gravity_Transform<<std::endl;    
      
  
    std::cout<<"IMU Data Summary"<<std::endl;
    std::cout<<"Gravity: "<<gravity.transpose()<<std::endl;
    std::cout<<"Gyroscope Bias: "<<gyr_bias.transpose()<<std::endl;
    std::cout<<"Accelerometer Bias: "<<acc_bias.transpose()<<std::endl;
    std::cout<<"Accelerometer Mean: "<<acc_mean.transpose()<<std::endl;
    std::cout<<"IMU Frequency: "<<imu_frequency<<" Hz"<<std::endl;
    std::cout<<"pitch offset gravity: "<<pitch_offset_gravity*180/M_PI<<std::endl;
    std::cout<<"roll offset gravity: "<<roll_offset_gravity*180/M_PI<<std::endl;
    std::cout<<"Roll Pitch Gravity Matrix: "<<Roll_Pitch_Gravity_Matrix<<std::endl;
      
  }

  // Function to convert a rotation matrix to roll, pitch, and yaw
  Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d& R) {
    Eigen::Vector3d rpy;

    // Extract the roll, pitch, and yaw from the rotation matrix
    double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));

    bool singular = sy < 1e-6; // If

    if (!singular) {
        rpy(0) = atan2(R(2,1), R(2,2)); // Roll
        rpy(1) = atan2(-R(2,0), sy);     // Pitch
        rpy(2) = atan2(R(1,0), R(0,0));  // Yaw
    } else {
        rpy(0) = atan2(-R(1,2), R(1,1)); // Roll
        rpy(1) = atan2(-R(2,0), sy);     // Pitch
        rpy(2) = 0;                      // Yaw
    }

    return rpy;
}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double gravity_norm = 9.8105;
  bool first_imu = true;
  double time;
  double imu_frequency = 200.0; // default 200 Hz
  Eigen::Vector3d gravity;
  Eigen::Vector3d gyr_bias;
  Eigen::Vector3d acc_bias;
  Eigen::Vector3d acc_mean; // mean of accelerometer measurement (m^2/sec)
  Eigen::Vector3d gyr_mean; // mean of gyroscope measurement (rad/s)
  Eigen::Vector3d acc; // accelerometer measurement (m^2/sec)
  Eigen::Vector3d gyr; // gyroscope measurement (rad/s)
  Eigen::Vector3d acc_cov; // covariance of accelerometer measurement
  Eigen::Vector3d gyr_cov; // covariance of gyroscope measurement
  Eigen::Quaterniond q_w_i;
  Eigen::Matrix3d Roll_Pitch_Gravity_Matrix;
  Eigen::Matrix3d imu_laser_R_Gravity;
  Transformd imu_laser_gravity_Transform;
  double pitch_offset_gravity;
  double roll_offset_gravity;
};

#endif // IMU_DATA_H
