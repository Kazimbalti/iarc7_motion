////////////////////////////////////////////////////////////////////////////
//
// Quad Velocity controller
//
// Accepts a target velocity and uses 4 PID loops (pitch, yaw, roll, thrust)
// To attempt to hit the target velocity.
//
////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <cmath>

// Associated header
#include "iarc7_motion/QuadVelocityController.hpp"

// ROS Headers
#include "ros_utils/LinearMsgInterpolator.hpp"
#include "ros_utils/ParamUtils.hpp"
#include "ros_utils/SafeTransformWrapper.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

// ROS message headers
#include "geometry_msgs/AccelWithCovarianceStamped.h"
#include "iarc7_msgs/Float64ArrayStamped.h"
#include "iarc7_msgs/Float64Stamped.h"
#include "iarc7_msgs/MotionPointStamped.h"

using namespace Iarc7Motion;

QuadVelocityController::QuadVelocityController(
        double vz_pid_settings[6],
        double vx_pid_settings[6],
        double vy_pid_settings[6],
        double (&position_p)[3],
        double yaw_p,
        const ThrustModel& thrust_model,
        const ThrustModel& thrust_model_side,
        const ros::Duration& battery_timeout,
        ros::NodeHandle& nh,
        ros::NodeHandle& private_nh,
        LandPlanner& land_planner)
    : vz_pid_(vz_pid_settings,
              "vz_pid",
              private_nh),
      vx_pid_(vx_pid_settings,
              "vx_pid",
              private_nh),
      vy_pid_(vy_pid_settings,
              "vy_pid",
              private_nh),
      thrust_model_(thrust_model),
      thrust_model_front_(thrust_model_side),
      thrust_model_back_(thrust_model_side),
      thrust_model_left_(thrust_model_side),
      thrust_model_right_(thrust_model_side),
      setpoint_(),
      last_setpoint_(),
      xy_mixer_(ros_utils::ParamUtils::getParam<std::string>(
              private_nh,
              "xy_mixer")),
      position_p_(position_p),
      yaw_p_(yaw_p),
      startup_timeout_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "startup_timeout")),
      update_timeout_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "update_timeout")),
      accel_interpolator_(
              nh,
              "accel/filtered",
              update_timeout_,
              ros::Duration(0),
              [](tf2::Vector3& accel, const geometry_msgs::AccelWithCovarianceStamped& msg) {
                  accel.setX(msg.accel.accel.linear.x);
                  accel.setY(msg.accel.accel.linear.y);
                  accel.setZ(msg.accel.accel.linear.z);
              },
              ros_utils::linearInterpolation<tf2::Vector3>,
              100,
              true),
      battery_interpolator_(nh,
                            "motor_battery",
                            update_timeout_,
                            battery_timeout,
                            [](double& data, const iarc7_msgs::Float64Stamped& msg) {
                                data = msg.data;
                            },
                            ros_utils::linearInterpolation<double>,
                            100),
      odom_interpolator_(nh,
                         "odometry/filtered",
                         update_timeout_,
                         ros::Duration(0),
                         [](nav_msgs::Odometry& data, const nav_msgs::Odometry& msg) { 
                              data = msg;
                         },
                         [](nav_msgs::Odometry& out,
                            const nav_msgs::Odometry& last,
                            const nav_msgs::Odometry& next,
                            const double alpha) {
                                // Linearly interpolate the euclidian components
                                Eigen::Matrix<double, 6, 1> last_v(6);
                                last_v[0] = last.twist.twist.linear.x;
                                last_v[1] = last.twist.twist.linear.y;
                                last_v[2] = last.twist.twist.linear.z;
                                last_v[3] = last.pose.pose.position.x;
                                last_v[4] = last.pose.pose.position.y;
                                last_v[5] = last.pose.pose.position.z;

                                Eigen::Matrix<double, 6, 1> next_v(6);
                                next_v[0] = next.twist.twist.linear.x;
                                next_v[1] = next.twist.twist.linear.y;
                                next_v[2] = next.twist.twist.linear.z;
                                next_v[3] = next.pose.pose.position.x;
                                next_v[4] = next.pose.pose.position.y;
                                next_v[5] = next.pose.pose.position.z;

                                Eigen::Matrix<double, 6, 1> current_v(6);
                                ros_utils::linearInterpolation<Eigen::Matrix<double, 6, 1>>(current_v, last_v, next_v, alpha);

                                // Linearly interpolate the rotation
                                Eigen::Quaterniond last_q(last.pose.pose.orientation.w,
                                                          last.pose.pose.orientation.x,
                                                          last.pose.pose.orientation.y,
                                                          last.pose.pose.orientation.z);

                                Eigen::Quaterniond next_q(next.pose.pose.orientation.w,
                                                          next.pose.pose.orientation.x,
                                                          next.pose.pose.orientation.y,
                                                          next.pose.pose.orientation.z);

                                Eigen::Quaterniond current_q = last_q.slerp(alpha, next_q);

                                // Put results in out
                                out.twist.twist.linear.x = current_v[0];
                                out.twist.twist.linear.y = current_v[1];
                                out.twist.twist.linear.z = current_v[2];
                                out.pose.pose.position.x = current_v[3];
                                out.pose.pose.position.y = current_v[4];
                                out.pose.pose.position.z = current_v[5];

                                out.pose.pose.orientation.w = current_q.w();
                                out.pose.pose.orientation.x = current_q.x();
                                out.pose.pose.orientation.y = current_q.y();
                                out.pose.pose.orientation.z = current_q.z();
                         },
                         100,
                         true),
      min_thrust_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "min_thrust")),
      max_thrust_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "max_thrust")),
      min_side_thrust_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "min_side_thrust")),
      max_side_thrust_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "max_side_thrust")),
      level_flight_required_height_(
          ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "level_flight_required_height")),
      level_flight_required_hysteresis_(
          ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "level_flight_required_hysteresis")),
      level_flight_active_(true),
      motion_point_interpolator_(nh),
      land_planner_(land_planner)
{
    derotated_twist_accel_ = private_nh.advertise<iarc7_msgs::Float64ArrayStamped>("derotated_twist_accel", 1000);

    // Create the publisher to send the current intended velocity target
    motion_point_target_ = nh.advertise<iarc7_msgs::MotionPointStamped>("cmd_motion_point", 50);
}

// Use a new thrust model
void QuadVelocityController::setThrustModel(const ThrustModel& thrust_model)
{
    thrust_model_ = thrust_model;
}

// Main update, runs all PID calculations and returns a desired uav_command
// Needs to be called at regular intervals in order to keep catching the latest velocities.
bool QuadVelocityController::update(const ros::Time& current_time,
                                    iarc7_msgs::OrientationThrottleStamped& uav_command,
                                    unsigned int setpoint_source,
                                    bool xy_passthrough_mode,
                                    double a_x,
                                    double a_y)
{
    // Get the current odometry of the quad
    nav_msgs::Odometry odometry_rotated;
    bool success = odom_interpolator_.getLatestMsgAfterTime(odometry_rotated, last_update_time_);
    if (!success) {
        ROS_ERROR("Failed to get current velocities in QuadVelocityController::update");
        return false;
    }

    ros::Time update_time(odometry_rotated.header.stamp);

    if (update_time <= last_update_time_) {
        ROS_ERROR("Tried to update QuadVelocityController with data less than or equal to last update time");
        return false;
    }

    // Get the current battery voltage of the quad
    double voltage;
    success = battery_interpolator_.getInterpolatedMsgAtTime(voltage, update_time);
    if (!success) {
        ROS_ERROR("Failed to get current battery voltage in QuadVelocityController::update");
        return false;
    }

    // Get the current acceleration of the quad
    tf2::Vector3 accel_rotated;
    success = accel_interpolator_.getInterpolatedMsgAtTime(accel_rotated, update_time);
    if (!success) {
        ROS_ERROR("Failed to get current acceleration in QuadVelocityController::update");
        return false;
    }

    ros::Time setpoint_time = current_time > update_time ? current_time : update_time;
    // ROS_INFO_STREAM("delta: " << update_time - last_update_time_ << " up: " << update_time - current_time << " set: " << setpoint_time - current_time);

    if (setpoint_source == 0) {
        // If nothing is wrong get a motion point target from the uav motion point interpolator
        motion_point_interpolator_.getTargetMotionPoint(setpoint_time, setpoint_);
    } else if (setpoint_source == 1) {
        bool success = land_planner_.getTargetMotionPoint(setpoint_time, setpoint_);
        if (!success) {
            ROS_ERROR("LowLevelMotion LandPlanner getTargetTwist failed");
            return false;
        }
    } else {
        ROS_ERROR("QuadVelocityController invalid setpoint source");
        return false;
    }

    // Publish the current target velocity
    motion_point_target_.publish(setpoint_);

    // Get current yaw angle from the odometry
    double current_yaw = yawFromQuaternion(odometry_rotated.pose.pose.orientation);

    Eigen::Vector3d position;
    position[0] = odometry_rotated.pose.pose.position.x;
    position[1] = odometry_rotated.pose.pose.position.y;
    position[2] = odometry_rotated.pose.pose.position.z;

    Eigen::Vector3d velocity;
    tf2::Vector3 vector;
    tf2::convert(odometry_rotated.twist.twist.linear, vector);
    derotateVector(velocity, vector, odometry_rotated.pose.pose.orientation);

    Eigen::Vector3d accel;
    derotateVector(accel, accel_rotated, odometry_rotated.pose.pose.orientation);

    iarc7_msgs::Float64ArrayStamped derotated_twist_accel_msg;
    derotated_twist_accel_msg.header.stamp = update_time;
    derotated_twist_accel_msg.data = {velocity[0], velocity[1], velocity[2],
                                      accel[0], accel[1], accel[2]};
    derotated_twist_accel_.publish(derotated_twist_accel_msg);

    // Update setpoints on PID controllers
    updatePidSetpoints(current_yaw, position);

    // Update Vz PID loop with position and velocity
    double x_accel_output = 0;
    double y_accel_output = 0;
    double z_accel_output = 0;
    success = vz_pid_.update(velocity[2],
                             update_time,
                             z_accel_output,
                             last_setpoint_.motion_point.accel.linear.z - accel[2], true);

    if (!success) {
        ROS_ERROR("Vz PID update failed in QuadVelocityController::update");
        return false;
    }

    // Assume center of lift height is the quad frame, technically it should
    // be the center of the propellers
    double col_height = position[2];
    if (level_flight_active_ && col_height
                                  > level_flight_required_height_
                                    + level_flight_required_hysteresis_) {
        level_flight_active_ = false;
        vx_pid_.resetAccumulator();
        vy_pid_.resetAccumulator();
    }

    // Final output variables
    if (xy_passthrough_mode) {
        x_accel_output = a_x;
        y_accel_output = a_y;
    }
    else if (col_height < level_flight_required_height_ || level_flight_active_) {
        x_accel_output = 0;
        y_accel_output = 0;
        level_flight_active_ = true;
    }
    else {
        // Update vx PID loop
        success = vx_pid_.update(velocity[0],
                                 update_time,
                                 x_accel_output,
                                 last_setpoint_.motion_point.accel.linear.x - accel[0],
                                 true);
        if (!success) {
            ROS_ERROR("Vx PID update failed in QuadVelocityController::update");
            return false;
        }

        // Update vy PID loop
        success = vy_pid_.update(velocity[1],
                                 update_time,
                                 y_accel_output,
                                 last_setpoint_.motion_point.accel.linear.y - accel[1],
                                 true);
        if (!success) {
            ROS_ERROR("Vy PID update failed in QuadVelocityController::update");
            return false;
        }
    }

    // Fill in the uav_command's information
    uav_command.header.stamp = update_time;

    double x_accel = x_accel_output + setpoint_.motion_point.accel.linear.x;
    double y_accel = y_accel_output + setpoint_.motion_point.accel.linear.y;
    double z_accel = g_ + z_accel_output + setpoint_.motion_point.accel.linear.z;

    double thrust_request;
    if(xy_mixer_ == "4dof") {
        double pitch_request, roll_request;
        {
            Eigen::Vector3d accel;
            accel(0) = std::cos(current_yaw) * x_accel
                       + std::sin(current_yaw) * y_accel;
            accel(1) = -std::sin(current_yaw) * x_accel
                       + std::cos(current_yaw) * y_accel;
            accel(2) = z_accel;

            commandForAccel(accel, pitch_request, roll_request, thrust_request);
        }

        uav_command.data.pitch = pitch_request;
        uav_command.data.roll = roll_request;

        uav_command.planar.front_throttle = 0;
        uav_command.planar.back_throttle = 0;
        uav_command.planar.left_throttle = 0;
        uav_command.planar.right_throttle = 0;
    }
    else if(xy_mixer_ == "6dof") {
        //ROS_ERROR_STREAM(x_accel << " " << y_accel);
        //ROS_ERROR_STREAM(min_side_thrust_ << " " << max_side_thrust_);

        thrust_request = z_accel;

        uav_command.data.pitch = 0;
        uav_command.data.roll = 0;

        uav_command.planar.front_throttle = thrust_model_front_.voltageFromThrust(
            std::min(std::max(-x_accel + min_side_thrust_, min_side_thrust_), max_side_thrust_),
            1,
            0.0)
            / voltage;
        uav_command.planar.back_throttle = thrust_model_back_.voltageFromThrust(
          std::min(std::max(x_accel + min_side_thrust_, min_side_thrust_), max_side_thrust_),
            1,
            0.0)
            / voltage;
        uav_command.planar.left_throttle = thrust_model_left_.voltageFromThrust(
            std::min(std::max(-y_accel + min_side_thrust_, min_side_thrust_), max_side_thrust_),
            1,
            0.0)
            / voltage;
        uav_command.planar.right_throttle = thrust_model_right_.voltageFromThrust(
            std::min(std::max(y_accel + min_side_thrust_, min_side_thrust_), max_side_thrust_),
            1,
            0.0)
            / voltage;
        //ROS_ERROR_STREAM(uav_command);
    }
    else {
      ROS_ERROR("Invalid XY Mixer type");
      return false;
    }

    ROS_DEBUG("Thrust: %f, Voltage: %f, height: %f", thrust_request, voltage, col_height);
    uav_command.throttle = thrust_model_.voltageFromThrust(
            std::min(std::max(thrust_request, min_thrust_), max_thrust_),
            4,
            col_height)
            / voltage;

    // Hack heading to straight ahead
    uav_command.data.yaw = -yaw_p_ * (0.0 - current_yaw);

    // Check that the PID loops did not return invalid values before returning
    if (!std::isfinite(uav_command.throttle)
     || !std::isfinite(uav_command.data.pitch)
     || !std::isfinite(uav_command.data.roll)
     || !std::isfinite(uav_command.data.yaw)) {
        ROS_ERROR(
            "Part of command is not finite in QuadVelocityController::update");
        return false;
    }

    // Print the velocity and throttle information
    ROS_DEBUG("Vz: %f Vx: %f Vy: %f",
             velocity[2],
             velocity[0],
             velocity[1]);
    ROS_DEBUG("Throttle: %f Pitch: %f Roll: %f Yaw: %f",
             uav_command.throttle,
             uav_command.data.pitch,
             uav_command.data.roll,
             uav_command.data.yaw);

    last_update_time_ = update_time;
    last_setpoint_ = setpoint_;

    return true;
}

bool QuadVelocityController::waitUntilReady()
{
    bool success = accel_interpolator_.waitUntilReady(startup_timeout_);
    if (!success) {
        ROS_ERROR("Failed to fetch initial acceleration");
        return false;
    }

    success = battery_interpolator_.waitUntilReady(startup_timeout_);
    if (!success) {
        ROS_ERROR("Failed to fetch battery voltage");
        return false;
    }

    success = odom_interpolator_.waitUntilReady(startup_timeout_);
    if (!success) {
        ROS_ERROR("Failed to fetch initial velocity");
        return false;
    }

    // Mark the last update time as 1ns later than the odometry message,
    // because we should always have a message older than the last update
    last_update_time_ = std::max({accel_interpolator_.getLastUpdateTime(),
                                  battery_interpolator_.getLastUpdateTime(),
                                  odom_interpolator_.getLastUpdateTime()});
    return true;
}

void QuadVelocityController::updatePidSetpoints(double /*current_yaw*/, const Eigen::Vector3d& position)
{
    double position_velocity_request[3] = {
        position_p_[0] * (setpoint_.motion_point.pose.position.x - position[0]),
        position_p_[1] * (setpoint_.motion_point.pose.position.y - position[1]),
        position_p_[2] * (setpoint_.motion_point.pose.position.z - position[2])
    };

    double map_z_velocity = position_velocity_request[2] + setpoint_.motion_point.twist.linear.z;

    vz_pid_.setSetpoint(map_z_velocity);

    // x and y velocities are transformed according to the last yaw
    // angle because the incoming target velocities are in the map frame
    double map_x_velocity = setpoint_.motion_point.twist.linear.x + position_velocity_request[0];
    double map_y_velocity = setpoint_.motion_point.twist.linear.y + position_velocity_request[1];

    vx_pid_.setSetpoint(map_x_velocity);
    vy_pid_.setSetpoint(map_y_velocity);
}

double QuadVelocityController::yawFromQuaternion(
        const geometry_msgs::Quaternion& rotation)
{
    tf2::Quaternion quaternion;
    tf2::convert(rotation, quaternion);

    tf2::Matrix3x3 matrix;
    matrix.setRotation(quaternion);

    double y, p, r;
    matrix.getEulerYPR(y, p, r);

    return y;
}

void QuadVelocityController::derotateVector(
        Eigen::Vector3d& result,
        const tf2::Vector3& vector,
        const geometry_msgs::Quaternion& rotation)
{
    tf2::Quaternion quaternion;
    tf2::convert(rotation, quaternion);

    tf2::Matrix3x3 matrix(quaternion);
    tf2::Matrix3x3 reverse_matrix = matrix.inverse();

    // Reverse rotation
    result[0] = reverse_matrix.tdotx(vector);
    result[1] = reverse_matrix.tdoty(vector);
    result[2] = reverse_matrix.tdotz(vector);
}

void QuadVelocityController::commandForAccel(
        const Eigen::Vector3d& accel,
        double& pitch,
        double& roll,
        double& thrust)
{
    // This transformation assumes the overall thrust vector
    // is upwards in the world frame
    Eigen::Vector3d limited_accel(accel[0],
                                  accel[1],
                                  std::max(0.0, accel[2]));

    const auto a_hat = limited_accel.normalized();
    roll = -std::asin(a_hat(1));
    const auto a_no_roll = a_hat - a_hat(1) * Eigen::Vector3d::UnitY();
    const auto a_hat_no_roll = a_no_roll.normalized();
    pitch = std::asin(a_hat_no_roll(0));
    thrust = limited_accel.norm();
}

bool QuadVelocityController::prepareForTakeover()
{
    vz_pid_.reset();
    vx_pid_.reset();
    vy_pid_.reset();
    return true;
}
