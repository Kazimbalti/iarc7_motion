////////////////////////////////////////////////////////////////////////////
//
// Takeoff Controller
//
// Handles takeoff
//
////////////////////////////////////////////////////////////////////////////

// Associated header
#include "iarc7_motion/TakeoffController.hpp"

// ROS Headers
#include <ros/ros.h>
#include "ros_utils/ParamUtils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <geometry_msgs/PointStamped.h>

using namespace Iarc7Motion;

TakeoffController::TakeoffController(
        ros::NodeHandle& nh,
        ros::NodeHandle& private_nh,
        const ThrustModel& thrust_model)
    : landing_detected_message_(),
      landing_detected_subscriber_(),
      landing_detected_message_received_(false),
      state_(TakeoffState::DONE),
      throttle_(),
      thrust_model_(thrust_model),
      post_arm_delay_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "post_arm_delay")),
      takeoff_throttle_ramp_duration_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "takeoff_throttle_ramp_duration")),
      last_update_time_(),
      startup_timeout_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "startup_timeout")),
      update_timeout_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "update_timeout")),
      battery_timeout_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "battery_timeout")),
      battery_interpolator_(nh,
                            "motor_battery",
                            update_timeout_,
                            battery_timeout_,
                            [](double& data, const iarc7_msgs::Float64Stamped& msg) {
                                data = msg.data;
                            },
                            ros_utils::linearInterpolation<double>,
                            100),
      odom_interpolator_(nh,
                         "odometry/filtered",
                         update_timeout_,
                         ros::Duration(0),
                         [](Eigen::Matrix<double, 6, 1>& v, const nav_msgs::Odometry& msg) {
                              v[0] = msg.twist.twist.linear.x;
                              v[1] = msg.twist.twist.linear.y;
                              v[2] = msg.twist.twist.linear.z;
                              v[3] = msg.pose.pose.position.x;
                              v[4] = msg.pose.pose.position.y;
                              v[5] = msg.pose.pose.position.z;
                         },
                         ros_utils::linearInterpolation<Eigen::Matrix<double, 6, 1>>,
                         100),
      uav_arm_client_(nh.serviceClient<iarc7_msgs::Arm>("uav_arm")),
      arm_time_(),
      ramp_start_time_()
{
    landing_detected_subscriber_ = nh.subscribe("landing_detected",
                                    10,
                                    &TakeoffController::processLandingDetectedMessage,
                                    this);
}

// Used to reset and check initial conditions for takeoff
// Update needs to begin being called shortly after this is called.
bool TakeoffController::prepareForTakeover(const ros::Time& time)
{
    if (time < last_update_time_) {
        ROS_ERROR("Tried to reset TakeoffHandler with time before last update");
        return false;
    }

    if(!landing_detected_message_.data) {
        ROS_ERROR("Tried to reset the takeoff controller without being on the ground");
        return false;
    } else if (state_ != TakeoffState::DONE) {
        ROS_ERROR("Tried to reset takeoff controller that wasn't in the done state");
        return false;
    }

    throttle_ = 0;
    state_ = TakeoffState::ARM;
    // Mark the last update time as the current time to prevent large throttle spikes
    last_update_time_ = time;
    return true;
}

// Main update
bool TakeoffController::update(const ros::Time& time,
                               iarc7_msgs::OrientationThrottleStamped& uav_command)
{
    if (time < last_update_time_) {
        ROS_ERROR("Tried to update TakeoffHandler with time before last update");
        return false;
    }

    if(state_ == TakeoffState::ARM) {
        // Sending arm request to fc_comms
        iarc7_msgs::Arm srv;
        srv.request.data = true;
        // Check if request was succesful
        if(uav_arm_client_.call(srv)) {
            if(srv.response.success == false) {
                ROS_ERROR("Service could not arm the controller");
                return false;
            }
        }
        else {
            ROS_ERROR("Arming service failed");
            return false;
        }

        arm_time_= time;
        state_ = TakeoffState::PAUSE;
    }
    else if (state_ == TakeoffState::PAUSE){
        if (time > arm_time_ + post_arm_delay_){
            state_ = TakeoffState::RAMP;
            ramp_start_time_ = time;
        }
    }
    else if(state_ == TakeoffState::RAMP) {
        if (time <= ramp_start_time_ + takeoff_throttle_ramp_duration_){
            double voltage;
            if (!battery_interpolator_.getInterpolatedMsgAtTime(voltage, time)) {
                ROS_ERROR("Failed to get battery voltage to interpret results of thrust model");
                return false;
            }

            // Get the current odometry of the quad.
            Eigen::Matrix<double, 6, 1> odometry(6);
            bool success = odom_interpolator_.getInterpolatedMsgAtTime(odometry, time);
            if (!success) {
                ROS_ERROR("Failed to get current odometry in TakeoffController::update");
                return false;
            }

            // Assume center of lift height is the quad frame, technically it should
            // be the center of the propellers
            double col_height = odometry[5];
            double hover_throttle = thrust_model_.voltageFromThrust(
                                                  9.8,
                                                  4,
                                                  col_height)/voltage;

            // Linearly ramp to hover throttle
            throttle_ = ((time-ramp_start_time_).toSec()
                          / takeoff_throttle_ramp_duration_.toSec())
                            * hover_throttle;
        }
        else{
            state_ = TakeoffState::DONE;
        }
    }
    else if(state_ == TakeoffState::DONE)
    {
        ROS_ERROR("Tried to update takeoff handler when in DONE state");
        return false;
    }
    else
    {
        ROS_ASSERT_MSG(false, "Invalid state in takeoff controller");
    }

    // Fill in the uav_command's information
    uav_command.header.stamp = time;
    uav_command.throttle = throttle_;

    // Check that none of the throttle values are infinite before returning
    if (!std::isfinite(uav_command.throttle)
     || !std::isfinite(uav_command.data.pitch)
     || !std::isfinite(uav_command.data.roll)
     || !std::isfinite(uav_command.data.yaw)) {
        ROS_ERROR(
            "Part of command is not finite in TakeoffHandler::update");
        return false;
    }

    // Print the throttle information
    ROS_DEBUG("Throttle: %f Pitch: %f Roll: %f Yaw: %f",
             uav_command.throttle,
             uav_command.data.pitch,
             uav_command.data.roll,
             uav_command.data.yaw);

    last_update_time_ = time;
    return true;
}

bool TakeoffController::waitUntilReady()
{
    bool success = battery_interpolator_.waitUntilReady(startup_timeout_);
    if (!success) {
        ROS_ERROR("Failed to fetch battery voltage");
        return false;
    }

    success = odom_interpolator_.waitUntilReady(startup_timeout_);
    if (!success) {
        ROS_ERROR("Failed to fetch initial odometry");
        return false;
    }

    const ros::Time start_time = ros::Time::now();
    while (ros::ok()
           && !landing_detected_message_received_
           && ros::Time::now() < start_time + startup_timeout_) {
        ros::spinOnce();
        ros::Duration(0.005).sleep();
    }

    if (!landing_detected_message_received_) {
        ROS_ERROR_STREAM("TakeoffController failed to fetch initial switch message");
        return false;
    }

    // This time is just used to calculate any ramping that needs to be done.
    last_update_time_ = std::max({landing_detected_message_.header.stamp,
                                  battery_interpolator_.getLastUpdateTime(),
                                  odom_interpolator_.getLastUpdateTime()});
    return true;
}

bool TakeoffController::isDone()
{
  return (state_ == TakeoffState::DONE);
}

const ThrustModel& TakeoffController::getThrustModel() const
{
  return thrust_model_;
}

void TakeoffController::processLandingDetectedMessage(
    const iarc7_msgs::BoolStamped::ConstPtr& message)
{
    landing_detected_message_received_ = true;
    landing_detected_message_ = *message;
}
