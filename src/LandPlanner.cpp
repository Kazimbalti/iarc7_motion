////////////////////////////////////////////////////////////////////////////
//
// Land Planner
//
// Handles landing
//
////////////////////////////////////////////////////////////////////////////

// Associated header
#include "iarc7_motion/LandPlanner.hpp"

// ROS Headers
#include "ros_utils/ParamUtils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace Iarc7Motion;

LandPlanner::LandPlanner(
        ros::NodeHandle& nh,
        ros::NodeHandle& private_nh)
    : transform_wrapper_(),
      state_(LandState::DONE),
      requested_x_(0.0),
      requested_y_(0.0),
      requested_height_(0.0),
      cushion_height_(0.0),
      actual_descend_rate_(0.0),
      descend_rate_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "descend_rate")),
      cushion_rate_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "cushion_rate")),
      descend_acceleration_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "descend_acceleration")),
      cushion_acceleration_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "cushion_acceleration")),
      landing_detected_height_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "landing_detected_height")),
      last_update_time_(),
      startup_timeout_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "startup_timeout")),
      update_timeout_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "update_timeout")),
      uav_arm_client_(nh.serviceClient<iarc7_msgs::Arm>("uav_arm"))
{
    ROS_ASSERT_MSG(descend_rate_ <= 0, "descend_rate_ loaded in with wrong sign!");
    ROS_ASSERT_MSG(cushion_rate_ <= 0, "cushion_rate_ loaded in with wrong sign!");
    ROS_ASSERT_MSG(cushion_acceleration_ > 0,"cushion_acceleration_ loaded in with wrong sign!");
    ROS_ASSERT_MSG(descend_acceleration_ < 0, "descend_acceleration_ loaded in with wrong sign!");
}

// Used to prepare and check initial conditions for landing
// getTargetTwist needs to begin being called shortly after this is called.
bool LandPlanner::prepareForTakeover(const ros::Time& time)
{
    if (time < last_update_time_) {
        ROS_ERROR("Tried to reset LandPlanner with time before last update");
        return false;
    }

    // Get the current transform (xyz) of the quad
    geometry_msgs::TransformStamped transform;
    bool success = transform_wrapper_.getTransformAtTime(transform,
                                                         "map",
                                                         "quad",
                                                         time,
                                                         update_timeout_);
    if (!success) {
        ROS_ERROR("Failed to get current transform in LandPlanner::prepareForTakeover");
        return false;
    }

    requested_x_ = transform.transform.translation.x;
    requested_y_ = transform.transform.translation.y;
    requested_height_ = transform.transform.translation.z;

    // height determined by the ratio of landing accelerations
    cushion_height_ = std::min(0.5 * (std::pow(descend_rate_,2)/cushion_acceleration_),
                               requested_height_ * ( 1 - ( 1 / ( 1 - descend_acceleration_/cushion_acceleration_))));
    actual_descend_rate_ = 0.0;

    state_ = LandState::DESCEND;
    // Mark the last update time as the current time since update may not have
    // Been called in a long time.
    last_update_time_ = time;
    return true;
}

// Main update
bool LandPlanner::getTargetMotionPoint(const ros::Time& time,
                         iarc7_msgs::MotionPointStamped& motion_point)
{
    if (time < last_update_time_) {
        ROS_ERROR("Tried to update LandPlanner with time before last update");
        return false;
    }

    // Get the current transform (xyz) of the quad
    geometry_msgs::TransformStamped transform;
    bool success = transform_wrapper_.getTransformAtTime(transform,
                                                         "map",
                                                         "quad",
                                                         time,
                                                         update_timeout_);
    if (!success) {
        ROS_ERROR("Failed to get current transform in LandPlanner::update");
        return false;
    }

    if(state_ == LandState::DESCEND)
    {
      // determines whether to speed up or slow down, depending on height
        if (transform.transform.translation.z > cushion_height_) {
            actual_descend_rate_ = std::max(descend_rate_,
                                            actual_descend_rate_
                                            + (descend_acceleration_
                                            * (time - last_update_time_).toSec()));
        }
        else{
            actual_descend_rate_ = std::min(cushion_rate_,
                                            actual_descend_rate_
                                            + (cushion_acceleration_
                                            * (time - last_update_time_).toSec()));
        }

        requested_height_ = std::max(0.0, requested_height_
                                          + (actual_descend_rate_
                                          * (time - last_update_time_).toSec()));

        if(transform.transform.translation.z <= landing_detected_height_) {
            // Sending disarm request to fc_comms
            iarc7_msgs::Arm srv;
            srv.request.data = false;
            // Check if request was succesful
            if(uav_arm_client_.call(srv)) {
                if(srv.response.success == false) {
                    ROS_ERROR("Service could not disarm the controller");
                    return false;
                }
            }
            else {
                ROS_ERROR("disarming service failed");
                return false;
            }
            state_ = LandState::DONE;
        }
    }
    else if(state_ == LandState::DONE)
    {
        ROS_ERROR("Tried to update takeoff handler when in DONE state");
        return false;
    }
    else
    {
        ROS_ASSERT_MSG(false, "Invalid state in takeoff controller");
    }

    // Fill in the uav_command's information
    motion_point.header.stamp = time;

    motion_point.motion_point.pose.position.x = requested_x_;
    motion_point.motion_point.pose.position.y = requested_y_;
    motion_point.motion_point.pose.position.z = requested_height_;
    motion_point.motion_point.twist.linear.z = actual_descend_rate_;

    if (transform.transform.translation.z > cushion_height_) {
        if(actual_descend_rate_ > descend_rate_) {
            motion_point.motion_point.accel.linear.z = descend_acceleration_;
        }
    }
    else {
        if (actual_descend_rate_ < cushion_rate_) {
            motion_point.motion_point.accel.linear.z = cushion_acceleration_;
        }
    }

    last_update_time_ = time;
    return true;
}

bool LandPlanner::waitUntilReady()
{
    geometry_msgs::TransformStamped transform;
    bool success = transform_wrapper_.getTransformAtTime(transform,
                                                         "map",
                                                         "quad",
                                                         ros::Time(0),
                                                         startup_timeout_);
    if (!success)
    {
        ROS_ERROR("Failed to fetch transform");
        return false;
    }

    // This time is just used to calculate any ramping that needs to be done.
    last_update_time_ = transform.header.stamp;
    return true;
}

bool LandPlanner::isDone()
{
  return (state_ == LandState::DONE);
}
