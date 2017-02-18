////////////////////////////////////////////////////////////////////////////
//
// LowLevelMotionController
//
// This is the top level class for the velocity controller.
// It uses an AccelerationPlanner to get velocities.
// Sends them to a QuadVelocityController who runs the PID loops that set the angles
// to output to the flight controller.
// The angles and throttle values are sent to a TwistLimiter class which limits the
// min, max, and max rate of change of those values.
// Finally the processed angles and throttle values are sent out onto a topic.
//
////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>

#include "iarc7_motion/AccelerationPlanner.hpp"
#include "iarc7_motion/QuadVelocityController.hpp"
#include "iarc7_motion/QuadTwistRequestLimiter.hpp"

#include "iarc7_safety/SafetyClient.hpp"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"

using namespace Iarc7Motion;
using geometry_msgs::TwistStamped;
using geometry_msgs::Twist;

// This is a helper function that will limit a iarc7_msgs::OrientationThrottleStamped using the twist limiter
// The twist limiter uses TwistStamped messages to do it's work so this function converts between the data types.
void limitUavCommand(QuadTwistRequestLimiter& limiter, iarc7_msgs::OrientationThrottleStamped& uav_command)
{
    TwistStamped uav_twist_stamped;
    Twist& uav_twist = uav_twist_stamped.twist;

    // Convert from uav command to twist
    uav_twist_stamped.header.stamp = uav_command.header.stamp;
    uav_twist.linear.z  = uav_command.throttle;
    uav_twist.angular.y = uav_command.data.pitch;
    uav_twist.angular.x = uav_command.data.roll;
    uav_twist.angular.z = uav_command.data.yaw;

    limiter.limitTwist(uav_twist_stamped);

    // Copy the twist to the uav command
    uav_command.header.stamp = uav_twist_stamped.header.stamp;
    uav_command.throttle     = uav_twist.linear.z;
    uav_command.data.pitch   = uav_twist.angular.y;
    uav_command.data.roll    = uav_twist.angular.x;
    uav_command.data.yaw     = uav_twist.angular.z;
}

// Loads the PID parameters from ROS param server into the passed in arrays
void getPidParams(ros::NodeHandle& nh, double throttle_pid[5], double pitch_pid[5], double roll_pid[5], double yaw_pid[5])
{
    // Throttle PID settings retrieve
    nh.param("throttle_p", throttle_pid[0], 0.0);
    nh.param("throttle_i", throttle_pid[1], 0.0);
    nh.param("throttle_d", throttle_pid[2], 0.0);
    nh.param("throttle_accumulator_max", throttle_pid[3], 0.0);
    nh.param("throttle_accumulator_min", throttle_pid[4], 0.0);

    // Pitch PID settings retrieve
    nh.param("pitch_p", pitch_pid[0], 0.0);
    nh.param("pitch_i", pitch_pid[1], 0.0);
    nh.param("pitch_d", pitch_pid[2], 0.0);
    nh.param("pitch_accumulator_max", pitch_pid[3], 0.0);
    nh.param("pitch_accumulator_min", pitch_pid[4], 0.0);

    // Roll PID settings retrieve
    nh.param("roll_p", roll_pid[0], 0.0);
    nh.param("roll_i", roll_pid[1], 0.0);
    nh.param("roll_d", roll_pid[2], 0.0);
    nh.param("roll_accumulator_max", roll_pid[3], 0.0);
    nh.param("roll_accumulator_min", roll_pid[4], 0.0);

    // Yaw PID settings retrieve
    nh.param("yaw_p", yaw_pid[0], 0.0);
    nh.param("yaw_i", yaw_pid[1], 0.0);
    nh.param("yaw_d", yaw_pid[2], 0.0);
    nh.param("yaw_accumulator_max", yaw_pid[3], 0.0);
    nh.param("yaw_accumulator_min", yaw_pid[4], 0.0);
}

// Loads the parameters for the TwistLimiter from the ROS paramater server
void getTwistLimiterParams(ros::NodeHandle& nh, Twist& min,  Twist& max,  Twist& max_rate)
{
    // Throttle Limit settings retrieve
    nh.param("throttle_max", max.linear.z, 0.0);
    nh.param("throttle_min", min.linear.z, 0.0);
    nh.param("throttle_max_rate", max_rate.linear.z, 0.0);

    // Pitch Limit settings retrieve
    nh.param("pitch_max", max.angular.y, 0.0);
    nh.param("pitch_min", min.angular.y, 0.0);
    nh.param("pitch_max_rate", max_rate.angular.y, 0.0);

    // Roll Limit settings retrieve
    nh.param("roll_max", max.angular.x, 0.0);
    nh.param("roll_min", min.angular.x, 0.0);
    nh.param("roll_max_rate", max_rate.angular.x, 0.0);

    // Yaw Limit settings retrieve
    nh.param("yaw_max", max.angular.z, 0.0);
    nh.param("yaw_min", min.angular.z, 0.0);
    nh.param("yaw_max_rate", max_rate.angular.z, 0.0);
}

// Main entry point for the low level motion controller
int main(int argc, char **argv)
{
    // Required by ROS before calling many functions
    ros::init(argc, argv, "Low_Level_Motion_Control");

    ROS_INFO("Low_Level_Motion_Control begin");

    // Create a node handle for the node
    ros::NodeHandle nh;
    // This node handle has a specific namespace that allows us to easily encapsulate parameters
    ros::NodeHandle param_nh ("low_level_motion_controller");

    // Form a connection with the node monitor. If no connection can be made assert because we don't
    // know what's going on with the other nodes.
    ROS_INFO("low_level_motion: Attempting to form safety bond");
    Iarc7Safety::SafetyClient safety_client(nh, "low_level_motion");
    ROS_ASSERT_MSG(safety_client.formBond(), "low_level_motion: Could not form bond with safety client");

    // Get the PID parameters from the ROS parameter server
    double throttle_pid[5];
    double pitch_pid[5];
    double roll_pid[5];
    double yaw_pid[5];
    getPidParams(param_nh, throttle_pid, pitch_pid, roll_pid, yaw_pid);

    // Create a quad velocity controller. It will output angles corresponding to our desired velocity
    QuadVelocityController quadController(throttle_pid, pitch_pid, roll_pid, yaw_pid);

    // Create an acceleration planner. It handles interpolation between timestamped velocity requests so that
    // Smooth accelerations are possible.
    AccelerationPlanner accelerationPlanner(nh);

    // Create the publisher to send the processed uav_commands out with (angles, throttle)
    ros::Publisher uav_control_ = nh.advertise<iarc7_msgs::OrientationThrottleStamped>("uav_direction_command", 50);

    // Check for empty uav_control_ as per http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers
    // section 1
    ROS_ASSERT_MSG(uav_control_, "Could not create uav_direction_command publisher");

    // Get the parameters for the twist limiter
    Twist min;
    Twist max;
    Twist max_rate;
    getTwistLimiterParams(param_nh, min, max, max_rate);

    // Create the twist limiter, it will limit min value, max value, and max rate of change.
    QuadTwistRequestLimiter limiter(min, max, max_rate);

    // Wait for a valid time in case we are using simulated time (not wall time)
    while (ros::ok() && ros::Time::now() == ros::Time(0)) {
        // wait
        ros::spinOnce();
    }

    // Cache the time
    ros::Time last_time = ros::Time::now();

    // Run until ROS says we need to shutdown
    while (ros::ok())
    {
        // Check the safety client before updating anything
        // If fatal is active the node monitor is telling everyone to shut down immediately
        ROS_ASSERT_MSG(!safety_client.isFatalActive(), "low_level_motion: fatal event from safety");

        // Get the time
        ros::Time current_time = ros::Time::now();

        // Make sure we don't call QuadVelocity controllers update unless we have a new timestamp to give.
        // This can be a problem with simulated time that does not update with high precision.
        if(current_time > last_time)
        {
            last_time = current_time;

            //  This will contain the target twist or velocity that we want to achieve
            geometry_msgs::TwistStamped target_twist;
            
            // Check for a safety state in which case we should execute our safety response
            if(safety_client.isSafetyActive())
            {
                // This is the safety response
                // Override whatever the Acceleration Planner wants to do and attempt to land
                target_twist.twist.linear.z = -0.2; // Try to descend
            }
            else
            {
                // If nothing is wrong get a target velocity from the acceleration planner
                accelerationPlanner.getTargetTwist(current_time, target_twist);
            }

            // Request the appropriate throttle and angle settings for the desired velocity
            quadController.setTargetVelocity(target_twist.twist);

            // Get the next uav command that is appropriate for the desired velocity
            iarc7_msgs::OrientationThrottleStamped uav_command;
            bool success = quadController.update(current_time, uav_command);
            ROS_ASSERT_MSG(success, "LowLevelMotion controller update failed");

            // Limit the uav command with the twist limiter before sending the uav command
            limitUavCommand(limiter, uav_command);

            // Publish the desired angles and throttle to the topic
            uav_control_.publish(uav_command);
        }

        // Handle all ROS callbacks
        ros::spinOnce();
    }

    // All is good.
    return 0;
}