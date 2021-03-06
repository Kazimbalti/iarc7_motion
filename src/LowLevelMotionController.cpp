////////////////////////////////////////////////////////////////////////////
//
// LowLevelMotionController
//
// This is the top level class for the velocity controller.
// It uses an MotionPointInterpolator to get velocities.
// Sends them to a QuadVelocityController who runs the PID loops that set the angles
// to output to the flight controller.
// The angles and throttle values are sent to a TwistLimiter class which limits the
// min, max, and max rate of change of those values.
// Finally the processed angles and throttle values are sent out onto a topic.
//
////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>

#include "actionlib/server/simple_action_server.h"
#include "dynamic_reconfigure/server.h"

#include "iarc7_motion/LandPlanner.hpp"
#include "iarc7_motion/LowLevelMotionConfig.h"
#include "iarc7_motion/QuadVelocityController.hpp"
#include "iarc7_motion/QuadTwistRequestLimiter.hpp"
#include "iarc7_motion/TakeoffController.hpp"
#include "iarc7_motion/ThrustModel.hpp"

#include "ros_utils/ParamUtils.hpp"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"

#include "iarc7_motion/GroundInteractionAction.h"

using namespace Iarc7Motion;
using geometry_msgs::TwistStamped;
using geometry_msgs::Twist;

typedef actionlib::SimpleActionServer<iarc7_motion::GroundInteractionAction> Server;

enum class MotionState { TAKEOFF,
                         LAND,
                         VELOCITY_CONTROL,
                         GROUNDED,
                         PASSTHROUGH };

// This is a helper function that will limit a
// iarc7_msgs::OrientationThrottleStamped using the twist limiter
//
// The twist limiter uses TwistStamped messages to do its work so this
// function converts between the data types.
void limitUavCommand(QuadTwistRequestLimiter& limiter,
                     iarc7_msgs::OrientationThrottleStamped& uav_command)
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

// Main entry point for the low level motion controller
int main(int argc, char **argv)
{
    // Required by ROS before calling many functions
    ros::init(argc, argv, "Low_Level_Motion_Control");

    ROS_INFO("Low_Level_Motion_Control begin");

    // Create a node handle for the node
    ros::NodeHandle nh;
    // This node handle has a specific namespace that allows us to easily
    // encapsulate parameters
    ros::NodeHandle private_nh ("~");

    // LOAD PARAMETERS
    const size_t pid_param_array_size = 6;
    double throttle_pid[pid_param_array_size];
    double pitch_pid[pid_param_array_size];
    double roll_pid[pid_param_array_size];
    double position_p[3];
    double yaw_p;

    ThrustModel thrust_model(private_nh, "thrust_model");
    ThrustModel thrust_model_side;
    if(ros_utils::ParamUtils::getParam<std::string>(
              private_nh,
              "xy_mixer")
                    == "6dof") {
        thrust_model_side.loadModel(private_nh, "thrust_model_side");
    }

    double battery_timeout;
    Twist min_velocity, max_velocity, max_velocity_slew_rate;
    double update_frequency;

    // Set up dynamic reconfigure
    bool dynamic_reconfigure_called = false;
    dynamic_reconfigure::Server<iarc7_motion::LowLevelMotionConfig> dynamic_reconfigure_server;
    boost::function<void(iarc7_motion::LowLevelMotionConfig &config,
                         uint32_t level)> dynamic_reconfigure_settings_callback =
        [&](iarc7_motion::LowLevelMotionConfig &config, uint32_t) {

            throttle_pid[0] = config.throttle_p;
            throttle_pid[1] = config.throttle_i;
            throttle_pid[2] = config.throttle_d;
            throttle_pid[3] = config.throttle_accumulator_max;
            throttle_pid[4] = config.throttle_accumulator_min;
            throttle_pid[5] = config.throttle_accumulator_enable_threshold;

            pitch_pid[0] = config.pitch_p;
            pitch_pid[1] = config.pitch_i;
            pitch_pid[2] = config.pitch_d;
            pitch_pid[3] = config.pitch_accumulator_max;
            pitch_pid[4] = config.pitch_accumulator_min;
            pitch_pid[5] = config.pitch_accumulator_enable_threshold;

            roll_pid[0] = config.roll_p;
            roll_pid[1] = config.roll_i;
            roll_pid[2] = config.roll_d;
            roll_pid[3] = config.roll_accumulator_max;
            roll_pid[4] = config.roll_accumulator_min;
            roll_pid[5] = config.roll_accumulator_enable_threshold;

            position_p[0] = config.position_p_x;
            position_p[1] = config.position_p_y;
            position_p[2] = config.position_p_z;

            yaw_p = config.yaw_p;

            dynamic_reconfigure_called = true;
        };
    dynamic_reconfigure_server.setCallback(dynamic_reconfigure_settings_callback);

    // Battery timeout setting
    ROS_ASSERT(private_nh.getParam("battery_timeout", battery_timeout));

    // Throttle Limit settings retrieve
    private_nh.param("throttle_max", max_velocity.linear.z, 0.0);
    private_nh.param("throttle_min", min_velocity.linear.z, 0.0);
    private_nh.param("throttle_max_rate", max_velocity_slew_rate.linear.z, 0.0);

    // Pitch Limit settings retrieve
    private_nh.param("pitch_max", max_velocity.angular.y, 0.0);
    private_nh.param("pitch_min", min_velocity.angular.y, 0.0);
    private_nh.param("pitch_max_rate", max_velocity_slew_rate.angular.y, 0.0);

    // Roll Limit settings retrieve
    private_nh.param("roll_max", max_velocity.angular.x, 0.0);
    private_nh.param("roll_min", min_velocity.angular.x, 0.0);
    private_nh.param("roll_max_rate", max_velocity_slew_rate.angular.x, 0.0);

    // Yaw Limit settings retrieve
    private_nh.param("yaw_max", max_velocity.angular.z, 0.0);
    private_nh.param("yaw_min", min_velocity.angular.z, 0.0);
    private_nh.param("yaw_max_rate", max_velocity_slew_rate.angular.z, 0.0);

    // Update frequency retrieve
    private_nh.param("update_frequency", update_frequency, 60.0);

    ros::Rate limit_check_for_simulated_time = ros::Rate(30);
    // Wait for a valid time in case we are using simulated time (not wall time)
    // Also wait for dynamic reconfigure to be called once
    while (ros::ok() && (ros::Time::now() == ros::Time(0) || !dynamic_reconfigure_called)) {
        // wait
        ros::spinOnce();
        limit_check_for_simulated_time.sleep();
    }

    Server server(nh,
                  "ground_interaction_action",
                  false);
    server.start();

    // This assumes that we start on the ground
    MotionState motion_state = MotionState::GROUNDED;

    LandPlanner landPlanner(nh, private_nh);
    if (!landPlanner.waitUntilReady())
    {
        ROS_ERROR("Failed during initialization of LandPlanner");
        return 1;
    }

    // Create a quad velocity controller. It will output angles corresponding
    // to our desired velocity
    QuadVelocityController quadController(throttle_pid,
                                          pitch_pid,
                                          roll_pid,
                                          position_p,
                                          yaw_p,
                                          thrust_model,
                                          thrust_model_side,
                                          ros::Duration(battery_timeout),
                                          nh,
                                          private_nh,
                                          landPlanner);
    if (!quadController.waitUntilReady())
    {
        ROS_ERROR("Failed during initialization of QuadVelocityController");
        return 1;
    }

    TakeoffController takeoffController(nh, private_nh, thrust_model);
    if (!takeoffController.waitUntilReady())
    {
        ROS_ERROR("Failed during initialization of TakeoffController");
        return 1;
    }

    // Create the publisher to send the processed uav_commands out with
    // (angles, throttle)
    ros::Publisher uav_control_
        = nh.advertise<iarc7_msgs::OrientationThrottleStamped>(
                "uav_direction_command", 50);

    // Check for empty uav_control_ as per
    // http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers
    // section 1
    ROS_ASSERT_MSG(uav_control_,
                   "Could not create uav_direction_command publisher");

    // Create the twist limiter, it will limit min value, max value, and max
    // rate of change.
    QuadTwistRequestLimiter limiter(min_velocity,
                                    max_velocity,
                                    max_velocity_slew_rate);

    ros::Time passthrough_start_time;
    boost::shared_ptr<const iarc7_msgs::OrientationThrottleStamped> last_msg;
    boost::function<void(const boost::shared_ptr<const iarc7_msgs::OrientationThrottleStamped>&)> passthrough_callback =
        [&](const boost::shared_ptr<const iarc7_msgs::OrientationThrottleStamped>& msg) -> void {
            if (last_msg == nullptr || last_msg->header.stamp < msg->header.stamp) {
                last_msg = msg;
            } else {
                ROS_ERROR("Bad stamp on passthrough message");
            }
        };
    ros::Subscriber passthrough_sub = nh.subscribe("passthrough_command", 2, passthrough_callback);

    // Cache the time
    ros::Time last_time = ros::Time::now();

    ros::Rate rate (update_frequency);

    iarc7_msgs::OrientationThrottleStamped last_uav_command;

    // Run until ROS says we need to shutdown
    while (ros::ok())
    {
        // Get the time
        ros::Time current_time = ros::Time::now();

        // Make sure we don't call QuadVelocity controllers update unless we
        // have a new timestamp to give. This can be a problem with simulated
        // time that does not update with high precision.
        if(current_time > last_time)
        {
            last_time = current_time;

            //cancellation inputs
            if(server.isPreemptRequested()) {

                if(motion_state == MotionState::PASSTHROUGH) {
                    ROS_INFO("Transitioning out of PASSTHROUGH");
                    bool success = quadController.prepareForTakeover();
                    if (success) {
                        motion_state = MotionState::VELOCITY_CONTROL;
                        server.setSucceeded();
                    } else {
                        ROS_ERROR("Failed to transition to quadController from passthrough mode");
                        server.setAborted();
                    }
                }

                else if (motion_state == MotionState::LAND){
                    ROS_INFO("Transitioning out of LAND");
                    bool success = quadController.prepareForTakeover();
                    if (success) {
                        motion_state = MotionState::VELOCITY_CONTROL;
                        server.setSucceeded();
                    } else {
                        ROS_ERROR("Failed to transition to quadController from land mode");
                        server.setAborted();
                    }
                }
            }

            //new goal inputs
            if(server.isNewGoalAvailable() && !server.isActive())
            {
                const iarc7_motion::GroundInteractionGoalConstPtr& goal = server.acceptNewGoal();
                if ("takeoff" == goal->interaction_type)
                {
                    if (motion_state == MotionState::GROUNDED)
                    {
                        bool success = takeoffController.prepareForTakeover(current_time);
                        if(success)
                        {
                            ROS_INFO("Transitioning to takeoff mode");
                            motion_state = MotionState::TAKEOFF;
                        }
                        else
                        {
                            ROS_ERROR("Failure transitioning to takeoff mode");
                            server.setAborted();
                            motion_state = MotionState::GROUNDED;
                        }
                    }
                    else
                    {
                        ROS_ERROR("Attempt to take off without LLM in the ground state");
                        server.setAborted();
                    }
                }
                else if("land" == goal->interaction_type)
                {
                    if (motion_state == MotionState::VELOCITY_CONTROL || motion_state == MotionState::PASSTHROUGH)
                    {
                        bool success = landPlanner.prepareForTakeover(current_time);
                        if(success)
                        {
                            ROS_INFO("Transitioning to land mode");
                            motion_state = MotionState::LAND;
                        }
                        else
                        {
                            ROS_ERROR("Failure transitioning to land mode");
                            server.setAborted();
                            motion_state = MotionState::VELOCITY_CONTROL;
                        }
                    }
                    else
                    {
                        ROS_ERROR("Attempt to land off without LLM in the velocity control state");
                        server.setAborted();
                    }
                }
                else if ("passthrough" == goal->interaction_type)
                {
                    if (motion_state == MotionState::VELOCITY_CONTROL) {
                        ROS_INFO("Transitioning to passthrough mode");
                        passthrough_start_time = current_time;
                        motion_state = MotionState::PASSTHROUGH;
                    } else {
                        ROS_ERROR("Attempt to passthrough when not in velocity control state");
                        server.setAborted();
                    }
                }
                else
                {
                    ROS_ERROR("Unknown ground interaction type");
                    server.setAborted();
                }
            }

            iarc7_msgs::OrientationThrottleStamped uav_command;

            if(motion_state == MotionState::VELOCITY_CONTROL)
            {
                // Get the next uav command that is appropriate for the desired velocity
                bool success = quadController.update(current_time, uav_command, 0);

                ROS_ASSERT_MSG(success, "LowLevelMotion quad velocity controller update failed");
            }
            else if(motion_state == MotionState::TAKEOFF)
            {
                bool success = takeoffController.update(current_time, uav_command);
                ROS_ASSERT_MSG(success, "LowLevelMotion takeoff controller update failed");

                if(takeoffController.isDone())
                {
                    server.setSucceeded();
                    ThrustModel new_model = takeoffController.getThrustModel();
                    quadController.setThrustModel(new_model);
                    success = quadController.prepareForTakeover();
                    ROS_ASSERT_MSG(success, "LowLevelMotion switching to velocity control failed");
                    motion_state = MotionState::VELOCITY_CONTROL;
                    ROS_INFO("Finished takeoff, hover throttle is %f",
                             uav_command.throttle);
                }
            }
            else if(motion_state == MotionState::LAND)
            {
                // Get the next uav command that is appropriate for the desired velocity
                bool success = quadController.update(current_time, uav_command, 1);
                ROS_ASSERT_MSG(success, "LowLevelMotion quad velocity controller update failed");

                if(landPlanner.isDone())
                {
                    ROS_INFO("Land completed");

                    success = quadController.prepareForTakeover();
                    ROS_ASSERT_MSG(success, "LowLevelMotion switching to velocity control failed");
                    motion_state = MotionState::GROUNDED;
                }
            }
            else if(motion_state == MotionState::GROUNDED)
            {
                ROS_DEBUG("Low level motion is grounded");
            } else if (motion_state == MotionState::PASSTHROUGH) {
                ROS_ERROR("PASSTHROUGH MODE CALLED");
                // if (last_msg != nullptr && last_msg->header.stamp >= passthrough_start_time) {
                //     geometry_msgs::Twist twist;
                //     twist.linear.z = last_msg->throttle;

                //     MotionPointStamped motion_point;
                //     motion_point.motion_point.twist.linear.z = twist.linear.z;

                //     quadController.setTargetVelocity(motion_point);
                //     bool success = quadController.update(current_time,
                //                                          uav_command,
                //                                          true,
                //                                          last_msg->data.pitch,
                //                                          last_msg->data.roll);
                //     ROS_ASSERT_MSG(success, "LowLevelMotion quad velocity controller update failed");
                // } else {
                //     ROS_WARN("No recent passthrough messages available");
                //     uav_command = last_uav_command;
                // }
            }
            else
            {
                ROS_ASSERT_MSG(false, "Low level motion does not know what state to be in");
            }

            //ROS_ERROR_STREAM("Pre limiter: " << uav_command);
            // Limit the uav command with the twist limiter before sending the uav command
            limitUavCommand(limiter, uav_command);
            //ROS_ERROR_STREAM("Post limiter: " << uav_command);

            // Publish the desired angles and throttle to the topic
            uav_control_.publish(uav_command);

            last_uav_command = uav_command;
        }

        // Handle all ROS callbacks
        ros::spinOnce();
        rate.sleep();
    }

    // All is good.
    return 0;
}
