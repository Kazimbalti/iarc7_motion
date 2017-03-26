#!/usr/bin/env python

import sys
import traceback
import actionlib
import rospy

from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool

from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_msgs.msg import TwistStampedArrayStamped
from iarc7_safety.SafetyClient import SafetyClient

from iarc_task_action_server import IarcTaskActionServer

import iarc_tasks.task_states as task_states
import iarc_tasks.task_commands as task_commands

class MotionPlanner:

    def __init__(self, _action_server, update_rate):
        self._action_server = _action_server
        self._update_rate = update_rate
        self._task = None
        self._velocity_pub = rospy.Publisher('movement_velocity_targets',   
                                             TwistStampedArrayStamped,
                                             queue_size=0)
        self._arm_service = rospy.ServiceProxy('uav_arm', SetBool)
        self._safety_client = SafetyClient('motion_planner')
        self._safety_land_complete = False
        self._safety_land_requested = False
        # Create action client to request a landing
        self._action_client = actionlib.SimpleActionClient(
                                        "motion_planner_server",
                                        QuadMoveAction)

        self._command_implementations = {
            task_commands.VelocityCommand: self._handle_velocity_command,
            task_commands.ArmCommand: self._handle_arm_command,
            task_commands.NopCommand: self._handle_nop_command
            }

    def run(self):
        rate = rospy.Rate(self._update_rate)

        rospy.logwarn('trying to form bond')
        if not self._safety_client.form_bond():
            rospy.logerr('Motion planner could not form bond with safety client')
            return
        rospy.wait_for_service('uav_arm')
        rospy.logwarn('done forming bond')

        while not rospy.is_shutdown():

            # Exit immediately if fatal
            if self._safety_client.is_fatal_active() or self._safety_land_complete:
                return

            # Land if put into safety mode
            if self._safety_client.is_safety_active() and not self._safety_land_requested:
                # Request landing
                goal = QuadMoveGoal(movement_type="land", preempt=True)
                self._action_client.send_goal(goal,
                        done_cb=self._safety_task_complete_callback)
                rospy.logwarn(
                        'motion planner attempting to execute safety land')
                self._safety_land_requested = True

            task_commands = self._get_task_command()
            for task_command in task_commands:
                try:
                    self._command_implementations[type(task_command)](task_command)
                except KeyError as e:
                    rospy.logerr("Task requested unimplemented command, noping: %s", type(task_command))
                    self._handle_nop_command(None)

            rate.sleep()

    def _handle_nop_command(self, nop_command):
        velocity = TwistStamped()
        velocity.header.frame_id = 'level_quad'
        velocity.header.stamp = rospy.Time.now()
        self._publish_twist(velocity)

    def _handle_velocity_command(self, velocity_command):
        self._publish_twist(velocity_command.target_twist)

    def _handle_arm_command(self, arm_command):
        arm_result = self._use_arm_service(arm_command.arm_state)
        self._call_tasks_arm_service_callback(arm_command.completion_callback, arm_result)

    def _call_tasks_arm_service_callback(self, callback, arm_result):
        try:
            callback(arm_result)
        except Exception as e:
            rospy.logerr('Exception setting result using an arm callback')
            rospy.logerr(str(e))
            rospy.logerr(traceback.format_exc())
            rospy.logwarn('Motion planner aborted task')
            self._action_server.set_aborted()
            self._task = None

    def _use_arm_service(self, arm):
        try:
            armed = self._arm_service(arm)
        except rospy.ServiceException as exc:
            rospy.logerr("Could not arm: " + str(exc))
            armed = False
        return armed

    def _publish_twist(self, twist):
        velocity_msg = TwistStampedArrayStamped()
        velocity_msg.header.stamp = rospy.Time.now()
        velocity_msg.data = [twist]
        self._velocity_pub.publish(velocity_msg)

    def _safety_task_complete_callback(self, status, response):
        if response.success:
            rospy.logwarn('Motion planner supposedly safely landed the aircraft')
        else:
            rospy.logerr('Motion planner did not safely land aircraft')
        self._safety_land_complete = True

    def _get_task_command(self):
        if (self._task is None) and self._action_server.has_new_task():
            self._task = self._action_server.get_new_task()

        if self._task:
            if self._action_server.is_canceled():
                try:
                    self._task.cancel()
                except Exception as e:
                    rospy.logerr('Exception canceling task')
                    rospy.logerr(str(e))
                    rospy.logerr(traceback.format_exc())
                    rospy.logwarn('Motion planner aborted task')
                    self._action_server.set_aborted()
                    self._task = None
                    return (task_commands.NopCommand(),)

            try:
                task_request = self._task.get_desired_command()
            except Exception as e:
                rospy.logerr('Exception getting tasks preferred velocity')
                rospy.logerr(str(e))
                rospy.logerr(traceback.format_exc())
                rospy.logwarn('Motion planner aborted task')
                self._action_server.set_aborted()
                self._task = None
                return (task_commands.NopCommand(),)
            
            task_state = task_request[0]
            if isinstance(task_state, task_states.TaskCanceled):
                self._action_server.set_canceled()
                self._task = None
            elif isinstance(task_state, task_states.TaskAborted):
                rospy.logwarn('Task aborted with: %s', task_state.msg)
                self._action_server.set_aborted()
                self._task = None
            elif isinstance(task_state, task_states.TaskFailed):
                rospy.logwarn('Task failed with: %s', task_state.msg)
                self._action_server.set_succeeded(False)
                self._task = None
            elif isinstance(task_state, task_states.TaskDone):
                self._action_server.set_succeeded(True)
                self._task = None
            else:
                assert isinstance(task_state, task_states.TaskRunning)
                return task_request[1:]
        # No action to take return a nop
        return (task_commands.NopCommand(),)

if __name__ == '__main__':
    rospy.init_node('motion_planner')
    action_server = IarcTaskActionServer()

    update_rate = rospy.get_param('~update_rate', False)

    motion_planner = MotionPlanner(action_server, update_rate)
    try:
        motion_planner.run()
    except Exception, e:
        rospy.logfatal("Error in motion planner while running.")
        rospy.logfatal(str(e))
        raise
    finally:
        rospy.signal_shutdown("Motion Planner shutdown")