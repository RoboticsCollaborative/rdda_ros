#!/usr/bin/env python
import rospy
import numpy as np

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from rdda_interface.msg import ControlState


class Gripper:

    def __init__(self):
        """Constructor."""

        # self.gripperSub = rospy.Subscriber('rdd/joint_stats', JointStates, self.updateJointStates)
        # self.gripperPub = rospy.Publisher('rdd/joint_cmds', JointCommands, queue_size=1)
        self.joint_pub = rospy.Publisher("rdda_interface/joint_cmds", JointTrajectoryPoint, queue_size=1)
        self.joint_sub = rospy.Subscriber("rdda_interface/joint_states", JointState, self.subjointstates_callback)
        self.ctrl_sub = rospy.Subscriber("rdda_interface/ctrl_states", ControlState, self.subctrlstates_callback)

        self.has_states_msg = False
        self.has_ctrl_msg = False

        """ Joint states """
        self.actual_positions = [0.0, 0.0]
        self.actual_velocities = [0.0, 0.0]
        self.external_efforts = [0.0, 0.0]
        self.applied_efforts = [0.0, 0.0]
        self.ts_nsec = 0.0
        self.ts_sec = 0.0

        self.joint_upper_bounds = [0.0, 0.0]
        self.joint_lower_bounds = [0.0, 0.0]
        self.joint_origins = [0.0, 0.0]

    def subjointstates_callback(self, joint_states_msg):
        self.has_states_msg = True
        self.actual_positions = joint_states_msg.position
        self.actual_velocities = joint_states_msg.velocity
        self.external_efforts = joint_states_msg.effort
        """ Ignore applied effort and time from ControlState. """
        # self.ts_nsec = JointStates_msg.header

    def subctrlstates_callback(self, ctrl_states_msg):
        self.has_ctrl_msg = True
        self.applied_efforts = ctrl_states_msg.applied_effort

    def set_positions(self, positions=(0.0, 0.0)):
        joint_cmds_msg = JointTrajectoryPoint()
        joint_cmds_msg.positions = positions
        self.joint_pub.publish(joint_cmds_msg)

    """ Fingers go back to origin. """
    # def close(self):
    #     has_home = rospy.get_param('/has_home')
    #     if has_home:
    #         origin = rospy.get_param('/origins')
    #         self.set_positions(positions=origin)
    #     else:
    #         print("Need home.")
    def close(self):
        has_home = rospy.get_param('/has_home')
        if has_home:
            # print("has_home: {}".format(has_home))
            origin = rospy.get_param('/origins')
            print("origins: {}".format(origin))
        else:
            print("Need home.")
