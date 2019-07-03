#!/usr/bin/env python
import rospy
import numpy as np
from rdda_interface.msg import JointCommands
from rdda_interface.msg import JointStates


class Gripper:

    def __init__(self):
        """Constructor."""

        self.gripperSub = rospy.Subscriber('rdd/joint_stats', JointStates, self.updateJointStates)
        self.gripperPub = rospy.Publisher('rdd/joint_cmds', JointCommands, queue_size=1)

        self.status = None

    def updateJointStates(self, states):
        """Obtain the status of the gripper."""
        self.status = states
        #rospy.loginfo("ref_pos[0]: %lf", self.status.act_pos[0])

