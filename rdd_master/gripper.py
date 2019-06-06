#!/usr/bin/env python
import rospy
from rdda.msg import JointCommands
from rdda.msg import JointStates

class Gripper:

    def __init__(self):
        """Constructor."""

        self.gripperSub = rospy.Subscriber('rdd/joint_stats', JointStates, self.updateJointStates)
        self.gripperPub = rospy.Publisher('rdd/joint_cmds', JointCommands, queue_size=1)

        self.status = None

    def updateJointStates(self, states):
        """Obtain the status of the gripper."""
        self.status = states
        rospy.loginfo("Actual position[0]: %lf", self.status.act_pos[0])

    def setGripperVel(self, ang_vel):
        """Set angular velocity """
        self.gripperPub.tg_pos = ang_vel

"""
    def closeGripper(self):
        print 'Closing gripper ...'
        cmd = JointCommands()
        cmd.tg_pos[0] = 0
        cmd.tg_pos[1] = 0
        self.gripperPub.publish(cmd)
        rospy.sleep(0.5)

    def openGripper(self):
        print 'Open gripper ...'
        cmd = JointCommands()
        cmd.tg_pos[0] = -0.5
        cmd.tg_pos[1] = -0.5
        rospy.sleep(0.5)
"""



