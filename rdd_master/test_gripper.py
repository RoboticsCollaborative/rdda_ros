#!/usr/bin/env python
import rospy
import numpy as np
from gripper import Gripper
from rdda_interface.msg import JointCommands


def main():
    gripper = Gripper()
    rospy.init_node('gripper_node', anonymous=True)
    rate = rospy.Rate(500)
    joint_cmds = JointCommands()
    time_interval = 0.0
    pos_ref = np.array([0.0, 0.0])
    vel_sat = np.array([3.0, 3.0])
    tau_sat = np.array([5.0, 5.0])
    stiffness = np.array([0.1, 0.1])
    freq_anti_alias = 500.0

    while not rospy.is_shutdown():
        time_interval += 2e-3
        pos_ref[0] = -1.0 * np.sin(time_interval)
        joint_cmds.pos_ref = pos_ref
        joint_cmds.vel_sat = vel_sat
        joint_cmds.tau_sat = tau_sat
        joint_cmds.stiffness = stiffness
        joint_cmds.freq_anti_alias = freq_anti_alias
        gripper.gripperPub.publish(joint_cmds)
        rospy.loginfo("pos_ref[0]: {}".format(joint_cmds.pos_ref[0]))
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
