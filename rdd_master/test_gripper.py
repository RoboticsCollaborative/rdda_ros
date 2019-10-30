#!/usr/bin/env python
import rospy
import numpy as np
from gripper import Gripper
from rdda_interface.msg import JointCommands


def main():
    gripper = Gripper()
    rospy.init_node('gripper_node', anonymous=True)
    rate = rospy.Rate(500)
    cmds = JointCommands()
#    time_interval = 0.0
#    vel_ref = np.array([0.0, 0.0])
    pos_ref = [0.0, 0.0]
    stiffness = [1.0, 1.0]

    while not rospy.is_shutdown():
#        vel_ref[0] = -4.0 * np.sin(time_interval)
#        vel_ref[1] = 0.0
#        time_interval += 2e-3
#        cmds.vel_sat = vel_ref
	cmds.pos_ref = pos_ref
	cmds.stiffness = stiffness
        gripper.gripperPub.publish(cmds)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
