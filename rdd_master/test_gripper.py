#!/usr/bin/env python
import rospy
import numpy as np
from gripper import Gripper


def main():

    try:
        rospy.init_node('gripper_node', anonymous=True)
        rospy.Rate(500)
        run()
    except rospy.ROSInterruptException:
        pass


def run():
    gripper = Gripper()
    time_interval = 0.0

    while not rospy.is_shutdown():
        vel_ref = -4.0 * np.sin(time_interval)
        time_interval += 0.5e-3
        gripper.setGripperVel(vel_ref)


if __name__ == '__main__':
    main()
