#!/usr/bin/env python
import rospy
from gripper import Gripper


def main():

    gripper = Gripper()

    while not rospy.is_shutdown():
        err = gripper.closeGripper()
        if err:
            exit(-1)


if __name__ == '__main__':
    main()
