#!/usr/bin/env python

import rospy
from gripper import Gripper


def main():
    gripper = Gripper()
    rospy.init_node('close', anonymous=True)

    gripper.close()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
