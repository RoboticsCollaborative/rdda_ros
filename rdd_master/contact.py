#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from rdda_interface.msg import ControlState

from rdda_interface.srv import SetStiffness
from rdda_interface.srv import SetMaxVelocity
from rdda_interface.srv import SetMaxEffort

import time
import numpy as np

from RddaProxy import RddaProxy


def main():
    rdda = RddaProxy()
    rospy.init_node('contact', anonymous = True)

    # rdda.homing()
    # time.sleep(1)

    # rdda.joint_origins = rospy.get_param('/rdda_interface/origins')
    # rdda.joint_lower_bounds = rospy.get_param('/rdda_interface/lower_bounds')
    # rdda.joint_upper_bounds = rospy.get_param('/rdda_interface/upper_bounds')


    rate = rospy.Rate(20)

    ### Harmonic wave ###
    rdda.harmonic_wave()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass