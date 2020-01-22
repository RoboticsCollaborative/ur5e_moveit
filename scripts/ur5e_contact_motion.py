#!/usr/bin/env python

# Import from system
import rospy
import time
import numpy as np
import sys

# Import from MoveIt (ROS) driver
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from controller_manager_msgs.srv import ListControllers, LoadController

# Import from ur5e driver
from ur5e_controller import Arm


def main():
    try:
        print 'OK'

    except rospy.ROSInterruptException:
        return

    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
