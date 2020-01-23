#!/usr/bin/env python

import rospy
import time
import numpy as np
import sys

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from controller_manager_msgs.srv import ListControllers, LoadController

from ur5e_controller import Arm
import ur5e_moveit_interface
from ur5e_moveit_interface import all_close
from ur5e_moveit_interface import MoveGroupInterface
from ur5e_moveit_interface import moveit_cart_plan_to_traj_list


def main():

    # Preset joint configurations for start and safe positions.
    safe_position = [0.0, -1.0, 1.0, 0.0, np.pi/2, 0.0]  # Neutral, above the table
    pos_A = [0.40261, -0.9254, 1.70722, -0.78288, 1.97121, 0.00194]
    pos_B = [-0.87946, -0.93835, 1.73203, -0.79074, 0.68918, 0.00084]

    # Start controller service.
    # rospy.wait_for_service('/controller_manager/list_controllers')
    # list_controllers = rospy.ServiceProxy('/controller_manager/list_controllers',
    #                                       ListControllers)

    # Velocity controller class.
    arm = Arm()
    # Move group, moveit abstraction class.
    mGroup = MoveGroupInterface()
    # Set moveit velocity.
    mGroup.group.set_max_velocity_scaling_factor(0.1)

    # Go to safe position using moveit.
    raw_input('Hit enter to go to safe position.')
    print('')

    # Call the planar to compute and execute.
    mGroup.group.go(safe_position, wait=True)
    # Calling `stop()` ensures that there is no residual movement.
    mGroup.group.stop()
    # It is always good to clear your targets after planning with poses.
    # A short delay to ensure a stop state.
    # Note: there is no equivalent function for clear_joint_value_targets().
    time.sleep(0.5)

    # Start position using MoveIt.
    raw_input('Hit enter to continue to start position')
    print('')
    mGroup.group.go(pos_A, wait=True)
    mGroup.group.stop()
    time.sleep(0.5)

    ### STEP1: INITIALIZE AND MOVE FORWARD ###

    # Move forward with velocity control.
    pose_goal = mGroup.group.get_current_pose().pose
    pose_goal.position.y -= 0.4
    raw_input('Hit enter to move forward') #wait for user input

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    # Execute planned positions with velocity control
    (plan, fraction) = mGroup.group.compute_cartesian_path([pose_goal], 0.01, 0.0)
    if fraction == 1.0:
        plan_positions = moveit_cart_plan_to_traj_list(plan) #extract positions
        arm.followTrajectory(plan_positions, gain=4.0, maxDistToTarget = 0.005, gamma=0.97)
    else:
        print('Trajectory not feasible. Fraction: {}'.format(fraction))
    time.sleep(0.5)

    ### STEP2: MOVE BACK TO SAFE POSITION ###

    # Move back to safe position.
    # raw_input('Hit enter to continue to safe position')
    # print('')
    # mGroup.group.go(safe_position, wait=True)
    # mGroup.group.stop()
    # time.sleep(0.5)

    # Move to start position for a next turn.
    # raw_input('Hit enter to continue to start position')
    print('')
    mGroup.group.go(pos_A, wait=True)
    mGroup.group.stop()
    time.sleep(0.5)

    print('Done')


if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass

    # except KeyboardInterrupt:
    #     pass
