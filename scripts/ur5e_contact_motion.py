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

from std_srvs.srv import Trigger

from ur5e_controller_contact import Arm
import ur5e_moveit_interface
from ur5e_moveit_interface import all_close
from ur5e_moveit_interface import MoveGroupInterface
from ur5e_moveit_interface import moveit_cart_plan_to_traj_list
from ur5e_moveit_interface import reconnect_ur_driver


def main():

    ### MAKE SURE UR_DRIVER CONNECTED ###
    reconnect_ur_driver()
    time.sleep(0.5)

    # Preset joint configurations for start and safe positions.
    safe_position = [0.0, -1.0, 1.0, 0.0, np.pi/2, 0.0] #neutral, above the table
    # pos_A = [0.40261, -0.9254, 1.70722, -0.78288, 1.97121, 0.00194]
    # pos_B = [-0.87946, -0.93835, 1.73203, -0.79074, 0.68918, 0.00084]
    pose_A_x = 0.538791827062
    pose_A_y = 0.332075010152
    pose_A_z = 0.126178557703 + 0.05
    offset = 0.4

    # Start orientation
    # x: -0.999998581591
    # y: -0.000991367569636
    # z: -0.000602488281656
    # w: 0.00122107108858

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
    # mGroup.group.go(pos_A, wait=True)
    # mGroup.group.stop()
    pose_goal = mGroup.group.get_current_pose().pose
    pose_goal.position.x = pose_A_x
    pose_goal.position.y = pose_A_y
    pose_goal.position.z = pose_A_z
    start_time = time.time()
    (plan, fraction) = mGroup.group.compute_cartesian_path([pose_goal], 0.001, 0.0)
    print('Planning Took {} seconds'.format(time.time()-start_time))
    mGroup.group.execute(plan, wait=True)
    mGroup.group.stop()
    time.sleep(0.5)

    ### STEP1: INITIALIZE AND MOVE FORWARD ###

    # Move forward with velocity control.
    pose_goal = mGroup.group.get_current_pose().pose
    pose_goal.position.y -= offset
    raw_input('Hit enter to move forward') #wait for user input

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    # Execute planned positions with velocity control
    (plan, fraction) = mGroup.group.compute_cartesian_path([pose_goal], 0.01, 0.0)
    if fraction == 1.0:
        plan_positions = moveit_cart_plan_to_traj_list(plan) #extract positions
        arm.followTrajectory(plan_positions, gain=5.0, maxDistToTarget = 0.005, gamma=0.97)
    else:
        print('Trajectory not feasible. Fraction: {}'.format(fraction))
    time.sleep(0.1)

    ### Reconnect ur_driver ###
    reconnect_ur_driver()
    time.sleep(0.5)

    ### STEP2: MOVE BACK TO SAFE POSITION ###

    # Move to start position for a next turn.
    # raw_input('Hit enter to continue to start position')
    # print('')
    # mGroup.group.go(pos_A, wait=True)
    # mGroup.group.stop()
    # time.sleep(0.5)

    # pose_goal = mGroup.group.get_current_pose().pose
    pose_goal.position.x = pose_A_x
    pose_goal.position.y = pose_A_y
    pose_goal.position.z = pose_A_z
    (plan, fraction) = mGroup.group.compute_cartesian_path([pose_goal], 0.001, 0.0)
    print('Planning Took {} seconds'.format(time.time()-start_time))
    mGroup.group.execute(plan, wait=True)
    mGroup.group.stop()
    time.sleep(0.5)

    # Move back to safe position.
    raw_input('Hit enter to continue to safe position')
    print('')
    mGroup.group.go(safe_position, wait=True)
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
