#!/usr/bin/env python

'''Inteded as a simple test moving from a to b using velocity control'''

import rospy
import time
import numpy as np
import sys
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
# from math import pi
from std_msgs.msg import String
from controller_manager_msgs.srv import ListControllers, LoadController


from tf.transformations import rotation_matrix
import trajectory_msgs.msg as traj_msgs

from ur5e_controller import Arm

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupTest(object):
    '''Tests cartesian path generation'''

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_test', anonymous = True)
        robot = moveit_commander.RobotCommander()
        ## Instantiate a `PlanningSceneInterface`_ object.
        # scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        group = moveit_commander.MoveGroupCommander(group_name)

        planning_frame = group.get_planning_frame()
        eef_link = group.get_end_effector_link()
        group_names = robot.get_group_names()
        # print()

        self.robot = robot
        # self.scene = scene
        self.group = group
        # self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        joint_goal = self.group.get_current_joint_values()
        print('Joints:')
        print(joint_goal)

        joint_goal[5] = joint_goal[5] + 8 #should rotate a small amount

        self.group.go(joint_goal, wait=True)
        self.group.stop() #ensures no ending movement

        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose(self):
        '''given a pose, moves the ur5e to the new pose (if feasible)'''

        #get current pose
        pose = self.group.get_current_pose().pose

        print('Pose:')
        print(pose)
        pose.position.z = pose.position.z + 0.01
        self.group.set_pose_target(pose)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets() #good practice
        current_pose = self.group.get_current_pose().pose
        return all_close(pose, current_pose, 0.01)

    def go(pose_goal):
        '''Takes a pose and computes a linear path and executes it'''
        (plan, fraction) = mGroup.group.compute_cartesian_path([pose_goal], 0.05, 0.0)
        if fraction == 1.0:
            plan_positions = moveit_cart_plan_to_traj_list(plan) #extract positions
            arm.followTrajectory(plan_positions, gain=2.0, maxDistToTarget = 0.005, gamma=0.97)
        else:
            print('Trajectory not feasible. Fraction: {}'.format(fraction))

def moveit_cart_plan_to_traj_list(plan):
    '''Iterates through a plan object to get just the position outputs
    Input:
        plan object from "compute_cartesian_path()"
    Output:
        plan_positions - list of joint positions to execute plan '''

    #cycle through trajectory and extract positions
    plan_positions = [] #list of tuples of joint positions
    points = plan.joint_trajectory.points
    for point in points:
        plan_positions.append(point.positions)
    return plan_positions


def main():
    '''Move the arm a short distance'''

    #preset joint configurations for start and safe psitions
    safe_position = [0.0, -1.0, 1.0, 0.0, np.pi/2, 0.0] #neutral, above the table
    pos_A = [0.40261, -0.9254, 1.70722, -0.78288, 1.97121, 0.00194]
    pos_B = [-0.87946, -0.93835, 1.73203, -0.79074, 0.68918, 0.00084]

    #start controller service
    rospy.wait_for_service('/controller_manager/list_controllers')
    list_controllers = rospy.ServiceProxy('/controller_manager/list_controllers',
                                            ListControllers)
    """To turn the robot back on use the '/ur_hardware_interface/resend_robot_program'
    service. Note, I could not get this to work"""

    #velocity controller class
    arm = Arm()
    #move group, moveit abstraction class
    mGroup = MoveGroupTest()
    #set moveit velocity
    mGroup.group.set_max_velocity_scaling_factor(0.1)

    # print(mGroup.group.get_current_joint_values())
    # pose_goal = mGroup.group.get_current_pose().pose
    # pose_goal.position.x -= 0.1
    # mGroup.group.set_pose_target(pose_goal)
    # plan = mGroup.group.go(wait=True)
    # mGroup.group.stop()
    # mGroup.group.clear_pose_targets()
    # print('At target')

    # go to safe position using moveit
    raw_input('Hit enter to go to safe position.')
    print('')
    # current_joints = mGroup.group.get_current_joint_values()
    # traj = [current_joints, safe_position]
    # arm.followTrajectory(traj, gain=2.0, maxDistToTarget = 0.005, gamma=0.97)
    mGroup.group.go(safe_position, wait=True) #plan and execute
    mGroup.group.stop() #ensure a stoped state (good practice)
    time.sleep(0.5)

    #start position uing moveit
    raw_input('Hit enter to continue to start position')
    print('')
    mGroup.group.go(pos_A, wait=True)
    mGroup.group.stop()
    time.sleep(0.5)

    #move forward with velocity control
    pose_goal = mGroup.group.get_current_pose().pose
    pose_goal.position.y -= 0.4
    raw_input('Hit enter to move forward') #wait for user input
    (plan, fraction) = mGroup.group.compute_cartesian_path([pose_goal], 0.01, 0.0)
    if fraction == 1.0:
        plan_positions = moveit_cart_plan_to_traj_list(plan) #extract positions
        arm.followTrajectory(plan_positions, gain=4.0, maxDistToTarget = 0.005, gamma=0.97)
    else:
        print('Trajectory not feasible. Fraction: {}'.format(fraction))

    # #cycle through trajectory and extract positions
    # plan_positions = [] #list of tuples of joint positions
    # points = plan.joint_trajectory.points
    # for point in points:
    #     plan_positions.append(point.positions)
    print('Done')


    # mGroup.group.execute(plan, wait=True)
    # # mGroup.group.set_pose_target(pose_goal)
    # # plan = mGroup.group.go(wait=True)
    # mGroup.group.stop()
    # mGroup.group.clear_pose_targets()

    # mGroup.group.set_max_velocity_scaling_factor(0.01)
    # mGroup.group.execute(plan, wait=True)
    # print(mGroup.group.get_jacobian_matrix())





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
