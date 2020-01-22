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


def moveit_cart_plan_to_traj_list(plan):
    '''
    Iterates through a plan object to get just the position outputs
    Input:
        plan object from "compute_cartesian_path()"
    Output:
        plan_positions - list of joint positions to execute plan
    '''
    # Cycle through trajectory and extract positions.
    plan_positions = []  # List of tuples of joint positions
    points = plan.joint_trajectory.points
    for point in points:
        plan_positions.append(point.positions)
    return plan_positions


class MoveGroupInterface(object):

    def __init__(self):
        super(MoveGroupInterface, self).__init__()

        ### SETUP ###
        # First initialize `moveit_commander` and `rospy` node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur5e_interface', anonymous=True)

        # Instantiate a `RobotCommander` object.
        # This object is the outer-level interface to the robot.
        robot = moveit_commander.RobotCommander

        # Instantiate a `PlanningSceneInterface` object.
        # This object is an interface to the world surrounding the robot.
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander` object.  This object is an interface
        # to one group of joints.  In this case the group is the joints in the UR5e
        # arm so we set ``group_name = manipulator``. If you are using a different robot,
        # you should change this value to the name of your robot arm planning group.
        # This interface can be used to plan and execute motions on the UR5e:
        group_name = "manipulator"
        group = moveit_commander.MoveGroupCommander(group_name)

        # Create a `DisplayTrajectory` publisher which is used later to publish
        # trajectories for RViz to visualize:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        ### BASIC INFO ###
        # Getting basic information
        # Get the name of the reference frame for this robot.
        planning_frame = group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # Print the name of the end-effector link for this group.
        eef_link = group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        # Get a list of all the groups in the robot.
        # group_names = robot.get_group_names()
        # print "============ Robot Groups: ", robot.get_group_names()

        # Print the entire state of the robot for debugging.
        # print "============ Printing robot state"
        # print robot.get_current_state()
        # print ""

        # Misc variable
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        # self.group_names = group_names
