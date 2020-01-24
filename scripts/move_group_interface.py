import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


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


class MoveGroupInterfaceTutorial(object):

    def __init__(self):
        super(MoveGroupInterfaceTutorial, self).__init__()

        ## SETUP
        # First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_interface', anonymous=True)

        # Instantiate a `RobotCommander`_ object. This object is the outer-level interface to the robot
        robot = moveit_commander.RobotCommander

        # Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        # to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        # to one group of joints.  In this case the group is the joints in the UR5e
        # arm so we set ``group_name = manipulator``. If you are using a different robot,
        # you should change this value to the name of your robot arm planning group.
        # This interface can be used to plan and execute motions on the UR5e:
        group_name = "manipulator"
        group = moveit_commander.MoveGroupCommander(group_name)

        # We create a `DisplayTrajectory`_ publisher which is used later to publish
        # trajectories for RViz to visualize:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        ## BASIC_INFO
        # Getting basic information
        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        # We can get a list of all the groups in the robot:
        # group_names = robot.get_group_names()
        # print "============ Robot Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        # print "============ Printing robot state"
        # print robot.get_current_state()
        print ""

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        # self.group_names = group_names

    def go_to_joint_state(self):
        # We can get the joint values from the group and adjust some of the values:
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi / 4
        joint_goal[2] = 0
        joint_goal[3] = -pi / 2
        joint_goal[4] = 0
        joint_goal[5] = pi / 3
        # joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()

        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self):
        # We can plan a motion for this group to a desired pose for the
        # end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4
        self.group.set_pose_target(pose_goal)

        # Now, we call the planner to compute the plan and execute it.
        plan = self.group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()

        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        # You can plan a Cartesian path directly by specifying a list of waypoints
        # for the end-effector to go through:
        waypoints = []

        wpose = self.group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append((copy.deepcopy(wpose)))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = self.group.compute_cartesian_path(
                                                waypoints=waypoints,    # waypoints to follow
                                                eef_step=0.01,          # eef_step
                                                jump_threshold=0.0)     # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def display_trajectory(self, plan):
        # A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        # We populate the trajectory_start with our current robot state to copy over
        # any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        # Use execute if you would like the robot to follow
        # the plan that has already been computed:
        self.group.execute(plan)

        # **Note:** The robot's current joint state must be within some tolerance of the
        # first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail


def main():
    try:
        print "============ Press `Enter` to begin the tutorial by " \
              "setting up the moveit_commander (press ctrl-d to exit) ..."
        raw_input()
        interface = MoveGroupInterfaceTutorial()

        print "============ Press `Enter` to execute a movement using a joint state goal ..."
        raw_input()
        interface.go_to_joint_state()

        print "============ Press `Enter` to execute a movement using a pose goal ..."
        raw_input()
        interface.go_to_pose_goal()

        print "============ Press `Enter` to plan and display a Cartesian path ..."
        raw_input()
        cartesian_path, fraction = interface.plan_cartesian_path()

        print "============ Press `Enter` to plan and display a Cartesian path ..."
        raw_input()
        interface.display_trajectory(cartesian_path)

        print "============ Press `Enter` to execute a saved path ..."
        raw_input()
        interface.execute_plan(cartesian_path)

        print "============ Demo complete!"

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
