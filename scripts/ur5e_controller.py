#!/usr/bin/env python

"""Custom velocity controller for UR5e"""

from time import time

import numpy as np
import rospy
import roslib; roslib.load_manifest('ur_driver')
import sensor_msgs.msg as sensor_msgs
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import geometry_msgs.msg as geometry_msgs

# ur_kinematics (IK)
#import ur_kin_py
#from ur_kinematics import ur_kin_py

class Arm:

    def __init__(self):
        '''Create arm object'''

        # basic assignments
        self.endEffector =  "ee_link"

        # joint values and joint limits
        self.jointValues = [0] * 6
        self.joint_velocities = [0] * 6
        self.joint_names = [''] * 6
        #TODO set joint limmits from movit
        self.joint_limits = np.zeros((2,6))
        self.joint_limits[0,:] = -np.pi
        self.joint_limits[1,:] = np.pi
        # self.robot.SetDOFLimits(self.joint_limits[0], self.joint_limits[1])

        # Ordering of joints.
        self.joint_names_speedj = ['shoulder_pan_joint', 'shoulder_lift_joint', \
           'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        # subscribers
        self.joints_sub = rospy.Subscriber("/joint_states", sensor_msgs.JointState, self.jointsCallback)
        self.joints_sub = rospy.Subscriber("/wrench", geometry_msgs.WrenchStamped, self.forceCallback)
        self.useURScript = True
        self.pub = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=10)

        #no collision checking
        self.obstacleCloud = None

        # register callback for CTRL+C
        rospy.on_shutdown(self.shutdownCallback)

    def calcFK(self, config, endEffectorName=None):
        '''Calculate the Cartesian poses for a given collection of joint positions.'''

        if endEffectorName is None:
          endEffectorName = self.endEffector

        self.robot.SetDOFValues(config, self.manip.GetArmIndices())
        return self.robot.GetLink(endEffectorName).GetTransform()

#    def calcIKForT(self, T, ignoreHandCollisions=False, forceOpenrave=False):
#        '''Uses UR IK solvers to find solutions.'''
#
#        # Call the IK solver in universal_robot/ur_kinematics
#        sols = ur_kin_py.inverse(np.ascontiguousarray(T), 0.0)
#
#        # Check for joint limits and collisions
#        valid = []
#        for i, sol in enumerate(sols):
#            if np.isnan(sol[0]): continue #ignore null solutions
#
#            # wrap joint positions such that they are within the limits.
#            larger2np.pi = np.logical_and(sol > self.joint_limits[1, :], sol - 2 * np.pi > self.joint_limits[0, :])
#            sol[np.where(larger2np.pi)] -= 2 * np.pi
#            smaller2np.pi = np.logical_and(sol < self.joint_limits[0, :], sol + 2 * np.pi < self.joint_limits[1, :])
#            sol[np.where(smaller2np.pi)] += 2 * np.pi
#
#            # Check if the config is within the joint limits.
#            isInLimits = self.isInJointLimits(sol)
#
#            if isInLimits:
#                valid.append(sol)
#
#        # Return results, if any.
#        return valid

    def findClosestIK(self, solutions):
        '''Find the joint positions closest to the current joint positions.'''

        return self.findClosestIKToGiven(solutions, self.jointValues)

    def findClosestIKToGiven(self, solutions, jointsStart):
        '''Find the joint positions closest to the given joint positions.'''

        closestSolution = None
        minDist = float('inf')
        for solution in solutions:
            diff = jointsStart - solution
            dist = np.dot(diff, diff)
            if dist < minDist:
                minDist = dist
                closestSolution = solution
        return closestSolution

    def publishVelocities(self, velocities_dict):
        '''Sends velocity message to arm '''
        a = 0.1
        t = 0.1
        vels = [velocities_dict['shoulder_pan_joint'],
                velocities_dict['shoulder_lift_joint'],
                velocities_dict['elbow_joint'],
                velocities_dict['wrist_1_joint'],
                velocities_dict['wrist_2_joint'],
                velocities_dict['wrist_3_joint']]
        self.pub.publish('speedj(' + str(vels) + ', ' + str(a) \
                             + ', ' + str(t) + ')')

    def followTrajectory(self, traj, gain, maxDistToTarget, gamma=0.97, breakGamma=0.75, maxErrorMag=0.80, isBraking=True):
        '''Simple leaky integrator with error scaling and breaking.'''

        first_half_len = int(len(traj)/2)
        gains = np.array([gain]*len(traj))
        gains[first_half_len : ] = np.linspace(gain, 0.8, len(traj) - first_half_len)
        maxDistsToTarget = [maxDistToTarget]*(len(traj)-1) + [maxDistToTarget - 0.002]

        leakySum = 0.0
        command = np.zeros(6)

        print("Moving arm...")
        avg_vels = []
        max_vels = []

        for i, target in enumerate(traj):

            if i >100:
                break

            error = target - self.jointValues

            while np.max(np.abs(error)) > maxDistsToTarget[i]:
                t0 = time()

                # scale to maximum error
                errorMag = np.linalg.norm(error)
                if errorMag > maxErrorMag:
                    scale = maxErrorMag / errorMag
                    error = scale * error

                # integrate error
                leakySum = gamma*leakySum + (1.0-gamma)*error
                command = gains[i]*leakySum

                self.publishVelocities(self.createJointVelocitiesDict(command))

                avg_vels.append(np.mean(command))
                max_vels.append(np.max(np.abs(command)))

                # sleepTime = 0.008 - (time() - t0)
                sleepTime = 0.008
                # print('  sleepTime:', sleepTime)
                rospy.sleep(sleepTime)
                # rospy.sleep(0.008)

                # compute error
                error = target - self.jointValues

            print('[followTrajectory] Reached viapoint #', i, 'with error:', np.max(np.abs(error)), np.abs(error))

        with open('/home/ethercat/.ros/avg_vels.txt', 'w') as f:
            for v in avg_vels:
                f.write('%.8f\n' % v)

        with open('/home/ethercat/.ros/max_vels.txt', 'w') as f:
            for v in max_vels:
                f.write('%.8f\n' % v)

        # Reduce velocity to 0.
        if isBraking:
            print("Braking...")
            self.brake(command, breakGamma)

        # set speed to 0.0
        self.publishVelocities(self.createJointVelocitiesDict(np.zeros(6)))

#    def brakeSmooth(self, command, thresh=1e-6):
#        diff = np.sum(command)
#
#        while np.max(np.abs(command)) > thresh:
#            t0 = time()
#            command = gamma*command
#            self.publishVelocities(self.createJointVelocitiesDict(command))
#            sleepTime = 0.008
#            rospy.sleep(sleepTime)

    def brake(self, command, gamma, thresh=1e-6):
        while max(np.abs(command)) > thresh:
            t0 = time()
            command = gamma*command
            self.publishVelocities(self.createJointVelocitiesDict(command))
            sleepTime = 0.008
            rospy.sleep(sleepTime)

    def forceCallback(self, msg):
        '''Callback function for the joint_states ROS tonp.pic.'''
        self.force_mag = msg.wrench.force.x ** 2 + msg.wrench.force.y ** 2 + msg.wrench.force.z ** 2

    def isInJointLimits(self, joint_positions):
        '''Returns true only if the given arm position is within the joint limits.'''
        return (joint_positions >= self.joint_limits[0, :]).all() and (joint_positions <= self.joint_limits[1, :]).all()

    def jointsCallback(self, msg):
        '''Callback function for the joint_states ROS tonp.pic.'''
        positions_dict = jointStateToDict(msg)
        self.joint_positions_dict = positions_dict
        self.jointValues = np.array([positions_dict[i] for i in self.joint_names_speedj])

    def shutdownCallback(self):
        '''Gradually reduces the joint velocities to zero when the program is trying to shut down.'''

        print("Received shutdown signal ...")

        v = np.asarray(self.joint_velocities)
        print 'arm velocities:', v

        if np.linalg.norm(v) >= 0.01:
            print("Arm is moving. Slowing down to avoid hard braking.")
            max_iterations = 100
            vel = v
            step = v / max_iterations

            for i in range(0, max_iterations):
                vel -= step
                self.publishVelocities(self.createJointVelocitiesDict(vel))
                rospy.sleep(0.01)

            # Reduce speed to zero.
            self.publishVelocities(self.createJointVelocitiesDict(np.zeros(6)))
            rospy.sleep(0.01)

        print("Exiting ...")
        exit(0)

    def createJointVelocitiesDict(self, velocities):
        dict_out = {}
        for i in range(len(velocities)):
            dict_out[self.joint_names_speedj[i]] = velocities[i]
        return dict_out

def jointStateToDict(msg):
    '''Convert JointState message to dictionary.'''
    dict_out = {}
    for i in range(len(msg.position)):
        dict_out[msg.name[i]] = msg.position[i]

    return dict_out
