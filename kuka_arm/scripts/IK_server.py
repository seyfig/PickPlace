#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *



def homogeneous_transform(alpha_n_1, a_n_1, q_n, d_n, s=None):
    s_q_n = sin(q_n)
    c_q_n = cos(q_n)
    s_a_n_1 = sin(alpha_n_1)
    c_a_n_1 = cos(alpha_n_1)
    Tn_n_1 =  Matrix([[        c_q_n,        -s_q_n,        0,          a_n_1],
                   [s_q_n*c_a_n_1, c_q_n*c_a_n_1, -s_a_n_1, -s_a_n_1 * d_n],
                   [s_q_n*s_a_n_1, c_q_n*s_a_n_1,  c_a_n_1,  c_a_n_1 * d_n],
                   [            0,             0,        0,              1]])
    if s is not None:
        Tn_n_1 = Tn_n_1.subs(s)
    return Tn_n_1

def sqsum(x, y):
    return pow(x,2) + pow(y,2)


class Transformer:
    def __init__(self):
        """
        Class to calculate required matrices in the beginning
        In order to, decrease computation time on the run.
        """
        # FK part
        # Create symbols for joint variables
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta_i
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Create Modified DH parameters
        # DH Parameters
        dh = {alpha0:     0, a0:      0, d1:  0.75,
             alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
             alpha2:     0, a2:   1.25, d3:     0,
             alpha3: -pi/2, a3: -0.054, d4:  1.50,
             alpha4:  pi/2, a4:      0, d5:     0,
             alpha5: -pi/2, a5:      0, d6:     0,
             alpha6:     0, a6:      0, d7: 0.303, q7:       0
        }

        # Set parameters as properties to access later
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3

        # Define Modified DH Transformation matrix
        # Homogeneous Transforms
        T0_1 = homogeneous_transform(alpha0, a0, q1, d1).subs(dh)
        T1_2 = homogeneous_transform(alpha1, a1, q2, d2).subs(dh)
        T2_3 = homogeneous_transform(alpha2, a2, q3, d3).subs(dh)
        T3_4 = homogeneous_transform(alpha3, a3, q4, d4).subs(dh)
        T4_5 = homogeneous_transform(alpha4, a4, q5, d5).subs(dh)
        T5_6 = homogeneous_transform(alpha5, a5, q6, d6).subs(dh)
        T6_G = homogeneous_transform(alpha6, a6, q7, d7).subs(dh)

        # Composition of Homogeneous Transforms
        # Set as property to access later
        self.T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

        # IK part
        # Create symbols for joint variables
        r, p, y = symbols('r p y')

        # roll
        Rot_x = Matrix([[      1,       0,       0],
                        [      0,  cos(r), -sin(r)],
                        [      0,  sin(r),  cos(r)]])

        # pitch
        Rot_y = Matrix([[ cos(p),       0,  sin(p)],
                        [      0,       1,       0],
                        [-sin(p),       0,  cos(p)]])

        # yaw
        Rot_z = Matrix([[ cos(y), -sin(y),       0],
                        [ sin(y),  cos(y),       0],
                        [      0,       0,       1]])

        Rot_EE = simplify(Rot_z * Rot_y * Rot_x)

        Rot_Error = Rot_z.subs(y, pi) * Rot_y.subs(p, -pi/2)

        self.Rot_EE = Rot_EE * Rot_Error
        self.R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # FK part was moved to the Transformer class
        # in order to prevent recalculation each time.

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            if transformer is None:
                continue
            joint_trajectory_point = JointTrajectoryPoint()

        # Extract end-effector position and orientation from request
        # px,py,pz = end-effector position
        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo

            Rot_EE = transformer.Rot_EE.subs({'r': roll, 'p': pitch, 'y':yaw})

            EE = Matrix([[px],[py],[pz]])

            # Calculate the wrist center
            # O_r_WC_0 = O_r_EE_0 - d * R_0_6 * [0,0,1] = [px,py,pz] - d * R_0_6 * [0, 0, 1]
            # (EE position) -  (EERM + the offset)
            WC = EE - (0.303) * Rot_EE[:,2]

            # Calculate joint angles using Geometric IK method
            # q1 by using WC
            theta1 = atan2(WC[1], WC[0])

            # Triangle
            side_a = 1.501
            WCxy = sqrt(sqsum(WC[0], WC[1])) - 0.35
            WCz = WC[2] - 0.75
            side_b = sqrt(sqsum(WCxy, WCz))
            side_c = 1.25

            side_a2 = pow(side_a, 2)
            side_b2 = pow(side_b, 2)
            side_c2 = pow(side_c, 2)

            angle_a = acos((side_b2 + side_c2 - side_a2) / (2 * side_b * side_c))
            angle_b = acos((side_a2 + side_c2 - side_b2) / (2 * side_a * side_c))
            angle_c = acos((side_a2 + side_b2 - side_c2) / (2 * side_a * side_b))

            theta2 = pi/2 - angle_a - atan2(WCz, WCxy)
            theta3 = pi/2 - (angle_b + 0.036)  # sag in link4 of -0.054m

            R0_3 = transformer.R0_3.evalf(subs={transformer.q1: theta1, transformer.q2: theta2, transformer.q3:theta3})

            R3_6 = R0_3.inv("LU") * Rot_EE

            # Euler angles from the rotation matrix
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(sqsum(R3_6[0,2], R3_6[2,2])), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            #
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
        joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
        joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    transformer = Transformer()
    global transformer
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
