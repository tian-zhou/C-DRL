#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published
## to the 'chatter' topic



## ==============================================================================
## Objective:
##          solution to hw 2 of ECET 581: Programming Robots with ROS
## Link to HW2:
##          http://web.ics.purdue.edu/~rvoyles/Classes/ROSprogramming/HW2.pdf
## Description:
##          This code will allow you to control Baxter to move in both
##          joint space (enter joint) and tip space (enter tip).
##          Use it together with talker.py which accepts key commands from users.
##          Also, either Gazebo simulator with Baxter or a physical Baxter needs
##          to be running
## Authors:
##          Maria Eugenia Cabrera (or commonly known as, Maru) (marulok@gmail.com)
##          PUID : 0026982137
##
##          Teerachart Soratana (or commonly known as, Taa) (tsoratan@purdue.edu)
##          PUID : 0025884248
##
##          Tian Zhou (or commonly known as, Tian) (zhou338@purdue.edu)
##          PUID : 0026110526
## Date:
##          Sep 7, 2016
## ==============================================================================

import rospy
import operator
import struct
import tf
from std_msgs.msg import String
import baxter_interface
from baxter_interface import CHECK_VERSION
from math import sin, cos, sqrt
import numpy as np
import collections

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

class BaxterMoveIT:
    def __init__(self):
        pass

    def init_codebook(self):
        set_j = self.set_j
        set_tip = self.set_tip
        left = self.left
        right = self.right
        grip_left = self.grip_left
        grip_right = self.grip_right
        lj = self.lj
        rj = self.rj

        unit = 0.05
        self.codebook_tip = {
            #   key: (function, args)
            '1': (set_tip, [left, 'x', unit, "left X increase"]),
            '!': (set_tip, [left, 'x', -unit, "left X decrease"]),
            '2': (set_tip, [left, 'y', unit, "left Y increase"]),
            '@': (set_tip, [left, 'y', -unit, "left Y decrease"]),
            '3': (set_tip, [left, 'z', unit, "left Z increase"]),
            '#': (set_tip, [left, 'z', -unit, "left Z decrease"]),
            '4': (set_tip, [left, 'r', unit, "left Roll increase"]),
            '$': (set_tip, [left, 'r', -unit, "left Roll decrease"]),
            '5': (set_tip, [left, 'p', unit, "left Pitch increase"]),
            '%': (set_tip, [left, 'p', -unit, "left Pitch decrease"]),
            '6': (set_tip, [left, 'yaw', unit, "left Yaw increase"]),
            '^': (set_tip, [left, 'yaw', -unit, "left Yaw decrease"]),

            'q': (set_tip, [right, 'x', unit, "right X increase"]),
            'Q': (set_tip, [right, 'x', -unit, "right X decrease"]),
            'w': (set_tip, [right, 'y', unit, "right Y increase"]),
            'W': (set_tip, [right, 'y', -unit, "right Y decrease"]),
            'e': (set_tip, [right, 'z', unit, "right Z increase"]),
            'E': (set_tip, [right, 'z', -unit, "right Z decrease"]),
            'r': (set_tip, [right, 'r', unit, "right Roll increase"]),
            'R': (set_tip, [right, 'r', -unit, "right Roll decrease"]),
            't': (set_tip, [right, 'p', unit, "right Pitch increase"]),
            'T': (set_tip, [right, 'p', -unit, "right Pitch decrease"]),
            'y': (set_tip, [right, 'yaw', unit, "right Yaw increase"]),
            'Y': (set_tip, [right, 'yaw', -unit, "right Yaw decrease"]),
        }

        self.codebook_joint = {
            #   key: (function, args)
            '1': (set_j, [left, lj[0], unit, "left_s0 increase"]),
            '!': (set_j, [left, lj[0], -unit, "left_s0 decrease"]),
            '2': (set_j, [left, lj[1], unit, "left_s1 increase"]),
            '@': (set_j, [left, lj[1], -unit, "left_s1 decrease"]),
            '3': (set_j, [left, lj[2], unit, "left_e0 increase"]),
            '#': (set_j, [left, lj[2], -unit, "left_e0 decrease"]),
            '4': (set_j, [left, lj[3], unit, "left_e1 increase"]),
            '$': (set_j, [left, lj[3], -unit, "left_e1 decrease"]),
            '5': (set_j, [left, lj[4], unit, "left_w0 increase"]),
            '%': (set_j, [left, lj[4], -unit, "left_w0 decrease"]),
            '6': (set_j, [left, lj[5], unit, "left_w1 increase"]),
            '^': (set_j, [left, lj[5], -unit, "left_w1 decrease"]),
            '7': (set_j, [left, lj[6], unit, "left_w2 increase"]),
            '&': (set_j, [left, lj[6], -unit, "left_w2 decrease"]),

            'q': (set_j, [right, rj[0], unit, "right_s0 increase"]),
            'Q': (set_j, [right, rj[0], -unit, "right_s0 decrease"]),
            'w': (set_j, [right, rj[1], unit, "right_s1 increase"]),
            'W': (set_j, [right, rj[1], -unit, "right_s1 decrease"]),
            'e': (set_j, [right, rj[2], unit, "right_e0 increase"]),
            'E': (set_j, [right, rj[2], -unit, "right_e0 decrease"]),
            'r': (set_j, [right, rj[3], unit, "right_e1 increase"]),
            'R': (set_j, [right, rj[3], -unit, "right_e1 decrease"]),
            't': (set_j, [right, rj[4], unit, "right_w0 increase"]),
            'T': (set_j, [right, rj[4], -unit, "right_w0 decrease"]),
            'y': (set_j, [right, rj[5], unit, "right_w1 increase"]),
            'Y': (set_j, [right, rj[5], -unit, "right_w1 decrease"]),
            'u': (set_j, [right, rj[6], unit, "right_w2 increase"]),
            'U': (set_j, [right, rj[6], -unit, "right_w2 decrease"]),
        }

    def init_comm(self):
        # member variable related to init node and Baxter
        rospy.init_node("teleBax_move")
        print("Initializing node... ")
        print("Getting robot state... ")
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled
        print("Enabling robot... ")
        rs.enable()

    def init_robot_arm(self):
        # GLOBAL variable related to Baxter joints and grippers
        self.left = baxter_interface.Limb('left')
        self.right = baxter_interface.Limb('right')
        self.grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
        self.grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
        self.lj = self.left.joint_names()
        self.rj = self.right.joint_names()

    # change a joint specified by @arg joint_name on arm @arg limb by @arg delta
    def set_j(self, limb, joint_name, delta, comment):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        limb.set_joint_positions(joint_command)

    # call IK function to calculate the joint points based on tip pose
    def calcJoint(self, updated_pose, limbString):
        # call IK to calculate joint positions from updated_pose
        ns = "ExternalTools/" + limbString + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        poses = PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x=updated_pose['position'][0],
                        y=updated_pose['position'][1],
                        z=updated_pose['position'][2],
                    ),
                    orientation=Quaternion(
                        x=updated_pose['orientation'][0],
                        y=updated_pose['orientation'][1],
                        z=updated_pose['orientation'][2],
                        w=updated_pose['orientation'][3],
                    ),
                )
        )

        ikreq.pose_stamp.append(poses)
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1

        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                                   resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            #print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
            #      (seed_str,))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            #print "\nIK Joint Solution:\n", limb_joints
            #print "------------------"
            #print "Response Message:\n", resp
            return limb_joints

        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
            return 0


    # Set tip position on @arg limb on @arg axis with @arg delta
    def set_tip(self, limb, axis, delta, comment):
        # get current pose
        current_pose = limb.endpoint_pose()
        #print "Current_pose: ", current_pose

        current_quat = current_pose['orientation']
        current_euler = tf.transformations.euler_from_quaternion(current_quat)
        updated_pose = current_pose

        # self.Compute_quat_from_rot(current_euler) # print out the rot to quat
        # print "Output current quaternion: ", current_quat

        # xyz movement
        if axis == 'x':
            updated_point = tuple(map(operator.add, updated_pose['position'], (delta,0,0)))
        elif axis == 'y':
            updated_point = tuple(map(operator.add, updated_pose['position'], (0,delta,0)))
        elif axis == 'z':
            updated_point = tuple(map(operator.add, updated_pose['position'], (0,0,delta)))

        # roll, pitch, yaw movement
        if axis == 'r':
            updated_euler = tuple(map(operator.add, current_euler, (delta,0,0)))
            updated_quat = tf.transformations.quaternion_from_euler(updated_euler[0], updated_euler[1], updated_euler[2])
        elif axis == 'p':
            updated_euler = tuple(map(operator.add, current_euler, (0,delta,0)))
            updated_quat = tf.transformations.quaternion_from_euler(updated_euler[0], updated_euler[1], updated_euler[2])
        elif axis == 'yaw':
            updated_euler = tuple(map(operator.add, current_euler, (0,0,delta)))
            updated_quat = tf.transformations.quaternion_from_euler(updated_euler[0], updated_euler[1], updated_euler[2])


        # update updated_pose with either updated_point or updated_quat
        if axis in ['x','y','z']:
            updated_pose['position'] = updated_point
        elif axis in ['r', 'p', 'yaw']:
            updated_pose['orientation'] = updated_quat

        #print "Updated_pose: ", updated_pose

        limb_joints = self.calcJoint(updated_pose, limbString = comment[:comment.find(' ')])
        if limb_joints:
            # call move_joint_positions with the calculated joint pos
            print "Start moving, wait..."
            limb.move_to_joint_positions(limb_joints, threshold=0.08726646)
            print "Finish moving. Yeah :-)"
            final_pose = limb.endpoint_pose()
            #print "Final_pose: ", final_pose

    # Compute transformation from rotation matrix to quat
    def Compute_quat_from_rot(self, current_euler):
        print "Input euler angle: \n", current_euler
        Rx = [[1, 0, 0], [0, cos(current_euler[0]), -sin(current_euler[0])], [0, sin(current_euler[0]), cos(current_euler[0])]]
        Ry = [[cos(current_euler[1]), 0, sin(current_euler[1])], [ 0, 1, 0], [-sin(current_euler[1]), 0, cos(current_euler[1])]]
        Rz = [[cos(current_euler[2]), -sin(current_euler[2]), 0], [ sin(current_euler[2]), cos(current_euler[2]), 0],[ 0, 0, 1]]

        R = np.dot(np.dot(Rz,Ry),Rx)
        b1 = 0.5*sqrt(1+R[0][0] + R[1][1] + R[2][2])
        b2 = (R[2][1] - R[1][2])/(4*b1)
        b3 = (R[0][2] - R[2][0])/(4*b1)
        b4 = (R[1][0] - R[0][1])/(4*b1)

        b = collections.OrderedDict()
        b['x'] = b2
        b['y'] = b3
        b['z'] = b4
        b['w'] = b1
        print "Input rotation matrix: \n", R
        print "Output quaternion from function: \n", b


    # select the codebook
    def select_control_mode(self, mode):
        if mode == 'joint':
            self.codebook = self.codebook_joint
        elif mode == 'tip':
            self.codebook = self.codebook_tip
        else:
            print "Unrecognized mode: %s" % mode

    # the call back function, when a message is received from topic "chatter"
    def callback(self, data):
        #rospy.loginfo('Motion data: %s', data.data)
        c = data.data
        if c in ['\x1b', '\x03']:
            rospy.signal_shutdown("Example finished.")
            exit()
        elif c in self.codebook:
            cmd = self.codebook[c]
            #expand binding to something like "set_j(right, 's0', 0.1, 'left x increase')"
            #print "cmd[1]: ", cmd[1]
            print("\ncommand: %s" % (cmd[1][-1],))
            cmd[0](*cmd[1])
        elif c == '?':
            print "\nHelp. key-command mappings: "
            for key, val in sorted(self.codebook.items(), key=lambda x: x[0][0]):
                print("  %s: %s" % (key, val[1][-1]))

        else:
            print "Unrecognized command: %s" % c

    # the listener
    def main_loop(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        # rospy.init_node('listener', anonymous=True)

        rospy.Subscriber('bax_tele', String, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

def main():
    b = BaxterMoveIT()
    b.init_comm()
    b.init_robot_arm()
    b.init_codebook()
    sel_mode = raw_input("Please enter control mode (either tip or joint): ")
    print "Ready to move. Go to talker.py to enter commands or press ? for help."
    b.select_control_mode(mode=sel_mode)
    b.main_loop()

if __name__ == '__main__':
    main()
