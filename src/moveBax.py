#!/usr/bin/env python
import sys, rospy, tf
import baxter_interface
from baxter_interface import CHECK_VERSION

class MoveBaxter:
    def __init__(self):
        print("Initializing node... ")
        rospy.init_node("move_baxter")

        self.left = baxter_interface.Limb('left')
        self.right = baxter_interface.Limb('right')
        self.grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
        self.grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
        self.lj = self.left.joint_names()
        self.rj = self.right.joint_names()

        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def initWayPoints(self):
        self.waypoints = {}
        self.waypoints['left_ready'] = {'left_w0': -0.2918398448952, 'left_w1': 1.527844864733872, 'left_w2': 0.7919175817458448, 'left_e0': -0.744364177321397, 'left_e1': 1.0711020851409903, 'left_s0': 0.31024761434982495, 'left_s1': 0.17333982903105175}
        self.waypoints['right_ready'] = {'right_s0': -0.23853401251618184, 'right_s1': 0.15301458359157002, 'right_w0': 0.27074760906177553, 'right_w1': 1.5876701154614032, 'right_w2': -0.7934515625337303, 'right_e0': 0.7547185476396235, 'right_e1': 1.0561457724591075}
        self.waypoints['left_hold'] = {'left_w0': -0.24198546928892403, 'left_w1': 1.2720535683539793, 'left_w2': 0.6937428113211783, 'left_e0': -0.768140879533621, 'left_e1': 1.3123205640359714, 'left_s0': 0.2906893593042859, 'left_s1': 0.09012137128826805}
        self.waypoints['right_hold'] = {'right_s0': -0.21705828148578604, 'right_s1': 0.20938837754635897, 'right_w0': 0.20785439675847356, 'right_w1': 1.4534467965214295, 'right_w2': -0.8026554472610428, 'right_e0': 0.8141603031701834, 'right_e1': 1.1236409271260657}

    def move(self, pose):
        if pose == 'neutral':
            print("Moving to neutral pose...")
            self.left.move_to_neutral()
            self.right.move_to_neutral()

        elif pose == 'ready':
            print("Moving to ready pose...")
            self.left.move_to_joint_positions(self.waypoints['left_ready'])
            self.right.move_to_joint_positions(self.waypoints['right_ready'])

        elif pose == 'hold':
            print("Moving to hold pose...")
            self.left.move_to_joint_positions(self.waypoints['left_hold'])
            self.right.move_to_joint_positions(self.waypoints['right_hold'])

        else:
            print "Unrecognized pose"
            print "usage: moveBax pose (pose can be neutral, ready and hold)"

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print "usage: moveBax pose (pose can be neutral, ready and hold)"
    else:
        pose = sys.argv[1]
        mb = MoveBaxter()
        mb.initWayPoints()
        mb.move(pose)
