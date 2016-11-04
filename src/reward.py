#!/usr/bin/env python
import sys, rospy, tf
import numpy as np
import operator
from std_msgs.msg import Float32
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Quaternion
import baxter_interface
from baxter_interface import CHECK_VERSION

class reward:
    def __init__(self):
        # init node
        rospy.init_node("get_reward")

        # init publisher for reward
        self.pub = rospy.Publisher('reward', Float32, queue_size=10)

        # init service for get gazebo model state
        rospy.wait_for_service("gazebo/get_model_state")
        self.get_mode_state = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
    
        # init parameters to get Baxter
        self.left = baxter_interface.Limb('left')
        self.right = baxter_interface.Limb('right')
        self.grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
        self.grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
        self.lj = self.left.joint_names()
        self.rj = self.right.joint_names()

        # default parameters for all to avoid error
        self.alpha = 1
        self.beta = 1
        self.gamma = 1
        self.desired_height = 0.8
        self.desired_dis = 0.4

    def get_cube(self):
        # get cube position and orientation
        cube = self.get_mode_state("cube", "world")
        return cube

    def get_tip_dis(self):
        # get robot hand distance
        left_pos = self.left.endpoint_pose()['position']
        right_pos = self.right.endpoint_pose()['position']
        offset = np.array((left_pos.x - right_pos.x,left_pos.y - right_pos.y, left_pos.z - right_pos.z))
        dis = np.linalg.norm(offset)
        return dis

    def init_reward_coeff(self, alpha, beta, gamma):
        # init coefficient for each reward
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        print "Init coefficient => alpha: %.3f, beta: %.3f, gamma: %.3f" % (alpha, beta, gamma)

    def init_desired_location(self, desired_height, desired_dis):
        # set desired height of table, and desired distance between hands
        self.desired_height = desired_height
        self.desired_dis = desired_dis
        print "Init target => desired_height: %.3f, desired_dis: %.3f" % (desired_height, desired_dis)

    def calc_reward(self, cube, dis):
        # get cube height
        current_height = cube.pose.position.y
        r1 = np.exp(-self.alpha * np.abs(current_height-self.desired_height))

        # get cube orientation
        current_quat = (cube.pose.orientation.x, cube.pose.orientation.y, cube.pose.orientation.z, cube.pose.orientation.w)
        current_euler = tf.transformations.euler_from_quaternion(current_quat)
        r2 = np.exp(-self.beta * np.abs(current_euler[0]))  

        # get distance between hands
        r3 = np.exp(-self.gamma * np.abs(dis-self.desired_dis))

        r = r1 + r2 + r3
        return r

    def get_reward(self):
        # update current state of the world
        cube = self.get_cube()
        dis = self.get_tip_dis()

        # calcualte current reward value
        r = self.calc_reward(cube, dis)
        return r
    
    def run(self):
        # publish at fixed rate 10hz
        rate = rospy.Rate(10)

        # start main loop
        while not rospy.is_shutdown():
            r = self.get_reward()
            rospy.loginfo(r)
            self.pub.publish(r)
            rate.sleep()
        
if __name__ == '__main__':
    try:
        # init reward object
        r = reward()

        # init coeff and ground truth for reward calculation 
        r.init_reward_coeff(alpha=0.1, beta=0.1, gamma=0.1)
        r.init_desired_location(desired_height=0.2, desired_dis=0.2)

        # start main loop
        r.run()

    except rospy.ROSInterruptException:
        pass

    