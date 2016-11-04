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

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

## ==============================================================================
## Objective:
##          solution to hw 2 of ECET 581: Programming Robots with ROS
## Link to HW2:
##          http://web.ics.purdue.edu/~rvoyles/Classes/ROSprogramming/HW2.pdf
## Description:
##          This code start a program to accept key commands from user. The command
##          will be sent to listener.py, which will control the movement of Baxter. 
##          Use together with listener.py
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
from std_msgs.msg import String
import baxter_external_devices


def talker():
    rospy.init_node('teleBax_getkey', anonymous=True)
    pub = rospy.Publisher('bax_tele', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    done = False
    while not rospy.is_shutdown() and not done:
        c = baxter_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
            hello_str = c
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            rate.sleep()
    print "Done! Quit!"

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
