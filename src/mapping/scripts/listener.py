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

import rospy
import sys
from std_msgs.msg import String
from hw1.srv import *
from sensor_msgs.msg import LaserScan




dataX = []
dataY = []
timeCurr = 0
plot_length = 20

def callback(data):
    global timeCurr, dataX, dataY, plot_length
    #rospy.loginfo(data.header.stamp.to_sec())
    if timeCurr == 0:
    	timeCurr = data.header.stamp
    time = data.header.stamp - timeCurr
    #print(time.to_sec())
    #print(len(data.ranges))    
    #print(data.range_max, data.range_min) 
   # print(data.angle_max, data.angle_min, data.angle_increment)    
    #print(data.angle_max*57.295779513, data.angle_min*57.295779513, data.angle_increment*57.295779513)   
    sumVal = 0
    it = 0
    for i in data.ranges[149:210]:
        
    	if i <= data.range_max and i >= data.range_min:
		sumVal += i
		it += 1
    #print(tmp)
    #print(data.ranges[149:210])    
    dataX.append(sumVal/it)
    dataY.append(time.to_sec())
    print("Data collection, stored: ", len(dataX))
    if len(dataX) >= plot_length:
    	callPlot(dataY, dataX)

def callPlot(dataX, dataY):
    rospy.wait_for_service('plot')
    try:
	print("Trying to plot!")
        plotting = rospy.ServiceProxy('plot', Plot)
        resp1 = plotting(dataX, dataY)
        #return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    print("Waiting for /scan topic")

    ourtopic = '/scan'
    another = 'rosout'
    found = False
    rospy.Subscriber(ourtopic, LaserScan, callback)
    print("Connected!")
    rospy.spin()
    

if __name__ == '__main__':
    listener()
