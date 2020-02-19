#!/usr/bin/env python

from hw1.srv import *
import rospy
import matplotlib.pyplot as plt
import numpy as np

def handle_plot(req):
    print "Plotting..."
    #print(type(req))
    #print(req.x)
    #fig, ax = plt.subplots()
    plt.title("Laser scanner data")
    plt.xlabel("Time, s")
    plt.ylabel("Distance, m")
    plt.plot(req.x, req.y, 'o-')

    #ax.tick_params(width=10)
    plt.ion()
    #plt.clear()
    plt.show()
    #x = input()
    #if (x): plt.close()
    #print(x)
    return

def plot_server():
    rospy.init_node('plot_server')
    s = rospy.Service('plot', Plot, handle_plot)
    print "Ready to plot! Waiting for data..."
    rospy.spin()

if __name__ == "__main__":
    plot_server()
