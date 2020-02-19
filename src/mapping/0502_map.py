#!/usr/bin/env python

import rospy
import sys
import math
from teensy.msg import pwm_high
from vesc_msgs.msg import VescStateStamped
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np
from scipy import spatial


class Mapping:
    def __init__(self):
        self.angleAccum = [0]
        self.angleSerial = [0, 0]
        self.lastDist = 0
        self.finalAngle = 0
        self.travelledTmp = 0
        self.trajectory = np.array(([[0,0]]))
        self.trajInd = 1
        self.state = "spawned"
        self.x = [0]
        self.y = [0]
        #self.hl, = plt.plot([], [])
        plt.ion()
        self.currentPos = [0 , 0]
        plt.show()
        self.plotLenght = 0
        self.updateCounter = 0
        self.ntd = False
        self.timer = rospy.Timer(rospy.Duration(20), self.checker)
        self.timer2 = rospy.Timer(rospy.Duration(0.25), self.plot)
        self.angleCal = 82


    def find_nearest(array, value):
        array = np.asarray(array)
        idx = (np.abs(array - value)).argmin()
        return array[idx]

    def checker(self, timer):
        #r = rospy.Rate(0.1) # 10hz
        #while not rospy.is_shutdown():
        if (self.ntd == False): self.ntd = True
        elif (self.ntd == True):
            print("DATA ENDED")
            self.timer.shutdown()
            self.plot()
            #r.sleep()

    def state_callback(self, state):
        #print(state.state.displacement)
        #print(state.state.distance_traveled)

        if (self.lastDist == 0): self.lastDist = state.state.distance_traveled


        elif (self.lastDist != state.state.distance_traveled):
            if (self.ntd == True): self.ntd = False
            if (self.state == "spawned"): self.state = "first_lap"

            tmplastDist = abs(state.state.distance_traveled - self.lastDist)
            self.travelledTmp += tmplastDist
            self.lastDist = state.state.distance_traveled

            if (self.travelledTmp >= 10):
                avgA = self.updateAngleAccum(len(self.angleAccum))
                self.updateAngleSerial(len(self.angleAccum), avgA, tmplastDist, state.state.distance_traveled)
                self.updateByDist(state.state.distance_traveled)
            #    self.updateCounter += 1
            #    if self.updateCounter >= 250:
            #        self.plot()
                #    self.updateCounter = 0
                #self.plot()
                #print(self.finalAngle)
            #print(self.angleSerial[0], self.travelledTmp, self.finalAngle)

    def updateXY(self, d):
        a = math.radians(self.finalAngle)


        if self.state == "first_lap":
            x_new = self.x[-1] + d*math.cos(a)
            y_new = self.y[-1] + d*math.sin(a)
            self.currentPos[0] = x_new
            self.currentPos[1] = y_new
            self.x.append(x_new)
            self.y.append(y_new)
            dist, ind = spatial.KDTree(self.trajectory).query([x_new, y_new])
            if (abs(d-dist)**2 > 10):
                self.state = "gotcha!"
                print("gotcha!")
                #print([x_new, y_new])
                #print(self.trajectory)
                self.x = [x_new]
                self.y = [y_new]
                traj_copy = np.array(([[x_new, y_new]]))
                for x in range(ind, len(self.trajectory)):
                    self.x.append(self.trajectory[x][0])
                    self.y.append(self.trajectory[x][1])
                    traj_copy = np.vstack((traj_copy, self.trajectory[x]))
                self.trajectory = traj_copy
                self.plotLenght = len(self.trajectory)
                self.trajInd = 1
            else:
                self.trajectory = np.vstack((self.trajectory, [x_new, y_new]))
                print(dist, d)

        elif self.state == "gotcha!":
            x_new = self.trajectory[self.trajInd][0] + d*math.cos(a)
            y_new = self.trajectory[self.trajInd][1] + d*math.sin(a)
            dist, ind = spatial.KDTree(self.trajectory).query([x_new, y_new])
            #print(dist)
            if (dist > 15 or ind == self.trajInd - 1):
                print(dist, ind, self.trajInd)
                self.state = "first_lap"
                print("first_lap")
                self.x = [x_new]
                self.y = [y_new]
                self.currentPos[0] = x_new
                self.currentPos[1] = y_new
                self.trajectory = np.array(([[x_new, y_new]]))
            elif ((ind >= self.trajInd and abs(self.trajInd - ind)<3) or \
            (ind < self.trajInd and abs(self.trajInd - len(self.trajectory) - ind)<4)):
                self.trajInd = ind
                print(ind, self.trajectory[ind][0], self.trajectory[ind][1])
                self.trajectory[ind][0] = (self.trajectory[ind][0] + (x_new - self.trajectory[ind][0])*0.33)
                self.trajectory[ind][1] = (self.trajectory[ind][1] + (y_new - self.trajectory[ind][1])*0.33)
                self.x[ind] = self.trajectory[ind][0]
                self.y[ind] = self.trajectory[ind][1]
                self.currentPos[0] = x_new
                self.currentPos[1] = y_new
                print(ind, self.trajectory[ind][0], self.trajectory[ind][1])
            else:
                print("WTF???")
                self.state = "first_lap"
                print("first_lap")
                self.x = [x_new]
                self.y = [y_new]
                self.currentPos[0] = x_new
                self.currentPos[1] = y_new
                self.trajectory = np.array(([[x_new, y_new]]))


    def updateByDist(self, dist):
        angles = ((self.angleSerial[0]-9361)*self.angleCal/2882.0)

        angles = math.radians(angles)

        c = 0.0
        if (angles == 0):
            self.updateXY(self.travelledTmp)
        elif (angles > 0):
            c = 2*(32.0 / math.sin((angles)))*math.pi
            deg = ((self.travelledTmp) / c) * 36
            self.finalAngle += deg
            if (self.finalAngle >= 360): self.finalAngle -= 360
            self.updateXY(self.travelledTmp)
        else:
            c = 2*(32.0 / math.sin((-angles)))*math.pi
            deg = ((self.travelledTmp) / float(c)) * 36
            self.finalAngle -= deg
            if (self.finalAngle <= 0): self.finalAngle += 360
            self.updateXY(self.travelledTmp)
        #print("uD", angles, self.finalAngle, dist)
        self.travelledTmp = 0
        self.angleSerial = [0, 0]

    def updateAngleAccum(self, tmp):
        updated = True
        avgA = 0
        if (tmp > 1):
            avgA =  sum(self.angleAccum) / float(tmp)
            del self.angleAccum[:]
            updated = False
        elif (tmp == 1):
            avgA = self.angleAccum[0]
            updated = False
        if (updated): print(self.angleAccum, tmp, "NOT UPDATED!")
        return avgA

    def updateAngleSerial(self, tmp, avgA, tmplastDist, dist):
        if (self.angleSerial[1] == 0):
            self.angleSerial[0] = avgA
            self.angleSerial[1] = tmp
        elif (avgA != 0 and abs(self.angleSerial[0] - avgA)/float(avgA) < 0.01):
            self.angleSerial[0] = (self.angleSerial[0]*self.angleSerial[1] + avgA )/(self.angleSerial[1]+1)
            self.angleSerial[1] += 1
        else: print("ELSE of self.updateAngleSerial")

        """
        else :
            angles = ((self.angleSerial[0]-9361)*self.angleCal/2882)
            angles = math.radians(angles)

            c = 0.0
            if (angles == 0):
                self.updateXY(self.travelledTmp - tmplastDist)
            elif (angles > 0):
                c = 2*(32 / math.sin(angles))*math.pi
                deg = ((self.travelledTmp - tmplastDist) / c) * 36
                self.finalAngle += deg
                if (self.finalAngle >= 360): self.finalAngle -= 360
                self.updateXY(self.travelledTmp - tmplastDist)
            else:
                c = 2*(32 / math.sin((-angles)))*math.pi
                deg = ((self.travelledTmp - tmplastDist) / c) * 36
                self.finalAngle -= deg
                if (self.finalAngle <= 0): self.finalAngle += 360
                self.updateXY(self.travelledTmp - tmplastDist)
            print("uA", angles, self.finalAngle, dist)
            self.travelledTmp = tmplastDist
            self.angleSerial = [avgA, 1]

            #print(self.finalAngle)
        """


    def pwm_callback(self, pwm):
        #print(pwm.period_str)
        if (self.state == "spawned"): self.angleAccum[0]=pwm.period_str
        else: self.angleAccum.append(pwm.period_str)

    def pwmToAngle(self, angle):
        #avgA -= 9305 34
        return math.radians(((angle-9361)*35/2882))

    def plot(self, timer):
        #print "Plotting..."

        #plt.xlim([0, 100])
        #plt.ylim([0, 100])
        #plt.pause(0.01)  # I ain't needed!!!
        #self.fig.canvas.draw()

        #y.append(yi)
        #x = range(len(y))
        #self.ax.clear()
        #self.ax.plot(self.x, self.y, 'o-')
        #
        #print i, ': ', yi
        #/self.hl.set_xdata(self.x)
        #/self.hl.set_ydata(self.y)
        #/plt.draw()
        plt.clf()
        plt.plot(self.currentPos[0], self.currentPos[1], 'rs')
        plt.plot(self.x, self.y)
        plt.pause(0.0000000001)
        #return



if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    myMap = Mapping()
    ourtopic1 = "/pwm_high"
    ourtopic2 = "/sensors/core"
    print("Waiting for ", ourtopic1, " and ", ourtopic2, " topic")
    #another = 'rosout'
    #found = False
    rospy.Subscriber(ourtopic1, pwm_high, myMap.pwm_callback)
    rospy.Subscriber(ourtopic2, VescStateStamped, myMap.state_callback)
    print("Connected!")
    rospy.spin()
