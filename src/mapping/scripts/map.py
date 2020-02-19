#!/usr/bin/env python

import rospy
import sys
import math
from teensy.msg import pwm_high, drive_values
from vesc_msgs.msg import VescStateStamped
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np
from scipy import spatial
from std_msgs.msg import Float32


class Mapping:
    def __init__(self, toPlot=False):
        self.toPlot = toPlot
        self.angleAccum = [0]
        self.angleSerial = [0, 0]
        self.lastDist = 0
        self.finalAngle = 0
        self.travelledTmp = 0
        self.trajectory = np.array(([[0,0]]))
        self.trajInd = 1
        self.state = "spawned"
        self.SU = [[],[]]
        self.SSU = [[], []]
        self.sameIndCnt = 0
        self.backPoint = 0
        self.compassList = [0]
        self.turnAhead = 0
        self.cntrPWM = 9150 #9361 9150
        self.pointsAhead = 25 #100 #65
        self.lapN = 0
        self.pntCntr = 0
        self.base = 32.0 #32.0
        self.x = [0]
        self.y = [0]
        #self.hl, = plt.plot([], [])
        if (self.toPlot):            
            plt.ion()
            plt.show()
            self.timer2 = rospy.Timer(rospy.Duration(1), self.plot)
        self.currentPos = [0 , 0]
        self.multCoeffs = [0.96, 0.9725]
        self.plotLenght = 0
        self.updateCounter = 0
        self.ntd = False
        self.timer = rospy.Timer(rospy.Duration(20), self.checker)
       # self.timer2 = rospy.Timer(rospy.Duration(5), self.plot)
        self.multPublisher = rospy.Publisher("speed_mult", Float32, queue_size=10)
        self.angleCal = 64 #82 #75 #63


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
            #self.plot()
            #r.sleep()

    def state_callback(self, state):
        now = rospy.get_rostime()
        diff = (now.secs - state.header.stamp.secs)*1000000000 + now.nsecs - state.header.stamp.nsecs
    
        if diff > 50000000: return

        #print(state.state.distance_traveled)

        if (self.lastDist == 0): self.lastDist = state.state.distance_traveled


        elif (self.lastDist != state.state.distance_traveled):
            if (self.ntd == True): self.ntd = False
            if (self.state == "spawned"): self.state = "first_lap"

            tmplastDist = abs(state.state.distance_traveled - self.lastDist)
            self.travelledTmp += tmplastDist
            self.lastDist = state.state.distance_traveled
           
            #print(self.travelledTmp, state.state.distance_traveled)
            if (self.travelledTmp >= 25):
                #self.travelledTmp *= 1.2
               # self.angleAccum = [0]

               # tmp = rospy.wait_for_message("/drive_pwm", drive_values)
               # self.angleAccum[0] = tmp.pwm_angle
               # print(diff, self.travelledTmp)
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
            #if (False):
            if (abs(d-dist)**2 > 5):
                self.state = "gotcha!"
                self.lapN = 1
                self.pntCntr = 0
                self.SU = [[],[]]
                self.SSU = [[], []]
                print("gotcha!")
                #print([x_new, y_new])
                #print(self.trajectory)
                self.x = [x_new]
                self.y = [y_new]
                compassList_copy = [a]
                traj_copy = np.array(([[x_new, y_new]]))
                for x in range(ind, len(self.trajectory)):
                    self.x.append(self.trajectory[x][0])
                    self.y.append(self.trajectory[x][1])
                    compassList_copy.append(self.compassList[x])
                    traj_copy = np.vstack((traj_copy, self.trajectory[x]))
                self.compassList = compassList_copy
                self.trajectory = traj_copy
                self.plotLenght = len(self.trajectory)
                self.trajInd = 1
                self.turnAhead = 0
                self.sameIndCnt = 0
                self.backPoint = 0
                if (self.pointsAhead <= self.plotLenght):
                   # f=open("1.csv", "w+")
                    for x in range(1, self.pointsAhead):
                        tmp = abs(self.compassList[x-1] - self.compassList[x])
                        if (abs(tmp)>= 4.0): tmp = abs(6.28319 - tmp)
                        self.turnAhead += tmp
                       # f.write(str(self.compassList[x-1])+"\t" + str(self.compassList[x])+"\t" + str(tmp) +"\n")
                   # f.write(str(self.turnAhead))
                   # f.close()
                    print("log file is made! Total points "+str(self.plotLenght)+str(len(self.compassList)))
                    print(self.turnAhead, self.calcAngle(0, self.pointsAhead))
                else: print("TOO FEW POINTS!")

            else:
                self.trajectory = np.vstack((self.trajectory, [x_new, y_new]))
                self.compassList.append(a)
                #print(dist, d)

        elif self.state == "gotcha!":
            self.pntCntr += 1
            if self.pntCntr >= self.plotLenght:
                self.lapN += 1
                print("Lap #"+str(self.lapN))
                self.pntCntr = 0
                self.SU = [[],[]]
                self.SSU = [[], []]
            x_new = self.trajectory[self.trajInd][0] + d*math.cos(a)
            y_new = self.trajectory[self.trajInd][1] + d*math.sin(a)
            dist, ind = spatial.KDTree(self.trajectory).query([x_new, y_new])
            #print(dist)
            badPoint = False
            distNext = math.sqrt((x_new - self.trajectory[self.trajInd][0])**2 \
            + (y_new - self.trajectory[self.trajInd][1])**2)
            if (distNext > 20): print(self.plotLenght, distNext, dist, ind, self.trajInd)
            if (self.trajInd == ind or self.trajInd == ind + 1):
                self.sameIndCnt += 1
                if (self.trajInd == ind + 1): self.backPoint += 1
                badPoint = True
                print(distNext, dist, ind, self.trajInd)
                if (ind + 1 != self.plotLenght): self.trajInd = ind + 1
                else: self.trajInd = 0
            else:
                self.sameIndCnt = 0
                self.backPoint = 0

            if (distNext > 150 or self.sameIndCnt > 50):
                print(dist, ind, self.trajInd)
                self.state = "first_lap"
                print("first_lap by distNext or sameIndCnt", self.sameIndCnt, ind, self.trajInd, distNext, dist )
                self.x = [x_new]
                self.y = [y_new]
                self.currentPos[0] = x_new
                self.currentPos[1] = y_new
                self.compassList = [a]
                self.trajectory = np.array(([[x_new, y_new]]))
            elif ((ind >= self.trajInd and (ind - self.trajInd)<3) or \
            (ind < self.trajInd and (self.plotLenght - self.trajInd - ind)<4) or \
            self.backPoint < 3):

                #print(ind, self.trajectory[ind][0], self.trajectory[ind][1])
                if (not badPoint):
                    self.trajectory[ind][0] = (self.trajectory[ind][0] + (x_new - self.trajectory[ind][0])/float(3+self.lapN))
                    self.trajectory[ind][1] = (self.trajectory[ind][1] + (y_new - self.trajectory[ind][1])/float(3+self.lapN))
                    tmp = (a - self.compassList[ind])
                    if (tmp >= 4.0): tmp = (6.28319 - tmp)
                    new_comp = self.compassList[ind] + tmp/float(3+self.lapN)
                    self.x[ind] = self.trajectory[ind][0]
                    self.y[ind] = self.trajectory[ind][1]
                    self.compassList[ind] = new_comp
                    #self.updateTurnInfo(new_comp, self.trajInd, ind, True)

                    self.turnAhead = self.calcAngle(ind, self.pointsAhead)
                    self.trajInd = ind
                else:
                   # pass
                    #self.updateTurnInfo(0, self.trajInd, self.trajInd+1, False )
                    self.turnAhead = self.calcAngle(ind, self.pointsAhead)
                if (self.turnAhead<(0.60+self.lapN/10) ):
                   # print("SSU!")
                    self.SSU[0].append(x_new)
                    self.SSU[1].append(y_new)
                    self.multPublisher.publish(self.multCoeffs[0])
                elif (self.turnAhead<(1.65 + self.lapN/10) and self.turnAhead >= (0.60 + self.lapN/10)):
                   # print("SU!")
                    self.SU[0].append(x_new)
                    self.SU[1].append(y_new)
                    self.multPublisher.publish(self.multCoeffs[1])
                else:
                    self.multPublisher.publish(1.0)
                self.currentPos[0] = x_new
                self.currentPos[1] = y_new
                if (self.turnAhead>=16.0):
                    self.state = "first_lap"
                    print("first_lap by TurnAhead", self.sameIndCnt, ind, self.trajInd, distNext, dist )
                    self.x = [x_new]
                    self.y = [y_new]
                    self.currentPos[0] = x_new
                    self.currentPos[1] = y_new
                    self.compassList = [a]
                    self.trajectory = np.array(([[x_new, y_new]]))
                #print(ind, self.trajectory[ind][0], self.trajectory[ind][1])
            else:
                print("WTF???")
                self.state = "first_lap"
                print("first_lap", self.sameIndCnt, ind, self.trajInd, distNext, dist )
                self.x = [x_new]
                self.y = [y_new]
                self.currentPos[0] = x_new
                self.currentPos[1] = y_new
                self.compassList = [a]
                self.trajectory = np.array(([[x_new, y_new]]))

    def calcAngle(self, start, points):
        sumOfAngles = 0
        if (points == 0): return sumOfAngles
        if ((start+points) <= self.plotLenght):
            for x in range(start+1, start+points):
                tmp = abs(self.compassList[x-1] - self.compassList[x])
                if (abs(tmp)>= 4.0): tmp = abs(6.28319 - tmp)
                sumOfAngles += tmp
        else:
            for x in range(start+1, self.plotLenght):
                tmp = abs(self.compassList[x-1] - self.compassList[x])
                if (abs(tmp)>= 4.0): tmp = abs(6.28319 - tmp)
                sumOfAngles += tmp
            tmp = abs(self.compassList[0] - self.compassList[-1])
            if (abs(tmp)>= 4.0): tmp = abs(6.28319 - tmp)
            sumOfAngles += tmp
            for x in range(1, (start + points - self.plotLenght)):
                tmp = abs(self.compassList[x-1] - self.compassList[x])
                if (abs(tmp)>= 4.0): tmp = abs(6.28319 - tmp)
                sumOfAngles += tmp
        return sumOfAngles


    def updateTurnInfo(self, new_comp, oldInd, ind, newData=False):
        if (newData):
            if (ind != (self.plotLenght-1)):

                tmp = abs(self.compassList[ind] - self.compassList[ind-1])
                tmp2 = abs(self.compassList[ind] - self.compassList[ind+1])
                if (abs(tmp)>= 4.0): tmp = abs(6.28319 - tmp)
                if (abs(tmp2)>= 4.0): tmp2 = abs(6.28319 - tmp2)
                tmp += tmp2
                self.turnAhead -= tmp

                self.compassList[ind] = new_comp
                tmp = abs(self.compassList[ind] - self.compassList[ind-1])
                tmp2 = abs(self.compassList[ind] - self.compassList[ind+1])
                if (abs(tmp)>= 4.0): tmp = abs(6.28319 - tmp)
                if (abs(tmp2)>= 4.0): tmp2 = abs(6.28319 - tmp2)
                tmp += tmp2
                self.turnAhead += tmp
                #self.turnAhead += abs(self.compassList[ind] - self.compassList[ind-1]) \
                #+ abs(self.compassList[ind] - self.compassList[ind+1])


            elif (ind == (self.plotLenght-1)):

                tmp = abs(self.compassList[ind] - self.compassList[ind-1])
                tmp2 = abs(self.compassList[ind] - self.compassList[0])
                if (abs(tmp)>= 4.0): tmp = abs(6.28319 - tmp)
                if (abs(tmp2)>= 4.0): tmp2 = abs(6.28319 - tmp2)
                tmp += tmp2
                self.turnAhead -= tmp

                self.compassList[ind] = new_comp
                tmp = abs(self.compassList[ind] - self.compassList[ind-1])
                tmp2 = abs(self.compassList[ind] - self.compassList[0])
                if (abs(tmp)>= 4.0): tmp = abs(6.28319 - tmp)
                if (abs(tmp2)>= 4.0): tmp2 = abs(6.28319 - tmp2)
                tmp += tmp2
                self.turnAhead += tmp

        if (ind >= oldInd):
            for x in range(oldInd, ind):
                tmp = abs(self.compassList[x] - self.compassList[x-1])
                if (abs(tmp)>= 4.0): tmp = abs(6.28319 - tmp)
                self.turnAhead -= tmp
        else:
            for x in range(oldInd, self.plotLenght):
                tmp = abs(self.compassList[x] - self.compassList[x-1])
                if (abs(tmp)>= 4.0): tmp = abs(6.28319 - tmp)
                self.turnAhead -= tmp

            for x in range(0, ind):
                tmp = abs(self.compassList[x] - self.compassList[x-1])
                if (abs(tmp)>= 4.0): tmp = abs(6.28319 - tmp)
                self.turnAhead -= tmp

        if (ind > oldInd):
            steps = abs(ind - oldInd)
            if ((ind + self.pointsAhead - 1) < (self.plotLenght)):
                for x in range((oldInd + self.pointsAhead - 1), (ind + self.pointsAhead - 1)):
                    tmp = abs(self.compassList[x] - self.compassList[x-1])
                    if (abs(tmp)>= 4.0): tmp = abs(6.28319 - tmp)
                    self.turnAhead += tmp
            else:
                for x in range((oldInd + self.pointsAhead - 1), self.plotLenght):
                    tmp = abs(self.compassList[x] - self.compassList[x-1])
                    if (abs(tmp)>= 4.0): tmp = abs(6.28319 - tmp)
                    self.turnAhead += tmp

                for x in range(0, (ind + self.pointsAhead - 1 - self.plotLenght)):
                    tmp = abs(self.compassList[x] - self.compassList[x-1])
                    if (abs(tmp)>= 4.0): tmp = abs(6.28319 - tmp)
                    self.turnAhead += tmp


    def updateByDist(self, dist):
        angles = ((self.angleSerial[0]-self.cntrPWM)*self.angleCal/2882.0)  #9361 9150
        #print self.angleSerial[0], angles
        angles = math.radians(angles)

        c = 0.0
        if (angles == 0):
            self.updateXY(self.travelledTmp)
        elif (angles > 0):
            c = 2*(self.base / math.sin((angles)))*math.pi #32
            deg = ((self.travelledTmp) / float(c)) * 36
            self.finalAngle += deg
            if (self.finalAngle >= 360): self.finalAngle -= 360
            self.updateXY(self.travelledTmp)
        else:
            c = 2*(self.base / math.sin((-angles)))*math.pi #32
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
            self.angleAccum = [avgA]
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
        if (self.state == "spawned"): self.angleAccum[0]=pwm.pwm_angle
        else: self.angleAccum.append(pwm.pwm_angle)
        #if (self.state == "spawned"): self.angleAccum[0]=pwm.period_str
        #else: self.angleAccum.append(pwm.period_str)

    def pwmToAngle(self, angle):
        #avgA -= 9305 34
        return math.radians(((angle-9361)*35/2882))

    def plot(self, timer):

        plt.clf()


        plt.plot(self.x, self.y, "r")
        if (self.state == "gotcha!"):
            plt.title(("Lap: " + str(self.lapN) + \
        ", Sum of angles ahead: " + str(round((self.turnAhead),4))))
            plt.plot(self.SSU[0], self.SSU[1], "go")
            plt.plot(self.SU[0], self.SU[1], "yo")
        else: plt.title("New Lap!")
        plt.plot(self.currentPos[0], self.currentPos[1], 'ks')
        plt.pause(0.0000000001)
        #return



if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    myMap = Mapping(True)
    ourtopic1 = "/pwm_high"
    ourtopic2 = "/sensors/core"
    ourtopic3 = "/drive_pwm"
    print("Waiting for ", ourtopic1, " and ", ourtopic2, " topic")
    #another = 'rosout'
    #found = False
    rospy.Subscriber(ourtopic3, drive_values, myMap.pwm_callback)
    #rospy.Subscriber(ourtopic1, pwm_high, myMap.pwm_callback)
    rospy.Subscriber(ourtopic2, VescStateStamped, myMap.state_callback)
    print("Connected!")
    rospy.spin()
