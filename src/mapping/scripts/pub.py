#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32

i = 0.0
i2 = 0
check = True
stage = 0

def talker():
    global i, i2, stage, check
    pub = rospy.Publisher('final_heading_angle', Float32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(25) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
	#if (i <= 7.0 and check):
	#	i += 0.1
	#	if (i >= 7.0):
	#		check = False

	#if (not check):
	#	i -= 0.1
	#	if (i <= -3.0):
	#		i = -3.0
	#		check = True
	if (stage == 0):
		i += 0.05
		if (i >= 1.0):
			i = 0
			stage = 1
	if (stage == 1):
		i2 += 1
		if (i2 >= 20):
			i2 = 0
			stage = 2
	if (stage == 2):
		i -= 0.05
		if (i <= -3.0):
			i = -3.0
			stage = 3
	if (stage == 3):
		i2 += 1
		if (i2 >= 30):
			return
			
	print(i)
        pub.publish(i)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
