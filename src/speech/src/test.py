#! /usr/bin/env python
import rospy
from std_msgs.msg import String, Int8, Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import time
import math

# going = np.array([0,0])
target_floor = 0
target_room = 0
heard = 0

def isKeepGoing(msg):
    global target_floor, target_room
    print(msg)
    if msg.data != '':
        target_floor = int(msg.data[0])
        target_room = int(msg.data[1:])
        print("target floor", target_floor)

def heard(msg):
    global heard
    heard = msg.data
    print("heard", heard)

if __name__ == '__main__':
    rospy.init_node("test", anonymous = True)
    pub_mention = rospy.Publisher("speech", Int8, queue_size = 20)
    # pub = rospy.Publisher("target_floor", Float64MultiArray, queue_size = 20)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        # rospy.Subscriber("isKeepGoing", Float64MultiArray, isKeepGoing)
        # pub_going.publish(going)
        rospy.Subscriber("target_floor", String, isKeepGoing)
        rospy.sleep(3)
        rospy.loginfo("see the elevator")
        # pub_mention.publish(1)
        rospy.loginfo("see the door open")
        # rospy.sleep(3)
        pub_mention.publish(2)
        # rospy.sleep(3)
        # pub_mention.publish(3)
        # rospy.Subscriber("HeardSomething", Int8, heard)
        # heard = input("it is reach the floor, you should input 3")
        # pub_hearing.publish(heard)
        rospy.sleep(1)
