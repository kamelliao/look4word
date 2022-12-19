#! /usr/bin/env python
import rospy
from std_msgs.msg import String, Float64MultiArray, Int8
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import cv2
import time
import math

depth_buffer = []
depth_msg = None
camera_info_buffer = []
camera_info_msg = None
camera_msg = None
msg_list = []
depth = []
cmd = Twist()
yaw = 0
tmp = 0
count = 0
count2 = 0
count3 = 0
inf = float("inf")
speed = 1
depth_image = np.array((720,1280))

center = [0,0]
degree = 0
ISee = 0


def camera_callback(msg):
    global camera_msg, msg_list
    camera_msg = msg
    if depth_msg is None or camera_info_msg is None or camera_msg is None:
        return
    msg_list.append([camera_msg, copy.deepcopy(depth_msg), copy.deepcopy(camera_info_msg)])

def depth_callback(msg):
    global depth_msg, depth_buffer, depth, depth_image
    depth = []
    depth_msg = msg
    depth_buffer.append(msg)
    
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(depth_msg)
    # rospy.loginfo(depth_image.shape)
    # shape = depth_image.shape[]
    # for i in range(5):
    # rospy.loginfo("depth_msg")
    # rospy.loginfo(np.nanmean(depth_image[:, 0:256]))
    
    depth.append(np.nanmean(depth_image[360:, 0:256]))
    # rospy.loginfo(np.nanmean(depth_image[:, 0:256]))
    depth.append(np.nanmean(depth_image[360:, 256:512]))
    # rospy.loginfo(depth)
    depth.append(np.nanmean(depth_image[360:, 512:768]))
    # rospy.loginfo(depth)
    depth.append(np.nanmean(depth_image[360:, 768:1024]))
    # rospy.loginfo(depth)
    depth.append(np.nanmean(depth_image[360:, 1024:1280]))
    # rospy.loginfo(depth)
    # cv2.imshow("show",depth_image)
    # cv2.waitKey(1)
    # cv2.destroyWindow("show")
    
    # rotate_degree()
    # rospy.sleep(2)

    # move_straight_time("forward", speed, depth[tmp]/speed)#0.5 change to depth[tmp]        
    # rospy.sleep(2)
    # rospy.loginfo("depth_buffer")
    # rospy.loginfo(depth_buffer)   

def camera_info_callback(msg):
    global camera_info_msg, camera_info_buffer
    camera_info_msg = msg
    camera_info_buffer.append(msg)


def move_straight_time(motion, speed, time):

    global count2
    rospy.loginfo("move forward")
    count2 = 0
    # Initilize velocities
    cmd.linear.y = 0
    cmd.linear.z = 0
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = 0

    if motion == "forward":
        cmd.linear.x = speed
        if cmd.linear.x == inf:
            cmd.linear.x = speed
    elif motion == "backward":
        cmd.linear.x = - speed

    i = 0
    # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
    print("=======Now moving========")
    print("time:",round(time))
    print ("cmd vel",cmd.linear.x)
    print("speed",speed)
    while (1):
        # Publish the velocity
        pub_vel.publish(cmd)
        # i += 0.1
        count2 += 1 
        rospy.sleep(0.1)
        if count2 == round(time)*10:
            # print("cmd:", cmd)
            print("time:", count2)
            break
        
    cmd.linear.x = 0
    # set velocity to zero to stop the robot
    # stop_robot()    

def rotate(degrees):
    global count
    count = 0
    rospy.loginfo("rotate")
    # time.sleep(1)
    target_rad = (degrees * math.pi/180)
    # print(depth)
    # print(depth[tmp])
    if (depth[tmp]<1.5):
        print("mode2")
        while (1):
            target_rad = (-44 * math.pi/180)
            # cmd.linear.x = 1
            cmd.angular.z = 10 * (target_rad - yaw)
            pub_vel.publish(cmd)
            print(cmd.linear.x)
            print(degrees)
            # print("target = {} current:{}",degrees, yaw)
            count += 1
            rospy.sleep(0.1)
            if count == 20:
                break
    else :
        print("mode1")
        while (1):
            # cmd.linear.x = 1
            cmd.angular.z = 2.5 * (target_rad - yaw)
            pub_vel.publish(cmd)
            print(cmd.linear.x)
            print(degrees)
            # print("target = {} current:{}",degrees, yaw)
            count += 1
            rospy.sleep(0.1)
            if count == 10:
                break

    print ("hahahahah")
    
    cmd.angular.z = 0
    # stop_robot()

def rotate_degree():
    global depth,tmp 
    tmp = 2
    longest = 0
    for i in range(len(depth)):
        if depth[i]>longest:
            longest = depth[i]
            tmp = i 
    print("tmp",tmp)
    if tmp == 0:
        rotate(44) 
    if tmp == 1:
        rotate(22) 
    if tmp == 2: 
        rotate(0)
    if tmp == 3:
        rotate(-22) 
    if tmp == 4:
        rotate(-44)

def CenterAndDegree(msg):
    global depth_image, count3, ISee
    if ISee == 1:
        # print(msg.data)
        count3 = 0
        center[0], center[1], degree = msg.data
        center[0] = round(center[0])
        center[1] = round(center[1])
        print("center", center)
        print("degree", degree)
        rospy.sleep(2)
        target_rad = (degree * math.pi/180)
        print("degree:", degree)
        while (1):
            cmd.angular.z = 2.5 * (target_rad)
            pub_vel.publish(cmd)
            # print("cmd:", cmd.angular.z)
            # print("target = {} current:{}",degrees, yaw)
            count3 += 1
            print("count3:", count3)
            rospy.sleep(0.1)
            if count3 == 10:
                break
        
        cmd.angular.z = 0
        rospy.sleep(3)
        ISee = 0
        print("blabla")
    else:
        return
    # move_straight_time("forward", speed, depth_image[center[1],center[0]]/speed)
    # rospy.sleep(3)

def SeeSomething(msg):
    global ISee
    ISee = msg.data
    # rospy.loginfo(ISee) 

if __name__ == '__main__':
    rospy.init_node("test_node", anonymous = True)
    pub_vel = rospy.Publisher('/temp_cmd_vel', Twist, queue_size = 10)
    rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, callback=depth_callback, queue_size=1)
    rospy.Subscriber("CenterAndDegree", Float64MultiArray, CenterAndDegree)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.Subscriber('SeeSomething', Int8, SeeSomething)#
        print("ISSee:", ISee)
        # rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, callback=depth_callback, queue_size=1)
        if(ISee == 1):
            rospy.loginfo("see the elevator")
            # rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, callback=depth_callback, queue_size=1)
            for i in range(10):
                # rospy.Subscriber("CenterAndDegree", Float64MultiArray, CenterAndDegree)
                rospy.sleep(0.1)
            print("go to sleep")
            rospy.sleep(15)
            ISee = 0
            print("Isee", ISee)
            # rospy.sleep(8)

            # while(1):
            #     rospy.Subscriber('SeeSomething', Int8, SeeSomething)
            #     if(ISee == 2):
            #         rospy.loginfo("see the elevator panel")
            #         rospy.Subcriber("CenterAndDegree", Float64MultiArray, CenterAndDegree)
            #         rospy.sleep(8)
            #         mention = 1
            #         rospy.loginfo("break1")
            #         break

        rospy.sleep(1)
        