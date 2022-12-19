#! /usr/bin/env python
import rospy
from std_msgs.msg import String, Float64MultiArray, Int8
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import cv2
import math

received_image = np.array([0,0,0,0,0,0,0])
old_received_image = []
target_floor = 0
target_room = 0
cmd = Twist()
ctrl_c = False
roll = 0
pitch = 0
yaw = 0
door_num = np.array([0,0])
door_num = Float64MultiArray(data = door_num)   
keepGoing = 0
ISee = 0
IHeard = 0
cur_door_num = 0
movemode = 0
waitForEleOpen = 0
mention = 0
depth_buffer = []
depth_msg = None
camera_info_buffer = []
camera_info_msg = None
camera_msg = None
msg_list = []
depth = np.array([0,0,0,0,0], dtype='f')
tmp = 0
count = 0
count2 = 0
count3 = 0
inf = float("inf")

speed = 1
center = [0,0]
degree = 0
depth_image = np.array((720,1280))
targetAchieve = 0
check = 0


def targetFloor(msg):
    global target_floor
    global target_room
    if msg.data != '':
        target_floor = int(msg.data[0])
        target_room = int(msg.data[1:])
        # rospy.loginfo("target floor")
        # rospy.loginfo(target_floor)
        # rospy.loginfo("target room")
        # rospy.loginfo(target_room)
        
def SeeSomething(msg):
    global ISee
    ISee = msg.data
    # rospy.loginfo(ISee)    

def SeeDoorNum(msg):
    global cur_door_num
    cur_door_num = msg.data
    door_num.data[1] = cur_door_num
    
    rospy.loginfo(door_num.data[0])
    rospy.loginfo(door_num.data[1])  

def HeardSomething(msg):
    global IHeard
    IHeard = msg.data
    # rospy.loginfo(IHeard)

def isKeepGoing(msg):
    global keepGoing
    keepGoing = msg.data
    # rospy.loginfo(keepGoing)

def depth_callback(msg):
    global depth_msg, depth_buffer, depth, depth_image
    # print("depth callback")
    # depth = np.array([0,0,0,0,0], dtype='f')
    depth_msg = msg
    depth_buffer.append(msg)
    
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(depth_msg)
    # cv2.imshow("1213",depth_image)
    # cv2.waitKey(1)
    # rospy.loginfo(depth_image.shape)
    # shape = depth_image.shape[]
    # for i in range(5):
    # rospy.loginfo("depth_msg")
    # rospy.loginfo(np.nanmean(depth_image[:, 0:256]))
    
    depth[0] = (np.nanmean(depth_image[360:, 0:256]))
    depth[1] = (np.nanmean(depth_image[360:, 256:512]))
    depth[2] = (np.nanmean(depth_image[360:, 512:768]))
    depth[3] = (np.nanmean(depth_image[360:, 768:1024]))
    depth[4] = (np.nanmean(depth_image[360:, 1024:1280]))
    # rospy.loginfo(depth)
    
    


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
    if (time == -inf or time == inf):
        time = 0
    # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
    print("=======Now moving========")
    print("time:",round(time))
    print ("cmd vel",cmd.linear.x)
    print("speed",speed)
    while (1):
        # Publish the velocity
        pub_vel.publish(cmd)
        # i += 0.1
        rospy.sleep(0.1)
        if count2 == round(time)*10:
            # print("cmd:", cmd)
            print("time:", count2)
            break
        count2 += 1 
        
    cmd.linear.x = 0
    # set velocity to zero to stop the robot
    # stop_robot()    

def rotate(degrees):
    global count
    count = 0
    rospy.loginfo("rotate")
    # time.sleep(1)
    target_rad = (degrees * math.pi/180)
    print(depth)
    print(depth[tmp])
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

    # print ("hahahahah")
    
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

def stop_robot():
        rospy.loginfo("shutdown time! Stop the robot")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        pub_vel.publish(cmd)
        

def CenterAndDegree(msg):
    global center, degree, check
    if check == 0:
        # print(msg.data)
        count3 = 0
        center[0], center[1], degree = msg.data
        center[0] = round(center[0])
        center[1] = round(center[1])
        print("center", center)
        print("degree", degree)
        # rospy.sleep(2)
    else:
        return

def rotate_target():
    global count3
    count3 = 0
    target_rad = (degree * math.pi/180)
    print("degree:", degree)
    while (1):
        cmd.angular.z = 2.5 * (target_rad)
        pub_vel.publish(cmd)
        count3 += 1
        print("count3:", count3)
        rospy.sleep(0.1)
        if count3 == 10:
            break
    
    cmd.angular.z = 0
    # rospy.sleep(5)
    ISee = 0
    print("blabla")

if __name__ == '__main__':
    print("hello it's me")
    rospy.init_node("main_program", anonymous = True)
    pub = rospy.Publisher('speech', Int8, queue_size = 20)
    pub_vel = rospy.Publisher('/temp_cmd_vel', Twist, queue_size = 10)
    pub_speech = rospy.Publisher('isKeepGoing', Float64MultiArray, queue_size = 10)
    # rospy.Subscriber ('/odom', Odometry, odom_callback)#
    rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, callback=depth_callback, queue_size=1)
    rospy.Subscriber("CenterAndDegree", Float64MultiArray, CenterAndDegree)
    rospy.Subscriber('SeeSomething', Int8, SeeSomething)  
    rospy.Subscriber('HeardSomething', Int8, HeardSomething)#1 
    rospy.Subscriber('SeeDoorNum', Int8, SeeDoorNum)#
    rospy.loginfo("hello")
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        print("original target floor", target_floor)
        if target_floor == 0:#waiting for the target floor
            rospy.Subscriber('target_floor', String, targetFloor)#
            rospy.sleep(1)

        while(target_floor == 2 or target_floor == 3 or target_floor == 4):#elevator mode
            # rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, callback=depth_callback, queue_size=1)
            rospy.loginfo("searching for elevator")
            while(1):#
                if(ISee == 1 or ISee == 2):
                    rospy.loginfo("see the elevator")
                    check = 1
                    rospy.sleep(1)
                    for i in range(5):
                        rotate(degree)
                        rospy.sleep(2)
                        move_straight_time("forward", speed, depth_image[center]/speed)
                        rospy.sleep(2)
                        
                    rospy.sleep(3)
                    # ISee = 0
                    while(1):
                        rospy.sleep(1)
                        rospy.Subscriber('SeeSomething', Int8, SeeSomething)
                        print("waiting for see elevator sign")
                        if(ISee == 2 or ISee == 1):
                            rospy.loginfo("see the elevator panel")
                            print("Isee:", ISee)
                            check = 0
                            rospy.sleep(1)
                            check = 1
                            rospy.sleep(1)
                            ISee = 0
                            
                            rospy.sleep(20)
                            mention = 1
                            pub.publish(mention)#
                            mention = 0
                            pub.publish(mention)#
                            rospy.loginfo("break1")
                            break
                    while(1):
                        rospy.loginfo("Waiting for the elevator")

                        if(IHeard == 1):#
                            rospy.loginfo("elevator arrived")
                            
                            move_straight_time("forward", speed, depth[tmp]/speed)

                            while(1):
                                # rospy.Subscriber('HeardSomething', Int8, HeardSomething)
                                rospy.loginfo("Waiting for the door closed")
                                
                                if(IHeard == 2):
                                    rospy.loginfo("elevator door closed")
                                    mention = 2
                                    pub.publish(mention)#
                                    mention = 0
                                    pub.publish(mention)#
                                    rotate(180)#rotate 180 degree to the elevator door
                                    waitForEleOpen = 1
                                    rospy.loginfo("break2")
                                    break
                        if(waitForEleOpen == 1):
                            rospy.loginfo("break3")
                            break
                        rospy.sleep(1)
                    while(1):
                        rospy.loginfo("Waiting for reaching the floor")
                        if(IHeard == 3):#
                            rospy.loginfo("reach the target floor")
                            movemode = 1
                            move_straight_time("forward", speed, depth[tmp]/speed)#
                            rospy.loginfo("break4")
                            print("=======================================go out the elevator===============================================")
                            rospy.sleep(100)
                            break
                else:
                    rospy.loginfo("elevator mode")
                    threshold = 100 #

                    if(threshold > 10000):#
                        rospy.loginfo("no way in front")

                    else:
                        rospy.loginfo("keep finding the elevator")
                        # rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, callback=depth_callback, queue_size=1)
                        rotate_degree()
                        rospy.sleep(2)
                        print(depth[tmp])
                        move_straight_time("forward", speed, depth[tmp]/speed)#0.5 change to depth[tmp]
                        rospy.sleep(5)
    
                if(movemode == 1):
                    rospy.loginfo("break5")
                    break
                rospy.sleep(1)

            if(movemode == 1):
                rospy.loginfo("break elevator mode")
                break
            rospy.sleep(1)

        while(target_floor == 1 or target_floor == 2 or target_floor == 3 or target_floor == 4):#move mode
            rospy.loginfo("Now is finding door mode")
            # rospy.Subscriber('SeeSomething', Int8, SeeSomething)#
        
            if(ISee == 3):
                rospy.loginfo("see the door")
                for i in range(5):
                    rotate(degree)
                    rospy.sleep(2)
                    move_straight_time("forward", speed, depth_image[center]/speed)
                    rospy.sleep(2)
                # rospy.Subscriber("CenterAndDegree", Float64MultiArray, CenterAndDegree)
                rospy.sleep(15)
                ISee = 0
                if(ISee == 4):
                    rospy.loginfo("see the door sign")
                    rospy.sleep(5)
                    if (ISee == 5):
                        rospy.loginfo("we find the target room")
                        mention = 3
                        pub.publish(mention)
                        mention = 0
                        pub.publish(mention)#
                        targetAchieve = 1
                        rospy.loginfo("break7")
                        break
                    else:#not the target room
                        rospy.sleep(15)
                        rotate(90)

                else:
                    rospy.loginfo("don't see door sign")
                    rotate(90)
                    # move_straight_time("forward", 1, 1)#
                    # 
            else:
                threshold = 100#
                print("door mode")

                if(threshold > 10000):#
                    rospy.loginfo("no way in front")
                    # 

                else:
                    rospy.loginfo("keep finding door") 
                    rotate_degree()
                    rospy.sleep(2)
                    move_straight_time("forward", speed, depth[tmp]/speed)#0.5 change to depth[tmp]
                    rospy.sleep(5)
            rospy.sleep(2)            
        if targetAchieve==1:
            rospy.loginfo("oh break")
            break
