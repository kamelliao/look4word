#! /usr/bin/env python
import rospy
from std_msgs.msg import String, Int8, Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import time
import math
import os

import speech_recognition as sr

from speech import speech2text
from oracle import Oracle
from play_audio import *


going = 0
mention_people = 0
floor = ''
heard = 0
oracle = Oracle()

def isKeepGoing(msg):
    global going
    is_keep_going, suggestion = oracle.check_go_forward(
        target=''.join([str(n) for n in floor]),
        history=msg.data
    )
    going = (is_keep_going == 'yes')


def mention(msg):
    global mention_people
    # print("messa")
    mention_people = msg.data
    if mention_people != 0:
        print("play audio")
        select_audio(mention_people) # play audio
        mention_people = 0


if __name__ == '__main__':
    rospy.init_node("speech", anonymous = True)
    pub = rospy.Publisher("target_floor", String, queue_size = 20)
    pub_going = rospy.Publisher("isKeepGoing_reply", Int8, queue_size = 10)
    pub_hearing = rospy.Publisher("HeardSomething", Int8, queue_size = 10)
    rospy.Subscriber("isKeepGoing", Float64MultiArray, isKeepGoing)
    rospy.Subscriber("speech", Int8, mention)

    rospy.loginfo("hello")

    r = sr.Recognizer()
    microphone = sr.Microphone(device_index=9, sample_rate=16000, chunk_size=2048)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        # speech recognition
        if not floor:
            get_instruction() # play audio
            utterance = speech2text(r, microphone, mode='google_api', language='en-US')
            room_no = oracle.get_room(utterance)
            # room_no = '412'
            rospy.loginfo(f"GoogleApi | room: '{room_no}' | transcript: '{utterance}'")
            # update floor only when valid room number is detected
            if room_no.isnumeric() and len(room_no)==3:
                floor = room_no
                if floor[0] == '1':
                    reply_instruction('first_floor')
                else:
                    reply_instruction('elevator')  # play audio
        else:
            # if mention_people != 0:
            #     print("play audio")
            #     select_audio(mention_people) # play audio
            #     mention_people = 0
            # set elevator state
            label, proba = speech2text(r, microphone, mode='elevator')
            rospy.loginfo(f'Elevator state | label {label} | proba {proba}')
            if label == '電梯上樓':
                heard = 1
            elif label == '關門中':
                heard = 2
            else:
                # state = oracle.elevator_out(floor[0], label)
                label_map = {
                    '一樓到了': '1',
                    '二樓到了': '2',
                    '三樓到了': '3',
                    '四樓到了': '4',
                }
                heard = 3 if label_map.get(label) == floor[0] else 4

        # publish something
        print("floor:", floor)
        pub.publish(floor)
        pub_going.publish(going)
        pub_hearing.publish(heard)
        rospy.loginfo(heard)
        # heard = input("it is reach the floor, you should input 3")
        # pub_hearing.publish(heard)
        print("mention", mention_people)
        rospy.sleep(1)
