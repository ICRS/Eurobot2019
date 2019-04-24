#!/usr/bin/env python

import rospy
import time
import RPi.GPIO as GPIO

from eurobot2019_messages.msg import collision_avoidance as CollisionAvoidanceMsg

def process_params():
    pass

def setup_gpio(triggers, echos):
    for trig in triggers:
        GPIO.setup(trig, GPIO.OUT)
    for echo in echos:
        GPIO.setup(echo, GPIO.IN)

def get_distance(trigger, echo):
    GPIO.output(trigger, True)
    time.sleep(0.00001)
    GPIO.output(tigger, False)

    StartTime = time.time()
    StopTime = time.time()

    while(GPIO.input(echo) == 0):
        StartTime = time.time()

    while(GPIO.input(echo) == 1):
        StartTime = time.time()

    dt = StopTime - StartTime

    distance = (TimeElapsed * 34300) / 2
    
    return distance

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(2, GPIO.IN)

    while(GPIO.input(2)):
        pass

    pub = rospy.Publisher('collision_avoidance', CollisionAvoidanceMsg)
    rospy.init_node('collision_avoidance_node')
    rate = rospy.Rate(20)

    triggers = [ 3, 17, 22, 9 ]
    echos    = [ 4, 27, 10, 11 ]
    
    setup_gpio(triggers, echos)

    data_msg = CollisionAvoidanceMsg
    distance_threshold = rospy.get_param('distance_threshold', 0.1)

    while not rospy.is_shutdown():
        data_msg.data = []
        for i in len(triggers):
            dist = get_distance(triggers[i], echos[i])

            if dist < distance_threshold:
                data_msg.data.append(True)
            else:
                data_msg.data.append(False)

        pub.publish(data_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

