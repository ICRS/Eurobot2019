#! /usr/bin/python

import sys
import argparse
import rospy
from std_msgs.msg import Empty
from eurobot2019_messages.srv import *

PATH_PLAN_TOPIC = "path_planning"
DROP_SERVICE_NAME = "drop_module"
PICKUP_SERVICE_NAME = "pickup_module"
COLLISION_AVOIDANCE_TOPIC = "collision_avoidance"

# Called when there's an obstacle coming towards us!
def collision_callback():
    pass

def main(args):
    # Wait for the drop and pickup services to come on line
    rospy.wait_for_service(DROP_SERVICE_NAME)
    rospy.wait_for_service(PICKUP_SERVICE_NAME)

    # Advertise as a publisher on the PATH_PLAN_TOPIC
    # Todo: Replace Empty with the correct message type
    path_pub = rospy.Publisher(PATH_PLAN_TOPIC, Empty, queue_size=1)
    
    # Listen on the COLLISION_AVOIDANCE topic
    rospy.Subscriber(COLLISION_AVOIDANCE_TOPIC, Empty, collision_callback)

    # 10 Hz update rate
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.spinOnce()

def parse_arguments():
    parser = argparse.ArgumentParser(description="Main entry point for ICRS' Eurobot entry")
    args = parser.parse_args()

if __name__ == "__main__":
    args = parse_arguments()
    main(args)
