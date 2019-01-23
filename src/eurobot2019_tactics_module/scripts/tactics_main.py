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

def main(args):
    rospy.wait_for_service(DROP_SERVICE_NAME)
    rospy.wait_for_service(PICKUP_SERVICE_NAME)

    pub = rospy.Publisher(PATH_PLAN_TOPIC, 2dPoint, queue_size=1)

    # 10 Hz
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pass

def parse_arguments():
    parser = argparse.ArgumentParser(description="Main entry point for ICRS' Eurobot entry")
    args = parser.parse_args()

if __name__ == "__main__":
    args = parse_arguments()
    main(args)
