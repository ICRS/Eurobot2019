#!/usr/bin/env python

import rospy

import tf_conversions

import tf2_ros

def get_laser_tf():
    lt = geometry_msgs.msg.TransformStamped()
    lt.header.stamp = rospy.Time.now()

    # Transform from robot frame to laser scanner frame
    lt.header.frame_id = "base"
    lt.child_frame_id = "laser_scanner"

    lt.transform.translation.x = 0
    lt.transform.translation.y = 0
    lt.transform.translation.z = 0

    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    lt.transform.rotation.x = q[0]
    lt.transform.rotation.y = q[1]
    lt.transform.rotation.z = q[2]
    lt.transform.rotation.w = q[3]
    
    return lt

def get_odometry_tf():
    ot = geometry_msgs.msg.TransformStamped()
    ot.header.stamp = rospy.Time.now()

    # Transform from robot frame to odometry frame
    ot.header.frame_id = "base"
    ot.child_frame_id = "odometry"

    ot.transform.translation.x = 0
    ot.transform.translation.y = 0
    ot.transform.translation.z = 0

    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    ot.transform.rotation.x = q[0]
    ot.transform.rotation.y = q[1]
    ot.transform.rotation.z = q[2]
    ot.transform.rotation.w = q[3]
    
    return ot


if __name__ == '__main__':
    broadcaster = tf2_ros.TransformBroadcaster()

    sleeper = rospy.Rate(100)

    while not rospy.is_shutdown():
        # Broadcast the laser scanner transform
        broadcaster.sendTransform(get_laser_tf())

        # Broadcast the odometry transform
        broadcaster.sendTransform(get_odometry_rf())

        # Maintain frequency
        sleeper.sleep()
