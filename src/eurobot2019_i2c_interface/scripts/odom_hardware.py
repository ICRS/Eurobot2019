#!/usr/bin/env python

import rospy

import tf_conversions as tf_c
import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform
from nav_msgs.msg import Odometry

prev_msg = Transform()

broadcaster = tf2_ros.TransformBroadcaster()
pub = rospy.Publisher("odom", Odometry, queue_size=10)

def callback(transform):
    global prev_msg
    global pub
    global broadcaster
    dur = rospy.Time.now() - callback.last_time
    callback.last_time += dur
    dt = dur.to_sec()

    dx = (transform.translation.x - prev_msg.translation.x) / dt
    dy = (transform.translation.y - prev_msg.translation.y) / dt

    q1 = transform.rotation
    q2 = prev_msg.rotation

    euler1 = tf_c.transformations.euler_from_quaternion([q1.x, q1.y, q1.z, q1.w])
    euler2 = tf_c.transformations.euler_from_quaternion([q2.x, q2.y, q2.z, q2.w])

    dyaw = (euler1[2] - euler2[2]) / dt

    tf_msg = TransformStamped()
    tf_msg.header.stamp = callback.last_time
    tf_msg.header.frame_id = "odom"
    tf_msg.child_frame_id = "base_link"

    tf_msg.transform = transform

    odom_msg = Odometry()
    odom_msg.header.stamp = callback.last_time
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"

    odom_msg.pose.pose.position.x = transform.translation.x
    odom_msg.pose.pose.position.y = transform.translation.y
    odom_msg.pose.pose.orientation = transform.rotation

    odom_msg.twist.twist.linear.x = dx
    odom_msg.twist.twist.linear.y = dy
    odom_msg.twist.twist.angular.z = dyaw

    broadcaster.sendTransform(tf_msg)
    pub.publish(odom_msg)

    prev_msg = transform


def main():
    rospy.init_node("odom_interface")
    callback.last_time = rospy.Time.now()
    sub = rospy.Subscriber("odom_raw", Transform, callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass;
