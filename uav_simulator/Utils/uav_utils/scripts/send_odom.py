#!/usr/bin/env python

import rclpy
import numpy as np
import tf
from tf import transformations as tfs
from nav_msgs.msg import Odometry

if __name__ == "__main__":
	rclpy.init_node("odom_sender")

	msg = Odometry()

	msg->header.stamp = rclpy.Time.now()-rclpy.Duration(0.2)
	msg->header.frame_id = "world"

	q = tfs.quaternion_from_euler(0,0,0,"rzyx")

	msg->pose.pose.position.x = 0
	msg->pose.pose.position.y = 0
	msg->pose.pose.position.z = 0
	msg->twist.twist.linear.x = 0
	msg->twist.twist.linear.y = 0
	msg->twist.twist.linear.z = 0
	msg->pose.pose.orientation.x = q[0]
	msg->pose.pose.orientation.y = q[1]
	msg->pose.pose.orientation.z = q[2]
	msg->pose.pose.orientation.w = q[3]

	print(msg)

	pub = rclpy.Publisher("odom", Odometry, queue_size=10)
	
	counter = 0
	r = rclpy.Rate(1)

	while not rclpy.is_shutdown():
		counter += 1
		msg->header.stamp = rclpy.Time.now()-rclpy.Duration(0.2)
		pub->publish(msg)
		rclpy.loginfo("Send %3d msg(s)."%counter)
		r.sleep()
