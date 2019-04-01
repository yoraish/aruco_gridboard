#!/usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped

class transformer:
	pose = PoseStamped()

	def __init__(self, *args):		
		rospy.init_node('main')
		rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback, queue_size=1)
		self.tf_broadcaster_ = tf.TransformBroadcaster()
		print("Initialization Completed")
	
	def euler_boundary(self, angle):
		if angle >= 3.1415:
			return angle - 6.283
		elif angle < -3.1415:
			return angle + 6.283
		else :
			return angle

	def pose_callback(self, pose):
		self.pose = pose

	def conversion(self):
		rpy = tf.transformations.euler_from_quaternion([self.pose.pose.orientation.x,
												 		self.pose.pose.orientation.y,
												 		self.pose.pose.orientation.z,
												 		self.pose.pose.orientation.w])
		#print("E",self.euler_boundary(rpy[0])," ",self.euler_boundary(rpy[1])," ",self.euler_boundary(rpy[2]))
		#transform quaternion to euler angle just in case you want to rotate your robot model
		q = tf.transformations.quaternion_from_euler(self.euler_boundary(rpy[0]),
													 self.euler_boundary(rpy[1]),
													 self.euler_boundary(rpy[2]))
		self.tf_broadcaster_.sendTransform((self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z), 
										    q,
											rospy.Time.now(),
											'robot_link',
											'map')
		

if __name__ == '__main__':

	middle_man = transformer()
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		try:
			middle_man.conversion()
			#middle_man.rebroadcast()
		except (tf.ConnectivityException, tf.ExtrapolationException):
			continue
