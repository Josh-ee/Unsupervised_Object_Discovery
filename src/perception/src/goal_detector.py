#!/usr/bin/env python

import rospy
import tf

from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header


class GoalDetector:

	def __init__(self):

		rospy.init_node("goal_detector", anonymous=True)

		self.tf_listener = tf.TransformListener()
		self.tfbr = tf.TransformBroadcaster()
		self.sub1 = rospy.Subscriber("/goal1_frame", Point, self.callback1)
		self.sub2 = rospy.Subscriber("/goal2_frame", Point, self.callback2)
		self.pub1 = rospy.Publisher("/goal1_center_coords", Point, queue_size=10)
		self.pub2 = rospy.Publisher("/goal2_center_coords", Point, queue_size=10)

		self.X1_base = None
		self.Y1_base = None
		self.Z1_base = 0

		self.X2_base = None
		self.Y2_base = None
		self.Z2_base = 0


	def callback1(self, msg):
		'''
		Computes coordinates of goal 1 in frame 'base'.
		Input:
			msg: Point = coordinates of goal 1 in frame 'camera_face'
		'''

		try:
			self.tf_listener.waitForTransform("/base", "/camera_face", rospy.Time(), rospy.Duration(10.0))
			center_in_base = self.tf_listener.transformPoint("/base", PointStamped(header=Header(stamp=rospy.Time(), frame_id="/camera_face"), point=msg))

			self.X1_base = center_in_base.point.x
			self.Y1_base = center_in_base.point.y

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			print("TF Error: " + e)


	def callback2(self, msg):
		'''
		Computes coordinates of goal 2 in frame 'base'.
		Input:
			msg: Point = coordinates of goal 2 in frame 'camera_face'
		'''

		try:
			self.tf_listener.waitForTransform("/base", "/camera_face", rospy.Time(), rospy.Duration(10.0))
			center_in_base = self.tf_listener.transformPoint("/base", PointStamped(header=Header(stamp=rospy.Time(), frame_id="/camera_face"), point=msg))

			self.X2_base = center_in_base.point.x
			self.Y2_base = center_in_base.point.y

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			print("TF Error: " + e)


	def main(self):
		'''
		Publishes TF transforms for goals 1 and 2 in frame 'base'.
		Also publishes these coordinates as a Point on a topic.
		'''

		while not rospy.is_shutdown():
			if self.X1_base is not None:
				self.tfbr.sendTransform((self.X1_base, self.Y1_base, self.Z1_base),
										tf.transformations.quaternion_from_euler(0, 0, 0),
										rospy.Time.now(),
										"goal1",
										"base")
				self.pub1.publish(Point(x=self.X1_base, y=self.Y1_base, z=self.Z1_base))
			if self.X2_base is not None:
				self.tfbr.sendTransform((self.X2_base, self.Y2_base, self.Z2_base),
										tf.transformations.quaternion_from_euler(0, 0, 0),
										rospy.Time.now(),
										"goal2",
										"base")
				self.pub2.publish(Point(x=self.X2_base, y=self.Y2_base, z=self.Z2_base))
			rospy.sleep(0.1)




if __name__ == "__main__":

	roomcenter = GoalDetector()
	roomcenter.main()
