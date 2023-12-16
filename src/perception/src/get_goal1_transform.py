#!/usr/bin/env python

import rospy
import tf2_ros
import sys

from geometry_msgs.msg import Point


def get_transformation(robot_frame, target_frame):
  """
  Gets coordinates of target_frame in robot_frame.
  Inputs:
  - robot_frame: the tf frame of your robot
  - target_frame: the tf frame of the target AR tag
  """

  pub = rospy.Publisher('/goal1_frame', Point, queue_size=10)
  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)
  
  r = rospy.Rate(10) # 10hz

  while not rospy.is_shutdown():

    try:
      trans = tfBuffer.lookup_transform(robot_frame, target_frame, rospy.Time())

      x = trans.transform.translation.x
      y = trans.transform.translation.y
      z = trans.transform.translation.z

      pt = Point()
      pt.x = x
      pt.y = -y
      pt.z = z

      pub.publish(pt)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
      print("error: ", e)
      pass

    r.sleep()

      


if __name__ == '__main__':

  rospy.init_node('AR_goal1', anonymous=True)

  try:
    get_transformation('camera_face', sys.argv[1])

  except rospy.ROSInterruptException:
    pass