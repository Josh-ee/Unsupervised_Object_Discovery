#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros
import tf

from tf.transformations import quaternion_from_euler
from tf2_geometry_msgs import do_transform_pose

from geometry_msgs.msg import Point, PoseStamped
from unitree_legged_msgs.msg import HighCmd

import time

from discovery_via_interaction.srv import walkToLocation, walkToLocationResponse  

class WalkToLocationService:

    def __init__(self):
        rospy.init_node('walk_to_location_service', anonymous=True)

        # Initialize the service server
        self.service = rospy.Service('walk_to_location', walkToLocation, self.handle_walk_request)

        self.Go1_frame = 'camera_face'
        self.reference_frame = 'base'
        self.cmd_topic = '/high_cmd_to_robot'
        self.error_threshold = 0.1
        self.Kp = np.diag([0.5, 0.5, 0.5])   # PID Proportional coefficient           
        self.Kd = np.diag([0.05, 0.05, 0])   # PID Derivative coefficient                   
        self.Ki = np.diag([0.05, 0.05, 0])   # PID Integral coefficient                     

        self.cup_sub = rospy.Subscriber('/cups/purple', Point, self.cup_sub_callback)
        self.cmd_pub = rospy.Publisher(self.cmd_topic, HighCmd, queue_size=10)

        self.waypoint = None
        self.idle = True



    def handle_walk_request(self, req):

        success = False

        print("handling request")
        self.waypoint = Point(req.x, req.y, req.z)
        print("starting controller")
        
        print("entering main")
        outcome = self.main()


        if outcome == True:
            print("controller was succesfull")
            # time.sleep(1)
            # outcome_2 = self.pick_up()
            # if outcome_2:
            #     print(" cup pickup is succesfull")
            success = True

        print("returning out of service")
        return walkToLocationResponse(success=success)



    def bow_down(self):

        while not rospy.is_shutdown():

            cmd_msg = self.init_highcmd()
            cmd_msg.bodyHeight = -0.1
            self.cmd_pub.publish(cmd_msg)



    def cup_sub_callback(self, point):
        self.waypoint = point
        if self.idle:
            self.main()
    
    
    def main(self):

        self.idle = False

        # Step 1: find cup (cup_sub takes care of that step)

        # Step 2: go to it
        if self.waypoint:
            self.controller()
            print('Reached cup, picking it up...')
            # Step 3: pick up the object
            time.sleep(1)
            self.pick_up()
            return True

            # DONE (hopefully)



    def controller(self):
        """
        Controls a Go1 robot whose position is denoted by Go1_frame,
        to go to a position denoted by target_frame
        Inputs:
        - Go1_frame: the tf frame of the AR tag on the Go1
        - target_frame: the tf frame of the target AR tag
        """

        print("starting controller")
        #Create a publisher and a tf buffer, which is primed with a tf listener
        ctrlpub = self.cmd_pub
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)
        
        r = rospy.Rate(10) # 10hz

        rospy.sleep(0.1)

        prev_time = rospy.get_time()
        integ = np.zeros((3, 1))
        derivative = np.zeros((3, 1))
        previous_error = np.zeros((3, 1))

        # Loop until the node is killed with Ctrl-C
        while not rospy.is_shutdown():
            try:
                trans_odom_to_base_link = tfBuffer.lookup_transform(self.Go1_frame, self.reference_frame, rospy.Time())

                (roll, pitch, baselink_yaw) = tf.transformations.euler_from_quaternion(
                    [trans_odom_to_base_link.transform.rotation.x, trans_odom_to_base_link.transform.rotation.y,
                        trans_odom_to_base_link.transform.rotation.z, trans_odom_to_base_link.transform.rotation.w])

                waypoint = self.waypoint

                waypoint_trans = PoseStamped()
                waypoint_trans.pose.position.x = waypoint.x
                waypoint_trans.pose.position.y = waypoint.y
                waypoint_trans.pose.position.z = 0

                quat = quaternion_from_euler(roll, pitch, baselink_yaw)
                waypoint_trans.pose.orientation.x = quat[0]
                waypoint_trans.pose.orientation.y = quat[1]
                waypoint_trans.pose.orientation.z = quat[2]
                waypoint_trans.pose.orientation.w = quat[3]

                # Use the transform to compute the waypoint's pose in the base_link frame
                waypoint_in_base_link = do_transform_pose(waypoint_trans, trans_odom_to_base_link)
                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                    [waypoint_in_base_link.pose.orientation.x, waypoint_in_base_link.pose.orientation.y,
                        waypoint_in_base_link.pose.orientation.z, waypoint_in_base_link.pose.orientation.w])

                curr_time = rospy.get_time()

                # some debug output below
                # print("Current: {}, {}, {}".format(trans_odom_to_base_link.transform.translation.x, trans_odom_to_base_link.transform.translation.y, baselink_yaw))
                # print("Target: {}".format(waypoint))

                # Process trans to get your state error

                # Generate a control command to send to the robot
                x_error = waypoint_in_base_link.pose.position.x
                y_error = - waypoint_in_base_link.pose.position.y       #TODO: Verify sign
                error = np.array([[x_error], [y_error], [yaw]])
                print('error: ', error)

                # proportional term
                proportional = np.dot(self.Kp, error).squeeze()
                
                # integral term
                dt = curr_time - prev_time
                integ += error * dt
                integral = np.dot(self.Ki, integ).squeeze()

                # dervative term
                error_deriv = (error - previous_error ) / dt
                derivative = np.dot(self.Kd, error_deriv).squeeze()

                # Control commands:
                msg_high_cmd = self.init_highcmd()

                msg_high_cmd.yawSpeed = proportional[2] + derivative[2] + integral[2]
                msg_high_cmd.velocity[0] = max(min(proportional[0] + derivative[0] + integral[0], 1), -1) # Forward velocity; value range: [0,1]
                msg_high_cmd.velocity[1] = max(min(proportional[1] + derivative[1] + integral[1], 1), -1) # Lateral velocity

                control_command = msg_high_cmd

                previous_error = error
                prev_time = curr_time
                ctrlpub.publish(control_command)

                if np.linalg.norm(error) < self.error_threshold :
                    # Successfully reached
                    print("reached cup")
                    break

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print("TF Error in Turtlebot Controller: ", e)

            # Use our rate object to sleep until it is time to publish again
            r.sleep()



    def pick_up(self):
        ctrlpub = self.cmd_pub
        print("crouching")
        msg_high_cmd = self.init_highcmd()
        # msg_high_cmd.bodyHeight = 0.0 # # (unit: m) -> WARNING: This is NOT an absolute position w.r.t the ground, but rather w.r.t the current height....
        # pub2high_cmd.publish(msg_high_cmd)
        time.sleep(1)

        msg_high_cmd.bodyHeight = -0.025 # # (unit: m) -> WARNING: This is NOT an absolute position w.r.t the ground, but rather w.r.t the current height....
        ctrlpub.publish(msg_high_cmd)
        #pub2high_cmd.publish(msg_high_cmd)
        # self.waypoint = None
        time.sleep(0.15)

        return True



    def init_highcmd(self):

        HIGHLEVEL = 0x00

        msg_high_cmd = HighCmd()
        msg_high_cmd.levelFlag = HIGHLEVEL
        msg_high_cmd.mode = 2
        msg_high_cmd.gaitType = 1 # 0.idle  1.trot  2.trot running  3.climb stair
        msg_high_cmd.velocity[0] = 0.0 # [-1,1] # (unit: m/s), forwardSpeed in body frame
        msg_high_cmd.velocity[1] = 0.0 # [-1,1] # (unit: m/s), sideSpeed in body frame
        msg_high_cmd.bodyHeight = 0.0 # # (unit: m) -> WARNING: This is NOT an absolute position w.r.t the ground, but rather w.r.t the current height....
        msg_high_cmd.yawSpeed = 0.0

        return msg_high_cmd
    


        
if __name__ == "__main__":
    try:
        service = WalkToLocationService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
