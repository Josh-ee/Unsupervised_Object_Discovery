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
import datetime


'''
Before running this code, make sure the vision node is running
(the one that publishes the cup Point to /cups/purple)
'''


class MainNode:


    def __init__(self, nb_cups = 2, arbitrary_target = None):

        rospy.init_node('main_node', anonymous=True)
        self.state = 'idle'     # Discrete State Machine initialization
        self.nb_cups = nb_cups  # Total number of cups to be picked up

        self.Go1_frame = 'camera_face'  # Name of frame linked to Go1 robot             
        self.reference_frame = 'base'   # Reference frame used in the control loop
        self.cmd_topic = '/high_cmd_to_robot'  # High commands control topic                                 

        self.error_threshold = 0.2  # Maximum acceptable norm of error   
        self.second_try = True      # Tells robot when to try picking up cup a second time          

        # Error defined as [dx, dy, dyaw]
        self.Kp = np.diag([0.5, 0.5, 0])   # PID Proportional coefficient            
        self.Kd = np.diag([0.1, 0.1, 0])   # PID Derivative coefficient                    
        self.Ki = np.diag([0.1, 0.1, 0])   # PID Integral coefficient                      

        self.cup_coords = None              # Last known coordinates of the cup
        self.goal1_coords = None            # Last known coordinates of the 1st goal
        self.goal2_coords = None            # Last known coordinates of the 2nd goal
        self.waypoint = self.cup_coords     # Coordinates used for the control loop

        self.cup_sub = rospy.Subscriber('/cups/purple', Point, self.cup_sub_callback)
        self.goal1_sub = rospy.Subscriber('/goal1_center_coords', Point, self.goal1_callback)
        self.goal2_sub = rospy.Subscriber('/goal2_center_coords', Point, self.goal2_callback)
        self.cmd_pub = rospy.Publisher(self.cmd_topic, HighCmd, queue_size=10)

        self.action_list = []       # List of actions taken by the robot (logging all actions)


    def goal1_callback(self, point):
        '''
        Callback function of teh goal 1 coordinates subscriber
        Updates the goal1_coords variable.
        Input:
            point: geometry_msgs/Point = goal1 coordinates given in 'base' frame
        '''
        self.goal1_coords = point
        self.goal1_coords.x += 0.1


    def goal2_callback(self, point):
        '''
        Callback function of teh goal 2 coordinates subscriber
        Updates the goal2_coords variable.
        Input:
            point: geometry_msgs/Point = goal2 coordinates given in 'base' frame
        '''
        self.goal2_coords = point
        self.goal2_coords.x += 0.1


    def cup_sub_callback(self, point):
        '''
        Callback function of the cup coordinates subscriber
        Updates the cup_coords variable.
        Input:
            point: geometry_msgs/Point = cup coordinates given in 'base' frame
        '''
        self.cup_coords = point

        # If robot is in the pickup cup phase, updates waypoint variable
        if self.state == 'pickup' or self.state == 'idle':
            self.waypoint = self.cup_coords

        # If robot has just been initialized, starts main loop
        if self.state == 'idle':
            rospy.loginfo('Starting')
            self.action_list.append('starting')
            self.action_list.append('cup detected')
            self.main()
    

    def main(self):
        '''
        Main loop of the software.
        Looks for a cup, goes to it, picks it up.
        '''
        self.state = 'pickup'

        ### STEP 1: find cup (cup_sub takes care of that step)

        ### STEP 2: go to the cup
        if self.cup_coords:
            self.action_list.append('going to cup')
            self.controller()   # Goes to current self.waypoint in current self.reference_frame
            # Logging
            rospy.loginfo('Reached cup, picking it up...')
            self.action_list.append('trying to pick up cup')

        ### STEP 3: pick up the object
            rospy.sleep(1)
            self.pickup_cup()

        ### DONE
    

    def controller(self):
        """
        Note: This code was taken and adapted from lab 8.
        Controls a Go1 robot whose position is denoted by Go1_frame,
        to go to a position denoted by self.waypoint given in self.current_frame
        """

        #Create a publisher and a tf buffer, which is primed with a tf listener
        ctrlpub = self.cmd_pub
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)
        rospy.sleep(0.5)
        
        r = rospy.Rate(10) # 10hz

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

                self.cup_sub.unregister()
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

                # Define state error [dx, dy, dyaw]
                x_error = waypoint_in_base_link.pose.position.x
                y_error = - waypoint_in_base_link.pose.position.y
                yaw_error = np.arctan(y_error / (x_error + 1e-6 * int(x_error == 0)))
                error = np.array([[x_error-0.12], [y_error], [yaw_error]])
                # print('error: ', error)

                # Proportional term
                proportional = np.dot(self.Kp, error).squeeze()
                
                # Integral term
                dt = curr_time - prev_time
                integ += error * dt
                integral = np.dot(self.Ki, integ).squeeze()

                # Derivative term
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

                self.cup_sub = rospy.Subscriber('/cups/purple', Point, self.cup_sub_callback)

                if np.linalg.norm(error) < self.error_threshold :       # If error small enough
                    break                                               # Exit control loop

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print("TF Error in Controller: ", e)
                pass

            # Use our rate object to sleep until it is time to publish again
            r.sleep()
    

    def pickup_cup(self):
        '''
        Attempts to pickup up a cup by crouching
        (if the cup is below, will be picked up by the robot's velcro beard).
        '''
        ctrlpub = self.cmd_pub
        print("Crouching")
        self.cup_coords = None      # Reset cup coords (useful when checking if cup was picked up)

        msg_high_cmd = self.init_highcmd()
        rospy.sleep(1.5)

        # Crouch down
        msg_high_cmd.bodyHeight = -0.099 # # (unit: m) -> WARNING: This is NOT an absolute position w.r.t the ground, but rather w.r.t the current height....
        ctrlpub.publish(msg_high_cmd)
        ctrlpub.publish(self.init_highcmd())
        rospy.sleep(1)
        # Get back up
        msg_high_cmd.bodyHeight = +0.099 # # (unit: m) -> WARNING: This is NOT an absolute position w.r.t the ground, but rather w.r.t the current height....
        ctrlpub.publish(msg_high_cmd)
        ctrlpub.publish(self.init_highcmd())
        rospy.sleep(1)

        self.state = 'check pickup'
        self.check_if_cup_moved()
    

    def check_if_cup_moved(self):
        '''
        Checks if the cup was successfully picked up by stepping
        back and checking if cup is still on the ground,
        and then calls the appropriate function.
        '''
        self.action_list.append('stepping back to check if cup is picked up')
        rospy.sleep(1)
        ctrlpub = self.cmd_pub
        self.state = None

        tt = 0      # Initialize timer
        msg_high_cmd = self.init_highcmd()
        rate_freq_send_commands = 120 # Hz
        time_tot = 1.0 # sec
        Nsteps_timeout = int(time_tot * rate_freq_send_commands)
        ros_loop = rospy.Rate(rate_freq_send_commands) # Hz

        rospy.loginfo("Moving back...")

        while tt < Nsteps_timeout:

            # Control commands:
            msg_high_cmd.yawSpeed = 0.0
            msg_high_cmd.velocity[0] = -0.2     # Forward velocity; value range: [0,1]
            msg_high_cmd.velocity[1] = 0.0      # Lateral velocity

            ctrlpub.publish(msg_high_cmd)

            ros_loop.sleep()
            tt += 1

        rospy.sleep(3)

        self.cup_sub.unregister()   # Required to use wait_for_message on that topic
        cup_found = None
        try:
            cup_found = rospy.wait_for_message('/cups/purple', Point, timeout=0.5)
        except:
            pass
        self.cup_sub = rospy.Subscriber('/cups/purple', Point, self.cup_sub_callback)
        
        if cup_found:   # if cup is still on the ground, it has NOT been picked up
            if self.second_try == True:
                self.second_try = False
                rospy.loginfo('Cup is still here, trying a second time')
                self.action_list.append('cup is still here, trying once more to pick it up')
                self.main()
            else:
                self.action_list.append('cup still there, it is immovable')
                rospy.loginfo('Cup is immovable')
                self.log()

                # # Searching for another cup:
                # self.state = 'looking for other cup'
                # while self.cup_coords is None:
                #     self.look_around()
                #     rospy.sleep(0.2)
                # self.state = 'pickup'
                # self.waypoint = self.cup_coords
                # self.main()

        else:   # Cup is not here anymore, meaning it was picked up
            self.error_threshold = 0.25
            self.action_list.append('successfully picked up the cup')
            rospy.loginfo("Cup was picked up!")

            self.cup_sub.unregister()   # Going to goal, no need to look for cup anymore (for now)
            if self.goal1_coords is not None:
                self.go_to_goal1()
            else:
                rospy.loginfo("Goal has not been seen yet, searching...")
                self.action_list.append('looking for goal')
                self.state = 'looking for goal'
                while self.goal1_coords is None:
                    self.look_around()
                    rospy.sleep(0.3)
                self.cmd_pub.publish(self.init_highcmd())
                self.go_to_goal1()
    

    def go_to_goal1(self):
        '''
        Get robot to goal 1 and ask if goal is correct for the current object.
        If wrong goal, look for 2nd one.
        If correct goal, look for other cup if there are others left.
        '''
        self.action_list.append('heading to goal')
        self.state = 'go to goal1'
        rospy.loginfo('Heading to goal')

        # self.reference_frame = 'goal1_coords'
        self.waypoint = self.goal1_coords   # Change destination of control loop
        self.controller()
        self.action_list.append('goal reached, is it the right one?')
        rospy.loginfo("Goal reached!")
        self.cmd_pub.publish(self.init_highcmd())

        # Ask user if it's the correct goal
        prompt = raw_input('Is this the right goal? [y/n]\n')

        if prompt == 'y':   # Correct goal
            self.nb_cups -= 1   # Reduces the number of cups yet to be picked up
            self.action_list.append('correct goal reached')
            self.cup_sub = rospy.Subscriber('cups/purple', Point, self.cup_sub_callback)
            rospy.loginfo('Correct goal reached! End of mission')

            if self.nb_cups == 0:   # Have all cups been picked up?
                self.log()
            else:
                self.action_list.append('looking for other cup')
                self.cup_coords = None
                while self.cup_coords is None:
                    self.look_around(1)
                    rospy.sleep(0.3)
                self.cmd_pub.publish(self.init_highcmd())
                self.action_list.append('found cup!')
                rospy.loginfo("New cup found!")
                self.error_threshold = 0.2
                self.main() # Start main loop again

        else:   # Wrong goal
            self.action_list.append('wrong goal, looking for other one')
            rospy.loginfo('Oh... Looking for other goal zone...')
            self.state = 'looking for goal'
            # Looking for other goal
            while self.goal2_coords is None:
                self.look_around()
                rospy.sleep(0.3)
            self.cmd_pub.publish(self.init_highcmd())
            self.go_to_goal2()
    

    def go_to_goal2(self):
        '''
        Get robot to goal 2 and ask if goal is correct for the current object.
        If wrong goal, look for 2nd one.
        If correct goal, look for other cup if there are others left.
        '''
        self.action_list.append('heading to goal')
        self.state = 'go to goal2'
        rospy.loginfo('Heading to other goal')

        # self.reference_frame = 'goal1_coords'
        self.waypoint = self.goal1_coords
        self.controller()
        self.action_list.append('goal reached, is it the right one?')
        rospy.loginfo("New goal reached!")
        self.cmd_pub.publish(self.init_highcmd())

        # Ask user if it's the correct goal
        prompt = raw_input('Is this the right goal? [y/n]\n')
        
        if prompt == 'y':   # Correct goal
            self.nb_cups -= 1   # Reduces the number of cups yet to be picked up
            self.action_list.append('correct goal reached')
            self.cup_sub = rospy.Subscriber('cups/purple', Point, self.cup_sub_callback)
            rospy.loginfo('Correct goal reached! Looking for other cup')
            
            if self.nb_cups == 0:   # Have all cups been picked up?
                self.log()
            else:
                self.action_list.append('looking for other cup')
                self.cup_coords = None
                while self.cup_coords is None:
                    self.look_around(1)
                    rospy.sleep(0.3)
                self.cmd_pub.publish(self.init_highcmd())
                self.action_list.append('found cup!')
                rospy.loginfo("New cup found!")
                self.error_threshold = 0.2
                self.main() # Start main loop again

        else:   # Wrong goal
            self.action_list.append('wrong goal, looking for other one')
            rospy.loginfo('Oh... Looking for other goal zone...')
            self.state = 'looking for goal'
            while self.goal1_coords is None:
                self.look_around()
                rospy.sleep(0.3)
            self.cmd_pub.publish(self.init_highcmd())
            self.go_to_goal1()


    def init_highcmd(self):
        '''
        Initializes a Go1 high command message
        '''

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


    def look_around(self, yaw=-1):
        '''
        Makes robot spin on itself to scan surroundings
        Input:
            yaw: float = angular velocity of the scan
        '''
        turn_msg = self.init_highcmd()
        turn_msg.yawSpeed = yaw
        self.cmd_pub.publish(turn_msg)


    def log(self):
        '''
        Writes all the mission logs contained in self.action_list in a text file.
        '''
        with open('logs_mission_' + str(datetime.datetime.now()), 'w') as f:
            for log in self.action_list:
                f.write(log+'\n')
            f.write('end')






if __name__ == "__main__":

    ctrl = MainNode()
    rospy.spin()
