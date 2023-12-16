#!/usr/bin/env python

import time
import rospy
import numpy as np
import tf2_ros
import tf


## Import ROS services
from discovery_via_interaction.srv import cupLocation, walkToLocation


################ Pseudocode ################ 

# While True: 

    # Look for cup: 
    #   If new cup:
        # Go to cup, pickup cup:
            # check if cup was picked up:
                # if still there, log cup at location as unmovable.
                # else: 
                    # Move cup to goal location 
                    # wait 5 seconds. 
            # return to origin. 


################ Pseudocode ################ 


class Activate:

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('activate', anonymous=True)

        # Ros Services
        self.find_cup_client = rospy.ServiceProxy('find_cup_location', cupLocation)
        self.walk_service = rospy.ServiceProxy('walk_to_location', walkToLocation)
        
        # Variables
        self.robustness_delta = 1
        self.immovable_cups = []
        self.goal_location = self.get_goal_location()
        self.origin = self.get_origin_location()


    def check_cup(self, cup):
        x, y, z = cup
        for dx in [-self.robustness_delta, 0, self.robustness_delta]:
            for dy in [-self.robustness_delta, 0, self.robustness_delta]:
                for dz in [-self.robustness_delta, 0, self.robustness_delta]:
                    if (x + dx, y + dy, z + dz) in self.immovable_cups:
                        return True
        return False



    def find_cup(self):

        try:
            # Call the service and wait for the response
            response = self.find_cup_client()
            if response:  # Check if the response is not empty
                return [response.x, response.y, response.z]
            else:
                rospy.logwarn("No cup found")
                return None
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)
            return None


    def walk_to_location(self, location):
        print("in walk to lcoation service call")
        try:
            rospy.wait_for_service('walk_to_location')
            print("sending request")
            response = self.walk_service(location[0], location[1], location[2])  
            print("sending request")
            if response.success:
                rospy.loginfo("Robot moved successfully.")
                return True
            else:
                rospy.logwarn("Failed to move the robot.")
                return False
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)
            return False


    # def move_and_pickup_cup(self, cup):
    #     # Walk to location
    #     print("going to walk to cup")
    #     outcome = self.walk_to_location(cup)
    #     return outcome


    def check_movability(self, cup):
        # TODO: Implement ROS functionality to check if the cup is movable
        # ROS function must: [1] walk back one step, [2] determine if the cup is in view
        pass

    def move_to_goal_location(self, goal_location):
        self.walk_to_location(goal_location)
        time.sleep(5)

    def move_to_origin(self, origin):
        self.walk_to_location(origin)

    def get_goal_location(self):
        # TODO: Implement ROS functionality to get the goal location
        return [0, 0, 0]

    def get_origin_location(self):
        # TODO: Implement ROS functionality to get the origin location
        return [0, 0, 0]
    


    def run(self):
        
        while not rospy.is_shutdown():

            cup = [0, 0, 0]
            while cup == [0, 0, 0]:
                print("going to find cup")
                found_cup = self.find_cup()
                print("cup found at: ", found_cup)
                if found_cup and not self.check_cup(found_cup):
                    print("cup is new")
                    cup = found_cup

            print("end of finding cup routine")

            print("starting move and pickup routine")
            move_and_pickup = self.walk_to_location(cup)

            print("finished move and pickup routine")
            print("going to restart the while loop")
    


        #     if self.check_movability(cup):
        #         self.move_to_goal_location(self.goal_location)

        #     else:
        #         self.immovable_cups.append(cup)

        #     self.move_to_origin(self.origin)




if __name__ == '__main__':
    
    try:
        algo = Activate()
        algo.run()
    except rospy.ROSInterruptException:
        pass
