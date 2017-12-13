#!/usr/bin/env python

# State Machine Implementation
# A python class for aggregating ROS inputs/outputs and performing state transitions
# Functions as the outer control loop for the drone

import rospy
import time
from std_msgs.msg import String
# Import other necessary ROS packages here

## Our classes
import drone_control as dcontr
import keyboard_controller as kcontr
import drone_cam as dcam
import window_center as wcenter
import drone_state as dstate
import drone_video_display as dvid



# Can add more states if necessary
# The following data structure is currently unused and is mostly for visual reference
AllStates = [
    "READY", # Waiting for user signal to move to SET
    "SET", # Spin rotors to check that they work, then wait for user takeoff signal
    "TAKEOFF", # Get the drone in the air
    "SEARCH", # Search for the window
    "MOVE_TO_WINDOW", # Fly toward the window, trying to keep centered
    "MOVE_THROUGH_WINDOW", # Flying through the window
    "FIND_FINAL_TAG", # Find the final tag. Can probably be done within other functions.
    "MOVE_TO_FINAL_TAG", # Move in front of the final tag
    "HOVER", #Hover in place and wait for command from user
    "LAND", #Land the drone and transition to READY
    "FAIL" #Take emergency measures like shutting down the drone or something
]



# State values can be extended later
# Can add progress variables within states
class State:
    def __init__(self, state):
        self.value = state

        self.seeCorner1 = False
        self.seeCorner2 = False
        self.seeCorner3 = False
        self.seeCorner4 = False
        self.seeWindow = False
        self.seeFinalTag = False
    
        # tag positions
        self.tagLowerLeft = None
        self.tagLowerRight = None
        self.tagUpperRight = None
        self.tagUpperLeft = None
        self.finalTag = None

        # Height of drone
        self.height = 0
        # pos with respect to tags
        self.pose = None



class StateMachine:
    def __init__(self):
        self.state = State("READY")

        # Objects that interface with the drone
        self.controller = dcontr.DroneControl()
        self.keyboard = kcontr.KeyboardController()
#        self.window = wcenter.??
        self.display = dvid.DroneVideoDisplay()

        ## Add drone control publishers here *****
        # The following publisher/rate is from the tutorial
        #self.pub = rospy.Publisher('chatter', String, queue_size=10)
        #self.rate = rospy.Rate(10) # 10hz


    def ready(self):
        pass

    def set(self):
        pass

    def takeoff(self):
        pass

    def search(self):
        pass

    def moveToWindow(self):
        pass

    def moveThroughWindow(self):
        pass

    def findFinalTag(self):
        pass

    def moveToFinalTag(self):
        pass

    def hover(self):
        pass

    def land(self):
        pass

    def fail(self):
        pass    




    def run(self):
      while True:
        if self.state.value == "READY":
            self.ready()
        elif self.state.value == "TAKEOFF":
            self.takeoff()
        elif self.state.value == "SEARCH":
            self.search()
        elif self.state.value == "MOVE_TO_WINDOW":
            self.moveToWindow()
        elif self.state.value == "MOVE_THROUGH_WINDOW":
            self.moveThroughWindow()
        elif self.state.value == "FIND_FINAL_TAG":
            self.findFinalTag()
        elif self.state.value == "MOVE_TO_FINAL_TAG":
            self.moveToFinalTag()
        elif self.state.value == "HOVER":
            self.hover()
        elif self.state.value == "LAND":
            self.land()
        elif self.state.value == "FAIL":
            self.fail()

        time.sleep(0.001)
       



    ## The following is from the tutorial
#        while not rospy.is_shutdown():
#            hello_str = "hello world %s" % rospy.get_time()
#            rospy.loginfo(hello_str)
#            self.pub.publish(hello_str)
#            self.rate.sleep()

    ## When actually running, we probably want
    # rospy.spin()
    # Because the subscriber callbacks should handle the state updates



if __name__ == '__main__':
    try:
        rospy.init_node('StateMachine', anonymous=True)
        S = StateMachine()
        S.run()
    except rospy.ROSInterruptException:
        pass
