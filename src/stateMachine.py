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
from drone_state import DroneStatus
import drone_video_display as dvid



# Can add more states if necessary
# The following data structure is currently unused and is mostly for visual reference

# Later: Make this into a class just like Params, use the class instead of strings
AllStates = [
    "READY", # Waiting for user signal to move to SET
    "TEST", # Spin rotors to check that they work, then wait for user takeoff signal
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


class Params:
    SearchYawVel = 0.1
    SearchUpVel = 0.1
    SearchDownVel = 0.1
    SearchRightVec = (1,0)
    SearchLeftVec = (-1,0)
    SearchForwardVec = (0,1)
    SearchBackwardVec = (0,-1)
    # def __init__(self, **kwargs):
    #     for key, val in kwargs.items():
    #         setattr(self, key, val)


         


# State values can be extended later
# Can add progress variables within states
class State:
    def __init__(self, state):
        self.value = state
        self.SearchState = 0
        self.SearchAttemptCounts = {1:0, 2:0, 3:0, 4:0, 5:0, 6:0, 7:0}
        self.SearchAttemptHistory = [0]
        self.SearchTimer = 0

        # Tag statuses (or use in subclass?)
        self.seeLowerLeft = False
        self.seeLowerRight = False
        self.seeUpperRight = False
        self.seeUpperLeft = False
        self.seeWindow = False
        self.seeFinalTag = False
    
        # tag positions (or use in subclass?)
        self.tagLowerLeft = None
        self.tagLowerRight = None
        self.tagUpperRight = None
        self.tagUpperLeft = None
        self.finalTag = None

        # Height of drone
        self.height = 0
        # pos with respect to tags
        self.pose = None

    def resetSearchState(self):
        self.SearchState = 0
        self.SearchAttemptCounts = {1:0, 2:0, 3:0, 4:0, 5:0, 6:0, 7:0}
        self.SearchAttemptHistory = [0]



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

    # Check if we need to transition to land, fail, or hover
    def emergencyChecks():
        if self.keyboard.EmergencyShutDownPressed or self.controller.status == DroneStatus.Emergency:
            self.state = "FAIL"
            # self.keyboard.EmergencyShutDownPressed = False
        elif self.keyboard.LandButtonPressed:
            self.state = "LAND"
            # self.keyboard.LandButtonPressed = False
        elif self.keyboard.HoverInPlaceButtonPressed:
            self.state = "HOVER"
            # self.keyboard.HoverInPlaceButtonPressed = False


    def ready(self):
        # Need a way to not keep redoing commands if button presses hold true over multiple iters
        if self.keyboard.StartButtonPressed:
            self.state = "TAKEOFF"
            # self.keyboard.StartButtonPressed = False
        elif self.keyboard.TestButtonPressed:
            self.state = "TEST"
            # self.keyboard.TestButtonPressed = False

    def test(self):
        self.controller.SendTest()
        self.state = "READY"

    def takeoff(self):
        if self.controller.status == DroneStatus.Hover:
            self.state.value = "SEARCH"
        elif not (self.controller.status == DroneStatus.Takeoff):
            self.controller.SendTakeoff()


    # Search does the following:
    # It yaws around for some number of seconds looking for the window
    # If it sees the window, it changes to move_to_window state
    # Otherwise, it moves a bit and tries yawing around again
    # Movement path: up up right right forward forward down down left left back back
    # After one cycle of not seeing the window, land the drone

    # Search states
    # 0: No search underway
    # 1: Yawing around to look for tags
    # 2: Moving up a little bit
    # 3: Moving right a little bit
    # 4: Moving forward a little bit
    # 5: Moving down a little bit
    # 6: Moving left a little bit
    # 7: Moving back a little bit
    def search(self):
        if self.state.seeWindow:
            self.state.resetSearchState()
            self.state.value = "MOVE_TO_WINDOW"
        else:
            if self.state.SearchState == 0:
                self.state.SearchState = 1
                self.state.SearchTimer = time.time()
                self.controller.SetCommand(yaw_velocity = Params.SearchYawVel) # 0.1 rad/s ?
            elif self.state.SearchState == 1:
                if time.time() - self.state.SearchTimer >= 10.0:
                    self.state.SearchTimer = time.time()
                    self.state.SearchAttemptHistory.append(1)
                    self.state.SearchAttemptCounts[1] += 1

                    if self.state.SearchAttemptCounts[2] <= 1:
                        self.state.SearchState = 2
                        self.controller.SetCommand(z_velocity = Params.SearchUpVel) # What are the units on velocity?

                    elif self.state.SearchAttemptCounts[3] <=1:
                        self.state.SearchState = 3
                        self.controller.SetCommandVector(..., ...) # Use Params.SearchRightVel

                    elif self.state.SearchAttemptCounts[4] <= 1:
                        self.state.SearchState = 4
                        self.controller.SetCommandVector(..., ...)

                    elif self.state.SearchAttemptCounts[5] <= 1:
                        self.state.SearchState = 5
                        self.controller.SetCommand(z_velocity = Params.SearchDownVel)

                    elif self.state.SearchAttemptCounts[6] <= 1:
                        self.state.SearchState = 6
                        self.controller.SetCommandVector(..., ...)

                    elif self.state.SearchAttemptCounts[7] <= 1:
                        self.state.SearchState = 7
                        self.controller.SetCommandVector(..., ...)

                    else:
                        # Drone has completed a full search loop and found nothing
                        self.state.resetSearchState()
                        self.state.value = "LAND"

            # If moving up or down
            elif self.state.SearchState in [2,5]:
                if time.time() - self.state.SearchTimer >= 1.0:
                    self.state.SearchState = 1
                    self.state.SearchTimer = time.time()
                    self.state.SearchAttemptHistory.append(self.state.SearchState)
                    self.state.SearchAttemptCounts[self.state.SearchState] += 1
                    self.controller.SetCommand(yaw_velocity = Params.SearchUpVel)

            # If moving left, right, forward, or backward
            elif self.state.SearchState in [3,4,6,7]:
                if time.time()-self.state.SearchTimer >= 2.0:
                    self.state.SearchState = 1
                    self.state.SearchTimer = time.time()
                    self.state.SearchAttemptHistory.append(self.state.SearchState)
                    self.state.SearchAttemptCounts[self.state.SearchState] += 1
                    self.controller.SetCommand(yaw_velocity = Params.SearchUpVel)
                   




    # The other functions can have a similar nested state machine to search
    def moveToWindow(self):
        pass

    def moveThroughWindow(self):
        pass

    def findFinalTag(self):
        pass

    def moveToFinalTag(self):
        pass

    def hover(self):
        if not (self.controller.status == DroneStatus.Hover):
            self.controller.SendReset() # Is that what reset does?
            # Or should I just set everything to 0?
            # self.controller.SetCommand() # Reset roll/pitch/yaw


    def land(self):
#        if not (self.controller.status == DroneStatus.Landing):
        self.controller.sendLand()
        if self.controller.status == DroneStatus.Landed:
            self.state.value = "READY"

    def fail(self):
        self.state.value = "LAND"
        self.SendLand()
        # Not sure how to do anything else during a fail




    def run(self):
      while True:
        self.emergencyChecks()

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

        time.sleep(0.005)
       



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
