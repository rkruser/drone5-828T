#!/usr/bin/env python

# State Machine Implementation
# A python class for aggregating ROS inputs/outputs and performing state transitions
# Functions as the outer control loop for the drone

import rospy
import time
from std_msgs.msg import String,Bool
from geometry_msgs.msg import Pose
# Import other necessary ROS packages here

import numpy as np
from numpy import pi,multiply

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
class Status:
    READY = 0 # Waiting for user signal to move to SET
    TEST = 1# Spin rotors to check that they work, then wait for user takeoff signal
    TAKEOFF = 2 # Get the drone in the air
    SEARCH = 3 # Search for the window
    MOVE_TO_WINDOW = 4 # Fly toward the window, trying to keep centered
    MOVE_THROUGH_WINDOW = 5 # Flying through the window
    FIND_FINAL_TAG = 6 # Find the final tag. Can probably be done within other functions.
    MOVE_TO_FINAL_TAG = 7 # Move in front of the final tag
    HOVER = 8 #Hover in place and wait for command from user
    LAND = 9 #Land the drone and transition to READY
    FAIL = 10 #Take emergency measures like shutting down the drone or something

# I have no idea what good params look like
class Params:
    SearchYawVel = 0.1
    SearchUpVel = 0.1
    SearchDownVel = 0.1
    SearchRightVec = (1,0)
    SearchLeftVec = (-1,0)
    SearchForwardVec = (0,1)
    SearchBackwardVec = (0,-1)
    SearchYawTime = 10
    SearchVerticalTime = 1.0
    SearchHorizontalTime = 2.0
    # def __init__(self, **kwargs):
    #     for key, val in kwargs.items():
    #         setattr(self, key, val)

    moveThroughWindowTime = 3.0


         


# State values can be extended later
# Can add progress variables within states
class State:
    def __init__(self, state):
        self.resetState(state)

    def resetSearchState(self):
        self.SearchState = 0
        self.SearchAttemptCounts = {1:0, 2:0, 3:0, 4:0, 5:0, 6:0, 7:0}
        self.SearchAttemptHistory = [0]

    def resetMoveToWindow(self):
        pass

    def resetMoveThroughWindow(self):
        pass

    def resetFindFinalTag(self):
        pass

    def resetMoveToFinalTag(self):
        pass

    def resetState(self, state=Status.READY):
        self.value = state

        self.SearchState = 0
        self.SearchAttemptCounts = {1:0, 2:0, 3:0, 4:0, 5:0, 6:0, 7:0}
        self.SearchAttemptHistory = [0]
        self.SearchTimer = 0

        self.moveToWindowState = 0
#        self.moveToWindowTimer = 0

        self.moveThroughWindowState = 0
        self.moveThroughWindowTimer = 0
        self.movedThroughWindow = False

        # Tag statuses (or use in subclass?)
        self.seeLowerLeft = False
        self.seeLowerRight = False
        self.seeUpperRight = False
        self.seeUpperLeft = False
        self.seeWindow = False
        self.closeToWindow = False
        self.atNormal = False
        self.seeFinalTag = False
        self.nearFinalTag = False
    
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



class StateMachine:
    def __init__(self):
        self.state = State(Status.READY)

        # Objects that interface with the drone
        self.controller = dcontr.DroneControl()
        self.keyboard = kcontr.KeyboardController()
#       self.window = wcenter.??
        self.display = dvid.DroneVideoDisplay()

        self.cmd_max=1.0
        self.cmd_min=-1.0
        self.position_error_max_expected=3
        self.yaw_error_max_expected=np.pi
        # [x,y,z,yaw]
        self.error=np.zeros(4)

        # Gains for pose_error -> vx/vy,vz,omegayaw
        # Logic: If we get position in m, we should expect
        #        error from approx -3 thru 3
        #        If we get yaw in rad, we should expect
        #        error from -pi thru pi
        self.gains=[self.cmd_max/self.position_error_max_expected,
                    self.cmd_max/self.position_error_max_expected,
                    self.cmd_max/self.position_error_max_expected,
                    self.cmd_max/self.yaw_error_max_expected]

        # Window and final tag poses as well as transverse and normal
        # components
        self.windowPose=Pose()
        self.windowPoseT=Pose()
        self.windowPoseN=Pose()
        self.lostWindow=Bool()
        self.finalTagPose=Pose()
        self.finalTagPoseT=Pose()
        self.finalTagPoseN=Pose()

        # Minimum normal distance upon approach to object
        droneWidth=.5 # meters (estimate)
        self.minNormDist=4*droneWidth

        self.windowPoseSub=rospy.Subscriber('/window_pose',Pose,
                self.windowPoseCB)
        self.seeWindowSub=rospy.Subscriber('/see_window',Bool,
                self.seeWindowCB)
        self.finalTagPoseSub=rospy.Subscriber('/final_tag_pose', Pose,
                self.finalTagPoseCB)
        ## Add drone control publishers here *****
        # The following publisher/rate is from the tutorial
        #self.pub = rospy.Publisher('chatter', String, queue_size=10)
        #self.rate = rospy.Rate(10) # 10hz


    ######## CALLBACKS ########
    def windowPoseCB(self,windowPose):
        self.windowPose=windowPose
        (self.windowPoseT,self.windowPoseN)=
            formVectorComponents(self.windowPose)

    def seeWindowCB(self,seeWindow):
        self.state.seeWindow=seeWindow

    def finalTagPoseCB(self,finalTagPose):
        self.finalTagPose=finalTagPose
        (self.finalTagPoseT,self.finalTagPoseN)=
            formVectorComponents(self.finalTagPose)
    ###########################

    #TODO find window pose error components along and transverse to
    #     window orientation quaternion
    # Transverse and normal components of pose
    # assuming that position defines distance from drone to centroid
    # and orientation defines normal to object principal axes
    def formVectorComponents(pose):
        pass

    # Form minimum position error to object
    # Useful for moving to object normal plane while not getting
    # too close
    # Defined by:
    #   current distance to object - minimum normal distance to object
    def minPositionError(pose):
        pass

    # The following function should take a pose error and map it to a 
    # xvel/yvel/yawdot/zdot command to the drone
    # Pose error of the form: [ex,ey,ez,eyaw]
    # In the case of moving to window line, error is defined by
    # perpendicular point on window normal line from current position
    # or minimum distance point from window
    # In the case of moving thru window, error is distance to other
    # side of window
    # In the case of deviated from window normal, error is distance
    # to normal line
    def SetCommandVector(self):
        vel=multiply(self.error,-self.gains)
        self.controller.SetCommand(xvel=vel[0],yvel=vel[1],zvel=vel[2],
                yawvel=vel[3])

    # Check if we need to transition to land, fail, or hover
    def emergencyChecks():
        if self.keyboard.EmergencyShutDownPressed or self.controller.status == DroneStatus.Emergency:
            self.state = Status.FAIL
            # self.keyboard.EmergencyShutDownPressed = False
        elif self.keyboard.LandButtonPressed:
            self.state = Status.LAND
            # self.keyboard.LandButtonPressed = False
        elif self.keyboard.HoverInPlaceButtonPressed:
            self.state = Status.HOVER
            # self.keyboard.HoverInPlaceButtonPressed = False


    def ready(self):
        # Need a way to not keep redoing commands if button presses hold true over multiple iters
        if self.keyboard.StartButtonPressed:
            self.state = Status.TAKEOFF
            # self.keyboard.StartButtonPressed = False
        elif self.keyboard.TestButtonPressed:
            self.state = Status.TEST
            # self.keyboard.TestButtonPressed = False

    def test(self):
        self.controller.SendTest()
        self.state = Status.READY

    def takeoff(self):
        if self.controller.status == DroneStatus.Hover:
            self.state.value = Status.SEARCH
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
            self.state.value = Status.MOVE_TO_WINDOW
        else:
            if self.state.SearchState == 0:
                self.state.SearchState = 1
                self.state.SearchTimer = time.time()
                self.controller.SetCommand(yawvel = Params.SearchYawVel) # 0.1 rad/s ?
            elif self.state.SearchState == 1:
                if time.time() - self.state.SearchTimer >= Params.SearchYawTime:
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
                        self.state.value = Status.LAND

            # If moving up or down
            elif self.state.SearchState in [2,5]:
                if time.time() - self.state.SearchTimer >= Params.SearchVerticalTime:
                    self.state.SearchState = 1
                    self.state.SearchTimer = time.time()
                    self.state.SearchAttemptHistory.append(self.state.SearchState)
                    self.state.SearchAttemptCounts[self.state.SearchState] += 1
                    self.controller.SetCommand(yawvel = Params.SearchUpVel)

            # If moving left, right, forward, or backward
            elif self.state.SearchState in [3,4,6,7]:
                if time.time()-self.state.SearchTimer >= Params.SearchHorizontalTime:
                    self.state.SearchState = 1
                    self.state.SearchTimer = time.time()
                    self.state.SearchAttemptHistory.append(self.state.SearchState)
                    self.state.SearchAttemptCounts[self.state.SearchState] += 1
                    self.controller.SetCommand(yawvel = Params.SearchUpVel)
                   

    def commandToNormal(self):
        pass

    def commandAlongNormal(self):
        pass

    # The other functions can have a similar nested state machine to search
    def moveToWindow(self):
        if not self.state.seeWindow and not self.state.closeToWindow:
            self.state.value = Status.SEARCH
            self.resetMoveToWindow()
        elif self.state.closeToWindow:
            self.state.value = Status.MOVE_THROUGH_WINDOW
            self.state.moveThroughWindowTimer = time.time()
        else:
            # Move to normal line
            if self.state.moveToWindowState == 0:
                if self.state.atNormal: #update in a separate state update
                    self.state.moveToWindowState = 1
                else:
                    # Need to yaw toward window while squaring up
                    self.commandToNormal()
            # Move toward window along normal
            elif self.state.moveToWindowState == 1:
                if not self.state.atNormal:
                    self.state.moveToWindowState = 0
                else:
                    self.commandAlongNormal()



    def moveThroughWindow(self):
        if time.time()-self.state.moveThroughWindowTimer < Params.moveThroughWindowTime:
            self.commandAlongNormal()
        else:
            self.state.movedThroughWindow = True
            self.state.value = Status.FIND_FINAL_TAG


        
    # The other functions can have a similar nested state machine to search
    # def moveToWindow(self):
    #     #TODO make norm compare a method
    #     if not self.state.seeWindow and np.linalg.norm([
    #                 self.windowPose.position.x,
    #                 self.windowPose.position.y,
    #                 self.windowPose.position.z])-\
    #                         self.minNormDist<0:
    #         self.state.value=Status.MOVE_THROUGH_WINDOW
    #     else if not self.state.seeWindow:
    #         self.state.value=Status.SEARCH
    #     else:
    #         normMinPositionCmp=\
    #             np.linalg.norm([
    #                 self.windowPoseN.position.x,
    #                 self.windowPoseN.position.y,
    #                 self.windowPoseN.position.z])-\
    #             self.minNormDist
    #         if normMinPositionCmp>0:
    #             self.error=self.windowPoseT
    #         else:
    #             self.error=minPositionError(self.windowPose)

    #         SetCommandVector()
    #         self.state.value=Status.MOVE_TO_WINDOW

    # def moveThroughWindow(self):
    #     if self.state.seeWindow:
    #         self.error=minPositionError(self.windowPose)
    #         self.state.value=Status.MOVE_TO_WINDOW
    #     else:
    #         # Move past window
    #         self.error=self.windowPoseN*1.1+self.windowPoseT
    #         #TODO motion estimate or open loop?

    #     SetCommandVector()

    # Do same thing as window for final tag
    def findFinalTag(self):
        if self.state.seeFinalTag:
            self.state.value = Status.MOVE_TO_FINAL_TAG
        else:
            pass # Do a search for final tag


    def CommandToFinalTag():
        pass

    def moveToFinalTag(self):
        if self.nearFinalTag:
            self.state.value = Status.HOVER
        elif not self.state.seeFinalTag:
            self.state.value = Status.FIND_FINAL_TAG
        else:
            self.CommandToFinalTag()


    def hover(self):
        if not (self.controller.status == DroneStatus.Hover):
#            self.controller.SendReset() # Is that what reset does?
            # Or should I just set everything to 0?
            self.controller.SetCommand() # Reset roll/pitch/yaw


    def land(self):
#        if not (self.controller.status == DroneStatus.Landing):
        self.controller.sendLand()
        if self.controller.status == DroneStatus.Landed:
            self.state.resetState()

    def fail(self):
        self.state.value = Status.LAND
        self.land()
        # Not sure how to do anything else during a fail


<<<<<<< HEAD
    # Look at the state of the controller, window, etc.
    # and update status variables like
    # seeFinalTag, nearFinalTag, closeToWindow, etc.
    def statusVariableUpdates(self):
        pass


=======
>>>>>>> 1e71a29d1482f445a176e0cdce9dfb2f5fd679a0
    def run(self):
      while True:
        self.statusVariableUpdates()
        self.emergencyChecks()

        if self.state.value == Status.READY:
            self.ready()
        elif self.state.value == Status.TEST
            self.test()
        elif self.state.value == Status.TAKEOFF:
            self.takeoff()
        elif self.state.value == Status.SEARCH:
            self.search()
        elif self.state.value == Status.MOVE_TO_WINDOW:
            self.moveToWindow()
        elif self.state.value == Status.MOVE_THROUGH_WINDOW:
            self.moveThroughWindow()
        elif self.state.value == Status.FIND_FINAL_TAG:
            self.findFinalTag()
        elif self.state.value == Status.MOVE_TO_FINAL_TAG:
            self.moveToFinalTag()
        elif self.state.value == Status.HOVER:
            self.hover()
        elif self.state.value == Status.LAND:
            self.land()
        elif self.state.value == Status.FAIL:
            self.fail()
       
# Don't need spin
#        rospy.spinOnce()

        # Should probably increase time to match slowest
        # update rate
        time.sleep(0.005)

if __name__ == '__main__':
    try:
        rospy.init_node('StateMachine', anonymous=True)
        S = StateMachine()
        S.run()
    except rospy.ROSInterruptException:
        pass
