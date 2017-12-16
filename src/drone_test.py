#!/usr/bin/env python


# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('drone5')
import rospy

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_control1 import DroneControl
from WindowCenter import Ar_center

# Drone status
from drone_state import DroneStatus

RIGHT = -1
LEFT = 1
Z_ERR = 0.15
Y_ERR = 0.10
YAW_ERR = 0.18
CENTER_DURATION = 0.5
rise = 0

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
    FOUND_WINDOW = 11
    PAUSE = 12


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


def search(direction):
	if direction == -1:
		control.SetCommand(0,0,-0.05,0)
		print "Yaw Right Search"
	elif direction == 1:
		print "Yaw Left Search"
		control.SetCommand(0,0,0.05,0)

if __name__ == '__main__':
	process_state = Status.READY
	direction = RIGHT
	rospy.init_node('drone_test')
	control = DroneControl()
	W_cen = Ar_center()


	while True:
		#print W_cen.cen_x, W_cen.cen_y, W_cen.cen_z
		state = control.status
		
		while process_state == Status.READY:
			control.SendTakeoff()
			
			if control.status == DroneStatus.Hover:
				process_state = Status.SEARCH
			
			print process_state
		
		while process_state == Status.SEARCH:
			start = rospy.get_rostime()
			while rospy.get_rostime() - start <= rospy.Duration(4) and W_cen.cen_x == None and rise == 0:
				control.SetCommand(0,0,0,0.15)
				print "Rising"

			rise = 1

			start = rospy.get_rostime()
			while rospy.get_rostime() - start <= rospy.Duration(5) and W_cen.cen_x == None:
				search(direction)

			direction = -direction

			if W_cen.cen_x != None:
				process_state = Status.FOUND_WINDOW
				
			
			print process_state

		while process_state == Status.FOUND_WINDOW:
			start = rospy.get_rostime()
			while rospy.get_rostime() - start <= rospy.Duration(1):
				control.SetCommand(0, 0, 0, 0)
				print "Found Window"
			process_state = Status.MOVE_TO_WINDOW

		while process_state == Status.MOVE_TO_WINDOW:
			print "Move to Window"
			if W_cen.cen_x == None:
				process_state = Status.SEARCH
			else:
				if abs(W_cen.cen_y) < Y_ERR and abs(W_cen.cen_z) < Z_ERR and abs(W_cen.y_err) < YAW_ERR:  #change the thresholds anytime y_err is 10*
					start = rospy.get_rostime()
					while rospy.get_rostime() - start <= rospy.Duration(3):
						control.SetCommand(0, 0, 0, 0)
						print "Build Anticipation"

					start = rospy.get_rostime()
					while rospy.get_rostime() - start <= rospy.Duration(3):
						control.SetCommand(0, 0.05, 0, 0)
						print "Move forward"
					process_state = Status.PAUSE #centered
					#break
				elif W_cen.cen_y > Y_ERR:
					start = rospy.get_rostime()
					while rospy.get_rostime() - start <= rospy.Duration(CENTER_DURATION):
						control.SetCommand(0.01,0,0,0)
						print "Moving Left"
					
					#continue
				elif W_cen.cen_y < -Y_ERR:
					start = rospy.get_rostime()
					while rospy.get_rostime() - start <= rospy.Duration(CENTER_DURATION):
						control.SetCommand(-0.01,0,0,0)
						print "Moving Right"
					
					#continue
				elif W_cen.cen_z > Z_ERR:
					start = rospy.get_rostime()
					while rospy.get_rostime() - start <= rospy.Duration(CENTER_DURATION):
						control.SetCommand(0,0,0,0.1)
						print "Moving Up"
					
					#continue
				elif W_cen.cen_z < -Z_ERR:
					start = rospy.get_rostime()
					while rospy.get_rostime() - start <= rospy.Duration(CENTER_DURATION):
						control.SetCommand(0,0,0,-0.1)
						print "Moving Down"
					
					#continue
				elif W_cen.y_err > -YAW_ERR:
					start = rospy.get_rostime()
					while rospy.get_rostime() - start <= rospy.Duration(CENTER_DURATION):
						control.SetCommand(0,0,-0.05,0)
						print "Rotating Right"
					
					#continue
				elif W_cen.y_err < YAW_ERR:
					start = rospy.get_rostime()
					while rospy.get_rostime() - start <= rospy.Duration(CENTER_DURATION):
						control.SetCommand(0,0,0.05,0)
						print "Rotating Left"
					
					#continue

			# while process_state == Status.MOVE_THROUGH_WINDOW:
			# 	start = rospy.get_rostime()
			# 	while rospy.get_rostime() - start <= rospy.Duration(5):
			# 		control.SetCommand(0, 0.1, 0, 0) 
			# 	process_state = Status.LAND
			
		while process_state == Status.PAUSE:
			start = rospy.get_rostime()
			while rospy.get_rostime() - start <= rospy.Duration(3):
				control.SetCommand(0,0,0,0)
			process_state = Status.LAND

		while process_state == Status.LAND:
			control.SendLand()
			if control.status == DroneStatus.Landed:
				process_state = 15
			print process_state
		
		if process_state == 15:
			print process_state
			break


