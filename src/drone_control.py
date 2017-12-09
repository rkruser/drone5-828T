#!/usr/bin/env python

# Drone control class using ardrone_autonomy

# Import libraries
import roslib
roslib.load_manifest('drone5')
import rospy

# Import messages
from geometry_msgs.msg import Twist	# Drone command messages
from std_msgs.msg import Empty		# For Landing, Takeoff and Reset
from ardrone_autonomy.msg import Navdata# For receiving navdata feedback

# Drone status
from drone_state import DroneStatus

# Command duration in ms
COMMAND_PERIOD = 100


# Controller class
class DroneControl(object):
	def __init__(self):
		# Current drone status
		self.status = -1
		
		# Subscribe to /ardrone/navdata, message type navdata, call self.ReceiveNavdata when message is revceived
		self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.ReceiveNavdata)

		# Publish to takeoff, land and reset
		self.pubLand	= rospy.Publisher('/ardrone/land', Empty)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff', Empty)
		self.pubReset	= rospy.Publisher('ardrone/reset', Empty)

		# Publish to /cmd_vel topic
		self.pubCommand = rospy.Publisher('/cmd_vel',Twist)

		# Publish frequency
		self.command = Twist()
		self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.sendCommand)

	def ReceiveNavdata(self, navdata):
		# Get State (add rest of the data later)
		self.status = navdata.state
		

	def SendTakeoff(self):
		# Send Takeoff command to ardrone_driver
		if(self.status == DroneStatus.Landed):
			self.pubTakeoff.publish(Empty())


	def SendLand(self):
		# Send Land command to ardrone_driver
		self.pubLand.publish(Empty())

	def SendReset(self):
		# Send Reset command to ardrone_driver
		self.pubReset.publish(Empty())

	def SetCommand(self, roll = 0, pitch = 0, yaw_velocity = 0, z_velocity = 0):
		# Called by main program to set current command
		self.command.linear.x = pitch
		self.command.linear.y = roll
		self.command.linear.z = z_velocity
		self.command.angular.z = yaw_velocity
	
	def SendCommand(self, event):
		# Previously set command is sent periodically if drone is flying
		if self.status == DroneStatus.Flying or self.status == DroneStatus.GoToHover or self.status == DroneStatus.Hover:
			self.pubCommand.publish(self.command)
			
