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
		
		# Subscribe to /ardrone/navdata, message type navdata,
                # call self.ReceiveNavdata when message is revceived
		self.subNavdata = rospy.Subscriber('/ardrone/navdata', 
                        Navdata, self.ReceiveNavdata)

		# Publish to takeoff, land and reset
		self.pubLand	= rospy.Publisher('/ardrone/land',
                        Empty)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',
                        Empty)
		self.pubReset	= rospy.Publisher('/ardrone/reset',
                        Empty)

		# Publish to /cmd_vel topic
		self.pubCommand = rospy.Publisher('/cmd_vel',
                        Twist)

                # Initial calibration service
                rospy.wait_for_service(
                        '/ardrone/flattrim')              
                #TODO is this correct way to set up client w/no params?
                self.flattrim=rospy.ServiceProxy('/ardrone/flattrim')

		# Publish frequency
		self.command = Twist()
		self.commandTimer = rospy.Timer(
                        rospy.Duration(COMMAND_PERIOD/1000.0),
                        self.SendCommand)

        def Calibrate(self):
            self.flattrim()

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

	def SetCommand(self, xvel = 0,
                yvel = 0, yawvel = 0, zvel = 0):
		# Called by main program to set current command
		self.command.linear.x = xvel
		self.command.linear.y = yvel 
		self.command.linear.z = zvel
		self.command.angular.z = yawvel
	
	def SendCommand(self, event):
		# Previously set command is sent periodically if drone is flying
		if self.status == DroneStatus.Flying or\
                   self.status == DroneStatus.GoToHover or\
                   self.status == DroneStatus.Hover:
			self.pubCommand.publish(self.command)
			
