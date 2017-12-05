#!/usr/bin/env python

# State Machine Implementation
# A python class for aggregating ROS inputs/outputs and performing state transitions
# Functions as the outer control loop for the drone

import rospy
from std_msgs.msg import String

class StateMachine:
  def __init__(self):
    self.pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('StateMachine', anonymous=True)
    self.rate = rospy.Rate(10) # 10hz

  def run(self):
    while not rospy.is_shutdown():
      hello_str = "hello world %s" % rospy.get_time()
      rospy.loginfo(hello_str)
      self.pub.publish(hello_str)
      self.rate.sleep()




if __name__ == '__main__':
    try:
        S = StateMachine()
        S.run()
    except rospy.ROSInterruptException:
        pass
