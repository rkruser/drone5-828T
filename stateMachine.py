#!/usr/bin/env python

# State Machine Implementation
# A python class for aggregating ROS inputs/outputs and performing state transitions
# Functions as the outer control loop for the drone

import rospy
from std_msgs.msg import String
# Import other necessary ROS packages here

# Can add more states if necessary
AllStates = [
    "READY", # Waiting for user signal to start the drone on its automatic path
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


class StateMachine:
    def __init__(self):
        self.state = State("READY")
        rospy.init_node('StateMachine', anonymous=True)

        ## Add drone control publishers here *****
        # The following publisher/rate is from the tutorial
        self.pub = rospy.Publisher('chatter', String, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz


        ## Add drone state subscribers here ******
        # e.g.
        # rospy.Subscriber("chatter", String, callback)
        # where "chatter" is what we subscribe to
        # String is the type of data obtained
        # callback is a function that handles the data and updates state if necessary
        # callback can be a function from this class instance, e.g. self.DoThing(msg)
        #   which has access to the class variables of this instance

        # We may want to use service / client instead of publisher subscriber,
        # depending on the drone interface

    def ready(self, msg):
        pass

    def takeoff(self, msg):
        pass

    def search(self, msg):
        pass

    def moveToWindow(self, msg):
        pass

    def moveThroughWindow(self, msg):
        pass

    def findFinalTag(self, msg):
        pass

    def moveToFinalTag(self, msg):
        pass

    def hover(self, msg):
        pass

    def land(self, msg):
        pass

    def fail(self, msg):
        pass    


    # The callback to feed into subscriber
    def callback(self, msg):
        if self.state.value == "READY":
            self.ready(msg)
        elif self.state.value == "TAKEOFF":
            self.takeoff(msg)
        elif self.state.value == "SEARCH":
            self.search(msg)
        elif self.state.value == "MOVE_TO_WINDOW":
            self.moveToWindow(msg)
        elif self.state.value == "MOVE_THROUGH_WINDOW":
            self.moveThroughWindow(msg)
        elif self.state.value == "FIND_FINAL_TAG":
            self.findFinalTag(msg)
        elif self.state.value == "MOVE_TO_FINAL_TAG":
            self.moveToFinalTag(msg)
        elif self.state.value == "HOVER":
            self.hover(msg)
        elif self.state.value == "LAND":
            self.land(msg)
        elif self.state.value == "FAIL":
            self.fail(msg)



    def run(self):
    ## The following is from the tutorial
        while not rospy.is_shutdown():
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            self.pub.publish(hello_str)
            self.rate.sleep()

    ## When actually running, we probably want
    # rospy.spin()
    # Because the subscriber callbacks should handle the state updates



if __name__ == '__main__':
    try:
        S = StateMachine()
        S.run()
    except rospy.ROSInterruptException:
        pass
