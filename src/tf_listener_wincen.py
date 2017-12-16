#!/usr/bin/env python  
import roslib
roslib.load_manifest('drone5')
import rospy
import math
import tf
import geometry_msgs.msg
from drone_control1 import DroneControl

if __name__ == '__main__':
    rospy.init_node('wincen_tf_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/ardrone_base_link', '/WinCen', rospy.Time(0))
            print (trans,rot)
            listener.clear()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "center lost"
            rospy.sleep(1)
            continue
        rate.sleep()