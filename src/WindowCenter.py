#!/usr/bin/env python

#This script takes the position of all the tags detected in the camera frame by ar_track_alvar package.
# If the number of tags detected is 4, it finds the center of the window from the 4 centers of tags


# rospy for the subscriber
import rospy
# ROS Image message
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
import numpy as np
 


def callback(data):

    #It calculates window center only when all 4 tags are detected.
    #If all tags are detected, they might be detected in random order.
    numtags = len(data.markers)
    if numtags < 4:
        yz=[] 
        y=[]
        z=[]
        if numtags==1:
            yz.append([data.markers[0].pose.pose.position.y,data.markers[0].pose.pose.position.z])
            if yz[0][0] > 0 and yz[0][1] > 0:
                print("move up and left")
            elif yz[0][0] > 0 and yz[0][1] < 0:
                print("move up and right")
            elif yz[0][0] < 0 and yz[0][1] > 0:
                print("move down and left")
            elif yz[0][0] < 0 and yz[0][1] < 0:
                print("move down and right")
        elif numtags == 2:
            for i in range(2):
                y.append([data.markers[i].pose.pose.position.y])
                z.append([data.markers[i].pose.pose.position.z])
            if (all(y) > 0):
                print("move left")
            elif (all(y) < 0):
                print("move right")
            elif (all(z) > 0):
                print("move down")
            elif (all(z) < 0):
                print("move up")


    if numtags==4:      
        yz=[] 
        yzn=[]
        for i in range(4):
            yz.append([data.markers[i].pose.pose.position.y,data.markers[i].pose.pose.position.z])
            yzn.append([data.markers[i].pose.pose.position.y,data.markers[i].pose.pose.position.z])

    #To find order of the tags
        tl_idx = yzn.index(max(yz))  #top left will have both +ve y and z wrt to base_link
        yz.remove(max(yz))
        br_idx = yzn.index(min(yz))  #bottom right will have both -ve y and z wrt to base_link
        yz.remove(min(yz))
        bl_idx = yzn.index(max(yz))
        yz.remove(max(yz))
        tr_idx = yzn.index(yz[0])

    #Tag id order - we need to publish along with the transformation 
        bl_tag_id = data.markers[bl_idx].id
        br_tag_id = data.markers[br_idx].id
        tl_tag_id = data.markers[tl_idx].id
        tr_tag_id = data.markers[tr_idx].id
        print(bl_tag_id,tl_tag_id,tr_tag_id,br_tag_id)

        bl_orientation = data.markers[bl_idx].pose.pose.orientation
        br_orientation = data.markers[br_idx].pose.pose.orientation
        tl_orientation = data.markers[tl_idx].pose.pose.orientation
        tr_orientation = data.markers[tr_idx].pose.pose.orientation

        bl_orientation = tf.transformations.euler_from_quaternion([bl_orientation.x, bl_orientation.y, bl_orientation.z, bl_orientation.w])
        tl_orientation = tf.transformations.euler_from_quaternion([tl_orientation.x, tl_orientation.y, tl_orientation.z, tl_orientation.w])
        tr_orientation = tf.transformations.euler_from_quaternion([tr_orientation.x, tr_orientation.y, tr_orientation.z, tr_orientation.w])
        br_orientation = tf.transformations.euler_from_quaternion([br_orientation.x, br_orientation.y, br_orientation.z, br_orientation.w])
        center_orientation = [bl_orientation[0]*(180/3.1495), bl_orientation[1]*(180/3.1495), bl_orientation[2]*(180/3.1495)]
        
        def_orientation = [1.5708, 0, -1.5708]
        error = [bl_orientation[0]-def_orientation[0], bl_orientation[1]-def_orientation[1], bl_orientation[2]-def_orientation[2]]
        c_error = [error[0]*(180/3.1495), error[1]*(180/3.1495), error[2]*(180/3.1495)]
        print "Center orientation error:"
        print c_error
        #print br_orientation
        #print tl_orientation
        #print tr_orientation

    #center coordinates
        cen_x1 = (data.markers[bl_idx].pose.pose.position.x + data.markers[tr_idx].pose.pose.position.x)/2
        cen_x2 = (data.markers[br_idx].pose.pose.position.x + data.markers[tl_idx].pose.pose.position.x)/2
        cen_x = (cen_x1 + cen_x2)/2
        cen_y1 = (data.markers[bl_idx].pose.pose.position.y + data.markers[tr_idx].pose.pose.position.y)/2
        cen_y2 = (data.markers[br_idx].pose.pose.position.y + data.markers[tl_idx].pose.pose.position.y)/2
        cen_y = (cen_y1 + cen_y2)/2
        cen_z1 = (data.markers[bl_idx].pose.pose.position.z + data.markers[tr_idx].pose.pose.position.z)/2
        cen_z2 = (data.markers[br_idx].pose.pose.position.z + data.markers[tl_idx].pose.pose.position.z)/2
        cen_z = (cen_z1 + cen_z2)/2
    #Orientation
        print "Center pos:"
        print cen_x,cen_y,cen_z 
        win = tf.TransformBroadcaster()   #Broadcasting data
        win.sendTransform((cen_x, cen_y,cen_z),
                     tf.transformations.quaternion_from_euler(bl_orientation[0], bl_orientation[1], bl_orientation[2]),
                     rospy.Time.now(),
                     "WinCen",
                     "/ardrone_base_link")
        
def main():
    #Created a node to subscribe the markers
    rospy.init_node('Markers')
    # Define your image topic
    info_topic = "ar_pose_marker"
    # Set up your subscriber and define its callback
    rospy.Subscriber(info_topic, AlvarMarkers, callback) #AlvarMarkers is the msg type. Look at the import statement
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
