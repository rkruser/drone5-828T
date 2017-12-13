#!/usr/bin/env python

#This script takes the position of all the tags detected in the camera frame by ar_track_alvar package.
# If the number of tags detected is 4, it finds the center of the window from the 4 centers of tags


# rospy for the subscriber
import rospy
# ROS Image message
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from numpy import median 

def callback(data):
    #If all tags are detected, they might be detected in random order
    if len(data.markers)==4:      
        idx=[]
        for i in range(4):
            idx.append(data.markers[i].id)
            
        #Getting index from tags detected
        bl_idx = idx.index(bl)
        br_idx = idx.index(br)
        tl_idx = idx.index(tl)
        tr_idx = idx.index(tr)

        
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

        # Normal orientation
        orientation=[median([data.markers[i].pose.pose.orientation.x
                        for i in [bl_idx,br_idx,tl_idx,tr_idx]]),
                     median([data.markers[i].pose.pose.orientation.y
                        for i in [bl_idx,br_idx,tl_idx,tr_idx]]),
                     median([data.markers[i].pose.pose.orientation.z
                        for i in [bl_idx,br_idx,tl_idx,tr_idx]]),
                     median([data.markers[i].pose.pose.orientation.w
                        for i in [bl_idx,br_idx,tl_idx,tr_idx]])]

        windowPose.position.x=cen_x
        windowPose.position.y=cen_y
        windowPose.position.z=cen_z
        windowPose.orientation.x=orientation[0]
        windowPose.orientation.y=orientation[1]
        windowPose.orientation.z=orientation[2]
        windowPose.orientation.w=orientation[3]
        pubWindowPose.publish(windowPose)
        seeWindow.data=True

        print(cen_x,cen_y,cen_z)
    else:
        seeWindow.data=False
        
    pubSeeWindow.publish(seeWindow)
        
def main():
    #The tags are not detected in single order so we need to specify the order by us
    global bl,tl,tr,br,windowPose,lostWindow,pubWindowPose,pubSeeWindow
    bl=8 
    tl=9 
    tr=10 
    br=11
    
    #Created a node to subscribe the markers
    rospy.init_node('Markers')
    # Define your image topic
    info_topic = "ar_pose_marker"
    # Set up your subscriber and define its callback
    rospy.Subscriber(info_topic, AlvarMarkers, callback)
    # Publish to /window_pose topic
    pubWindowPose = rospy.Publisher('/window_pose',
            Pose)
    pubSeeWindow=rospy.Publisher('/see_window',
            Pose)

    windowPose=Pose()
    seeWindow=Bool()
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
