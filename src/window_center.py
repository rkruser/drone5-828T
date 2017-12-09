#!/usr/bin/env python

#This script takes the position of all the tags detected in the camera frame by ar_track_alvar package.
# If the number of tags detected is 4, it finds the center of the window from the 4 centers of tags


# rospy for the subscriber
import rospy
# ROS Image message
from ar_track_alvar_msgs.msg import AlvarMarkers
 


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
        
        print(cen_x,cen_y,cen_z)
        
def main():
    #The tags are not detected in single order so we need to specify the order by us
    global bl,tl,tr,br
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
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
