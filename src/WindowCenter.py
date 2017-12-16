#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
import numpy as np
import math
 

class Ar_center:
    def __init__(self):
        self.subAR = rospy.Subscriber('/ar_pose_marker',AlvarMarkers,self.ReceivePose)
        self.tfBr = tf.TransformBroadcaster()
        self.cen_x = None
        self.cen_y = None
        self.cen_z = None
        self.y_err = None
        self.z_err = None

    def ReceivePose(self, data):
        #It calculates window center only when all 4 tags are detected.
        #If all tags are detected, they might be detected in random order.
        numtags = len(data.markers)
        if numtags < 4:
            self.cen_x = None
            self.cen_y = None
            self.cen_z = None
            self.y_err = None
            self.z_err = None

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
            #print(bl_tag_id,tl_tag_id,tr_tag_id,br_tag_id)

            bl_orientation = data.markers[bl_idx].pose.pose.orientation
            br_orientation = data.markers[br_idx].pose.pose.orientation
            tl_orientation = data.markers[tl_idx].pose.pose.orientation
            tr_orientation = data.markers[tr_idx].pose.pose.orientation

            bl_orientation = tf.transformations.euler_from_quaternion([bl_orientation.x, bl_orientation.y, bl_orientation.z, bl_orientation.w])
            tl_orientation = tf.transformations.euler_from_quaternion([tl_orientation.x, tl_orientation.y, tl_orientation.z, tl_orientation.w])
            tr_orientation = tf.transformations.euler_from_quaternion([tr_orientation.x, tr_orientation.y, tr_orientation.z, tr_orientation.w])
            br_orientation = tf.transformations.euler_from_quaternion([br_orientation.x, br_orientation.y, br_orientation.z, br_orientation.w])
            bl_orientation = [bl_orientation[0]*(180/3.1495), bl_orientation[1]*(180/3.1495), bl_orientation[2]*(180/3.1495)]
            tl_orientation = [tl_orientation[0]*(180/3.1495), tl_orientation[1]*(180/3.1495), tl_orientation[2]*(180/3.1495)]
            tr_orientation = [tr_orientation[0]*(180/3.1495), tr_orientation[1]*(180/3.1495), tr_orientation[2]*(180/3.1495)]
            br_orientation = [br_orientation[0]*(180/3.1495), br_orientation[1]*(180/3.1495), br_orientation[2]*(180/3.1495)]

        #center coordinates
            cen_x1 = (data.markers[bl_idx].pose.pose.position.x + data.markers[tr_idx].pose.pose.position.x)/2
            cen_x2 = (data.markers[br_idx].pose.pose.position.x + data.markers[tl_idx].pose.pose.position.x)/2
            self.cen_x = (cen_x1 + cen_x2)/2
            cen_y1 = (data.markers[bl_idx].pose.pose.position.y + data.markers[tr_idx].pose.pose.position.y)/2
            cen_y2 = (data.markers[br_idx].pose.pose.position.y + data.markers[tl_idx].pose.pose.position.y)/2
            self.cen_y = (cen_y1 + cen_y2)/2
            cen_z1 = (data.markers[bl_idx].pose.pose.position.z + data.markers[tr_idx].pose.pose.position.z)/2
            cen_z2 = (data.markers[br_idx].pose.pose.position.z + data.markers[tl_idx].pose.pose.position.z)/2
            self.cen_z = (cen_z1 + cen_z2)/2

            ###############################################
        #center orientation and errors
            #print "Center pos:"
            #print cen_x,cen_y,cen_z

            cen_th1 = ((bl_orientation[0]+tl_orientation[0]+tr_orientation[0]+br_orientation[0])/4)*(3.1415/180)
            cen_th2 = ((bl_orientation[1]+tl_orientation[1]+tr_orientation[1]+br_orientation[1])/4)*(3.1415/180)
            cen_th3 = ((bl_orientation[2]+tl_orientation[2]+tr_orientation[2]+br_orientation[2])/4)*(3.1415/180)

            self.y_err = math.atan(self.cen_y/self.cen_x)
            self.z_err = math.atan(self.cen_z/self.cen_x)
            #print("Center orientation and error of drone right now")
            #print(cen_th1,cen_th2,cen_th3,y_err,z_err)
        
            ###############################################
            self.tfBr.sendTransform((self.cen_x, self.cen_y,self.cen_z),
                         tf.transformations.quaternion_from_euler(cen_th1,cen_th2,cen_th3),
                         rospy.Time.now(),
                         "WinCen",
                         "/ardrone_base_link")
            return self.cen_x,self.cen_y,self.cen_z

    def printit(self):
        print (self.cen_x,self.cen_y,self.cen_z)
        
def main():
    #Created a node to subscribe the markers
    window = Ar_center()
    print window
    rospy.init_node('Markers')
    while True:
        print (window.cen_x, window.cen_y, window.cen_z,window.y_err,window.z_err)
    

     

if __name__ == '__main__':
    main()
    
