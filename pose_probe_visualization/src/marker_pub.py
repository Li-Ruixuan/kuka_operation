#!/usr/bin/python 

import sys
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker,MarkerArray
import json
from tf.transformations import quaternion_from_euler, quaternion_multiply 
from math import pi


def fill_in_marker(pose,id):
   q_or=quaternion_from_euler(pose['R'],pose['P'],pose['Y'])
   q_rot = quaternion_from_euler(0, -pi/2, 0)
   q = quaternion_multiply(q_rot, q_or)
   marker = Marker()
   if id==0:
       marker.color.b = 0.8
   else:
       marker.color.r = 0.8
   marker.color.a = 0.6    
   marker.id = id;
   marker.ns = "pos_probe";
   marker.header.frame_id = "calib_lwr_arm_base_link"
   marker.header.stamp    = rospy.get_rostime()
   marker.type = marker.ARROW
   marker.action = marker.ADD
   marker.scale.x = 0.15
   marker.scale.y = 0.02
   marker.scale.z = 0.02
   
   marker.pose.orientation.x = q[0]
   marker.pose.orientation.y = q[1]
   marker.pose.orientation.z = q[2]
   marker.pose.orientation.w = q[3]
   marker.pose.position.x = pose['x']
   marker.pose.position.y = pose['y']
   marker.pose.position.z = pose['z']
   marker.lifetime = rospy.Duration(1);
   return marker
    
def update_markers(file_name):
    marray=MarkerArray()
    with open(file_name) as json_file:
        data = json.load(json_file)
        id=0
        m=fill_in_marker(data['center_frame'],id)
        marray.markers.append(m)
        for p in data['probe_frames']:
            id+=1
            m=fill_in_marker(p,id)
            marray.markers.append(m)
    return marray
            
            

def marker_pub():
    pub = rospy.Publisher('pose_marker', MarkerArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) 
    file_name=rospy.get_param('pose_file_name', "/media/ruixuan/Volume/ruixuan/Documents/etasl/ws/my_new_workspace/src/probe_a_scan/poses/probe_poses.json")
    
    while not rospy.is_shutdown():
        marray=update_markers(file_name)
        pub.publish(marray)
        rate.sleep()

if __name__ == '__main__':
    try:
        marker_pub()
    except rospy.ROSInterruptException:
        pass