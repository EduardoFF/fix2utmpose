#!/usr/bin/env python
import rospy
import utm  # install with pip install utm

from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped

import tf2_ros
import tf2_geometry_msgs
import tf


pub = None
tf_buffer = None
tf_listener = None
last_ori = None

def fix_callback( msg):
    rospy.loginfo("I heard %s", msg)
    (x,y,z,l) = utm.from_latlon(msg.latitude, msg.longitude)
    p = PoseStamped()
    p.header.frame_id = 'map3'
    p.header.stamp = rospy.Time.now()
    p.pose.position.x = x
    p.pose.position.y = y
    if last_ori is None:
        return
    p.pose.orientation.x = last_ori.y
    p.pose.orientation.y = last_ori.x
    p.pose.orientation.z = -last_ori.z
    p.pose.orientation.w = last_ori.w
    trans = None
    try:
        trans = tf_buffer.lookup_transform('map','map3',rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print("ERROR DING TF")
        return
    pose_transformed = tf2_geometry_msgs.do_transform_pose(p, trans)
    pub.publish(pose_transformed)

def imu_callback( msg):
    global last_ori
    rospy.loginfo("I heard %s", msg)
    try:
        trans = tf_buffer.lookup_transform('base_link',msg.header.frame_id,rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print("ERROR DING TF")
        return

    p = PoseStamped()
    p.pose.orientation = msg.orientation
    last_ori = tf2_geometry_msgs.do_transform_pose(p,trans).pose.orientation


if __name__=='__main__':
    rospy.init_node('fix2utmpose')
    sub=rospy.Subscriber('fix', NavSatFix, fix_callback)
    sub=rospy.Subscriber('imu', Imu, imu_callback)
    pub = rospy.Publisher('gnss_pose', PoseStamped, queue_size=10)
    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    transformation_matrix = [[-0.048,0.989,0.137,24.544],
                             [-0.999,-0.051,0.018,99.713],
                             [0.025,-0.136,0.990,-0.640],
                             [0.0000,0.000,0.000,1.000]]

    scale, shear, rpy_angles, translation_vector, perspective = \
        tf.transformations.decompose_matrix(transformation_matrix)
#    print scale
#    print shear
    print rpy_angles
    print translation_vector
#b    print transformation_matrix
#    print perspective

    rospy.spin()
