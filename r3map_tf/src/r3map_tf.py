#!/usr/bin/env python  
import roslib
import numpy

import rospy
import tf 
from nav_msgs.msg import Odometry

def body_to_world(msg):
    br = tf.TransformBroadcaster()
    orientation = numpy.zeros(4)
    orientation[0] = msg.pose.pose.orientation.x
    orientation[1] = msg.pose.pose.orientation.y
    orientation[2] = msg.pose.pose.orientation.z
    orientation[3] = msg.pose.pose.orientation.w
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0), orientation,
                     rospy.Time.now(),
                     "base_link",
                     "enu")


if __name__ == '__main__':
    rospy.init_node('tf')
    br = tf.TransformBroadcaster()
    rospy.Subscriber('odom', Odometry, body_to_world)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.1778, 0.0635, 0.2032),
                          tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "velodyne",
                         "base_link")

        br.sendTransform((0.127, -0.127, 0.0),
                          tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "ins",
                         "base_link")

        br.sendTransform((0.127, 0.01, 0.254),
                          tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "antenna",
                         "base_link")
        rate.sleep()