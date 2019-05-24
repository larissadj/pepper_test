#!/usr/bin/env python

import rospy
import smach_ros
import smach
from naoqi_bridge_msgs.msg import HeadTouch
from std_msgs.msg import String

rospy.init_node('smach_touch_react')
pub = rospy.Publisher('/speech', String, queue_size=1)
hello_str = "ich bin pepper, mein Kopf wird angefasst."
rospy.loginfo(hello_str)
pub.publish(hello_str)
#rate.sleep()
