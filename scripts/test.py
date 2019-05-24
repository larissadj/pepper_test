#!/usr/bin/env python
import rospy
from naoqi_bridge_msgs.msg import HeadTouch

def callback(data):
    rospy.loginfo( "I heard %u %u" % (data.button, data.state))
    #if (HeadTouch.state) :
    #rospy.loginfo(HeadTouch.state)
    #rospy.loginfo(HeadTouch.button)

def tactilelistener():

    rospy.init_node('tactilelistener', anonymous=True)

    rospy.Subscriber("/pepper_robot/head_touch", HeadTouch, callback)
    rospy.loginfo(HeadTouch.state)
    rospy.spin()

if __name__ == '__main__':
    tactilelistener()
