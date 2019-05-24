#!/usr/bin/env python

'''
	1. $ roslaunch roslaunch pepper_bringup pepper_full.launch nao_ip:=<robot_ip> roscore_ip:=<roscore_ip> [network_interface:=<eth0|wlan0|vpn0>]
	2. $ python smach_touch_react_1.py
	3. $ rosrun smach_viewer smach_viewer.py 
'''

import rospy
import smach_ros
import smach
from naoqi_bridge_msgs.msg import Bumper, HeadTouch, HandTouch
from std_msgs.msg import String 
from std_srvs.srv import Empty
from smach_ros import ServiceState
import time

class TouchCheck(smach.State):
    """ A module to detect if pepper's head is touched
    """
    def __init__(self):	
        smach.State.__init__(self, outcomes = ['success','waiting','failed'])
	self.state_received = False
        self.body = ""
	rospy.Subscriber("/pepper_robot/bumper", Bumper, self.bumper_callback)
	rospy.Subscriber("/pepper_robot/head_touch", HeadTouch, self.head_callback)
	rospy.Subscriber("/pepper_robot/hand_touch", HandTouch, self.hand_callback)
    def bumper_callback(self, bumper):
	#block and unblock the data of state
	if bumper.state == 1:
	  self.bumper_received = True
          self.body = "Bumper"

    def head_callback(self, head):
	#block and unblock the data of state
	if head.state == 1:
	  self.state_received = True
          self.body = "Kopf"

    def hand_callback(self, hand):
	#block and unblock the data of state
	if hand.state == 1:
	  self.state_received = True
	  self.body = "Hand"	

    def execute(self, userdata):
	#wait for maximum 1 min 
	for i in range(0,600):
	    if self.state_received:
		tts_publisher = rospy.Publisher('/speech', String, queue_size=1)
		rospy.sleep(1.0)
		tts_publisher.publish(String(" Du hast meinen " + self.body + " angefasst.   Hast du vielleicht Lust,mir ein Paar fragen zu stellen?"))
		rospy.sleep(10.0)
		return 'success'
 
	    #waiting for touch 0.1 second
	    time.sleep(0.1)
	    return 'waiting'
	return 'failed'
	

#class SaySpeech(smach.State):
    #''' give a speech
    #'''
    #def __init__(self):	
        #smach.State.__init__(self, outcomes = ['success'])
    #def execute(self, userdata):
       
	#tts_publisher = rospy.Publisher('/speech', String, queue_size=1)
	#rospy.sleep(1.0)
	#tts_publisher.publish(String(" Du hast meinen " + self.body + " angefasst.   Hast du vielleicht Lust,mir ein Paar fragen zu stellen?"))
       # rospy.sleep(1.0)
	#return 'success' 
 
class SayHi(smach.State):
    ''' give a speech
    '''
    def __init__(self):	
        smach.State.__init__(self, outcomes = ['success'])
    def execute(self, userdata):
	tts_publisher = rospy.Publisher('/speech', String, queue_size=1)
	rospy.sleep(1.0)
        tts_publisher.publish(String("Herzlich Willkommen am Lehrstuhl FAPS,mein Name ist Pepper"))
        rospy.sleep(1.0)
	return 'success'         

class SayHi_2(smach.State):
    ''' give a speech
    '''
    def __init__(self):	
        smach.State.__init__(self, outcomes = ['success'])
    def execute(self, userdata):
	tts_publisher = rospy.Publisher('/speech', String, queue_size=1)
	rospy.sleep(1.0)
        tts_publisher.publish(String("Schade.Vielleicht koennen wir ja etwas anderes zusammen machen"))
        rospy.sleep(15.0)
	return 'success'         


def main():
    rospy.init_node('smach_touch_react')
    rate = rospy.Rate(10)
    sm_root = smach.StateMachine(outcomes=['success','failed','succeeded','aborted','preempted'])

    with sm_root:
        smach.StateMachine.add('STAND_UP', ServiceState('/pepper_robot/pose/wakeup', Empty), transitions={'succeeded':'WILLKOMMEN'})
        smach.StateMachine.add('WILLKOMMEN', SayHi(), transitions={'success':'TOUCH_CHECKER'})
        smach.StateMachine.add('TOUCH_CHECKER', TouchCheck(), transitions={'waiting':'TOUCH_CHECKER','success':'SAYHI_2'})
        smach.StateMachine.add('SAYHI_2', SayHi_2(),transitions={'success':'SLEEP'})
        smach.StateMachine.add('SLEEP', ServiceState('/pepper_robot/pose/rest', Empty))
	#smach.StateMachidata of state
	#if bumper.state == 1:ne.add('SAY_SPEECH', SaySpeech())

    # Execute SMACH plan
    sis = smach_ros.IntrospectionServer('SM_Pepper', sm_root, '/SM_Pepper')
    sis.start()
    # Execute SMACH plan
    outcome = sm_root.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
if __name__ == '__main__':
    main()
