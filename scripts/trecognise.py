#!/usr/bin/env python
import rospy
import smach_ros
import smach
from std_msgs.msg import String 
from std_srvs.srv import Empty
from smach_ros import ServiceState
import time
import sys, threading

class GSpeech(smach.State):
     """Speech Recogniser """
    
		def __init__(self):	
		smach.State.__init__(self, outcomes = ['success','failed'])

		# configure ROS settings
		rospy.on_shutdown(self.shutdown)
		self.pub_speech = rospy.Publisher('/speech', String, queue_size=10)
		self.srv_wakeup = rospy.Service('/pepper_robot/pose/wakeup', Empty,self.start)
		self.srv_rest= rospy.Service('/pepper_robot/pose/rest', Empty,self.stop)

		# run speech recognition

		self.started = True
		self.recog_thread = threading.Thread(target=self.do_recognition, args=())
		self.recog_thread.start()

     def start(self, req):
		"""Start speech recognition"""
		if not self.started:
		   self.started = True
		   if not self.recog_thread.is_alive():
		      self.recog_thread = threading.Thread(target=self.do_recognition, args=())
		      self.recog_thread.start()
		   rospy.loginfo("gspeech recognizer started")
		else:
		   rospy.loginfo("gspeech is already running")           		 
		return EmptyResponse()

     def stop(self, req):
	    """Stop speech recognition"""
		if self.started:
		   self.started = False
		   if self.recog_thread.is_alive():
		      self.recog_thread.join()
		   rospy.loginfo("gspeech recognizer stopped")
		else:
		   rospy.loginfo("gspeech is already stopped")
		return EmptyResponse()

     def shutdown(self):
            """Stop all system process before killing node"""
            self.started = False
		if self.recog_thread.is_alive():
		self.recog_thread.join()
		self.srv_start.shutdown()
		self.srv_stop.shutdown()
		os.remove("recording.flac")

     def do_recognition(self):
           """Do speech recognition"""
           while self.started:






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
	    # start ROS node
	    rospy.init_node('gspeech')
	    rate = rospy.Rate(10)
	    sm_root = smach.StateMachine(outcomes=['success','failed','succeeded','aborted','preempted'])

	    with sm_root:
		smach.StateMachine.add('SPEECH_RECOGNITION', GSpeech(), transitions={'succeeded':'WILLKOMMEN'})
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

