#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import *
import timeit
from std_msgs.msg import Int32


class StopRotate():
	def __init__(self):
		self.ID = rospy.Publisher('centeringID', Int32, queue_size=1)
		self.currenPub = rospy.Publisher('/node', String, queue_size=1)
		self.startLooking = True
		self.finish_copy = True
		nn_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", darknet_ros_msgs.msg.BoundingBoxes, self.callback)
    		follow = rospy.Subscriber("/start_follow" , String, self.restart_callback)
		restart = rospy.Subscriber("/copy" , String, self.copy_callback)

		self.count = [[0 for x in range(10)] for y in range(10)] 

		self.pub_time = 0
	def callback(self, data):
        
	    if not self.finish_copy or not self.startLooking:
		return
            
	    self.currenPub.publish('stop rotating is currentlyrunning')

	    devices = [0, 0, 0]
	    for faces in data.bounding_boxes:
	    	if faces.xmin < 640:
		    devices[0] += 1
		elif faces.xmin < 1280:
		    devices[1] += 1
		else:
		    devices[2] += 1
	    centering = devices.index(max(devices))
	    self.addToCount(centering)

	    if sum(self.count[centering]) > 7:
		self.startLooking = False
		self.finish_copy = False
	    	self.ID.publish(centering)



        def addToCount(self, centering):
		self.count[0].pop(0)
		self.count[1].pop(0)
		self.count[2].pop(0)
		if centering == 0:
			self.count[0].append(1)
			self.count[1].append(0)
			self.count[2].append(0)
		elif centering == 1:
			self.count[0].append(0)
			self.count[1].append(1)
			self.count[2].append(0)
		elif centering == 2:
			self.count[0].append(0)
			self.count[1].append(0)
			self.count[2].append(1)
		

	def restart_callback(self, data):

		self.pub_time = timeit.default_timer()
		self.startLooking = True

	def copy_callback(self, data):

		self.finish_copy = True


def listener():
    
    rospy.init_node('listener', anonymous=True)
    StopRotate()
    rospy.spin()


if __name__ == '__main__':
    listener()
