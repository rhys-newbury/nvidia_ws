#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import math
from sound_play.libsoundplay import SoundClient
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Align():
	def __init__(self):
		
		self.debug = rospy.Publisher("/debug2", String, queue_size=10)
		self.sound_client = SoundClient()

		self.vel = rospy.Publisher("/RosAria/cmd_vel", Twist, queue_size=10)
		self.current = rospy.Publisher("/node", String, queue_size=10)
		self.centre = rospy.Publisher("take_pic", Odometry, queue_size=1)
		self.rotation = rospy.Publisher("rotation", Int32, queue_size=1)
		self.starting_pose = None
		self.starting_angle = None
		self.startRotate = False
		self.factor = 0
		self.angle_to_rotate = 130
		pose_sub = rospy.Subscriber("/RosAria/pose", Odometry, self.savePose)
    		center_sub = rospy.Subscriber("centeringID", Int32, self.callback)
		self.direction = 0
	def callback(self, centeringID):

		if str(centeringID.data) == "0":
			self.angle_to_rotate = 40
			self.factor = 0
		elif str(centeringID.data) == "1":
			self.angle_to_rotate = 130

			self.factor = -1
		elif str(centeringID.data) == "2":
			self.angle_to_rotate = 130
			self.factor = 1

		self.sound_client.say('Take me to your leader.')
		self.startRotate = True
	
	def savePose(self, msg):
	
		if not self.startRotate:
			return
	
		self.current.publish("align is currently running")

		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)


		if self.starting_pose == None:
			self.starting_pose = msg
			self.starting_angle = (yaw + math.pi) * 180 / math.pi	
	
		#if self.factor == 0:
		#	self.startRotate = False
	        #	self.startingPose = None
                 #       self.startingAngle = None
		#
		#	self.centre.publish(self.starting_pose)
		#	self.rotation.publish(self.factor)
		#	return
		
		
		#change to 0 to 2 pi
		angle = yaw + math.pi
		#change to 0 to 360
		angle = angle * 180 / math.pi

		angular_diff = 180 - abs(abs(angle - self.starting_angle) - 180); 
	
		if abs(angular_diff - self.angle_to_rotate) < 10:
			self.centre.publish(self.starting_pose)
			self.rotation.publish(self.factor)

			self.startRotate = False
			self.starting_pose = None
			self.starting_angle = None
			
			
		else:
			speed = Twist()
			if self.factor == 0:
				speed.angular.z = abs(self.angle_to_rotate - angular_diff) * 0.5
			else:
				speed.angular.z = abs(self.angle_to_rotate - angular_diff) * 0.5 * self.factor
			
			self.vel.publish(speed)
	
	



def listener():

    rospy.init_node('listener', anonymous=True)
    Align()
    rospy.spin()


if __name__ == '__main__':
    listener()
