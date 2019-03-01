#!/usr/bin/env python
import rospy
#from sensor_msgs.msg import Image
import sensor_msgs
import cv2
from std_msgs.msg import String, Int32
from google.cloud import storage
from cv_bridge import CvBridge
from google.cloud import vision
from google.cloud.vision import types
from PIL import Image, ImageDraw
import timeit

class TakePicture():
	def __init__(self):
		self.image_sub = rospy.Subscriber('/images', sensor_msgs.msg.Image, self.image_callback)
		self.bridge = CvBridge()
		self.debug = rospy.Publisher('/debug12', String, queue_size=1)
		self.count = 0

	def image_callback(self, image):

		cv2_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
		
		#cv2_image = cv2.cvtColor(cv2_image,cv2.COLOR_BGR2RGB)
		cv2.imwrite("/home/nvidia/captured_images/" + str(timeit.default_timer()) + ".jpg", cv2_image)
 		
		#storage_client = storage.Client()

		#bucket = storage_client.get_bucket('monash-robot-photos')
    		#blob = bucket.blob(str(timeit.default_timer()) + ".jpg")
		#converted = cv2.imencode('.jpg', cv2_image)[1].tostring()
		#blob.upload_from_string(converted)







rospy.init_node('take_pic')
follower = TakePicture()

rospy.spin()
