#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import message_filters
import cv2
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import String
bridge = CvBridge()
image_publish = rospy.Publisher('joined_images', Image, queue_size=10)
debug = rospy.Publisher('debug', String, queue_size=10)
count = 0
def callback(main_image, side1_image, side2_image):
    global count
    debug.publish("hello")

    main_image = bridge.imgmsg_to_cv2(main_image, "bgr8")
    side1_image = bridge.imgmsg_to_cv2(side1_image, "bgr8")
    side2_image = bridge.imgmsg_to_cv2(side2_image, "bgr8")
    
    side1_image = cv2.resize(side1_image, (640, 480)) 
    side2_image = cv2.resize(side2_image, (640, 480)) 

    final = np.hstack((main_image, side1_image))
    final = np.hstack((final, side2_image))
   
    count += 1
    converted = bridge.cv2_to_imgmsg(final, encoding="bgr8")

    image_publish.publish(converted)

def listener():

	rospy.init_node('join', anonymous=True)

	main_sub = message_filters.Subscriber('/main_camera/image_raw', Image)
	side1_sub = message_filters.Subscriber('/side1_camera/image_raw', Image)
	side2_sub = message_filters.Subscriber('/side2_camera/image_raw', Image)

	ts = message_filters.ApproximateTimeSynchronizer([main_sub, side1_sub, side2_sub], 10, 0.1, allow_headerless=True)
	ts.registerCallback(callback)
	
	rospy.spin()

if __name__ == '__main__':
    listener()

   
