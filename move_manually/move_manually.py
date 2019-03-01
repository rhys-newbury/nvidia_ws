#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import timeit



bridge = CvBridge()
x = rospy.Publisher("debug", String, queue_size=10)
sub = None
take_pic = False
count = 0
def callback(data):
    global take_pic
    x.publish("hey")
    if data.buttons[6] == 1:
	sub.unregister()
	take_pic = True

start = None
curr = 0

def callback_cam(data):
	global count
	global take_pic
	global curr
	global start

	if not take_pic:
		return 
	
	if curr == 30:
		start = None
		take_pic = False
		curr = 0
        	sub = rospy.Subscriber("joy", Joy, callback)
	elif curr == 0:
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
                cv2.imwrite("/home/nvidia/" + str(count) + ".png", cv_image)
    
                start = timeit.default_timer()
                curr = 1

	else:
                while timeit.default_timer() - start < 1.0/15.0:
                        continue

		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
		cv2.imwrite("/home/nvidia/" + str(count) + ".png", cv_image)
    
		x.publish(str(timeit.default_timer() - start))
		start = timeit.default_timer()
		curr += 1
		count += 1
def listener():
    global sub
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    sub = rospy.Subscriber("joy", Joy, callback)
    cam_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, callback_cam)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

