#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import numpy as np

bridge = CvBridge()


def rotate_im(pub, image):
	gray = True
	img = bridge.imgmsg_to_cv2(image)
	out = np.zeros(img.shape).astype(np.uint8)
	out[:,:,0] = img[:,:,0].T
	out[:,:,1] = img[:,:,1].T
	out[:,:,2] = img[:,:,2].T
	out = out[-1:0:-1,:,:]
	if gray:
		out = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
		out_msg = bridge.cv2_to_imgmsg(out.astype(np.uint8),"mono8")
	else:
		out_msg = bridge.cv2_to_imgmsg(out.astype(np.uint8),"rgb8")
	out_msg.header = image.header 
	pub.publish(out_msg)

if __name__ == "__main__":
	rospy.init_node('rotator', anonymous=True)    
	pub = rospy.Publisher("image_rot", Image)
	sub = rospy.Subscriber("image_raw", Image, lambda x : rotate_im(pub, x), queue_size = 10)
	rospy.spin()
	



