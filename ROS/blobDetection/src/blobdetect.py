#!/usr/bin/env python

import cv2
import numpy as np;



if __name__ == '__main__':
    rospy.init_node('blobDetectionNode')
	rospy.Subscriber("raspicam_node/image", image_transport::TransportHints, teddyPosCallback)
	rospy.spin()
