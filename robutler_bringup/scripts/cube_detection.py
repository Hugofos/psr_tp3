#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class CubeDetector:
    def __init__(self):
        rospy.init_node('sphere_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_acquired = False
        self.count_pub = rospy.Publisher('/cubes/count', Int32, queue_size=10)
        self.image_sub = rospy.Subscriber('/camera2/rgb/image_raw', Image, self.callback)
        rospy.loginfo('Subscribing to cameras topic')
        rospy.sleep(1)

    def callback(self, data):
        if not self.image_acquired:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            except CvBridgeError as e:
                rospy.logerr(e)
                return

            mask = cv2.inRange(cv_image, (0,230,0), (20,255,20))

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            count = 0

            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 200:
                    count += 1

            self.count_pub.publish(count)
            self.image_acquired = True

def main():
    
    cube_detector = CubeDetector()

if __name__ == '__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass        