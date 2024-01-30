#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class SphereDetector:
    def __init__(self):
        rospy.init_node('sphere_detector', anonymous=True)
        self.bridge = CvBridge()
        self.count_pub = rospy.Publisher('/spheres/count', Int32, queue_size=10)
        self.image_sub = rospy.Subscriber('/camera2/rgb/image_raw', Image, self.callback)
        rospy.loginfo('Subscribing to cameras topic')
        rospy.sleep(1)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        mask = cv2.inRange(cv_image, (90,0,90), (255,20,255))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        count = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 200:
                perimeter = cv2.arcLength(contour, True)
                circularity = 4 * np.pi * (area / (perimeter * perimeter))

                circularity_threshold = 0.8

                if circularity > circularity_threshold:
                    count += 1

        self.count_pub.publish(count)

def main():
    
    sphere_detector = SphereDetector()
    rospy.spin()

if __name__ == '__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass        