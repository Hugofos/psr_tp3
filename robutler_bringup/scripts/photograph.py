#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
from datetime import datetime

class CameraSubscriber:
    def __init__(self):
        rospy.init_node('camera_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.image_saved = False
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)
        rospy.sleep(1)

    def callback(self, data):
        if not self.image_saved:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            except CvBridgeError as e:
                rospy.logerr(e)
                return

            script_path = os.path.dirname(os.path.realpath(__file__))
            img_filename = str(datetime.now())
            cv2.imwrite(script_path + '/saved_images/' + img_filename + '.jpg', cv_image)
            rospy.loginfo('Image saved as {}'.format(img_filename))
            self.image_saved = True

def main():
    
    camera_subscriber = CameraSubscriber()

if __name__ == '__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass        