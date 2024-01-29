#!/usr/bin/env python3

import cv2
import rospy
from colorama import Fore, Style, init
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO
from labels import idsY
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from ultralytics_ros.msg import YoloResult


def callback_detector(data):
    for d in data.detections.detections:
        l = d.results[0].id
        if l in idsY:

            print(f"{idsY[l]} found: ", end='')
            print(f" x: {d.bbox.center.x} ; y:{d.bbox.center.y} ; z:{d.results[0].score}")
            print("#################################################################################")
        else:
            print("-------------------Nothing Found-------------------") 
def main():
    init(autoreset=True)
    rospy.Subscriber('/yolo_result', YoloResult, callback_detector)
    rospy.init_node('yoloSubscriber', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    main()