#!/usr/bin/env python

#Importing Libraries
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#Defining the bridge
bridge = CvBridge()

#Callback Function
def callback(data):
    try:
        #Converting to CV2 Image
        cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
        #Diplaying the image
        cv2.imshow("Picture", cv_img)
        cv2.waitKey(3)

    except CvBridgeError as e:
        print(e)

#Main Function
def main():
    #Initialzing the node
    rospy.init_node("Image_processing", anonymous=True)
    #Subscribing the image
    rospy.Subscriber("/rrbot/camera1/image_raw", Image, callback)
    #Updating the master
    rospy.spin()

#Calling the main thread
if __name__ == "__main__":
    main()
