#!/usr/bin/env python3

import os
import cv2                                          # to display the images
from cv_bridge import CvBridge, CvBridgeError       # to convert ros image messages to OpenCV images
import rospy                                        # ROS Python Interface
from sensor_msgs.msg import CompressedImage         # Allows for faster transportation of the image
import numpy as np
import imutils

class DisplayStitched(object):

    def __init__(self):
        rospy.init_node("MiRo_Stitch")
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        
        # Bridge allows for ROS image data to be converted 
        # to OpenCV image data so OpenCV can use it 
        self.bridge = CvBridge()
        self.miro_cams = [None, None]

        # Subscribers for the MiRo's two cameras
        self.left_cam_sub = rospy.Subscriber(topic_base_name + "/sensors/caml/compressed",
                                             CompressedImage,
                                             self.callback_left_cam,
                                             queue_size = 1, 
                                             tcp_nodelay = True)
        
        self.right_cam_sub = rospy.Subscriber(topic_base_name + "/sensors/camr/compressed",
                                             CompressedImage,
                                             self.callback_right_cam,
                                             queue_size = 1, 
                                             tcp_nodelay = True)
        
        # Calibration features for un-distorting the MiRo fish-eye images
        # Taken from MiRo-projects/basic_functions/miro_constants.py
        self.mtx = np.array([
	        [1.04358065e+03, 0, 3.29969935e+02],
	        [0, 1.03845278e+03, 1.68243114e+02],
	        [0, 0, 1]])
        self.dist = np.array([[-3.63299415e+00, 1.52661324e+01, -7.23780207e-03, -7.48630198e-04, -3.20700124e+01]])
        self.focal_length = 330

        # For FPS counter
        self.fps_count = 0 
        self.cur_fps = 0 
        self.stop = False

        rospy.on_shutdown(self.shutdown_hook)

    # Individual Camera Callbacks
    def callback_left_cam(self, ros_image):  
        self.callback_cam(ros_image, 0)
        rospy.sleep(0.05)

    def callback_right_cam(self, ros_image):  
        self.callback_cam(ros_image, 1)
        rospy.sleep(0.05)

    # General Camera Callback, used by both the left and right cams
    def callback_cam(self, ros_image, index):
        try:
            # Converts compressed ROS image to raw CV image
            image = self.bridge.compressed_imgmsg_to_cv2(ros_image, "bgr8")

            # Undistorts the fish eye image retrieved from the MiRo cameras
            # Returns image with, height = 360, width = 640
            unfisheye_img = cv2.undistort(image,self.mtx,self.dist, None)

            # Allows for testing with different image crop values
            crop_width = 50
            crop_height = 0

            # Left 
            if index == 0:    
                cropped_img = unfisheye_img[crop_height:360, crop_width:640]
            # Right
            else : 
                cropped_img = unfisheye_img[crop_height:360, 0:640-crop_width]

            # Ignores empty images
            if cropped_img.all != None:
                self.miro_cams[index] = cropped_img
    
        # Ignores corrupted frames
        except CvBridgeError as e:
            pass

    def stitchImages(self):   

        status = 1 
        try:
            # Creates the stitcher, stitches and adds a black border for clarity
            stitcher = cv2.Stitcher.create()
            (status, stitched) = stitcher.stitch((self.miro_cams[0], self.miro_cams[1]))
        except: 
            print("General Failure")

        # Status Code of 0 means the stitch was a SUCCESS, anything else is a failure of some kind
        if status == 0:
            try:
                # Crops the stitched image to remove the empty space
                grey = cv2.cvtColor(stitched, cv2.COLOR_BGR2GRAY)
                # All the empty space will be black (0) so it can be thresholded 
                thresh = cv2.threshold(grey, 0,255, cv2.THRESH_BINARY)[1]
 
                # Finds the contours (boundary) between the stitched image and the empty space
                # boundingRect finds the 
                cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cnts = imutils.grab_contours(cnts)
                c = max(cnts, key=cv2.contourArea)
                mask = np.zeros(thresh.shape, dtype="uint8")
                wid,hei = thresh.shape

                (x, y, w, h) = cv2.boundingRect(c)
                cv2.rectangle(mask,(1,1),(hei-2,wid-2),255,-1)

                # Actual mask
                minRect = mask.copy()

                # Counter for number of pixels to be reduced
                sub = mask.copy()

                # Gradually makes the mask smaller until no more empty space can be found
                while cv2.countNonZero(sub) > 0:
                    minRect = cv2.erode(minRect, None)
                    sub = cv2.subtract(minRect, thresh)

                # Finds the contours of the completed mask and the corresponding bounding rectangle
                # which is the resulting 
                cnts = cv2.findContours(minRect.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                newCnts = imutils.grab_contours(cnts)
                c = max(newCnts, key=cv2.contourArea)
                mask = np.zeros(thresh.shape, dtype="uint8")
                (x, y, w, h) = cv2.boundingRect(c)

                # Crops the stitched image using the values from the bounding rectangle
                # to create the final stitch
                stitchedImg = stitched[y:y + h, x:x + w]

                # Resizes the image to make it larger for users when displaying
                finalStitch = cv2.resize(stitchedImg,None, fx = 2.5, fy=2.5)
                finalH, finalW, _ = np.shape(finalStitch)

                # Thresholds the stitches based on their heights / widths, if they are below these thresholds
                # the stitch is likely still wonky and so is discarded
                if finalH > 550 and finalW > 1530:
                    
                    # Crops image to the threshold for every image,
                    # creating a consistent size when displaying the stitch
                    finalStitch = finalStitch[0:550, 0:1530]

                    # Adds the FPS counter to the stitched image
                    color = (255,255,255)
                    finalImg = cv2.putText(finalStitch, str(self.cur_fps), (50,50),cv2.FONT_HERSHEY_SIMPLEX,1,color,2, cv2.LINE_AA, False)
                    cv2.imshow("Result", finalImg)
                    cv2.waitKey(10)
                    self.fps_count = self.fps_count + 1 
            
            # Prevents random crashes
            except Exception as e:
                print(e)
    
        # Status code information provided by the cv2.stitcher documentation
        # Provides feedback for user as to what went wrong
        elif status == 1:
            print("ERROR : Stitcher requires more images")
        elif status == 2:
            print("ERROR : Homography estimation failed")
        elif status == 3:
            print("ERROR : Camera parameter adjustment failed")
        else:
            # Shouldn't occur since there are only status codes 0 - 3
            print(f"Status Code : n/a")

    def shutdown_hook(self):
        print("Program Done!")
        self.stop = True

if __name__ == '__main__':
    node = DisplayStitched()

    while not rospy.is_shutdown():
        # Doesn't attempt to stitch if the images are empty
        if node.miro_cams[0] is not None and node.miro_cams[1] is not None:

            time_now = rospy.get_time()

            # Counts up how many frames are shown in 5 seconds 
            # then averages out to find the current fps 
            node.fps_count = 0
            while rospy.get_time() < time_now + 5:
                if node.stop == False:
                    node.stitchImages()
            node.cur_fps = (node.fps_count / 5)