#!/usr/bin/env python3.8
#impelement a ros node that subscribes to /odometry/filtered and gets z orientation
import rospy
from nav_msgs.msg import Odometry
import numpy as np
import math
import time
import tf
#import compressed image type and openCV
from sensor_msgs.msg import CompressedImage 
import cv2


class GetZOrientation:
    def __init__(self):
        #current_yaw is in radians
        self.current_yaw = 0
        self.initial_yaw = 0
        self.initial_u = 0
        self.odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.callback, queue_size=1)
        self.camera_sub = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.image_callback, queue_size=1)
        self.last_image = None
        self.circle_location = None
        
    
    def callback(self, data):
        quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.current_yaw = euler[2]
        #print degrees
        print("degrees: ",math.degrees(self.current_yaw))

    def find_blue_circle(self):
        camera_image = self.last_image
        #convert image to hsv
        hsv_image = cv2.cvtColor(camera_image, cv2.COLOR_BGR2HSV)
        #define range of blue color in HSV
        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])
        #threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
        #bitwise-AND mask and original image
        res = cv2.bitwise_and(camera_image, camera_image, mask=mask)
        #convert to grayscale
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        #find the center of the blue circle
        #find contours
        contours, hierarchy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #find the largest contour
        largest_contour = None
        largest_contour_area = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > largest_contour_area:
                largest_contour_area = area
                largest_contour = contour
        #find the center of the largest contour
        if largest_contour is not None:
            M = cv2.moments(largest_contour)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            #draw a circle at the center of the largest contour
            cv2.circle(camera_image, (cx,cy), 5, (0,0,255), -1)
            #draw a circle around the largest contour
            x,y,w,h = cv2.boundingRect(largest_contour)
            cv2.rectangle(camera_image, (x,y), (x+w,y+h), (0,255,0), 2)
            #draw a line from the center of the image to the center of the largest contour
            cv2.line(camera_image, (320,240), (cx,cy), (255,0,0), 2)
            #write the distance from the center of the image to the center of the largest contour on the image
            cv2.putText(camera_image, str(cx-320)+","+str(cy-240), (cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
            #show the image
            cv2.imshow("image", camera_image)
            cv2.waitKey(1)
            #return the x,y coordinates of the center of the largest contour with respect to the center of the image
            return cx-320, cy-240
        else:
            return None
        

        
    def estimate_u2(self, alpha, u1, f):
        #calculate f*(tan(alpha) +(u1/f))/(1-(u1/f)*tan(alpha))
        #alpha is in radians
        #u1 is in pixels
        #f is in pixels
        gamma = math.atan(u1/f)

        return f*(math.tan(alpha+gamma))
    
    def estimate_linear_u2(self, alpha, u1, f):
        #calculate u1+(u1^2/f^2)+1)*f*alpha 
        return u1+(u1**2/f**2+1)*f*alpha


    def image_callback(self, data):
        #convert image to cv2 image
        np_arr = np.fromstring(data.data, np.uint8)
        self.last_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imshow("image", self.last_image)
        if cv2.waitKey(5) == ord('a'):
            #if 'a' is pressed, print the current yaw
            print("A PRESSED")
            print("current yaw: ", math.degrees(get_z_orientation.current_yaw))
            self.initial_yaw = self.current_yaw
            self.initial_u = self.circle_location[0]
        #shutdown ros if 'q' is pressed on the keyboard
        # if cv2.waitKey(5) == ord('q'):
        #     print("q is pressed")
        #     rospy.signal_shutdown("q pressed")
        #find the location of the blue circle in the image with respect to the center of the image
        #if the circle is not found, return None
        #if the circle is found, return the x,y coordinates of the circle with respect to the center of the image
        self.circle_location = self.find_blue_circle()
        print ("                               estimated u2: ",self.estimate_u2((self.current_yaw-self.initial_yaw), self.initial_u, 470.0))
        #print 
        print ("                        estimated linear u2: ",self.estimate_linear_u2((self.current_yaw-self.initial_yaw), self.initial_u, 470.0))
        

    

if __name__ == '__main__':
    rospy.init_node('get_z_orientation',anonymous=True)
    print("KIIER")
    get_z_orientation = GetZOrientation()
    rospy.spin()
