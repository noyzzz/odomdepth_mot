#!/usr/bin/env python3.8
# impelement a ros node that subscribes to /odometry/filtered and gets z orientation
import rospy
from nav_msgs.msg import Odometry
import numpy as np
import math
import time
import tf
# import compressed image type and openCV
from sensor_msgs.msg import CompressedImage, Image
from tf2_msgs.msg import TFMessage
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys

#define a struct for the three f values, F_U_ROT, F_U_TRANS, F_V, a set of values for simulation and a set of values for real life and initialize them to 0 
# and generate getter function with the argument is_sim

class F_values:
    __F_U_ROT = 656.24609375
    __F_U_TRANS = 656.0
    __F_V = 656.0
    __F_U_ROT_SIM = 462.1379699707031
    __F_U_TRANS_SIM = 510.0
    __F_V_SIM = 100.0
    def get_u_rot(is_sim):
        if is_sim:
            return F_values.__F_U_ROT_SIM
        else:
            return F_values.__F_U_ROT
    def get_u_trans(is_sim):
        if is_sim:
            return F_values.__F_U_TRANS_SIM
        else:
            return F_values.__F_U_TRANS
    def get_v(is_sim):
        if is_sim:
            return F_values.__F_V_SIM
        else:
            return F_values.__F_V
    def decrease_u_rot(is_sim):
        if is_sim:
            F_values.__F_U_ROT_SIM -= 10
        else:
            F_values.__F_U_ROT -= 10
    def increase_u_rot(is_sim):
        if is_sim:
            F_values.__F_U_ROT_SIM += 10
        else:
            F_values.__F_U_ROT += 10
    def decrease_u_trans(is_sim):
        if is_sim:
            F_values.__F_U_TRANS_SIM -= 10
        else:
            F_values.__F_U_TRANS -= 10
    def increase_u_trans(is_sim):
        if is_sim:
            F_values.__F_U_TRANS_SIM += 10
        else:
            F_values.__F_U_TRANS += 10
    def decrease_v(is_sim):
        if is_sim:
            F_values.__F_V_SIM -= 10
        else:
            F_values.__F_V -= 10

    def increase_v(is_sim):
        if is_sim:
            F_values.__F_V_SIM += 10
        else:
            F_values.__F_V += 10

class GetZOrientation:
    def __init__(self, is_sim = False):
        self.is_sim = is_sim
        # current_yaw is in radians
        self.current_yaw = 0
        self.current_position = None
        self.current_depth = None
        self.initial_yaw = 0
        self.initial_u = 0
        self.initial_v = 0
        self.initial_position = np.array([0, 0])
        self.initial_depth = 0.
        self.initial_rot_matrix = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
        self.last_u_estimate = 0
        self.last_v_estimate = 0
        self.odom_sub = rospy.Subscriber(
            "/odometry/filtered", Odometry, self.callback, queue_size=1)
        if self.is_sim:
            self.camera_sub = rospy.Subscriber(
                "/camera/color/image_raw/compressed", CompressedImage, self.image_callback, queue_size=1)
            self.depth_image_sub = rospy.Subscriber(
                "/camera/aligned_depth_to_color/image_raw",Image, self.callback_depth)
        else:
            self.camera_sub = rospy.Subscriber(
                "/color_image", CompressedImage, self.image_callback, queue_size=1)
            self.depth_image_sub = rospy.Subscriber(
                "/depth_image",CompressedImage, self.callback_depth)

        self.tf_sub = rospy.Subscriber(
            "/tf", TFMessage, self.callback_tf, queue_size=1)
        self.last_image = None
        self.circle_location = None
        self.depth_image = None
        self.bridge = CvBridge()

    def callback(self, data):
        quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                      data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.current_yaw = euler[2]
        # update current position with data.pose.pose.position and convert to numpy array
        self.current_position = np.array(
            [data.pose.pose.position.x, data.pose.pose.position.y])

        # print degrees
        # print("degrees: ", math.degrees(self.current_yaw))

    def callback_depth(self, data):
        try:
            if self.is_sim:
                cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
            else:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)
            # check if cv_image has three dimensions
        if len(cv_image.shape) == 3:
            (rows, cols, channels) = cv_image.shape
        else:
            (rows, cols) = cv_image.shape

        cv_image = np.array(cv_image, dtype=np.float32)
        # change all of the zero values to nan
        cv_image[np.where(cv_image == 0)] = np.nan
        self.depth_image = cv_image

    def callback_tf(self, data):
        # I want to have the rotation between base_link and odom
        if data.transforms[0].header.frame_id == 'odom' and data.transforms[0].child_frame_id == 'base_link':
            quaternion = (data.transforms[0].transform.rotation.x, data.transforms[0].transform.rotation.y,
                          data.transforms[0].transform.rotation.z, data.transforms[0].transform.rotation.w)
            # convert  quaternion to rotation matrix and take the only the rotation part
            self.rot_matrix = tf.transformations.quaternion_matrix(quaternion)
            self.rot_matrix = self.rot_matrix[:3, :3]
            # print('rotation matrix:', self.rot_matrix)

    def find_blue_circle(self):
        camera_image = self.last_image
        # convert image to hsv
        hsv_image = cv2.cvtColor(camera_image, cv2.COLOR_BGR2HSV)
        # define range of blue color in HSV
        lower_blue = np.array([110, 50, 50])
        upper_blue = np.array([130, 255, 255])
        #define the same range for the color vibrant orange
        lower_orange = np.array([0, 150, 50])
        upper_orange = np.array([10, 255, 255])
        # threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
        # bitwise-AND mask and original image
        res = cv2.bitwise_and(camera_image, camera_image, mask=mask)
        # convert to grayscale
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        # find the center of the blue circle
        # find contours
        contours, hierarchy = cv2.findContours(
            gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # find the largest contour
        largest_contour = None
        largest_contour_area = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > largest_contour_area:
                largest_contour_area = area
                largest_contour = contour
        # find the center of the largest contour
        if largest_contour is not None:
            M = cv2.moments(largest_contour)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            # draw a circle at the center of the largest contour
            cv2.circle(camera_image, (cx, cy), 5, (0, 0, 255), -1)
            # print depth at the center of the largest contour
            if self.depth_image is not None:
                # calculate the depth at the center of the circle by getting the meean of the depth values on the largest contour
                # get the x,y coordinates of the largest contour
                x, y, w, h = cv2.boundingRect(largest_contour)
                # get the depth values on the largest contour
                depth_values = self.depth_image[y:y+h, x:x+w]
                # get the mean of the depth values nanmean ignores nan values
                self.current_depth = np.nanmean(depth_values)/1000.0
                # also put a text on top of the image with the depth
                cv2.putText(camera_image, str(self.current_depth), (20, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            # draw a circle around the largest contour
            x, y, w, h = cv2.boundingRect(largest_contour)
            #print x,y,w,h with space as delimiter erach with 2 decimal points and it's name as a string
            # print("x: ", x, "y: ", y, "w: ", w, "h: ", h)
            cv2.rectangle(camera_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            #draw a rectangle iwth last_u_estimate and last_v_estimate if the two are not nan, these two values are with respect to the center of the image so add the center of the image to them
            if not math.isnan(self.last_u_estimate) and not math.isnan(self.last_v_estimate):
                cv2.rectangle(camera_image, (int(self.last_u_estimate+camera_image.shape[1]/2-w/2), int(self.last_v_estimate+camera_image.shape[0]/2-h/2)), (int(self.last_u_estimate+camera_image.shape[1]/2+w/2), int(self.last_v_estimate+camera_image.shape[0]/2+h/2)), (0, 0, 255), 2)
            
            # draw a line from the center of the image to the center of the largest contour
            #define the center of the image as varialles derived from the image size
            center_x = int(camera_image.shape[1]/2)
            center_y = int(camera_image.shape[0]/2)
            cv2.line(camera_image, (center_x, center_y), (cx, cy), (255, 0, 0), 2)
            # write the distance from the center of the image to the center of the largest contour on the image
            cv2.putText(camera_image, str(cx-center_x)+","+str(cy-center_y),
                        (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            # show the image
            cv2.imshow("image", camera_image)
            cv2.waitKey(1)
            # return the x,y coordinates of the center of the largest contour with respect to the center of the image
            return cx-center_x, cy-center_y
        else:
            return None

    def estimate_rot_u2(self, alpha, u1, f):
        # calculate f*(tan(alpha) +(u1/f))/(1-(u1/f)*tan(alpha))
        # alpha is in radians
        # u1 is in pixels
        # f is in pixels
        gamma = math.atan(u1/f)

        return f*(math.tan(alpha+gamma))

    def estimate_rot_linear_u2(self, alpha, u1, f):
        # calculate u1+(u1^2/f^2)+1)*f*alpha
        return u1+(u1**2/f**2+1)*f*alpha

    def estimate_trans_u2(self, d1, D, u1, f):
        # calculate (d1*sin(atan(u1/f)))/(D-cos(atan(u1/f))))
        # d1 is in meters
        # D is in meters
        # u1 is in pixels
        # f is in pixels
        # calcualte all with numpy functions

        gamma = math.atan(u1/f)
        # B = ((d1+D)*tan(gamma/2))/(d1-D)
        B = (d1+D)*math.tan(gamma/2)/(d1-D)
        # gamma2_tan = (tan(gamma/2) + B)/(1-tan(gamma/2)*B)
        gamma2 = (math.tan(gamma/2) + B)/(1-math.tan(gamma/2)*B)
        return f*gamma2

    def image_callback(self, data):
        # convert image to cv2 image
        np_arr = np.fromstring(data.data, np.uint8)
        self.last_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # cv2.imshow("image", self.last_image)
        if cv2.waitKey(5) == ord('a'):
            # if 'a' is pressed, print the current yaw
            print("A PRESSED")
            print("current yaw: ", math.degrees(get_z_orientation.current_yaw))
            self.initial_yaw = self.current_yaw
            self.initial_u = self.circle_location[0]
            self.initial_v = self.circle_location[1]
            self.initial_depth = self.current_depth
            self.initial_position = self.current_position
            self.initial_rot_matrix = self.rot_matrix

        #if down arrow is pressed, decrease the f_u_rot by 10
        pressed_key = cv2.waitKey(5)
        if pressed_key == ord('s'):
            print("S PRESSED")
            F_values.decrease_v(self.is_sim)
            print("f_v: ", F_values.get_v(self.is_sim))
        #if up arrow is pressed, increase the f_u_rot by 10
        if pressed_key == ord('w'):
            print("W PRESSED")
            F_values.increase_v(self.is_sim)
            print("f_v: ", F_values.get_v(self.is_sim))

        if pressed_key == ord('d'):
            print("D PRESSED")
            F_values.increase_u_trans(self.is_sim)
            print("f_u_trans: ", F_values.get_u_trans(self.is_sim))
        #if up arrow is pressed, increase the f_u_trans by 10
        if pressed_key == ord('e'):
            print("E PRESSED")
            F_values.increase_u_trans(self.is_sim)
            print("f_u_trans: ", F_values.get_u_trans(self.is_sim))

        #if p is pressed, print the current estimated u and v
        if pressed_key == ord('p'):
            print("P PRESSED")
            print("estimated u: ", self.last_u_estimate, "estimated v: ", self.last_v_estimate)

        # shutdown ros if 'q' is pressed on the keyboard
        # if cv2.waitKey(5) == ord('q'):
        #     print("q is pressed")
        #     rospy.signal_shutdown("q pressed")
        # find the location of the blue circle in the image with respect to the center of the image
        # if the circle is not found, return None
        # if the circle is found, return the x,y coordinates of the circle with respect to the center of the image
        self.circle_location = self.find_blue_circle()
        # print ("                               estimated u2: ",self.estimate_rot_u2((self.current_yaw-self.initial_yaw), self.initial_u, 470.0))
        # #print
        # print ("                        estimated linear u2: ",self.estimate_rot_linear_u2((self.current_yaw-self.initial_yaw), self.initial_u, 470.0))

        # using the inital position which is (x,y) and the current position which is (x',y') calculate the distance between the two points
        # distance = np.linalg.norm(self.current_position-self.initial_position)
        robot_movement = self.current_position-self.initial_position
        robot_movement_3x1 = np.array([robot_movement[0], robot_movement[1], 0])
        translation_b1 = np.matmul(self.initial_rot_matrix, robot_movement_3x1)
        # print("estimated u2 with translational move", self.estimate_trans_u2(self.initial_depth, distance, self.initial_u, 250.0))
        superpos_estimate_u = (self.estimate_rot_u2((self.current_yaw-self.initial_yaw), self.initial_u, F_values.get_u_rot(self.is_sim)) -
                             self.initial_u) + self.estimate_trans_u2(self.initial_depth, translation_b1[0], self.initial_u, F_values.get_u_trans(self.is_sim))
        
        # superpos_estimate_u = self.estimate_rot_u2((self.current_yaw-self.initial_yaw), self.initial_u, self.f_u_rot)
        superpos_estimate_v = self.estimate_trans_u2(self.initial_depth, translation_b1[1], self.initial_v, F_values.get_v(self.is_sim))

        self.last_u_estimate = superpos_estimate_u
        self.last_v_estimate = superpos_estimate_v

 
        
        #print estimated u and v
        # print("estimated u: ", superpos_estimate_u , "estimated v: ", superpos_estimate_v)
        # print("translation wrt B1: ", translation_b1)
        
        


if __name__ == '__main__':
    #get the argument is_sim from the command line and put it in the variable is_sim if there is no argument given in the command line, set is_sim to False
    is_sim = False
    if len(sys.argv) > 1:
        #parse the arguments with argparse.ArgumentParser()
        import argparse
        parser = argparse.ArgumentParser()
        parser.add_argument('--is_sim', action='store_true')
        args = parser.parse_args()
        is_sim = args.is_sim
    print("is_sim: ", is_sim)
    
    rospy.init_node('get_z_orientation', anonymous=True)
    print("KIIER")
    get_z_orientation = GetZOrientation(is_sim = is_sim)
    rospy.spin()

