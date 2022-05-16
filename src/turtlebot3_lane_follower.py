#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from controllers import PIDController, NonholomonicController
from moving_window_filter import MovingWindowFilter
import warnings
warnings.simplefilter('ignore', np.RankWarning)

# Constants
DTYPE = np.float32
STOP_DISTANCE = 0.08
STOP_TIME = 5
kernel = cv2.getGaussianKernel(5, 2)
rot_90 = np.array([0,1,-1,0], dtype=DTYPE).reshape((2,2))

# Camera params and Homography
# resolution (320,240)
distCoeffs = np.array([0.135529, -0.197880, 0.009788, -0.003316, 0.000000], dtype=DTYPE)
cameraMatrix = np.array([273.9783, 0.0, 151.81899, 0.0, 273.03021, 127.88242, 0.0, 0.0, 1.0],
                dtype=DTYPE).reshape((3,3))
top_x = 68 # 47
top_y = 20 # 38
bottom_x = 155
bottom_y = 116 
# selecting 4 points from the original image
pts_src = np.array([[160 - top_x, 180 - top_y], 
                    [160 + top_x, 180 - top_y], 
                    [160 + bottom_x, 120 + bottom_y], 
                    [160 - bottom_x, 120 + bottom_y]])

# selecting 4 points from image that will be transformed
pts_dst = np.array([[64, 0], [256, 0], [256, 240], [64, 240]])
# finding homography matrix
H, status = cv2.findHomography(pts_src, pts_dst)
H /= H[2,2]
print(H)

# Controllers and filters
controller = NonholomonicController(0.01, 1.5, 0.5, max_v=0.22, max_w=2.84) # 0.031, 2.5, 0.5
filter_x = MovingWindowFilter(1, dim=1)
filter_y = MovingWindowFilter(1, dim=1)
filter_t = MovingWindowFilter(2, dim=1)

# ArUco stuff
ARUCO_TAG = cv2.aruco.DICT_6X6_50
aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_TAG)
aruco_parameters = cv2.aruco.DetectorParameters_create()

def check_coord(coord):
    if coord[0]<0 and coord[1]<0:
        return -coord
    elif coord[0]<0 and coord[1]>0:
        return -coord
    else:
        return coord

def get_lane_theta(mask):
    theta = 0.0
    mask_eroded = cv2.erode(mask, kernel, iterations=2)
    contours, _ = cv2.findContours(mask_eroded, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)[-2:]
    if contours:
        line_param = cv2.fitLine(contours[0], distType=cv2.DIST_L2, param=0, reps=0.01, aeps=0.01)
        new_coord = rot_90.dot(np.array([line_param[0], line_param[1]],dtype=DTYPE).reshape((2,1)))
        new_coord = check_coord(new_coord)
        theta = np.arctan2(new_coord[1], new_coord[0])
        
    return theta

class Follower:

    def __init__(self):

        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber('camera/image', Image, self.image_callback)

        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.twist = Twist()

        self.stop_flag = False
        self.stop_once = False
        self.timer = 0.0
        self.positions = np.zeros(3, dtype=DTYPE)
        self.stop_pos = None
        self.crossing_counter = 0
    
    def odom_callback(self, msg):
        self.positions[0] = msg.pose.pose.position.x
        self.positions[1] = msg.pose.pose.position.y
        self.positions[2] = msg.pose.pose.position.z
        
    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        if self.crossing_counter == 4:
            corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dictionary, parameters=aruco_parameters)

            if len(corners) > 0:
                cv2.aruco.drawDetectedMarkers(image, corners, ids)
                _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, cameraMatrix, distCoeffs)

                if np.linalg.norm(tvecs.squeeze()) < STOP_DISTANCE*10 and not self.stop_flag and not self.stop_once:
                    self.stop_flag = True
                    print("Stop Flag Triggered")
                    self.timer = rospy.get_time()
            

        img_bird_view = cv2.warpPerspective(image, H, (image.shape[1], image.shape[0]))
        # img_bird_view = image
        # cv2.imshow("BEV", img_bird_view)

        img_hsv = cv2.cvtColor(img_bird_view, cv2.COLOR_BGR2HSV)
        # cv2.imshow("HSV", img_hsv)
        h, w, d = image.shape # (240, 320, 3)

        lower_black = np.array([0, 0, 0], dtype=DTYPE)
        upper_black = np.array([180, 255, 70], dtype=DTYPE)

        mask1 = cv2.inRange(img_hsv[:,:int(w/2)], lower_black, upper_black)
        theta1 = get_lane_theta(mask1)

        mask2 = cv2.inRange(img_hsv[:,int(w/2):], lower_black, upper_black)
        theta2 = get_lane_theta(mask2)

        theta = filter_t.calculate_average((theta1 + theta2) / 2)

        search_top = int(h/3)
        # Blind top 1/3
        mask1[0:search_top, 0:w] = 0
        mask2[0:search_top, 0:w] = 0
        # TODO 这里维护一下左右边线数组
        # Blind left and right 1/6
        mask1[:, 0:int(w/6)] = 0
        mask2[:, 2*int(w/6):] = 0

        mask_add = np.zeros((h,w), dtype=DTYPE)
        mask_add[:,:int(w/2)] = mask1
        mask_add[:,int(w/2):] = mask2
        cv2.imshow("masks", mask_add)

        M1 = cv2.moments(mask1)
        M2 = cv2.moments(mask2)

        if M1['m00'] > 0:
            if M1['m00'] == 0:
                cx1 = 0
                cy1 = 0
            else:
                cx1 = int(M1['m10']/M1['m00'])
                cy1 = int(M1['m01']/M1['m00'])

            if M2['m00'] == 0:
                cx2 = 0
                cy2 = 0
            else:
                cx2 = int(M2['m10']/M2['m00']) + int(w/2)
                cy2 = int(M2['m01']/M2['m00'])

            fpt_x = int((cx1 + cx2)/2)
            fpt_y = int((cy1 + cy2)/2)

            cv2.circle(img_bird_view, (cx1, cy1), 10, (0, 255, 255), -1)
            cv2.circle(img_bird_view, (cx2, cy2), 10, (255, 255, 255), -1)
            cv2.circle(img_bird_view, (fpt_x, fpt_y), 10, (128, 128, 128), -1)

            # dx = fpt_y/10.0
            # dy = (w/2 - fpt_x)
            dx = filter_x.calculate_average(fpt_y/10.0)
            dy = filter_y.calculate_average(w/2 - fpt_x)

            v, omega = controller.apply(dx, dy, theta)

            # print("dx:", dx)
            # print("dy:", dy)
            # print("theta:", rad2deg(theta))
            print("omega:", omega)
            print("veloc:", v)

            cv2.line(img_bird_view, (int(w/2-dy*5), h), (int(w/2-dy*5), h-int(dx*5)), (0, 0, 255), 2)
            cv2.line(img_bird_view, (int(w/2), h-2), (int(w/2-dy*5), h-2), (0, 0, 255), 2)
            
            if self.stop_flag:
                v = 0.0
                # omega = 0.0
                print("stop time: %.2f"%(rospy.get_time()-self.timer))
                if rospy.get_time()-self.timer>=STOP_TIME:
                    self.stop_flag = False
                    self.stop_once = True
                    self.stop_pos = self.positions.copy()
            
            if self.stop_once:
                distance = np.linalg.norm((self.positions-self.stop_pos))
                # print(distance)
                if distance > STOP_DISTANCE:
                    self.stop_once = False
                    self.crossing_counter = 0
                    print("Refresh Stop State")

            self.twist.linear.x = 0.22#v
            self.twist.angular.z = omega

            self.cmd_vel_pub.publish(self.twist)
        
        img_pair = np.concatenate((image, img_bird_view), axis=1)
        cv2.imshow("image", img_pair)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('lane_follower')
    follower = Follower()
    rospy.spin()
