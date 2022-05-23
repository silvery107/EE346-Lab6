#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from controllers import PIDController, NonholomonicController
from moving_window_filter import MovingWindowFilter
import Queue # this is for python 2
import warnings
warnings.simplefilter('ignore', np.RankWarning)
####################################################
# CONSTANTS
DTYPE = np.float32
STOP_DISTANCE = 0.08
STOP_TIME = 5
kernel = cv2.getGaussianKernel(5, 5)
rot_90 = np.array([0,1,-1,0], dtype=DTYPE).reshape((2,2))

# Camera params and Homography
# resolution (320, 240)
IMG_H = 240
IMG_W = 320
distCoeffs = np.array([0.135529, -0.197880, 0.009788, -0.003316, 0.000000], dtype=DTYPE)
cameraMatrix = np.array([273.9783, 0.0, 151.81899, 0.0, 273.03021, 127.88242, 0.0, 0.0, 1.0],
                dtype=DTYPE).reshape((3,3))
top_x = 105 #88 # 47
top_y = 12 # 38
bottom_x = 190
bottom_y = 90 #120 
# selecting 4 points from the original image
pts_src = np.array([[160 - top_x, 180 - top_y], 
                    [160 + top_x, 180 - top_y], 
                    [160 + bottom_x, 120 + bottom_y], 
                    [160 - bottom_x, 120 + bottom_y]],
                    dtype=DTYPE)

# selecting 4 points from image that will be transformed
LANE_LEFT = 53.3333
LANE_RIGHT = 266.6667
pts_dst = np.array([[LANE_LEFT, 0], [LANE_RIGHT, 0], [LANE_RIGHT, IMG_H], [LANE_LEFT, IMG_H]], dtype=DTYPE)
# finding homography matrix
homography, status = cv2.findHomography(pts_src, pts_dst)
homography /= homography[2,2]
print(homography)

template = cv2.imread("corner_template.png", 0)

# ArUco stuff
ARUCO_TAG = cv2.aruco.DICT_6X6_50
aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_TAG)
aruco_parameters = cv2.aruco.DetectorParameters_create()

CROSS_DISTANCE = 0.28 # 28 cm
####################################################

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
        new_coord = check_coord(new_coord) + 1e-5
        theta = np.arctan2(new_coord[1], new_coord[0])
        # print(np.abs(theta)-np.pi/2)
        if np.isclose(np.abs(theta), np.pi/2, 1e-4):
            theta = 0.0

    return theta

def match_corner(img):
    # TODO match max
    # tempMatch = 1e10
    match_res = cv2.matchTemplate(img, template, cv2.TM_SQDIFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(match_res)
    # print(min_val)
    if min_val < 0.65:
        return True
    else:
        return False

class Follower:

    def __init__(self):

        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/image', Image, self.image_callback)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.twist = Twist()

        # Stop State
        self.stop_flag = False
        self.stop_once = False
        self.timer = 0.0
        self.positions = np.zeros(3, dtype=DTYPE)
        self.stop_pos = None

        # Controllers and filters
        self.controller = NonholomonicController(0.01, 1.0, 0.5, max_v=0.22, max_w=2.84)
        self.filter_x = MovingWindowFilter(1, dim=1)
        self.filter_y = MovingWindowFilter(1, dim=1)
        self.filter_t = MovingWindowFilter(2, dim=1)
        self.cmd_queue = Queue.Queue(maxsize=10)

        # Turning State
        self.turn_left = False
        self.turn_right = False
        self.mis_left = False
        self.mis_right = False

        # Crossing State
        self.cross_flag = False
        self.cross_once = False
        self.cross_counter = 1
        self.cross_pos = None

        # Start & Exit State
        self.start = False
        self.start_once = False
        self.exit = False
        self.exit_once = False

        self.disable_motor = False
    
    def print_state(self):
        print("Turn Left:", self.turn_left, "Turn Right:", self.turn_right)
        print("Miss Left:", self.mis_left, "Miss Right:", self.mis_right)
        print("Cross Flag:", self.cross_flag, "Cross Counter:", self.cross_counter)
        # print("Stop Flag:", self.stop_flag)
    
    def odom_callback(self, msg):
        self.positions[0] = msg.pose.pose.position.x
        self.positions[1] = msg.pose.pose.position.y
        self.positions[2] = msg.pose.pose.position.z
        
    def image_callback(self, msg):
        self.turn_left = False
        self.turn_right = False
        self.mis_left = False
        self.mis_right = False
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        #### *ArUco Detection #####
        if self.cross_counter >= 4:
            corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dictionary, parameters=aruco_parameters)

            if len(corners) > 0:
                cv2.aruco.drawDetectedMarkers(image, corners, ids)
                _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, cameraMatrix, distCoeffs)

                if np.linalg.norm(tvecs.squeeze()) < STOP_DISTANCE*10 and not self.stop_flag and not self.stop_once:
                    self.stop_flag = True
                    print("[Stop] Flag Triggered")
                    self.timer = rospy.get_time()

        #### *Perspective Transform #####
        img_bird_view = cv2.warpPerspective(image, homography, (IMG_W, IMG_H))

        span = np.linspace(0, 72, IMG_H)
        # print(span)
        for row in range(5, IMG_H):
            img_bird_view[row, 0:int(span[row])] = 255
            img_bird_view[row, IMG_W - int(span[row]):] = 255

        #### *Image Processing #####
        img_hsv = cv2.cvtColor(img_bird_view, cv2.COLOR_BGR2HSV)
        h, w, d = image.shape # (240, 320, 3)

        lower_black = np.array([0, 0, 0], dtype=DTYPE)
        upper_black = np.array([180, 255, 70], dtype=DTYPE)

        mask1 = cv2.inRange(img_hsv[:,:w/2], lower_black, upper_black)
        mask2 = cv2.inRange(img_hsv[:,w/2:], lower_black, upper_black)

        # !Blind top 1/3
        # search_top = h*1/3
        # mask1[0:search_top, 0:w] = 0
        # mask2[0:search_top, 0:w] = 0
        
        # Blind left and right 1/6
        # mask1[:, 0:int(w/6)] = 0
        # mask2[:, 2*int(w/6):] = 0

        mask_add = np.zeros((h,w), dtype=DTYPE)
        mask_add[:, :w/2] = mask1
        mask_add[:, w/2:] = mask2
        # remove two small white lines from perspective transform
        mask_add = cv2.morphologyEx(mask_add, cv2.MORPH_OPEN, kernel, iterations=3)
        # mask_add = cv2.dilate(mask_add, kernel, iterations = 2)

        #### *Calc Lane Orientation #####
        theta1 = get_lane_theta(mask1)
        theta2 = get_lane_theta(mask2)
        theta = 0.0

        if abs(theta1) > abs(theta2) and abs(theta1) > 0.3:
            theta = theta2
            self.turn_left = True
            # print("Turning Left")
        elif abs(theta2) > abs(theta1) and abs(theta2) > 0.3:
            theta = theta1
            self.turn_right = True
            # print("Turning Right")
        else:
            theta = (theta1 + theta2) / 2.0
        # print("theta1:%.3f, theta2: %.3f"%(theta1, theta2))
        # TODO decide which to use
        theta = self.filter_t.calculate_average((theta1 + theta2) / 2)
        # print("theta: %.3f"%theta)

        #### *Calc Lane Moment #####
        M1 = cv2.moments(mask1)
        M2 = cv2.moments(mask2)
        # TODO are they good init vals?
        cx1 = LANE_LEFT
        cx2 = LANE_RIGHT
        cy1 = IMG_H/2
        cy2 = IMG_H/2

        MOM_THR = 450000 # TODO tuning 12000
        if M1['m00'] > MOM_THR:
            cx1 = M1['m10']/M1['m00']
            cy1 = M1['m01']/M1['m00']
            # print("M1:", M1['m00'])
        else:
            self.mis_left = True
            # print("Miss Left")

        if M2['m00'] > MOM_THR:
            cx2 = M2['m10']/M2['m00'] + w/2
            cy2 = M2['m01']/M2['m00']
            # print("M2:", M2['m00'])
        else:
            self.mis_right = True
            # print("Miss Right")

        #### *Corner Detect #####



        #### *Desition Logic #####
        # TODO tuning turn center
        # print(match_corner(mask2))
        if self.turn_left:
            cx1 = cx2 - 150
            cy1 = cy2
        elif self.turn_right:
            cx2 = cx1 + 150
            cy2 = cy1
        else:
            if self.mis_left and self.mis_right and not self.start and not self.start_once:
                self.start = True
                print("[Start] Flag Triggered")
            elif self.mis_right and not self.mis_left:
                cx2 = IMG_W - cx1 -10
                cy2 = cy1
                if match_corner(mask2) and not self.cross_flag and not self.cross_once:
                    self.cross_flag = True
                    print("[Cross] Flag Triggered")
                    self.cross_counter += 1
                    print("[Cross] Counts: %d"%self.cross_counter)
                    if self.cross_counter == 4:
                        self.exit = True
            elif self.mis_left and not self.mis_right:
                cx1 = IMG_W - cx2 -10
                cy1 = cy2
                # if not self.cross_flag and not self.cross_once:
                #     self.cross_flag = True
                #     print("[Cross] Flag Triggered")

        # self.print_state()
        fpt_x = (cx1 + cx2)/2
        fpt_y = (cy1 + cy2)/2

        dx = self.filter_x.calculate_average(fpt_y/10.0)
        dy = self.filter_y.calculate_average(w/2 - fpt_x)

        v, omega = self.controller.apply(dx, dy, theta)

        #### *Stop Logic #####
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
            if distance > STOP_DISTANCE:
                self.stop_once = False
                self.cross_counter = 1
                print("[Cross] Counts: %d"%self.cross_counter)
                print("[Stop] Refresh State")
        
        #### *Crossing Logic #####
        if self.cross_flag:
            self.cross_pos = self.positions.copy()
            self.cross_once = True
            self.cross_flag = False
        
        if self.cross_once:
            distance = np.linalg.norm((self.positions-self.cross_pos))
            if distance > CROSS_DISTANCE:
                self.cross_once = False
                print("[Cross] Refresh  State")


        #### *Delay Logic #####
        self.twist.linear.x = 0.25 #v
        self.twist.angular.z = omega

        # self.cmd_queue.put(omega)
        # if self.cmd_queue.full():
        #     omega_old = self.cmd_queue.get()
        #     if self.turn_left or self.turn_right:
        #         self.twist.angular.z = omega_old

        #     self.cmd_queue.task_done()

        #### *Start & Exit Logic #####
        # TODO Param Tuning
        if self.start:
            self.start = False
            self.start_once = True
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
            if not self.disable_motor:
                self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(2)
            self.twist.linear.x = 0
            self.twist.angular.z = -1.5
            if not self.disable_motor:
                self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(1)
        
        if self.exit:
            self.exit = False
            self.exit_once = True
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
            if not self.disable_motor:
                self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(2)
            self.twist.linear.x = 0
            self.twist.angular.z = -1.5
            if not self.disable_motor:
                self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(1)

        #### *Command Publish #####
        if not self.disable_motor:
            self.cmd_vel_pub.publish(self.twist)

        #### *Image Display #####
        # cv2.imshow("BEV", img_bird_view)
        # cv2.imshow("HSV", img_hsv)
        cv2.imshow("masks", mask_add)

        cv2.circle(img_bird_view, (int(cx1), int(cy1)), 10, (0, 255, 255), -1)
        cv2.circle(img_bird_view, (int(cx2), int(cy2)), 10, (255, 255, 255), -1)
        cv2.circle(img_bird_view, (int(fpt_x), int(fpt_y)), 10, (128, 128, 128), -1)

        cv2.line(img_bird_view, (w/2-dy*5, h), (w/2-dy*5, h-dx*5), (0, 0, 255), 2)
        cv2.line(img_bird_view, (w/2, h-2), (w/2-dy*5, h-2), (0, 0, 255), 2)
        img_pair = np.concatenate((image, img_bird_view), axis=1)
        cv2.imshow("image", img_pair)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('lane_follower')
    follower = Follower()
    rospy.spin()
