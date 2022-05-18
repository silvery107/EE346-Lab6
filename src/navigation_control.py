#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import actionlib
import roslib
import rospy
import time

from actionlib_msgs.msg import GoalID, GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


status_list = []
status = -1

def callback(data):
    global status_list, status
    # rospy.loginfo(rospy.get_caller_id(), data)
    # flag = data.status_list[-2]
    status_list = data.status_list
    if status_list:
        status = status_list[0].status
        # print(status)
        # print(status_list.status)

# frame_Id_lab = 'map_lab'

rospy.Subscriber('move_base/status', GoalStatusArray, callback)

goalPoints = [ 
    # from point1 to point2, to point3, to point4 and then back to point1
    # position[XYZ] and pose[quaternion]
    [(3.3856, 2.5524, 0.0), (0.0, 0.0, 0.9334, 0.3587)],
    [(0.6613, 5.3532, 0.0), (0.0, 0.0, 0.9434, -0.3315)],
    [(-2.3455, 2.4712, 0.0), (0.0, 0.0, -0.4172, 0.9088)],
    [(0.6152, -0.4165, 0.0), (0.0, 0.0, 0.9231, 0.3846)]
]



def send_pose(pose): 
    # To produce and return the goal pose variable which contains position and orientation
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
    return goal_pose


def set_initial_pose():
    # To set the 2D pose estimate of initial moment
    init_pose = PoseWithCovarianceStamped()
    init_pose.header.frame_id = 'map'
    init_pose.pose.pose.position.x = 0.6567
    init_pose.pose.pose.position.y = -0.4185
    init_pose.pose.pose.position.z = 0.0
    init_pose.pose.pose.orientation.x = 0.0
    init_pose.pose.pose.orientation.y = 0.0
    init_pose.pose.pose.orientation.z = 0.3917
    init_pose.pose.pose.orientation.w = 0.9201
    init_pose.pose.covariance[0] = 0.25
    init_pose.pose.covariance[7] = 0.25
    init_pose.pose.covariance[35] =  0.06853892326654787
    # msg_pub = actionlib.SimpleActionClient('amcl', PoseWithCovarianceStamped)
    # msg_pub.send_goal(init_pose)
    # msg_pub = rospy.Publisher('/amcl_pose', PoseWithCovarianceStamped, queue_size=1)
    msg_pub = rospy.Publisher("initialpose",PoseWithCovarianceStamped,latch=True, queue_size=1)
    msg_pub.publish(init_pose)
    pointPose = goalPoints[3]
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    goal = send_pose(pointPose)
    client.send_goal(goal)



def move_forward(n):
    pointPose = goalPoints[n]
    print("Start to move to the position point"+str(n+2))
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    goal = send_pose(pointPose)
    client.send_goal(goal)



def callback22(msg):
    ti = PoseWithCovarianceStamped()

    cmd_vel = Twist()
    print(msg.data)
    pub2 = rospy.Publisher('xfwords', String, queue_size=1)
    if msg.data.find('professor') > -1:
        pose = goalPoints[0]
        print()
        pub2.publish('准备出发！')
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        print("goal:x=%f y=%f" % (pose[0][0], pose[0][1]))
        goal = send_pose(pose)
        client.send_goal(goal)

    elif msg.data.find('Stop') > -1:
        print()
        cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        cancel_msg = GoalID()
        cancel_pub.publish(cancel_msg)

    else:
        return


if __name__ == '__main__':


    rospy.init_node("navController", anonymous=True)
    # pub = rospy.Publisher('voiceWakeup', String, queue_size=1)
    # rospy.sleep(1)
    # pub.publish('wake up')

    # sub = rospy.Subscriber('voiceWords', String, callback)

    set_initial_pose()
    flag = input (" Start now?(Y/N)\n ")
    if flag == "Y" or flag == "y":
        # move_forward(0)

    # flag = input ("Move to next point?(Y/N)\n")
    # if flag == "Y" or flag == "y":
    #     move_forward(1)
    
    # flag = input("Move to next point?(Y/N)\n")
    # if flag == "Y" or flag == "y":
    #     move_forward(2)
    
    # flag = input("Move back to the start?(Y/N)\n")
    # if flag == "Y" or flag == "y":
    #     move_forward(3)
    
        for i in range(4):
            
            move_forward(i)
            time.sleep(3)
            while(True):
            #     complete_status()
                if status == 3:
                    print("Arrive the point"+str(i+2))
                    break


    rospy.spin()
