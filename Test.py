#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from math import *
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64, Bool

def callback (data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    cir_low = (0, 98, 0)
    cir_high = (87, 220, 161)
    nimage = cv2.inRange(cv_image, cir_low, cir_high)
    #moments = cv2.moments(nimage, 1) #Settings of centralization
    #x_moment = moments['m01']
    #y_moment = moments['m10']
    #area = moments['m00']
    #x = int(y_moment / area)
    #y = int(x_moment / area)
    count_cir = np.sum(nimage == 255)
    if count_cir > 1700:
        pub_move = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        vel = Twist()
        vel.linear.x = 0.1
        pub_move.publish(vel)
        rospy.sleep(2)
        vel.angular.x = 0
        pub_move.publish(vel)
        vel.angular.z = -0.1
        pub_move.publish(vel)
        rospy.sleep(13)
        vel.angular.z = 0
        pub_move.publish(vel)
        pub_extract = rospy.Publisher("servo", Bool, queue_size=10)
        cmd_msg = Bool()
        cmd_msg.data = True
        pub_extract.publish(cmd_msg)
        rospy.sleep(4)
        cmd_msg.data = False
        pub_extract.publish(cmd_msg)
        vel.angular.z = 0.1
        pub_move.publish(vel)
        rospy.sleep(13)
        vel.angular.z = 0
        pub_move.publish(vel)
        vel.linear.x = 0.1
        pub_move.publish(vel)
        rospy.sleep(2)
        vel.angular.x = 0
        pub_move.publish(vel)


def start_mission (pushed_msg):
    pointx = [0.3, 1.2, 1.3, 0.6, 0.0]
    pointy = [-0.1, -0.2, -1.8, -1.0, 0.0]
    if pushed_msg.data == 1:
        print(1)
        pub_move = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        vel = Twist()
        x_glob, y_glob = 0, 0
        mod_angle = 0
        # while counter_point
        for turtlebot in range (5):
            detect_ex = 0
            x_tr, y_tr = pointx[turtlebot], pointy[turtlebot]
            x_point, y_point = float(x_tr)-x_glob, float(y_tr)-y_glob
            measure = sqrt(x_point**2 + y_point**2)
            if x_point >= 0 and y_point >= 0:
                bas_angle = degrees(acos(abs(x_point)/measure))
                count_angle = 0.1
            elif x_point >= 0 and y_point <= 0:
                bas_angle = degrees(acos(abs(x_point)/measure))
                count_angle = -0.1
            elif x_point <= 0 and y_point >= 0:
                bas_angle = degrees(acos(abs(x_point)/measure)) + 90
                count_angle = 0.1
            elif x_point <= 0 and y_point <= 0:
                bas_angle = degrees(acos(abs(x_point)/measure)) + 90
                count_angle = -0.1
            if x_tr == 0 and y_tr == 0:
                bas_angle, mod_angle, count_angle = 10, 0, -1

            print(measure)

            x_glob = float(x_tr)
            y_glob = float(y_tr)
            total_angle = bas_angle + mod_angle
            mod_angle = bas_angle*(count_angle*(-10))

            vel.angular.z = count_angle
            pub_move.publish(vel)
            rospy.sleep(13.2*(total_angle)/90)
            vel.angular.z = 0
            pub_move.publish(vel)

            vel.linear.x = 0.1
            pub_move.publish(vel)
            rospy.sleep(measure*10) #Find structure for 0.1x per 1s
            vel.linear.x = 0
            pub_move.publish(vel)
            
            while detect_ex != 6:
                rospy.Subscriber("/front_camera/image_raw", Image, callback)
                detect_ex += 1
                vel.angular.z = 0.1
                pub_move.publish(vel)
                rospy.sleep(13.2*60/90)
                vel.angular.z = 0
                pub_move.publish(vel)
        
rospy.init_node("cmd_mission")
rospy.Subscriber("/pushed", Int64, start_mission)
rospy.spin()
