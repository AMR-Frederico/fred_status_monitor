#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool,Float32,Int32


current_position_x = 0 
current_position_y = 0
current_speed_angular = 0
current_speed_linear = 0 
cmd_vel_safe = 0

KD_linear = 0
KI_linear = 0 
KP_linear = 0 

KD_angular = 0 
KI_angular = 0
KP_angular = 0

emergency = False

heading = 0
goal = 0 


def kp_linear_callback(msg):
    global KP_linear
    KP_linear = msg.data
    

def ki_linear_callback(msg):
    global KI_linear 
    KI_linear = msg.data


def kd_linear_callback(msg): 
    global KD_linear
    KD_linear = msg.data


def kp_angular_callback(msg):
    global KP_angular 
    KP_angular = msg.data


def kd_angular_callback(msg):
    global KI_angular 
    KI_angular = msg.data 


def ki_angular_callback(msg):
    global KD_angular 
    KD_angular = msg.data

def goal_callback(msg):
    global goal 
    goal = msg.data


def headingCB(msg):
    global heading
    heading = round(msg.data,3)


def odom_callback(msg):
    global current_position_x
    global current_position_y
    global current_speed_angular
    global current_speed_linear

    current_position_x = round(msg.pose.pose.position.x,2)
    current_position_y =  round(msg.pose.pose.position.y,2)
    current_theta = round(msg.pose.,2)

    current_speed_linear = round(msg.twist.twist.linear.x,3)
    current_speed_angular = round(msg.twist.twist.angular.z,3)

    
def cmd_vel_safe_callback(msg):
    global cmd_vel_safe_linear
    global cmd_vel_safe_angular
    cmd_vel_safe_linear = round(msg.linear.x ,3)
    cmd_vel_safe_linear = round(msg.angular.z,3)
    
def emergency_break_callback(msg):
    global emergency
    emergency = msg.data


if __name__ == '__main__':
    rospy.init_node("status_monitor_node")
    r = rospy.Rate(10)
    #locomotion related topics 

    #odom 
    rospy.Subscriber("/odom",Odometry,odom_callback)
    #odom.twist 

    #cmd_vel/safe
    rospy.Subscriber("cmd_vel/safe",Twist,cmd_vel_safe_callback)
    #emergency topics 
    rospy.Subscriber("control/position/x",Float64,goal_callback)

    #emergency break 
    rospy.Subscriber("/safety/emergency/stop",Bool, emergency_break_callback)
    
    rospy.Subscriber("/control/position/setup/linear/kp", Float64, kp_linear_callback)
    rospy.Subscriber("/control/position/setup/linear/ki", Float64, ki_linear_callback)
    rospy.Subscriber("/control/position/setup/linear/kd", Float64, kd_linear_callback)

    rospy.Subscriber("/control/position/setup/angular/kp", Float64, kp_angular_callback)
    rospy.Subscriber("/control/position/setup/angular/ki", Float64, kd_angular_callback)
    rospy.Subscriber("/control/position/setup/angular/kd", Float64, ki_angular_callback)

    #status topics 

    #control mode 
    while not rospy.is_shutdown():


        # print(f"Goal: {goal}|Pos:[x:{current_position_x},y:{current_position_y}]|Speed:[l:{current_speed_linear},a:{current_speed_angular}]|CMD_vel:[{cmd_vel}, safe:{cmd_vel_safe}]|Abort:{emergency}|Ultrassonic:[{distance_detected_left}|{distance_detected_middle}|{distance_detected_right}]|Ticks:[R:{right_ticks}|L:{left_ticks}]|RPM:[R{right_rpm}| L{left_rpm}]IMU:{heading}")
        print(f"Goal: {goal}|Pos:[x:{current_position_x},y:{current_position_y},th:{th} ]|Speed:[l:{current_speed_linear},a:{current_speed_angular}]|CMD_vel:[{cmd_vel}, safe:{cmd_vel_safe}]|Abort:{emergency}|Ultrassonic:[{distance_detected_left}|{distance_detected_middle}|{distance_detected_right}]|Ticks:[R:{right_ticks}|L:{left_ticks}]|RPM:[R{right_rpm}| L{left_rpm}]")

        r.sleep()