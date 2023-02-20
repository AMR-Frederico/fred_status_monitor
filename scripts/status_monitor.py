#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool,Float32,Int32


current_position_x = 0 
current_position_y = 0
current_speed_angular = 0
current_speed_linear = 0 
cmd_vel = 0
cmd_vel_safe = 0

emergency = False

distance_detected_left = 0
distance_detected_middle = 0
distance_detected_right = 0
left_ticks = 0
left_rpm = 0
right_ticks = 0
right_rpm = 0
heading = 0


def leftRpmCallback(msg):
    global left_rpm
    left_rpm = msg.data

def rightRpmCallback(msg):
    global right_rpm
    right_rpm = msg.data

def leftTicksCallback(msg):
    global left_ticks
    left_ticks = msg.data


def rightTicksCallback(msg):
    global right_ticks
    right_ticks = msg.data


def headingCB(msg):
    global heading
    heading = round(msg.data,3)

def callbackSensorLeft(msg):
    global distance_detected_left
    distance_detected_left = msg.data
    #print(distance_detected_left)


def callbackSensorMiddle(msg):
    global distance_detected_middle
    distance_detected_middle = msg.data
    #print(distance_detected_middle)


def callbackSensorRight(msg):
    global distance_detected_right
    distance_detected_right = msg.data

def odom_callback(msg):
    global current_position_x
    global current_position_y
    global current_speed_angular
    global current_speed_linear

    current_position_x = round(msg.pose.pose.position.x,2)
    current_position_y =  round(msg.pose.pose.position.y,2)

    current_speed_linear = round(msg.twist.twist.linear.x,3)
    current_speed_angular = round(msg.twist.twist.angular.z,3)


def cmd_vel_callback(msg):
    global cmd_vel
    cmd_vel = round(msg.linear.x ,3)
    
def cmd_vel_safe_callback(msg):
    global cmd_vel_safe
    cmd_vel_safe = round(msg.linear.x ,3)
    
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
    #cmd_vel 
    rospy.Subscriber("cmd_vel",Twist,cmd_vel_callback)
    #cmd_vel/safe
    rospy.Subscriber("cmd_vel/safe",Twist,cmd_vel_safe_callback)
    #emergency topics 

    #emergency break 
    rospy.Subscriber("/safety/emergency/stop",Bool, emergency_break_callback)

    #ultrassonic distance sensor
    rospy.Subscriber('sensor/ultrasonic/right/distance', Int32, callbackSensorRight)
    rospy.Subscriber('sensor/ultrasonic/left/distance', Int32, callbackSensorLeft)
    rospy.Subscriber('sensor/ultrasonic/middle/distance', Int32, callbackSensorMiddle)
    
    #motor sensor
    left_ticks_sub = rospy.Subscriber("power/status/distance/ticks/left", Float32, leftTicksCallback)
    right_ticks_sub = rospy.Subscriber("power/status/distance/ticks/right", Float32, rightTicksCallback)
    heading_sub = rospy.Subscriber("sensor/imu/yaw", Float32, headingCB)
    left_rpm_sub = rospy.Subscriber("power/status/speed/rpm/left", Float32, leftRpmCallback)
    right_rpm_sub = rospy.Subscriber("power/status/speed/rpm/right", Float32, rightRpmCallback)

    #status topics 

    #control mode 
    while not rospy.is_shutdown():


        print(f"Pos:[x:{current_position_x},y:{current_position_y}]|Speed:[l:{current_speed_linear},a:{current_speed_angular}]|CMD_vel:[{cmd_vel}, safe:{cmd_vel_safe}]|Abort:{emergency}|Ultrassonic:[{distance_detected_left}|{distance_detected_middle}|{distance_detected_right}]|Ticks:[R:{right_ticks}|L:{left_ticks}]|RPM:[R{right_rpm}| L{left_rpm}]IMU:{heading}")
        r.sleep()