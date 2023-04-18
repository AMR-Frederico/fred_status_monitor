#!/usr/bin/env python

import rospy
import os
from std_msgs.msg import Float32

def publish_temp():
    pub = rospy.Publisher('/status/temp', Float32, queue_size=10)
    rospy.init_node('temp_publisher', anonymous=True)
    rate = rospy.Rate(1) # 1Hz
    while not rospy.is_shutdown():
        temp = os.popen("vcgencmd measure_temp").readline()
        temp = float(temp.replace("temp=","").replace("'C\n",""))
        pub.publish(temp)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_temp()
    except rospy.ROSInterruptException:
        pass
