#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
#import utm
import sys
import time
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
#from math import sin, pi
from std_msgs.msg import String
from std_msgs.msg import Header
from imu_driver.msg import imu_msg
#from std_msgs.msg import Float64
#from nav_msgs.msg import Odometry

def euler_to_quaternion_conversion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]


if __name__ == '__main__':
    SENSOR_NAME = "imu"
    rospy.init_node('imu_data')
    args = sys.argv
    print(args)
    serial_port = args[1]

    serial_baud = 115200
    #sampling_rate = 1.0
    imu_data_pub = rospy.Publisher('/imu', imu_msg, queue_size=10)
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    port.write(b'$VNWRG,07,40*XX\r')
    # s_bytes = imu+module.write(b'$VNWRG,06, 14*XX\r')
    n = 0
    line = port.readline()
    try:
        while not rospy.is_shutdown():
            str_line = port.readline()
            if str_line == '':
                rospy.logwarn("No data")
            else :
                str_line=str_line.decode('UTF-8')
                imu_data=str_line.split("*")[0]
                imu_collected_data = imu_data.split(",")
                #print(imu_collected_data)
                #print("GNGGA" in gpgga_collected_data[0])
                
                if ("VNYMR" in imu_collected_data[0]):                  
                    # time_seconds =int(time.time())
                    # time_nano_seconds =int(time.time()) - int(time.time())*(10**9)
                    
                    yaw= np.radians(float(imu_collected_data[1]))
                    
                    pitch= np.radians(float(imu_collected_data[2]))
                    roll= np.radians(float(imu_collected_data[3]))
                    #print("yaw,pitch, ro;;", yaw, pitch, roll)
                    #quat = quaternion_from_euler (roll, pitch,yaw)
                    #print("----",quat)
                    q1,q2,q3,q4 =euler_to_quaternion_conversion(yaw,pitch,roll)
                    #print(q1,q2,q3,q4)
                    n = n + 1
                    imu_msg_values = imu_msg()
                    imu_msg_values.Header.seq= n
                    #imu_msg_values.header.stamp.secs = int(time_seconds)
                    #imu_msg_values.header.stamp.nsecs = int(time_nano_seconds)
                    imu_msg_values.Header.stamp = rospy.Time.now()
                    imu_msg_values.Header.frame_id = 'IMU_FRAME1'
                    #imu_msg_values.Header.frame_id = 'IMU_FRAME1'
                    imu_msg_values.IMU.orientation.x = q1
                    imu_msg_values.IMU.orientation.y = q2
                    imu_msg_values.IMU.orientation.z = q3
                    imu_msg_values.IMU.orientation.w = q4
                    imu_msg_values.IMU.angular_velocity.x = float(imu_collected_data[10])
                    imu_msg_values.IMU.angular_velocity.y = float(imu_collected_data[11])
                    imu_msg_values.IMU.angular_velocity.z = float(imu_collected_data[12])
                    imu_msg_values.IMU.linear_acceleration.x = float(imu_collected_data[7])
                    imu_msg_values.IMU.linear_acceleration.y = float(imu_collected_data[8])
                    imu_msg_values.IMU.linear_acceleration.z = float(imu_collected_data[9])
                    imu_msg_values.MagField.magnetic_field.x = float(imu_collected_data[4])
                    imu_msg_values.MagField.magnetic_field.y = float(imu_collected_data[5])
                    imu_msg_values.MagField.magnetic_field.z = float(imu_collected_data[6])

                    print(imu_msg_values)
                    imu_data_pub.publish(imu_msg_values)

    except rospy.ROSInterruptException:
        port.close()
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down IMU node...")


