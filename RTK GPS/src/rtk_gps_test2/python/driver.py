#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import utm
import sys
#from math import sin, pi
from std_msgs.msg import String
from std_msgs.msg import Header
from rtk_gps_test2.msg import gps_msg
#from std_msgs.msg import Float64
#from nav_msgs.msg import Odometry
def Convert_time(x):
    hh=int(x/10000)
    mm=int((x-hh*10000)/100)
    ss=int(x-hh*10000-mm*100)
    ns=(x-hh*10000-mm*100-ss)*1000000000
    second=hh*3600+mm*60+ss
    return second,ns

def Convert_gpggs_degrees(x):
    dd=int(x/100)
    mm=(x-dd*100)/60
    deg=dd+mm
    return deg

if __name__ == '__main__':
    SENSOR_NAME = "gps"
    rospy.init_node('gps_data')
    args = rospy.myargv(argv=sys.argv)
        
    #serial_port = rospy.get_param('~port','/dev/ttyUSB0')
    serial_port = args[1]
    #serial_port = '/dev/pts/5'
    serial_baud = 57600
    #serial_baud = 4800
    sampling_rate = 1.0
    gpggs_data_pub = rospy.Publisher('gps', gps_msg, queue_size=10)
    port = serial.Serial(serial_port, serial_baud, timeout=3.)

    line = port.readline()
    try:
        while not rospy.is_shutdown():
            line = port.readline()
            
            str_line = line
            str_line=str_line.decode('UTF-8')

            gpgga_collected_data = str_line.split(",")
            print(gpgga_collected_data)
            #print("GNGGA" in gpgga_collected_data[0])
            if str_line == '':
                rospy.logwarn("No data")
            elif ("GNGGA" in gpgga_collected_data[0]):
                
                if gpgga_collected_data[5] == 'W':
                    lat = -1
                else:
                    lat = 1
                if gpgga_collected_data[3] == 'S':
                    lon = -1 
                else:
                    lon = 1
                gpgga_msg_values = gps_msg()

                (sec,nsec)=Convert_time(float(gpgga_collected_data[1]))
                gpgga_msg_values.Header.stamp.secs = int(sec)
                gpgga_msg_values.Header.stamp.nsecs = int(nsec)
                gpgga_msg_values.Header.frame_id = 'GPS1_FRAME'
                Latitude = Convert_gpggs_degrees(float(gpgga_collected_data[2]))*lat
                gpgga_msg_values.Latitude = Latitude
                Longitude = Convert_gpggs_degrees(float(gpgga_collected_data[4]))*lon
                gpgga_msg_values.Longitude = Longitude 
                gpgga_msg_values.Altitude = float(gpgga_collected_data[9])
                (gpgga_msg_values.UTM_easting,gpgga_msg_values.UTM_northing,gpgga_msg_values.Zone,gpgga_msg_values.Letter)=utm.from_latlon(Latitude,Longitude)
                gpgga_msg_values.Quality = int(gpgga_collected_data[6])

                print(gpgga_msg_values)
                gpggs_data_pub.publish(gpgga_msg_values)

    except rospy.ROSInterruptException:
        port.close()
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down GPS node...")
