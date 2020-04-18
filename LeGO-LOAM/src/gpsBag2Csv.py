#!/usr/bin/env python

import rospy
import csv
from datetime import datetime
from sensor_msgs.msg import NavSatFix

f = open('gps_loam.csv','w')
fieldnames = ['date','time','latitude', 'longitude']
writer = csv.DictWriter(f, fieldnames=fieldnames)
writer.writeheader()

def callback(data):
    # rospy.loginfo(float(str(data.header.stamp))*0.000000001)
    timestamp = datetime.fromtimestamp(float(str(data.header.stamp))*0.000000001)
    writer.writerow({'date': timestamp.strftime("%m/%d/%Y"),'time': timestamp.strftime("%H:%M:%S"), 'latitude': str(data.latitude), 'longitude': str(data.longitude)})

if __name__ == '__main__':
    rospy.init_node('bag2csv')
    pub = rospy.Subscriber('/vehicle/gps/fix', NavSatFix, callback)
    
    rospy.spin()
    
    f.close()