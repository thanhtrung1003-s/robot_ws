#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from sensor_msgs.msg import LaserScan

class node_lidar():
    def __init__(self):
        rospy.init_node("node_lidar", anonymous=True)

        self.port_name = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.frame_id = rospy.get_param("~frame_id", "laser")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.min_range = rospy.get_param("~min_range", 0.1)
        self.max_range = rospy.get_param("~max_range", 5.0)
        self.downsample_step = rospy.get_param("~downsample_step", 3)  # mỗi 3 điểm lấy 1

        self.pub_rate = rospy.get_param("~pub_rate", 5)  # Hz
        self.last_pub_time = rospy.Time.now()

        self.pub = rospy.Publisher("/scan_data", LaserScan, queue_size=10)
        self.sub = rospy.Subscriber(self.scan_topic, LaserScan, self.callback)

        rospy.loginfo("node_lidar started, repub to /scan_data at %d Hz (downsample_step=%d)", 
                      self.pub_rate, self.downsample_step)

    def callback(self, scan):
        if (rospy.Time.now() - self.last_pub_time).to_sec() < (1.0 / self.pub_rate):
            return

        scan_filler = LaserScan()
        scan_filler.header = scan.header
        scan_filler.header.frame_id = self.frame_id

        # Góc min, max thay đổi theo downsample
        scan_filler.angle_min = scan.angle_min
        scan_filler.angle_max = scan.angle_max
        scan_filler.angle_increment = scan.angle_increment * self.downsample_step
        scan_filler.time_increment = scan.time_increment * self.downsample_step
        scan_filler.scan_time = scan.scan_time
        scan_filler.range_min = self.min_range
        scan_filler.range_max = self.max_range

        # Downsample ranges
        data = []
        intensities = []
        for i in range(0, len(scan.ranges), self.downsample_step):
            r = scan.ranges[i]
            if r < self.min_range or r > self.max_range:
                data.append(float('inf'))
            else:
                data.append(r)

            if i < len(scan.intensities):
                intensities.append(scan.intensities[i])

        scan_filler.ranges = data
        scan_filler.intensities = intensities

        self.pub.publish(scan_filler)
        self.last_pub_time = rospy.Time.now()

if __name__ == "__main__":
    try:
        node_lidar()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
