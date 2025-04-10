#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage, Imu

def image_callback(msg):
    # 将时间戳转换为秒
    secs = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
    rospy.loginfo("Image timestamp: %.6f seconds", secs)

def imu_callback(msg):
    # 将时间戳转换为秒  
    secs = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
    rospy.loginfo("IMU timestamp: %.6f seconds", secs)

def listener():
    rospy.init_node('timestamp_subscriber', anonymous=True)
    
    rospy.Subscriber("/camera/image_raw/compressed", CompressedImage, image_callback)
    rospy.Subscriber("/imu/data", Imu, imu_callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass