#!/usr/bin/env python3
import rosbag
import numpy as np
import cv2
import os
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu, CompressedImage
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3

from utils.get_imu_data import GoProIMUParser
from utils.get_timestamp import VideoTimestampParser
from utils.get_rosbag import GoProToRosBagConverter

if __name__ == "__main__":
    
    try:
        rospy.init_node('gopro_to_rosbag_converter', anonymous=True, disable_signals=True)
    except rospy.exceptions.ROSException:
        pass
    
    root_dir = "/home/clp/workspace/lias_gopro/data/video3/"
    
    video_path = root_dir + "gopro.mp4"
    frame_timestamps_path = root_dir+ "gopro_frame_timestamp.npy"
    imu_data_path = root_dir + "imu_data.npy"
    output_bag_path = root_dir + "gopro.bag"
    
    
    # STEP 1 Process imu
    parser = GoProIMUParser(video_path)

    # 获取相机信息
    camera_info = parser.get_camera_info()
    print("Camera Info:", camera_info)

    # 保存IMU数据
    saved_path = parser.save_imu_data()
    # saved_path = parser.save_imu_data(output_file)
    print("Data saved to:", saved_path)

    # 加载IMU数据
    loaded_data = parser.load_imu_data()
    # loaded_data = parser.load_imu_data(output_file)
    print("Loaded data shape:", loaded_data.shape)
    
    
    # STEP 2 Get timestamp
    # 创建解析器实例
    parser = VideoTimestampParser(video_path)
    
    # 获取并打印视频流信息
    # stream_info = parser.get_stream_info()
    # print("Stream信息:")
    # print(stream_info)
    
    # 保存时间戳
    npy_file, txt_file = parser.save_timestamps()
    print(f"已保存NPY文件到: {npy_file}")
    print(f"已保存文本文件到: {txt_file}")
    
    # 加载时间戳
    timestamps = parser.load_timestamps()
    print(f"加载了 {len(timestamps)} 个时间戳")
    
    
    # STEP 3 Get ROSBAG
    
    converter = GoProToRosBagConverter(
        video_path=root_dir + "gopro.mp4",
        frame_timestamps_path=frame_timestamps_path,
        imu_data_path=imu_data_path,
        output_bag_path=output_bag_path,
        frame_rate=10,  # 降低帧率
        scale_factor=1  # 图像尺寸减半
    )
    
    converter.convert()