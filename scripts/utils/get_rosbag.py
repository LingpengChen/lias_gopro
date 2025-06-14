#!/usr/bin/env python3
import rosbag
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu, CompressedImage
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
import os

class GoProToRosBagConverter:
    def __init__(self, 
                 video_path,
                 frame_timestamps_path,
                 imu_data_path,
                 output_bag_path,
                 frame_topic="/camera/image_raw/compressed",
                 imu_topic="/imu/data",
                 frame_rate=10,
                 scale_factor=1):
        """
        初始化GoPro到ROS包转换器
        
        Args:
            video_path (str): 视频文件路径
            frame_timestamps_path (str): 帧时间戳文件路径
            imu_data_path (str): IMU数据文件路径
            output_bag_path (str): 输出ROS包路径
            frame_topic (str): 图像话题名称
            imu_topic (str): IMU话题名称
            frame_rate (int): 目标帧率
            scale_factor (float): 图像缩放因子
        """
        self.video_path = video_path
        self.frame_timestamps_path = frame_timestamps_path
        self.imu_data_path = imu_data_path
        self.output_bag_path = output_bag_path
        self.frame_topic = frame_topic
        self.imu_topic = imu_topic
        self.frame_rate = frame_rate
        self.scale_factor = scale_factor
        
        self.bridge = CvBridge()
        
        # 初始化ROS节点
        try:
            rospy.init_node('gopro_to_rosbag_converter', anonymous=True, disable_signals=True)
        except rospy.exceptions.ROSException:
            pass
        
    def load_data(self):
        """加载时间戳和IMU数据"""
        self.frame_timestamps = np.load(self.frame_timestamps_path)
        self.imu_data = np.load(self.imu_data_path)
        
    def process_video(self, bag):
        """处理视频帧"""
        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            raise Exception(f"无法打开视频文件: {self.video_path}")
        
        fps = cap.get(cv2.CAP_PROP_FPS)
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        frame_interval = int(fps / self.frame_rate)
        
        print(f"视频信息: {frame_count}帧, {fps}FPS")
        print(f"目标帧率: {self.frame_rate}FPS, 每{frame_interval}帧取1帧")
        
        frame_counter = 0
        processed_frames = 0
        
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                if frame_counter % frame_interval != 0:
                    frame_counter += 1
                    continue
                    
                if frame_counter < len(self.frame_timestamps):
                    timestamp = self.frame_timestamps[frame_counter] + 1000
                    ros_time = rospy.Time.from_sec(timestamp/1000)
                    
                    # 缩放图像
                    if self.scale_factor != 1.0:
                        new_size = (int(frame.shape[1] * self.scale_factor), 
                                  int(frame.shape[0] * self.scale_factor))
                        frame = cv2.resize(frame, new_size)
                    
                    # 转换为压缩图像消息
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
                    _, jpeg_data = cv2.imencode('.jpg', frame, encode_param)
                    
                    compressed_msg = CompressedImage()
                    compressed_msg.header.stamp = ros_time
                    compressed_msg.header.frame_id = "camera"
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = jpeg_data.tobytes()
                    
                    bag.write(self.frame_topic, compressed_msg, ros_time)
                    processed_frames += 1
                    
                    if processed_frames % 100 == 0:
                        print(f"已处理 {processed_frames} 帧...")
                    
                frame_counter += 1
                
        finally:
            cap.release()
            
        return processed_frames
    
    def process_imu(self, bag):
        """处理IMU数据"""
        print("开始处理IMU数据...")
        imu_counter = 0
        
        for imu_entry in self.imu_data:
            timestamp_ms = imu_entry['timestamp_ms'] + 1000
            gyro_x, gyro_y, gyro_z = imu_entry['gyro_x'], imu_entry['gyro_y'], imu_entry['gyro_z']
            accl_x, accl_y, accl_z = imu_entry['accl_x'], imu_entry['accl_y'], imu_entry['accl_z']
            
            imu_msg = Imu()
            ros_time = rospy.Time.from_sec(timestamp_ms / 1000.0)
            imu_msg.header = Header(stamp=ros_time, frame_id="imu")
            
            imu_msg.angular_velocity = Vector3(gyro_x, gyro_y, gyro_z)
            imu_msg.linear_acceleration = Vector3(accl_x, accl_y, accl_z)
            
            imu_msg.angular_velocity_covariance = [0] * 9
            imu_msg.linear_acceleration_covariance = [0] * 9
            imu_msg.orientation_covariance = [-1] * 9
            
            bag.write(self.imu_topic, imu_msg, ros_time)
            imu_counter += 1
            
            if imu_counter % 1000 == 0:
                print(f"已处理 {imu_counter} 条IMU数据...")
                
        return imu_counter
    
    def convert(self):
        """执行转换过程"""
        print(f"开始处理: {self.video_path}")
        print(f"输出ROS bag: {self.output_bag_path}")
        
        self.load_data()
        print(f"时间戳数量: {len(self.frame_timestamps)}")
        print(f"IMU数据数量: {len(self.imu_data)}")
        
        bag = rosbag.Bag(self.output_bag_path, 'w')
        
        try:
            processed_frames = self.process_video(bag)
            print(f"视频处理完成: 共添加 {processed_frames} 帧到ROS bag")
            
            processed_imus = self.process_imu(bag)
            print(f"IMU数据处理完成: 共添加 {processed_imus} 条IMU数据到ROS bag")
            
        finally:
            bag.close()
            print(f"ROS bag文件已创建: {self.output_bag_path}")
  
    def convert_imu(self):
        """执行转换过程"""
        
        self.imu_data = np.load(self.imu_data_path)
        print(f"IMU数据数量: {len(self.imu_data)}")
        
        bag = rosbag.Bag(self.output_bag_path, 'w')
        
        try:
            processed_imus = self.process_imu(bag)
            print(f"IMU数据处理完成: 共添加 {processed_imus} 条IMU数据到ROS bag")
            
        finally:
            bag.close()
            print(f"ROS bag文件已创建: {self.output_bag_path}")
            

if __name__ == "__main__":
    # root_dir = "/home/clp/workspace/lias_gopro/data/video1/"
    root_dir = "/home/clp/workspace/lias_gopro/data/video2/"
    
    converter = GoProToRosBagConverter(
        video_path=root_dir + "gopro.mp4",
        frame_timestamps_path=root_dir + "gopro_frame_timestamp.npy",
        imu_data_path=root_dir + "imu_data.npy",
        output_bag_path=root_dir + "gopro.bag",
        frame_rate=10,  # 降低帧率
        scale_factor=1  # 图像尺寸减半
    )
    
    # converter.convert()
    converter.convert_imu()