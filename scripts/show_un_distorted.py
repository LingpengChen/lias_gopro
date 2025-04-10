#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge
import math

class CameraViewer:
    def __init__(self):
        # 相机内参
        self.fx = 789.0448593807412
        self.fy = 789.7109365933173
        self.cx = 961.0688435438332
        self.cy = 840.6204460644894
        self.fov_dist = 0.8317727778027759
        self.width = 1920
        self.height = 1680
        
        # 创建内参矩阵
        self.K = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])
        
        # 预计算去畸变映射
        self.mapx, self.mapy = self.create_undistort_maps()
        
        # 初始化ROS节点和订阅器
        rospy.init_node('camera_viewer', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/image_raw/compressed",
            CompressedImage,
            self.callback,
            queue_size=1
        )
        
    def fov_undistort_point(self, x, y):
        # FOV模型去畸变
        x_dist = (x - self.cx) / self.fx
        y_dist = (y - self.cy) / self.fy
        r_dist = np.sqrt(x_dist**2 + y_dist**2)
        
        if r_dist < 1e-8:
            return x, y
            
        rd = math.atan(r_dist * 2 * math.tan(self.fov_dist / 2)) / self.fov_dist
        scale = rd / r_dist if r_dist > 0 else 1.0
        
        x_undist = x_dist * scale
        y_undist = y_dist * scale
        
        return (x_undist * self.fx + self.cx,
                y_undist * self.fy + self.cy)
    
    def create_undistort_maps(self):
        print("Creating undistortion maps...")
        # 创建去畸变映射表
        mapx = np.zeros((self.height, self.width), dtype=np.float32)
        mapy = np.zeros((self.height, self.width), dtype=np.float32)
        
        for y in range(self.height):
            for x in range(self.width):
                x_undist, y_undist = self.fov_undistort_point(x, y)
                mapx[y,x] = x_undist
                mapy[y,x] = y_undist
        
        print("Undistortion maps created!")
        return mapx, mapy
    
    def callback(self, data):
        try:
            # 将ROS压缩图像转换为OpenCV格式
            np_arr = np.frombuffer(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                rospy.logwarn("Failed to decode image")
                return
                
            # 检查图像尺寸是否匹配
            if cv_image.shape[:2] != (self.height, self.width):
                rospy.logwarn(f"Image size mismatch. Expected {(self.height, self.width)}, got {cv_image.shape[:2]}")
                cv_image = cv2.resize(cv_image, (self.width, self.height))
            
            # 使用映射表进行去畸变
            undistorted = cv2.remap(cv_image, self.mapx, self.mapy, 
                                  cv2.INTER_LINEAR)
            
            # 显示原始图像和去畸变后的图像
            cv2.imshow('Original', cv_image)
            cv2.imshow('Undistorted', undistorted)
            cv2.waitKey(1)
            
        except Exception as e:
            rospy.logerr(f"Error in callback: {str(e)}")

if __name__ == '__main__':
    try:
        viewer = CameraViewer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()