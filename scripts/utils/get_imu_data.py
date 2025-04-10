import telemetry_parser
import numpy as np
import os

class GoProIMUParser:
    def __init__(self, video_path):
        """
        初始化GoProIMU解析器
        
        Args:
            video_path (str): GoPro视频文件的完整路径
        """
        self.video_path = video_path
        self.root_dir = os.path.dirname(video_path)
        self.parser = telemetry_parser.Parser(video_path)
        
        # 定义IMU数据的结构
        self.dtype = [
            ('timestamp_ms', 'f8'),
            ('gyro_x', 'f8'), ('gyro_y', 'f8'), ('gyro_z', 'f8'),
            ('accl_x', 'f8'), ('accl_y', 'f8'), ('accl_z', 'f8')
        ]

    def get_camera_info(self):
        """获取相机信息"""
        return {
            'camera': self.parser.camera,
            'model': self.parser.model
        }

    def get_imu_data(self):
        """获取标准化的IMU数据"""
        return self.parser.normalized_imu()

    def convert_to_structured_array(self, imu_data):
        """
        将IMU数据转换为结构化数组
        
        Args:
            imu_data (list): IMU数据列表
            
        Returns:
            np.ndarray: 结构化数组
        """
        structured_data = np.zeros(len(imu_data), dtype=self.dtype)
        
        for i, data in enumerate(imu_data):
            structured_data[i] = (
                data['timestamp_ms'],
                data['gyro'][0], data['gyro'][1], data['gyro'][2],
                data['accl'][0], data['accl'][1], data['accl'][2]
            )
        
        return structured_data

    def save_imu_data(self, output_path=None):
        """
        保存IMU数据到NPY文件
        
        Args:
            output_path (str, optional): 输出文件路径。如果为None，则保存到视频同目录下
            
        Returns:
            str: 保存文件的路径
        """
        if output_path is None:
            output_path = os.path.join(self.root_dir, 'imu_data.npy')
            
        imu_data = self.get_imu_data()
        structured_data = self.convert_to_structured_array(imu_data)
        np.save(output_path, structured_data)
        
        return output_path

    def load_imu_data(self, file_path=None):
        """
        加载NPY文件中的IMU数据
        
        Args:
            file_path (str, optional): NPY文件路径。如果为None，则从默认位置加载
            
        Returns:
            np.ndarray: 加载的IMU数据
        """
        if file_path is None:
            file_path = os.path.join(self.root_dir, 'imu_data.npy')
            
        return np.load(file_path)
    
if __name__ == "__main__":
    # 创建解析器实例 
    # root_dir = "/home/clp/workspace/lias_gopro/data/video1/"
    root_dir = "/home/clp/workspace/lias_gopro/data/video2/"
    video_path = root_dir + "imu1.MP4"
    output_file = root_dir + "imu_data1.npy"
    video_path = root_dir + "imu2.MP4"
    output_file = root_dir + "imu_data2.npy"
    
    
    parser = GoProIMUParser(video_path)

    # 获取相机信息
    camera_info = parser.get_camera_info()
    print("Camera Info:", camera_info)

    # 保存IMU数据
    saved_path = parser.save_imu_data(output_file)
    # saved_path = parser.save_imu_data(output_file)
    print("Data saved to:", saved_path)

    # 加载IMU数据
    loaded_data = parser.load_imu_data(output_file)
    # loaded_data = parser.load_imu_data(output_file)
    print("Loaded data shape:", loaded_data.shape)