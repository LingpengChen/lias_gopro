import subprocess
import json
import numpy as np
import os

class VideoTimestampParser:
    def __init__(self, video_path):
        """
        初始化视频时间戳解析器
        
        Args:
            video_path (str): 视频文件的完整路径
        """
        self.video_path = video_path
        self.root_dir = os.path.dirname(video_path)
        
    def get_stream_info(self):
        """
        获取视频流信息
        
        Returns:
            str: JSON格式的视频流信息
        """
        cmd = [
            'ffprobe',
            '-v', 'quiet',
            '-print_format', 'json',
            '-show_streams',
            self.video_path
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)
        return result.stdout
    
    def get_first_frame_info(self):
        """
        获取第一帧信息
        
        Returns:
            str: JSON格式的第一帧信息
        """
        cmd = [
            'ffprobe',
            '-v', 'quiet',
            '-select_streams', 'v:0',
            '-print_format', 'json',
            '-show_frames',
            '-read_intervals', '%+#1',  # 只读取第一帧
            self.video_path
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)
        return result.stdout
    
    def get_timestamps(self):
        """
        获取所有帧的时间戳
        
        Returns:
            np.ndarray: 时间戳数组（毫秒）
        """
        cmd = [
            'ffprobe',
            '-v', 'quiet',
            '-select_streams', 'v:0',
            '-show_entries', 
            'frame=pkt_pts_time',
            '-print_format', 'json',
            self.video_path
        ]
        
        print("Get and save timestamp from mp4 stream, may take a few seconds")
        result = subprocess.run(cmd, capture_output=True, text=True)
        data = json.loads(result.stdout)
        timestamps = [1000*float(frame['pkt_pts_time']) for frame in data['frames']]
        
        return np.array(timestamps)
    
    def save_timestamps(self, output_file=None):
        """
        保存时间戳到文件
        
        Args:
            output_file (str, optional): 输出文件路径。如果为None，则使用默认路径
            
        Returns:
            tuple: (npy文件路径, txt文件路径)
        """
        if output_file is None:
            output_file = os.path.join(self.root_dir, "gopro_frame_timestamp.npy")
            
        timestamps_array = self.get_timestamps()
        
        # 保存NPY文件
        np.save(output_file, timestamps_array)
        
        # 保存文本文件
        text_file = output_file.replace('.npy', '.txt') if output_file.endswith('.npy') else output_file + '.txt'
        np.savetxt(text_file, timestamps_array, fmt='%.6f')
        
        return output_file, text_file
    
    def load_timestamps(self, file_path=None):
        """
        从NPY文件加载时间戳
        
        Args:
            file_path (str, optional): NPY文件路径。如果为None，则使用默认路径
            
        Returns:
            np.ndarray: 时间戳数组
        """
        if file_path is None:
            file_path = os.path.join(self.root_dir, "gopro_frame_timestamp.npy")
            
        return np.load(file_path)

    

if __name__ == "__main__":
    # video_path = "/home/clp/workspace/lias_gopro/data/video1/gopro.mp4"
    video_path = "/home/clp/workspace/lias_gopro/data/video2/imu.MP4"
    
    # 创建解析器实例
    parser = VideoTimestampParser(video_path)
    
    # 获取并打印视频流信息
    stream_info = parser.get_stream_info()
    print("Stream信息:")
    print(stream_info)
    
    # 保存时间戳
    npy_file, txt_file = parser.save_timestamps()
    print(f"已保存NPY文件到: {npy_file}")
    print(f"已保存文本文件到: {txt_file}")
    
    # 加载时间戳
    timestamps = parser.load_timestamps()
    print(f"加载了 {len(timestamps)} 个时间戳")



# def get_all_frames_timestamps(video_path):
#     cmd = [
#         'ffprobe',
#         '-v', 'quiet',
#         '-select_streams', 'v:0',
#         '-show_frames',
#         '-print_format', 'json',
#         video_path
#     ]
    
#     result = subprocess.run(cmd, capture_output=True, text=True)
#     data = json.loads(result.stdout)
    
#     timestamps = []
#     for frame in data['frames']:
#         frame_info = {
#             'pts_time': float(frame['pkt_pts_time']),
#             'duration': float(frame['pkt_duration_time'])
#         }
#         timestamps.append(frame_info)
    
#     return timestamps