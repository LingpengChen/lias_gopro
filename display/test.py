import cv2
import requests
import tkinter as tk
from tkinter import Button, Frame
import threading
import socket
import numpy as np

class GoPro:
    def __init__(self, ip="172.28.197.51", control_port=8080, stream_port=8554):
        self.ip = ip
        self.control_port = control_port
        self.stream_port = stream_port
        self.base_url = f"http://{ip}:{control_port}/gopro"
        self.streaming = False
        self.recording = False
        self.stream_thread = None
        self.stop_event = threading.Event()
        
    def start_stream(self, hq=False):
        """启动预览流"""
        try:
            url = f"{self.base_url}/camera/stream/start"
            if hq:
                url += "?hq=true"
            response = requests.get(url)
            if response.status_code == 200:
                print("预览流已启动")
                self.streaming = True
                return True
            else:
                print(f"启动预览流失败: {response.status_code}")
                return False
        except Exception as e:
            print(f"启动预览流时出错: {e}")
            return False
    
    def stop_stream(self):
        """停止预览流"""
        try:
            response = requests.get(f"{self.base_url}/camera/stream/stop")
            if response.status_code == 200:
                print("预览流已停止")
                self.streaming = False
                return True
            else:
                print(f"停止预览流失败: {response.status_code}")
                return False
        except Exception as e:
            print(f"停止预览流时出错: {e}")
            return False
    
    def start_recording(self):
        """开始录制"""
        try:
            response = requests.get(f"{self.base_url}/camera/shutter/start")
            if response.status_code == 200:
                print("开始录制")
                self.recording = True
                return True
            else:
                print(f"开始录制失败: {response.status_code}")
                return False
        except Exception as e:
            print(f"开始录制时出错: {e}")
            return False
    
    def stop_recording(self):
        """停止录制"""
        try:
            response = requests.get(f"{self.base_url}/camera/shutter/stop")
            if response.status_code == 200:
                print("停止录制")
                self.recording = False
                return True
            else:
                print(f"停止录制失败: {response.status_code}")
                return False
        except Exception as e:
            print(f"停止录制时出错: {e}")
            return False
    
    def get_camera_status(self):
        """获取相机状态"""
        try:
            response = requests.get(f"{self.base_url}/camera/state")
            if response.status_code == 200:
                return response.json()
            else:
                print(f"获取相机状态失败: {response.status_code}")
                return None
        except Exception as e:
            print(f"获取相机状态时出错: {e}")
            return None

def display_stream(gopro, stop_event):
    # 设置接收UDP视频流的管道
    url = f'udp://@:{gopro.stream_port}'
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    
    if not cap.isOpened():
        print("无法打开视频流")
        return
    
    cv2.namedWindow("GoPro Stream", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("GoPro Stream", 640, 480)
    
    while not stop_event.is_set():
        ret, frame = cap.read()
        if ret:
            cv2.imshow("GoPro Stream", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print("读取视频帧失败")
            break
    
    cap.release()
    cv2.destroyAllWindows()

def main():
    # 创建GoPro实例
    gopro = GoPro(ip="172.28.197.51")
    
    # 创建一个简单的UI
    root = tk.Tk()
    root.title("GoPro 控制器")
    
    # 定义UI函数
    def toggle_stream():
        if not gopro.streaming:
            if gopro.start_stream(hq=True):
                stream_btn.config(text="停止预览")
                # 启动视频流显示线程
                gopro.stop_event.clear()
                gopro.stream_thread = threading.Thread(target=display_stream, args=(gopro, gopro.stop_event))
                gopro.stream_thread.daemon = True
                gopro.stream_thread.start()
        else:
            gopro.stop_event.set()
            if gopro.stream_thread:
                gopro.stream_thread.join(timeout=1.0)
            if gopro.stop_stream():
                stream_btn.config(text="开始预览")
    
    def toggle_recording():
        if not gopro.recording:
            if gopro.start_recording():
                record_btn.config(text="停止录制")
        else:
            if gopro.stop_recording():
                record_btn.config(text="开始录制")
    
    def exit_app():
        gopro.stop_event.set()
        if gopro.streaming:
            gopro.stop_stream()
        if gopro.recording:
            gopro.stop_recording()
        root.destroy()
    
    # 创建按钮
    control_frame = Frame(root)
    control_frame.pack(pady=10)
    
    stream_btn = Button(control_frame, text="开始预览", command=toggle_stream)
    stream_btn.pack(side=tk.LEFT, padx=5)
    
    record_btn = Button(control_frame, text="开始录制", command=toggle_recording)
    record_btn.pack(side=tk.LEFT, padx=5)
    
    exit_btn = Button(control_frame, text="退出", command=exit_app)
    exit_btn.pack(side=tk.LEFT, padx=5)
    
    # 运行主循环
    root.protocol("WM_DELETE_WINDOW", exit_app)
    root.mainloop()

if __name__ == "__main__":
    main()