import requests
import time
import socket
import re
import subprocess

class GoProUSBControl:
    def __init__(self, serial_last_3=None):
        """
        初始化GoPro USB控制
        
        Args:
            serial_last_3: 相机序列号的最后三位，用于IP地址构建
        """
        self.base_url = None
        if serial_last_3:
            # 如果提供了序列号最后三位，则构建IP地址
            x = int(serial_last_3[0])
            yz = int(serial_last_3[1:3])
            self.base_url = f"http://172.2{x}.1{yz}.51:8080"
        else:
            # 尝试通过mDNS发现相机
            self.discover_camera()
            
        if not self.base_url:
            print("无法确定GoPro IP地址，请提供相机序列号的最后三位数字")
    
    def discover_camera(self):
        """尝试通过mDNS发现GoPro相机"""
        try:
            # 使用avahi-browse查找GoPro服务
            result = subprocess.run(
                ["avahi-browse", "-rt", "_gopro-web._tcp"], 
                capture_output=True, 
                text=True
            )
            
            # 解析输出以获取IP地址
            for line in result.stdout.splitlines():
                if "address = [" in line:
                    ip_match = re.search(r"address = \[(.*?)\]", line)
                    if ip_match:
                        ip = ip_match.group(1)
                        self.base_url = f"http://{ip}:8080"
                        print(f"发现GoPro相机，IP地址: {ip}")
                        return
        except Exception as e:
            print(f"mDNS发现失败: {e}")
            print("请确保已安装avahi-utils并且GoPro相机已连接")
    
    def enable_usb_control(self):
        """启用GoPro有线USB控制"""
        if not self.base_url:
            print("未设置相机IP地址")
            return False
        
        try:
            response = requests.get(f"{self.base_url}/gp/gpControl/usb")
            if response.status_code == 200:
                print("成功启用USB控制")
                return True
            else:
                print(f"启用USB控制失败: {response.status_code}")
                return False
        except requests.exceptions.RequestException as e:
            print(f"启用USB控制请求错误: {e}")
            return False
    
    def get_camera_state(self):
        """获取相机状态信息"""
        try:
            response = requests.get(f"{self.base_url}/gp/gpControl/status")
            if response.status_code == 200:
                return response.json()
            else:
                print(f"获取相机状态失败: {response.status_code}")
                return None
        except requests.exceptions.RequestException as e:
            print(f"获取相机状态请求错误: {e}")
            return None
    
    def take_photo(self):
        """拍摄照片"""
        # 首先检查相机状态
        state = self.get_camera_state()
        if state:
            if state.get("status", {}).get("1") == 1 or state.get("status", {}).get("2") == 1:
                print("相机忙碌或正在编码，无法拍照")
                return False
        
        # 发送拍照命令
        try:
            response = requests.get(f"{self.base_url}/gp/gpControl/command/shutter?p=1")
            if response.status_code == 200:
                print("成功拍摄照片")
                return True
            else:
                print(f"拍照失败: {response.status_code}")
                return False
        except requests.exceptions.RequestException as e:
            print(f"拍照请求错误: {e}")
            return False
    
    def start_recording(self):
        """开始录制视频"""
        try:
            response = requests.get(f"{self.base_url}/gp/gpControl/command/shutter?p=1")
            if response.status_code == 200:
                print("开始录制视频")
                return True
            else:
                print(f"开始录制失败: {response.status_code}")
                return False
        except requests.exceptions.RequestException as e:
            print(f"开始录制请求错误: {e}")
            return False
    
    def stop_recording(self):
        """停止录制视频"""
        try:
            response = requests.get(f"{self.base_url}/gp/gpControl/command/shutter?p=0")
            if response.status_code == 200:
                print("停止录制视频")
                return True
            else:
                print(f"停止录制失败: {response.status_code}")
                return False
        except requests.exceptions.RequestException as e:
            print(f"停止录制请求错误: {e}")
            return False
    
    def get_media_list(self):
        """获取媒体列表"""
        try:
            response = requests.get(f"{self.base_url}/gp/gpMediaList")
            if response.status_code == 200:
                return response.json()
            else:
                print(f"获取媒体列表失败: {response.status_code}")
                return None
        except requests.exceptions.RequestException as e:
            print(f"获取媒体列表请求错误: {e}")
            return None
    
    def set_camera_control(self, enable=True):
        """设置相机控制状态"""
        value = 1 if enable else 0
        try:
            response = requests.get(f"{self.base_url}/gp/gpControl/command/control/app?p={value}")
            if response.status_code == 200:
                print(f"相机控制已{'启用' if enable else '禁用'}")
                return True
            else:
                print(f"设置相机控制失败: {response.status_code}")
                return False
        except requests.exceptions.RequestException as e:
            print(f"设置相机控制请求错误: {e}")
            return False
    
    def keep_alive(self):
        """发送保持活动信号"""
        try:
            response = requests.get(f"{self.base_url}/gp/gpControl/status")
            return response.status_code == 200
        except:
            return False

# 使用示例
def main():
    # 方法1：如果知道相机序列号的最后三位数字（例如C3456789，使用"789"）
    gopro = GoProUSBControl("897")
    
    # 方法2：尝试自动发现相机
    # gopro = GoProUSBControl()
    
    # 如果未成功初始化IP地址，可以手动设置
    if not gopro.base_url:
        gopro.base_url = "http://172.2X.1YZ.51:8080"  # 替换X和YZ为正确的值
    
    # 启用USB控制
    if gopro.enable_usb_control():
        # 声明控制权
        gopro.set_camera_control(True)
        
        # 获取相机状态
        state = gopro.get_camera_state()
        if state:
            print("相机状态:", state)
        
        # 拍摄照片
        gopro.take_photo()
        
        # 休眠一秒
        time.sleep(1)
        
        # 开始录制视频
        gopro.start_recording()
        
        # 录制5秒
        time.sleep(5)
        
        # 停止录制
        gopro.stop_recording()
        
        # 获取媒体列表
        media = gopro.get_media_list()
        if media:
            print("媒体列表:", media)
        
        # 释放控制权
        gopro.set_camera_control(False)

if __name__ == "__main__":
    main()