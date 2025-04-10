import cv2
import requests
import time
import yaml
import pygame
import threading
import json
import numpy as np

class GoProWebcam:
    def __init__(self, config_path):
        # 加载配置文件
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)
        
        self.camera_ip = self.config['camera']['ip']
        self.webcam_config = self.config['camera']['webcam']
        self.hypersmooth_config = self.config['camera']['hypersmooth']
        
        self.cap = None
        self.running = True
        self.video_frame = None
        self.camera_status = {}
        self.frame_shape = None
        self.is_recording = False
        self.is_powered_on = True  # 假设初始状态是开机的
        
        # 初始化pygame
        pygame.init()
        pygame.font.init()
        
        # 设置初始屏幕大小（可以根据第一帧调整）
        self.screen = pygame.display.set_mode((800, 600), pygame.RESIZABLE)
        pygame.display.set_caption("GoPro Control")
        
        # 字体
        self.font = pygame.font.SysFont('Arial', 24)
        self.small_font = pygame.font.SysFont('Arial', 18)
        
        # 按钮定义
        self.buttons = {
            'power': {'rect': pygame.Rect(20, 20, 150, 50), 'color': (50, 150, 50), 'text': 'Power On/Off'},
            'record': {'rect': pygame.Rect(190, 20, 150, 50), 'color': (150, 50, 50), 'text': 'Start Recording'},
            'preset': {'rect': pygame.Rect(360, 20, 150, 50), 'color': (50, 50, 150), 'text': 'Change Preset'},
            'exit': {'rect': pygame.Rect(530, 20, 150, 50), 'color': (150, 150, 150), 'text': 'Exit'}
        }
        
        # 启动状态更新线程
        self.status_thread = threading.Thread(target=self.update_camera_status)
        self.status_thread.daemon = True
        self.status_thread.start()

    def send_request(self, endpoint, params=None):
        """发送HTTP请求到GoPro，并返回响应对象"""
        url = f"http://{self.camera_ip}:8080{endpoint}"
        try:
            response = requests.get(url, params=params, timeout=5)
            return response
        except requests.exceptions.RequestException as e:
            print(f"请求失败: {e}")
            return None

    def power_camera(self, power_on=True):
        """开关机控制"""
        endpoint = '/gopro/camera/control/power/sleep'
        if power_on:
            endpoint = '/gopro/camera/control/power/wake'
        
        response = self.send_request(endpoint)
        if response and response.status_code == 200:
            self.is_powered_on = power_on
            print(f"相机电源已{'开启' if power_on else '关闭'}")
            # 如果关机，停止视频流
            if not power_on and self.cap:
                self.cleanup_camera()
            # 如果开机，等待一下再初始化摄像头
            elif power_on:
                time.sleep(3)  # 等待相机启动
                self.initialize_webcam()
            return True
        return False

    def toggle_recording(self):
        """开始/停止录制"""
        if not self.is_powered_on:
            print("相机未开机，无法控制录制")
            return False
            
        endpoint = '/gopro/camera/shutter/stop'
        new_status = False
        
        if not self.is_recording:
            endpoint = '/gopro/camera/shutter/start'
            new_status = True
            
        response = self.send_request(endpoint)
        if response and response.status_code == 200:
            self.is_recording = new_status
            print(f"录制已{'开始' if new_status else '停止'}")
            return True
        return False

    def change_preset(self):
        """切换预设"""
        if not self.is_powered_on:
            print("相机未开机，无法切换预设")
            return False
            
        # 获取当前预设列表
        response = self.send_request('/gopro/camera/presets/get')
        if response and response.status_code == 200:
            presets = response.json()
            if 'presets' in presets:
                # 简单地选择下一个预设
                current_preset = presets.get('current_preset', 0)
                next_preset = (current_preset + 1) % len(presets['presets'])
                
                # 设置新预设
                set_response = self.send_request('/gopro/camera/presets/load', 
                                               params={'id': next_preset})
                if set_response and set_response.status_code == 200:
                    print(f"已切换到预设 {next_preset}")
                    return True
        return False

    def set_hypersmooth(self):
        """设置Hypersmooth"""
        if not self.hypersmooth_config['enabled']:
            response = self.send_request(
                '/gopro/camera/setting',
                params={'setting': 135, 'option': self.hypersmooth_config['option']}
            )
            return response and response.status_code == 200
        return True

    def initialize_webcam(self):
        """初始化webcam"""
        if not self.is_powered_on:
            print("相机未开机，无法初始化")
            return False
            
        # 禁用USB控制
        response = self.send_request('/gopro/camera/control/wired_usb', params={'p': 0})
        if not response or response.status_code != 200:
            return False
        time.sleep(1)

        # 设置Hypersmooth
        if not self.set_hypersmooth():
            return False
        time.sleep(1)

        # 启动webcam
        params = {
            'res': self.webcam_config['resolution'],
            'fov': self.webcam_config['fov'],
            'protocol': self.webcam_config['protocol'],
            'port': self.webcam_config['port']
        }
        response = self.send_request('/gopro/webcam/start', params=params)
        success = response and response.status_code == 200
        
        if success:
            # 打开视频流
            self.cap = cv2.VideoCapture(f'udp://@:{self.webcam_config["port"]}')
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        return success

    def update_camera_status(self):
        """更新相机状态的线程函数"""
        while self.running:
            if self.is_powered_on:
                response = self.send_request('/gopro/camera/state')
                if response and response.status_code == 200:
                    self.camera_status = response.json()
            time.sleep(1)  # 每秒更新一次状态

    def capture_video_frame(self):
        """捕获视频帧"""
        if self.cap and self.is_powered_on:
            ret, frame = self.cap.read()
            if ret:
                # 保存帧形状信息
                self.frame_shape = frame.shape
                # 将OpenCV的BGR格式转换为Pygame的RGB格式
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                self.video_frame = frame
                return True
        return False

    def render_ui(self):
        """渲染Pygame UI"""
        # 填充背景
        self.screen.fill((30, 30, 30))
        
        # 如果有视频帧，显示它
        if self.video_frame is not None:
            # 创建Pygame表面
            h, w = self.video_frame.shape[:2]
            surface = pygame.surfarray.make_surface(self.video_frame.swapaxes(0, 1))
            
            # 调整屏幕大小以适应视频（如果需要）
            screen_w, screen_h = self.screen.get_size()
            if w != screen_w or h + 200 != screen_h:  # 添加额外空间用于UI元素
                self.screen = pygame.display.set_mode((w, h + 200), pygame.RESIZABLE)
            
            # 显示视频帧
            self.screen.blit(surface, (0, 100))
        
        # 渲染按钮
        for name, button in self.buttons.items():
            # 更新按钮文字
            if name == 'power':
                button['text'] = 'Power Off' if self.is_powered_on else 'Power On'
                button['color'] = (50, 150, 50) if self.is_powered_on else (150, 50, 50)
            elif name == 'record':
                button['text'] = 'Stop Recording' if self.is_recording else 'Start Recording'
                button['color'] = (200, 50, 50) if self.is_recording else (150, 50, 50)
            
            # 绘制按钮
            pygame.draw.rect(self.screen, button['color'], button['rect'])
            pygame.draw.rect(self.screen, (200, 200, 200), button['rect'], 2)
            
            # 绘制按钮文字
            text = self.font.render(button['text'], True, (255, 255, 255))
            text_rect = text.get_rect(center=button['rect'].center)
            self.screen.blit(text, text_rect)
        
        # 显示相机状态
        y_pos = 550
        if self.camera_status:
            # 获取一些关键状态信息
            status_items = [
                f"电池: {self.camera_status.get('status', {}).get('battery', 'N/A')}%",
                f"存储: {self.camera_status.get('status', {}).get('remaining_space', 'N/A')} MB",
                f"模式: {self.camera_status.get('status', {}).get('current_mode', 'N/A')}",
                f"预设: {self.camera_status.get('status', {}).get('current_preset', 'N/A')}"
            ]
            
            # 如果有录制信息
            if self.is_recording and 'recording_progress' in self.camera_status.get('status', {}):
                rec_progress = self.camera_status['status']['recording_progress']
                status_items.append(f"录制时间: {rec_progress}")
            
            # 显示帧尺寸
            if self.frame_shape:
                status_items.append(f"分辨率: {self.frame_shape[1]}x{self.frame_shape[0]}")
            
            # 渲染状态文本
            status_x = 20
            for item in status_items:
                text = self.small_font.render(item, True, (200, 200, 200))
                self.screen.blit(text, (status_x, y_pos))
                status_x += text.get_width() + 20
        else:
            text = self.small_font.render("相机状态: 未连接或已关机", True, (200, 200, 200))
            self.screen.blit(text, (20, y_pos))
        
        # 更新显示
        pygame.display.flip()

    def handle_events(self):
        """处理Pygame事件"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                return False
            
            if event.type == pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                
                # 检查是否点击了按钮
                for name, button in self.buttons.items():
                    if button['rect'].collidepoint(pos):
                        if name == 'power':
                            self.power_camera(not self.is_powered_on)
                        elif name == 'record':
                            self.toggle_recording()
                        elif name == 'preset':
                            self.change_preset()
                        elif name == 'exit':
                            self.running = False
                            return False
        
        return True

    def run(self):
        """主循环"""
        # 初始化摄像头
        self.initialize_webcam()
        
        # 主循环
        clock = pygame.time.Clock()
        while self.running:
            # 处理事件
            if not self.handle_events():
                break
            
            # 捕获视频帧
            self.capture_video_frame()
            
            # 渲染UI
            self.render_ui()
            
            # 控制帧率
            clock.tick(30)
        
        self.cleanup()

    def cleanup_camera(self):
        """清理相机资源"""
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        
        # 停止webcam
        self.send_request('/gopro/webcam/stop')
        self.send_request('/gopro/webcam/exit')

    def cleanup(self):
        """清理所有资源"""
        self.running = False
        self.cleanup_camera()
        pygame.quit()

def main():
    webcam = GoProWebcam('webcam_config.yaml')
    webcam.run()

if __name__ == "__main__":
    main()