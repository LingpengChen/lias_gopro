import cv2
import requests
import time
import yaml
import logging
from datetime import datetime
import sys

class GoProWebcam:
    def __init__(self, config_path):
        # 加载配置文件
        with open(config_path, 'r', encoding='utf-8') as file:
            self.config = yaml.safe_load(file)
        
        # 初始化配置
        self.camera_ip = self.config['camera']['ip']
        self.webcam_config = self.config['camera']['webcam']
        self.hypersmooth_config = self.config['camera']['hypersmooth']
        self.window_config = self.config['camera']['window']
        self.stream_config = self.config['camera']['stream']
        self.advanced_config = self.config['camera']['advanced']
        self.debug_config = self.config['camera']['debug']
        
        self.cap = None
        self.setup_logging()

    def setup_logging(self):
        """设置日志系统"""
        if self.debug_config['enabled']:
            logging.basicConfig(
                level=getattr(logging, self.debug_config['level']),
                format='%(asctime)s - %(levelname)s - %(message)s',
                handlers=[
                    logging.StreamHandler(sys.stdout),
                    logging.FileHandler(f'gopro_webcam_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log')
                ]
            )
        self.logger = logging.getLogger(__name__)

    def send_request(self, endpoint, params=None, retry_count=0):
        """发送HTTP请求到GoPro，支持重试机制"""
        url = f"http://{self.camera_ip}:8080{endpoint}"
        try:
            response = requests.get(
                url, 
                params=params, 
                timeout=self.advanced_config['request_timeout']
            )
            if response.status_code == 200:
                self.logger.debug(f"请求成功: {endpoint}")
                return True
            else:
                self.logger.warning(f"请求失败: {endpoint}, 状态码: {response.status_code}")
        except requests.exceptions.RequestException as e:
            self.logger.error(f"请求异常: {endpoint}, 错误: {e}")
            if retry_count < self.advanced_config['max_retries']:
                self.logger.info(f"尝试重试 ({retry_count + 1}/{self.advanced_config['max_retries']})")
                time.sleep(self.advanced_config['retry_delay'])
                return self.send_request(endpoint, params, retry_count + 1)
        return False

    def set_hypersmooth(self):
        """设置Hypersmooth"""
        if not self.hypersmooth_config['enabled']:
            self.logger.info("设置Hypersmooth: 关闭")
            return self.send_request(
                '/gopro/camera/setting',
                params={'setting': 135, 'option': self.hypersmooth_config['option']}
            )
        return True

    def initialize_webcam(self):
        """初始化webcam"""
        self.logger.info("开始初始化Webcam...")

        # 禁用USB控制
        if not self.send_request('/gopro/camera/control/wired_usb', params={'p': 0}):
            self.logger.error("禁用USB控制失败")
            return False
        time.sleep(self.advanced_config['init_delay'])

        # 设置Hypersmooth
        if not self.set_hypersmooth():
            self.logger.error("设置Hypersmooth失败")
            return False
        time.sleep(self.advanced_config['init_delay'])

        # 进入预览模式
        if not self.send_request('/gopro/webcam/preview'):
            self.logger.error("进入预览模式失败")
            return False
        time.sleep(self.advanced_config['init_delay'])

        # 启动webcam
        params = {
            'res': self.webcam_config['resolution'],
            'fov': self.webcam_config['fov'],
            'protocol': self.webcam_config['protocol'],
            'port': self.webcam_config['port']
        }
        if not self.send_request('/gopro/webcam/start', params=params):
            self.logger.error("启动Webcam失败")
            return False

        self.logger.info("Webcam初始化成功")
        return True

    def calculate_fps(self):
        """计算实际FPS"""
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.fps_time >= 1.0:
            self.current_fps = self.frame_count
            self.frame_count = 0
            self.fps_time = current_time
        return self.current_fps

    def start_streaming(self):
        """开始视频流显示"""
        if not self.initialize_webcam():
            self.logger.error("无法启动视频流")
            return

        # 打开视频流
        stream_url = f'udp://@:{self.webcam_config["port"]}'
        self.logger.info(f"正在连接视频流: {stream_url}")
        self.cap = cv2.VideoCapture(stream_url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, self.stream_config['buffer_size'])

        # FPS计算初始化
        self.frame_count = 0
        self.fps_time = time.time()
        self.current_fps = 0

        try:
            while True:
                ret, frame = self.cap.read()
                if ret:
                    # 调整图像大小
                    frame = cv2.resize(frame, 
                                    (self.window_config['width'], 
                                     self.window_config['height']))

                    # 显示性能统计
                    if self.debug_config['show_stats']:
                        fps = self.calculate_fps()
                        cv2.putText(frame, f"FPS: {fps}", (10, 30),
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                    cv2.imshow(self.window_config['name'], frame)

                    # FPS限制
                    if self.stream_config['fps_limit'] > 0:
                        time.sleep(1.0 / self.stream_config['fps_limit'])

                    # 按'q'退出
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                else:
                    self.logger.warning("帧获取失败")
                    time.sleep(0.1)

        except KeyboardInterrupt:
            self.logger.info("用户中断流程")
        except Exception as e:
            self.logger.error(f"视频流发生错误: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        """清理资源"""
        self.logger.info("开始清理资源...")
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        
        # 停止webcam
        self.send_request('/gopro/webcam/stop')
        self.send_request('/gopro/webcam/exit')
        self.logger.info("资源清理完成")


if __name__ == "__main__":
    try:
        webcam = GoProWebcam('webcam_config.yaml')
        webcam.start_streaming()
    except Exception as e:
        print(f"程序发生错误: {e}")
        sys.exit(1)
