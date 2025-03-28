import cv2
import requests
import time
import yaml
#   ip: "172.28.197.51"

class GoProWebcam:
    def __init__(self, config_path):
        # 加载配置文件
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)
        
        self.camera_ip = self.config['camera']['ip']
        self.webcam_config = self.config['camera']['webcam']
        self.hypersmooth_config = self.config['camera']['hypersmooth']
        self.window_config = self.config['camera']['window']
        
        self.cap = None

    def send_request(self, endpoint, params=None):
        """发送HTTP请求到GoPro"""
        url = f"http://{self.camera_ip}:8080{endpoint}"
        try:
            response = requests.get(url, params=params)
            return response.status_code == 200
        except requests.exceptions.RequestException as e:
            print(f"请求失败: {e}")
            return False

    def set_hypersmooth(self):
        """设置Hypersmooth"""
        if not self.hypersmooth_config['enabled']:
            return self.send_request(
                '/gopro/camera/setting',
                params={'setting': 135, 'option': self.hypersmooth_config['option']}
            )
        return True

    def initialize_webcam(self):
        """初始化webcam"""
        # 禁用USB控制
        if not self.send_request('/gopro/camera/control/wired_usb', params={'p': 0}):
            return False
        time.sleep(1)

        # 设置Hypersmooth
        if not self.set_hypersmooth():
            return False
        time.sleep(1)

        # # 进入预览模式
        # if not self.send_request('/gopro/webcam/preview'):
        #     return False
        # time.sleep(1)

        # 启动webcam
        params = {
            'res': self.webcam_config['resolution'],
            'fov': self.webcam_config['fov'],
            'protocol': self.webcam_config['protocol'],
            'port': self.webcam_config['port']
        }
        return self.send_request('/gopro/webcam/start', params=params)

    def start_streaming(self):
        """开始视频流显示"""
        if not self.initialize_webcam():
            print("Webcam初始化失败")
            return

        # 打开视频流
        self.cap = cv2.VideoCapture(f'udp://@:{self.webcam_config["port"]}')
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        try:
            while True:
                ret, frame = self.cap.read()
                print(frame.shape)
                if ret:
                    # 调整图像大小以匹配配置
                    frame = cv2.resize(frame, 
                                    (self.window_config['width'], 
                                     self.window_config['height']))
                    cv2.imshow(self.window_config['name'], frame)
                    
                    # 按'q'退出
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
        finally:
            self.cleanup()

    def cleanup(self):
        """清理资源"""
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        
        # 停止webcam
        self.send_request('/gopro/webcam/stop')
        self.send_request('/gopro/webcam/exit')

def main():
    webcam = GoProWebcam('webcam_config.yaml')
    webcam.start_streaming()

if __name__ == "__main__":
    main()