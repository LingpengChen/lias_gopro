import cv2
import requests
import time

def start_webcam():
    # 先禁用USB控制
    requests.get('http://172.28.197.51:8080/gopro/camera/control/wired_usb?p=0')
    time.sleep(1)
    
    # 启动webcam预览
    requests.get('http://172.28.197.51:8080/gopro/webcam/preview')
    time.sleep(1)
    
    # 启动webcam, 使用TS协议
    params = {
        'res': 4,  # 1080p
        'fov': 0,   # Wide
        'protocol': 'TS',
        'port': 8554
    }
    requests.get('http://172.28.197.51:8080/gopro/webcam/start', params=params)
    
    # 使用UDP接收视频流
    cap = cv2.VideoCapture('udp://@:8554')
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    try:
        while True:
            ret, frame = cap.read()
            if ret:
                cv2.imshow('GoPro Webcam', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        # 停止webcam
        requests.get('http://172.28.197.51:8080/gopro/webcam/stop')
        requests.get('http://172.28.197.51:8080/gopro/webcam/exit')

if __name__ == "__main__":
    start_webcam()