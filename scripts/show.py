import cv2
import sys

def read_video_stream(source):
    """
    读取视频流
    :param source: 可以是摄像头索引(通常0表示默认摄像头)或视频文件路径
    """
    # 创建视频捕获对象
    cap = cv2.VideoCapture(source)
    
    # 检查是否成功打开
    if not cap.isOpened():
        print("无法打开视频源")
        sys.exit()

    try:
        while True:
            # 逐帧读取
            ret, frame = cap.read()
            
            # 如果正确读取帧，ret为True
            if not ret:
                print("无法获取帧")
                break
            
            # 显示视频帧
            cv2.imshow('Video Stream', frame)
            
            # 按'q'键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        # 释放资源
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    
    # 或者读取视频文件
    read_video_stream("/home/clp/workspace/lias_gopro/data/video2/GX010072.MP4")