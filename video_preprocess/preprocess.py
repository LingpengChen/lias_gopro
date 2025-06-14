import cv2
import sys

def play_video(video_path, speed=1.0, width=None, height=None):
    # 打开视频文件
    cap = cv2.VideoCapture(video_path)
    
    if not cap.isOpened():
        print("错误：无法打开视频文件")
        return
    
    # 获取原始视频的FPS和尺寸
    fps = cap.get(cv2.CAP_PROP_FPS)
    original_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    original_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    # 如果没有指定新的尺寸，则计算一个默认的缩放尺寸
    if width is None and height is None:
        # 默认将宽度缩放到1280，高度等比例缩放
        scale = 1280 / original_width
        width = int(original_width * scale)
        height = int(original_height * scale)
    elif width is None:
        # 如果只指定了高度，宽度等比例缩放
        scale = height / original_height
        width = int(original_width * scale)
    elif height is None:
        # 如果只指定了宽度，高度等比例缩放
        scale = width / original_width
        height = int(original_height * scale)
    
    # 计算新的等待时间（毫秒）
    wait_ms = int(1000/(fps*speed))
    
    print(f"原始尺寸: {original_width}x{original_height}")
    print(f"显示尺寸: {width}x{height}")
    print(f"播放速度: {speed}x")
    print("按'q'退出，空格键暂停/继续")
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            break
            
        # 调整帧的大小
        resized_frame = cv2.resize(frame, (width, height))
        
        # 显示帧
        cv2.imshow('Video Player', resized_frame)
        
        # 等待按键事件
        key = cv2.waitKey(wait_ms)
        
        # 按'q'键退出
        if key & 0xFF == ord('q'):
            break
        # 按空格键暂停/继续
        elif key & 0xFF == ord(' '):
            cv2.waitKey(0)
    
    # 释放资源
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # 设置视频路径和播放参数
    video_path = "gopro.mp4"  # 替换为你的视频路径
    speed = 2.0  # 播放速度，1.0为正常速度，2.0为2倍速
    
    # 可以通过以下三种方式之一来设置显示尺寸：
    
    # 1. 指定宽度和高度
    play_video(video_path, speed, width=1280, height=720)
    
    # 2. 只指定宽度，高度等比例缩放
    # play_video(video_path, speed, width=1280)
    
    # 3. 只指定高度，宽度等比例缩放
    # play_video(video_path, speed, height=720)
    
    # 4. 使用默认缩放（宽度1280，高度等比例缩放）
    # play_video(video_path, speed)