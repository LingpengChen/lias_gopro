import cv2
import time
from datetime import datetime

def display_stream(stream_port=8554):
    cap = cv2.VideoCapture(f'udp://@:{stream_port}', cv2.CAP_FFMPEG)
    
    if not cap.isOpened():
        print("无法打开视频流")
        return
    
    cv2.namedWindow("GoPro Stream", cv2.WINDOW_NORMAL)

    frame_count = 0
    last_show_time = None
    try:
        while True:
            start_time = time.time()
            ret, frame = cap.read()
            acquisition_time = time.time()
            
            if ret:
                cv2.imshow("GoPro Stream", frame)
                show_time = time.time()
                frame_count += 1

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
                acquisition_time_interval = acquisition_time - start_time
                show_interval = show_time - acquisition_time

                if last_show_time:
                    delta_t = show_time - last_show_time
                    print(f"\nFrame {frame_count}:")
                    print(f"Frame acquisition time: {acquisition_time_interval*1000:.1f}ms")
                    print(f"Show interval: {show_interval*1000:.1f}ms")
                    print(f"Total processing time: {delta_t*1000:.1f}ms, FPS = {1/delta_t:.1f}")
                last_show_time = show_time
                
            else:
                print("读取视频帧失败")
                break
    
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    display_stream()