import requests

# GoPro API基础URL
BASE_URL = "http://172.28.197.51:8080"


# 获取陀螺仪校准参数
def get_gyro_calib():
    response = requests.get(f"{BASE_URL}/gopro/camera/gyro_calib")
    if response.status_code == 200:
        return response.json()
    return None

# 获取加速度计校准参数
def get_accel_calib():
    response = requests.get(f"{BASE_URL}/gopro/camera/accel_calib")
    if response.status_code == 200:
        return response.json()
    return None

# 获取完整IMU状态
def get_imu_status():
    response = requests.get(f"{BASE_URL}/gopro/camera/imu_status")
    if response.status_code == 200:
        return response.json()
    return None

# 使用示例
gyro_data = get_gyro_calib()
print(gyro_data)
accel_data = get_accel_calib()
print(accel_data)
imu_status = get_imu_status()
print(imu_status)