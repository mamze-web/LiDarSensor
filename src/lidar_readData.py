import PyLidar3
import time
import math 

def connect_lidar(port):
    Obj = PyLidar3.YdLidarX4(port)
    if Obj.Connect():
        print("LiDAR 장치에 연결되었습니다.")
        print(Obj.GetDeviceInfo())
        return Obj
    else:
        print("장치에 연결할 수 없습니다.")
        return None

def start_scanning(Obj, duration=30):
    gen = Obj.StartScanning()
    start_time = time.time()
    data = []
    while (time.time() - start_time) < duration:
        scan_data = next(gen)
        data.append(scan_data)
        time.sleep(0.5)
    Obj.StopScanning()
    return data

def process_scan_data(scan_data):
    angle_distance = {}
    for scan in scan_data:
        for angle, distance in scan.items():
            angle_distance[angle] = distance
    return angle_distance

def print_angle_distance(angle_distance):
    for angle in sorted(angle_distance):
        deg_radians = math.radians(angle) 
        cos_value = math.cos(deg_radians) 
        cos_value = angle_distance[angle]*cos_value 
        
        sin_value = math.sin(deg_radians) 
        sin_value = angle_distance[angle]*sin_value 
        
        print(f"({cos_value,sin_value})")

def main():
    port = input("LiDAR가 연결된 포트 이름을 입력하세요: ")  # 예: COM3 또는 /dev/ttyUSB0
    lidar = connect_lidar(port)
    if lidar:
        raw_data = start_scanning(lidar, duration=30)
        angle_distance = process_scan_data(raw_data)
        
        # 디버깅 출력을 추가합니다.
        print("수집된 데이터의 개수:", len(raw_data))
        if raw_data:
            print("첫 번째 스캔 데이터 샘플:", raw_data[0])
        
        print_angle_distance(angle_distance)
        
        lidar.Disconnect()

if __name__ == "__main__":
    main()