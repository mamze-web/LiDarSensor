import sys
sys.path.append('/opt/homebrew/lib/python3.12/site-packages')  # 'PyLidar3'가 설치된 경로
import time
import PyLidar3
import math

# Lidar 초기화
port = "/dev/tty.usbserial-0001"  # 포트 설정 (예: Windows에서는 'COM3' 등)
lidar = PyLidar3.YdLidarX4(port)
if not lidar.Connect():
    print("Lidar 연결 실패!")
    exit()

zones = {
    'A': (346, 15),
    'B': (16, 45),
    'C': (46, 75),
    'D': (76, 105),
    'E': (106, 135),
    'F': (136, 165),
    'G': (166, 195),
    'H': (196, 225),
    'I': (226, 255),
    'J': (256, 285),
    'K': (286, 315),
    'L': (316, 345)
}

excluded_zones = ['E', 'F', 'G', 'H', 'I']

def get_zone(angle):
    """각도에 해당하는 구역을 반환"""
    for zone, (start, end) in zones.items():
        if start < end:
            if start <= angle <= end:
                return zone
        else:  # 구역이 0도를 넘어가는 경우
            if angle >= start or angle <= end:
                return zone
    return None

def is_in_excluded_zone(angle):
    """주어진 각도가 제외된 구역에 있는지 확인"""
    zone = get_zone(angle)
    return zone in excluded_zones

def scan_and_get_longest_distance(scan):
    """제외된 구역을 제외하고 가장 긴 거리의 각도를 반환"""
    longest_distance = 0
    best_angle = None
    for angle in scan:
        if is_in_excluded_zone(angle):
            continue
        distance = scan[angle]
        if distance > longest_distance:
            longest_distance = distance
            best_angle = angle
    return best_angle

def get_front_distance(scan):
    """A 구역에서 전방 거리를 측정"""
    distances = []
    for angle in scan:
        if 346 <= angle <= 359 or 0 <= angle <= 15:
            distances.append(scan[angle])
    return min(distances) if distances else float('inf')

def rotate_to_angle(target_angle):
    """주어진 각도로 회전"""
    print(f"{target_angle}도로 회전합니다.")
    # 실제 로봇 회전 코드를 여기에 추가

try:
    gen = lidar.StartScanning()
    while True:
        scan = next(gen)
        front_distance = get_front_distance(scan)
        print(f"전방 거리: {front_distance} cm")

        if front_distance <= 20:
            print("장애물 감지! 3초 정지합니다.")
            time.sleep(3)

            scan = next(gen)
            front_distance = get_front_distance(scan)
            if front_distance <= 20:
                best_angle = scan_and_get_longest_distance(scan)
                target_zone = get_zone(best_angle)
                print(f"가장 긴 거리의 각도: {best_angle}도, 구역: {target_zone}")

                rotate_to_angle(best_angle)
            else:
                print("전방 장애물이 제거되었습니다. 계속 전진합니다.")
        else:
            print("전진합니다.")
            # 로봇 전진 코드 여기에 추가

        time.sleep(0.1)  # 반복 주기 설정

except KeyboardInterrupt:
    print("중지합니다.")
finally:
    lidar.StopScanning()
    lidar.Disconnect()
