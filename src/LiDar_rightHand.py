import sys
sys.path.append('/opt/homebrew/lib/python3.12/site-packages')  # 'PyLidar3'가 설치된 경로
import time
import PyLidar3
import cv2
import numpy as np

# LiDAR 센서 초기화
lidar = PyLidar3.YdLidarX4(port='/dev/tty.usbserial-0001')
if lidar.Connect():
    print("LiDAR 연결 성공!")
    gen = lidar.StartScanning()
else:
    print("LiDAR 연결 실패!")
    sys.exit()

# 카메라 초기화
camera = cv2.VideoCapture(0)
if not camera.isOpened():
    print("카메라를 열 수 없습니다.")
    sys.exit()

# 검은색 선 감지 함수
def detect_black_line(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 임계값 조정: 검은색 선을 더 잘 감지하도록 조정
    _, binary = cv2.threshold(gray, 30, 100, cv2.THRESH_BINARY_INV)
    
    # 모폴로지 연산: 노이즈 제거 및 선의 두께 조정
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    # 선을 감지하고 이미지에 표시
    contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 3)
        return True
    return False

# 거리 측정 함수
def get_scan_data():
    return next(gen)

def average_distance(start_angle, end_angle, step=5):
    data = get_scan_data()
    angles = range(start_angle, end_angle + 1, step)
    distances = [data[angle % len(data)] for angle in angles]
    return sum(distances) / len(distances) if distances else float('inf')

def get_front_distance():
    return average_distance(0, 10)

def get_right_distance():
    return average_distance(80, 100)

def turn_left_90_degrees():
    print("좌측으로 90도 회전합니다.")
    time.sleep(1)

def turn_right():
    print("우회전합니다.")
    time.sleep(1)

def autonomous_drive():
    while True:
        front_distance = get_front_distance()
        print(f"전방 거리: {front_distance:.2f} cm")
        
        if front_distance >= 20:
            print("전진")
            time.sleep(1)
        else:
            print("전진 중지 - 장애물 감지됨")
            time.sleep(3)
            
            front_distance = get_front_distance()
            print(f"전방 거리: {front_distance:.2f} cm")
            if front_distance >= 20:
                print("장애물이 사라졌습니다. 전진합니다.")
                time.sleep(1)
            else:
                turn_left_90_degrees()
                
                while True:
                    ret, frame = camera.read()
                    if not ret:
                        print("카메라에서 프레임을 읽을 수 없습니다.")
                        break

                    black_line_detected = detect_black_line(frame)
                    
                    if black_line_detected:
                        print("검은색 선 감지됨 - 자율주행 초기화")
                        break
                    
                    right_distance = get_right_distance()
                    front_distance = get_front_distance()
                    print(f"전방 거리: {front_distance:.2f} cm")
                    print(f"우측 거리: {right_distance:.2f} cm")
                    
                    if right_distance <= 20:
                        print("전진")
                        time.sleep(1)
                        
                        if front_distance < 20:
                             print("오류")
                             turn_left_90_degrees()
                    else:
                        print("우회전")
                        turn_right()
                        time.sleep(1)

try:
    autonomous_drive()
except KeyboardInterrupt:
    print("자율주행을 종료합니다.")
finally:
    lidar.StopScanning()
    lidar.Disconnect()
    camera.release()
    cv2.destroyAllWindows()