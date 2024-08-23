import cv2
import numpy as np
import RPi.GPIO as GPIO
import PyLidar3
import sys
import time
import threading
import queue

# LiDAR 센서 초기화
lidar = PyLidar3.YdLidarX4(port='/dev/tty.usbserial-0001')
if lidar.Connect():
    print("LiDAR 연결 성공!")
    gen = lidar.StartScanning()
else:
    print("LiDAR 연결 실패!")
    sys.exit()

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
AN2, AN1, DIG2, DIG1 = 12, 13, 19, 16
GPIO.setup(AN2, GPIO.OUT)
GPIO.setup(AN1, GPIO.OUT)
GPIO.setup(DIG2, GPIO.OUT)
GPIO.setup(DIG1, GPIO.OUT)
p1 = GPIO.PWM(AN1, 100)
p2 = GPIO.PWM(AN2, 100)

# 카메라 초기화
camera = cv2.VideoCapture(0)
if not camera.isOpened():
    print("카메라를 열 수 없습니다.")
    sys.exit()

# Shared variables
direction_queue = queue.Queue()
exit_flag = threading.Event()

def detect_line(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 30])
    mask = cv2.inRange(hsv, lower_black, upper_black)
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        moments = cv2.moments(largest_contour)
        
        if moments["m00"] != 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            cv2.drawContours(frame, [largest_contour], 0, (0,255,0), 2)
            cv2.circle(frame, (cx, cy), 5, (0,0,255), -1)
            return frame, cx
    
    return frame, None

def detect_black_line(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 30, 100, cv2.THRESH_BINARY_INV)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 3)
        return True
    return False

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

def camera_thread():
    global line_tracing_active
    while not exit_flag.is_set():
        ret, frame = camera.read()
        if not ret:
            print("카메라에서 프레임을 읽을 수 없습니다.")
            break
        
        if line_tracing_active:
            frame, center_x = detect_line(frame)
            
            if center_x is not None:
                if center_x < frame.shape[1] / 3:
                    direction_queue.put("LEFT")
                elif center_x > 2 * frame.shape[1] / 3:
                    direction_queue.put("RIGHT")
                else:
                    direction_queue.put("STRAIGHT")
            else:
                direction_queue.put("STOP")
        else:
            black_line_detected = detect_black_line(frame)
            if black_line_detected:
                print("검은색 선 감지됨 - 라인 트레이싱 모드로 돌아갑니다.")
                line_tracing_active = True  # 라인 트레이싱 모드로 돌아감
                direction_queue.queue.clear()  # 큐 비우기
        
        cv2.imshow('Line Tracer', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit_flag.set()
    
    camera.release()
    cv2.destroyAllWindows()

def motor_control_thread():
    while not exit_flag.is_set():
        try:
            direction = direction_queue.get(timeout=1)
            if direction == "LEFT":
                print("Turn Left")
                p1.start(30)
                p2.start(70)
            elif direction == "RIGHT":
                print("Turn Right")
                p1.start(70)
                p2.start(30)
            elif direction == "STRAIGHT":
                print("Go Straight")
                GPIO.output(DIG1, GPIO.HIGH)
                GPIO.output(DIG2, GPIO.HIGH)
                p1.start(50)
                p2.start(50)
            else:  # STOP
                print("Stop")
                p1.start(0)
                p2.start(0)
        except queue.Empty:
            pass
    
    p1.stop()
    p2.stop()
    GPIO.cleanup()

def autonomous_drive_thread():
    global line_tracing_active
    while not exit_flag.is_set():
        if line_tracing_active:
            front_distance = get_front_distance()
            print(f"전방 거리: {front_distance:.2f} cm")
            if front_distance < 20:
                print("장애물 감지됨, 라인 트레이싱 모드 중지")
                line_tracing_active = False
                direction_queue.queue.clear()
                time.sleep(1)  # 일시 중지
        else:
            front_distance = get_front_distance()
            print(f"전방 거리: {front_distance:.2f} cm")
            if front_distance >= 20:
                print("장애물이 사라졌습니다. 전진합니다.")
                time.sleep(1)
            else:
                turn_left_90_degrees()
                while not line_tracing_active:
                    ret, frame = camera.read()
                    if not ret:
                        print("카메라에서 프레임을 읽을 수 없습니다.")
                        break
                    
                    black_line_detected = detect_black_line(frame)
                    
                    if black_line_detected:
                        print("검은색 선 감지됨 - 라인 트레이싱 모드로 돌아갑니다.")
                        line_tracing_active = True
                        direction_queue.queue.clear()
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

line_tracing_active = True

camera_thread = threading.Thread(target=camera_thread)
motor_thread = threading.Thread(target=motor_control_thread)
auto_drive_thread = threading.Thread(target=autonomous_drive_thread)

camera_thread.start()
motor_thread.start()
auto_drive_thread.start()

camera_thread.join()
motor_thread.join()
auto_drive_thread.join()

lidar.StopScanning()
lidar.Disconnect()

print("프로그램 종료")