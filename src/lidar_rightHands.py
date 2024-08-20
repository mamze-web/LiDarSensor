import cv2
import numpy as np
import RPi.GPIO as GPIO
from time import sleep
import threading
import queue
import sys
sys.path.append('/opt/homebrew/lib/python3.12/site-packages')  # 'PyLidar3'가 설치된 경로
import time
import PyLidar3
import tkinter as tk

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

# LiDAR setup
lidar = PyLidar3.YdLidarX4(port='/dev/tty.usbserial-0001')  # 포트는 환경에 맞게 설정
if lidar.Connect():
    print("LiDAR 연결 성공!")
    gen = lidar.StartScanning()

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

def camera_thread():
    cap = cv2.VideoCapture(0)
    while not exit_flag.is_set():
        ret, frame = cap.read()
        if not ret:
            break
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
        
        cv2.imshow('Line Tracer', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit_flag.set()
    cap.release()
    cv2.destroyAllWindows()

def motor_control_thread():
    while not exit_flag.is_set():
        try:
            direction = direction_queue.get(timeout=1)
            if direction == "LEFT":
                p1.start(30)
                p2.start(70)
            elif direction == "RIGHT":
                p1.start(70)
                p2.start(30)
            elif direction == "STRAIGHT":
                GPIO.output(DIG1, GPIO.HIGH)
                GPIO.output(DIG2, GPIO.HIGH)
                p1.start(50)
                p2.start(50)
            else:  # STOP
                p1.start(0)
                p2.start(0)
        except queue.Empty:
            pass
    p1.stop()
    p2.stop()
    GPIO.cleanup()

def obstacle_avoidance_thread():
    while not exit_flag.is_set():
        front_distance = get_front_distance()
        if front_distance < 20:  # 장애물 감지
            direction_queue.queue.clear()  # 기존 라인트레이싱 명령 클리어
            # 장애물 회피 알고리즘 시작
            turn_left_90_degrees()
            while True:
                front_distance = get_front_distance()
                if front_distance > 20:
                    break
                time.sleep(0.1)
            direction_queue.put("RESUME")
        time.sleep(0.1)

def turn_left_90_degrees():
    print("좌측으로 90도 회전합니다.")
    GPIO.output(DIG1, GPIO.LOW)
    GPIO.output(DIG2, GPIO.HIGH)
    p1.start(50)
    p2.start(50)
    time.sleep(1)  # 임의로 회전 시간 설정, 조정 필요
    p1.start(0)
    p2.start(0)

def turn_right():
    print("우회전합니다.")
    GPIO.output(DIG1, GPIO.HIGH)
    GPIO.output(DIG2, GPIO.LOW)
    p1.start(50)
    p2.start(50)
    time.sleep(1)  # 임의로 회전 시간 설정, 조정 필요
    p1.start(0)
    p2.start(0)

camera_thread = threading.Thread(target=camera_thread)
motor_thread = threading.Thread(target=motor_control_thread)
obstacle_thread = threading.Thread(target=obstacle_avoidance_thread)

camera_thread.start()
motor_thread.start()
obstacle_thread.start()

camera_thread.join()
motor_thread.join()
obstacle_thread.join()

lidar.StopScanning()
lidar.Disconnect()

print("Program ended")