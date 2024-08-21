import sys
sys.path.append('/opt/homebrew/lib/python3.12/site-packages')  # 'PyLidar3'가 설치된 경로
import time
import PyLidar3  # LiDAR 센서를 제어하기 위해 사용
import cv2  # OpenCV 라이브러리 사용
import tkinter as tk  # GUI 라이브러리 사용

# LiDAR 센서 초기화
lidar = PyLidar3.YdLidarX4(port='/dev/tty.usbserial-0001')  # 포트는 환경에 맞게 설정
if lidar.Connect():
    print("LiDAR 연결 성공!")
    gen = lidar.StartScanning()  # 스캔 데이터를 생성하는 제너레이터

# 거리 측정 함수
def get_scan_data():
    return next(gen)  # 제너레이터에서 스캔 데이터를 가져옴

def average_distance(start_angle, end_angle, step=5):
    data = get_scan_data()
    angles = range(start_angle, end_angle + 1, step)
    distances = [data[angle % len(data)] for angle in angles]  # 모듈로 연산으로 데이터 배열의 인덱스 처리
    return sum(distances) / len(distances) if distances else float('inf')

def get_front_distance():
    return average_distance(0, 10)  # 전방 방향 (0도 ~ 10도)

def get_right_distance():
    return average_distance(80, 100)  # 우측 방향 (80도 ~ 100도)

# 좌회전 함수
def turn_left_90_degrees():
    print("좌측으로 90도 회전합니다.")
    status_label.config(text="좌회전 중")
    time.sleep(0)  # 회전 시간 시뮬레이션 (임시)

# 우회전 함수
def turn_right():
    print("우회전합니다.")
    status_label.config(text="우회전 중")
    time.sleep(0)  # 회전 시간 시뮬레이션 (임시)

# 자율주행 중단 플래그
stop_flag = False

# 키보드 입력 감지 함수
def on_key_press(event):
    global stop_flag
    if event.keysym == 'l':
        print("L 키가 눌려 자율주행을 종료합니다.")
        stop_flag = True
        root.quit()  # Tkinter 루프 종료

# Tkinter 윈도우 설정
root = tk.Tk()
root.title("로봇 자율주행 시각화")

# 상태 표시 레이블
status_label = tk.Label(root, text="대기 중", font=("Arial", 20))
status_label.pack(pady=10)

# 거리 표시 레이블
front_distance_label = tk.Label(root, text="전방 거리: ", font=("Arial", 16))
front_distance_label.pack()

right_distance_label = tk.Label(root, text="우측 거리: ", font=("Arial", 16))
right_distance_label.pack()

root.bind('<KeyPress>', on_key_press)  # 모든 키보드 입력에 대해 이벤트 핸들러 호출
root.protocol('WM_DELETE_WINDOW', lambda: root.quit())  # 창 닫기 버튼 클릭 시 종료

# 카메라 초기화
cap = cv2.VideoCapture(0)  # 카메라 인덱스는 환경에 따라 다를 수 있음

def detect_black_line():
    ret, frame = cap.read()  # 카메라에서 프레임을 읽어옴
    if not ret:
        return False

    # 이미지를 그레이스케일로 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 임계처리로 검은 선 감지
    _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
    
    # 컨투어(윤곽선) 찾기
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 500:  # 검은 선의 면적이 특정 값 이상이면 감지된 것으로 판단
            return True
    
    return False

# 자율주행 루프
def autonomous_drive():
    global stop_flag
    camera_active = False  # 카메라로 검은선 감지를 시작했는지 여부를 추적
    
    while not stop_flag:
        if camera_active:  # 카메라로 검은선을 감지하는 중인지 확인
            if detect_black_line():
                print("검은 선이 감지되었습니다. 초기화합니다.")
                status_label.config(text="검은 선 감지 - 초기화")
                camera_active = False  # 카메라 감지 비활성화
                continue  # 알고리즘 초기화(다시 전진 단계로 돌아감)
        
        front_distance = get_front_distance()
        front_distance_label.config(text=f"전방 거리: {front_distance:.2f} cm")
        
        if front_distance >= 20:
            print("전진")
            status_label.config(text="전진 중")
            # 전진 로직 추가 필요
            time.sleep(0)  # 전진 시간 시뮬레이션 (임시)
        else:
            print("전진 중지")
            status_label.config(text="전진 중지 - 장애물 감지됨")
            time.sleep(3)  # 3초 대기
            
            front_distance = get_front_distance()
            front_distance_label.config(text=f"전방 거리: {front_distance:.2f} cm")
            if front_distance >= 20:
                print("장애물이 사라졌습니다. 전진합니다.")
                status_label.config(text="장애물 사라짐 - 전진 중")
                # 전진 로직 추가 필요
                time.sleep(0)  # 전진 시간 시뮬레이션 (임시)
            else:
                turn_left_90_degrees()
                camera_active = True  # 좌회전 후 카메라를 활성화하여 검은선 감지 시작
                
                while not stop_flag:
                    if camera_active:  # 카메라로 검은선을 감지하는 중인지 확인
                        if detect_black_line():
                            print("검은 선이 감지되었습니다. 초기화합니다.")
                            status_label.config(text="검은 선 감지 - 초기화")
                            camera_active = False  # 카메라 감지 비활성화
                            break  # 알고리즘 초기화(다시 전진 단계로 돌아감)
                    
                    right_distance = get_right_distance()
                    front_distance = get_front_distance()
                    front_distance_label.config(text=f"전방 거리: {front_distance:.2f} cm")
                    right_distance_label.config(text=f"우측 거리: {right_distance:.2f} cm")
                    
                    if right_distance <= 20:
                        print("전진")
                        status_label.config(text="우측 확인 - 전진 중")
                        # 전진 로직 추가 필요
                        time.sleep(0)  # 전진 시간 시뮬레이션 (임시)
                        
                        if front_distance < 20:
                            turn_left_90_degrees()
                    else:
                        print("우회전")
                        status_label.config(text="우회전 중")
                        turn_right()
                        # 우회전 후 다시 거리 측정을 위해 약간의 대기 시간 추가
                        time.sleep(0)

        root.update_idletasks()
        root.update()

try:
    autonomous_drive()
except KeyboardInterrupt:
    print("자율주행을 종료합니다.")
finally:
    lidar.StopScanning()
    lidar.Disconnect()  # LiDAR 센서 정지 및 연결 해제
    cap.release()  # 카메라 자원 해제
    root.destroy()  # Tkinter 윈도우 종료