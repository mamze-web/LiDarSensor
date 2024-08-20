import sys
sys.path.append('/opt/homebrew/lib/python3.12/site-packages')  # 'PyLidar3'가 설치된 경로
import time
import PyLidar3  # LiDAR 센서를 제어하기 위해 사용
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

# 자율주행 루프
def autonomous_drive():
    global stop_flag
    while not stop_flag:
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
                
                while not stop_flag:
                    right_distance = get_right_distance()
                    front_distance = get_front_distance()
                    front_distance_label.config(text=f"전방 거리: {front_distance:.2f} cm")
                    right_distance_label.config(text=f"우측 거리: {right_distance:.2f} cm")
                    print(right_distance)
                    
                    if right_distance <= 20:
                        print("전진")
                        status_label.config(text="우측 확인 - 전진 중")
                        # 전진 로직 추가 필요
                        time.sleep(0)  # 전진 시간 시뮬레이션 (임시)
                        
                        if front_distance < 20:
                             
                             print("오류")
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
    root.destroy()  # Tkinter 윈도우 종료