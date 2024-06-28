import PyLidar3
import time  # 시간 모듈

# LiDAR가 연결된 시리얼 포트 설정, 윈도우의 경우 장치 관리자에서 확인
# 리눅스의 경우 터미널에 -- ls /dev/tty* 입력하여 확인
port = input("LiDAR가 연결된 포트 이름을 입력하세요:")  # 윈도우의 경우
#port = "/dev/ttyUSB0"  # 리눅스의 경우

# LiDAR 객체 생성
Obj = PyLidar3.YdLidarX4(port)  # PyLidar3.your_version_of_lidar(port, chunk_size)

# LiDAR 연결 시도
if(Obj.Connect()):
    print(Obj.GetDeviceInfo())  # LiDAR 장치 정보 출력
    gen = Obj.StartScanning()  # 스캔 시작
    t = time.time()  # 시작 시간 기록
    while (time.time() - t) < 30:  # 30초 동안 스캔
        print(next(gen))  # 스캔 데이터 출력
        time.sleep(0.5)  # 0.5초 대기
    Obj.StopScanning()  # 스캔 중지
    Obj.Disconnect()  # LiDAR 연결 해제
else:
    print("장치에 연결할 수 없습니다.")  # 연결 실패 시 메시지 출력