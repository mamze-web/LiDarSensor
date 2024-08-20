import time
import sys
sys.path.append('/opt/homebrew/lib/python3.12/site-packages')  # 'PyLidar3'가 설치된 경로
import PyLidar3

# 라이다 포트와 Baud Rate 설정 (필요에 맞게 수정)
port = "/dev/tty.usbserial-0001" # 적절한 포트로 수정
baud_rate = 115200

# PyLidar3 객체 생성
lidar = PyLidar3.YdLidarX4(port, baud_rate)

# 존 정의 (각도 범위)
zones = {
    'A': (346, 15),    # A존: 346도 ~ 15도 (순환각도 고려)
    'D': (76, 105)     # D존: 76도 ~ 105도
}

def average_distance(distances, start_angle, end_angle):
    """주어진 각도 범위 내에서 평균 거리 계산"""
    total_distance = 0
    count = 0
    # 각도 순환 고려
    if start_angle <= end_angle:
        for angle in range(start_angle, end_angle + 1):
            distance = distances.get(angle, float('inf'))
            if distance < float('inf'):
                total_distance += distance
                count += 1
    else:  # 각도가 순환하는 경우
        for angle in range(start_angle, 360):
            distance = distances.get(angle, float('inf'))
            if distance < float('inf'):
                total_distance += distance
                count += 1
        for angle in range(0, end_angle + 1):
            distance = distances.get(angle, float('inf'))
            if distance < float('inf'):
                total_distance += distance
                count += 1

    return total_distance / count if count > 0 else float('inf')

def get_distance_readings(gen):
    scan = next(gen)  # 첫 번째 스캔 데이터를 가져옴
    return scan

def main():
    try:
        if lidar.Connect():
            print("Lidar 연결됨!")
        else:
            print("Lidar 이미 연결됨.")

        print("스캔 시작...")
        gen = lidar.StartScanning()
        
        while True:
            distances = get_distance_readings(gen)

            if not distances:
                continue

            # A존 및 D존의 각도 범위
            a_start_angle, a_end_angle = zones['A']
            d_start_angle, d_end_angle = zones['D']

            # 각 존 내 거리 평균 계산
            a_avg_distance = average_distance(distances, a_start_angle, a_end_angle)
            d_avg_distance = average_distance(distances, d_start_angle, d_end_angle)

            print(f"A존 평균 전방 거리: {a_avg_distance} cm")
            print(f"D존 평균 우측 거리: {d_avg_distance} cm")

            # 전방거리가 10cm 이하일 때 좌측 회전
            if a_avg_distance <= 10:
                print("장애물 감지! 좌측으로 회전")
                time.sleep(1)  # 좌측 회전하는 시간

            # 우측거리가 20cm 이하일 때까지 전진
            elif d_avg_distance <= 20:
                print("우측 거리 20cm 이하, 전진")
                time.sleep(1)  # 전진하는 시간

            # 우측거리가 20cm 이상일 때 우측 회전
            elif d_avg_distance > 20:
                print("우측 거리 20cm 이상, 우측으로 회전")
                time.sleep(1)  # 우측 회전하는 시간

    except KeyboardInterrupt:
        print("종료 중...")
    finally:
        lidar.StopScanning()
        lidar.Disconnect()

if __name__ == "__main__":
    main()
