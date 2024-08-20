import sys
sys.path.append('/opt/homebrew/lib/python3.12/site-packages')  # 'PyLidar3'가 설치된 경로
import time
import math
import PyQt5.QtWidgets as QtWidgets
import PyQt5.QtCore as QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import PyLidar3
from sklearn.cluster import DBSCAN
import numpy as np
import matplotlib.pyplot as plt

class MplCanvas(FigureCanvas):

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.ax = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)

class LidarWidget(QtWidgets.QWidget):

    def __init__(self, port, duration=30):
        super().__init__()
        self.initUI()
        self.port = port
        self.duration = duration
        self.lidar = None
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.connect_lidar()
        self.map_data = []  # 맵 데이터 저장을 위한 리스트

    def initUI(self):
        self.setWindowTitle('LiDAR 실시간 맵핑')
        self.canvas = MplCanvas(self, width=5, height=4, dpi=100)
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)

    def connect_lidar(self):
        self.lidar = PyLidar3.YdLidarX4(self.port)
        if self.lidar.Connect():
            print("LiDAR 장치에 연결되었습니다.")
            print(self.lidar.GetDeviceInfo())
            self.gen = self.lidar.StartScanning()
            self.timer.start(100)  # 100ms마다 업데이트
        else:
            print("장치에 연결할 수 없습니다.")

    def update_plot(self):
        try:
            scan_data = next(self.gen)
            x_coords, y_coords = self.process_scan_data(scan_data)
            x_coords, y_coords = self.remove_noise(x_coords, y_coords)
            self.canvas.ax.clear()
            self.update_map(x_coords, y_coords)
            self.plot_clusters(x_coords, y_coords)
            self.canvas.ax.set_xlim(-750, 750)
            self.canvas.ax.set_ylim(-750, 750)
            self.canvas.ax.set_xlabel('X(cm)')
            self.canvas.ax.set_ylabel('Y(cm)')
            self.canvas.ax.set_title('Clustering')
            self.canvas.ax.plot(0, 0, 'ro', markersize=10)  # 원점을 크게 표시
            self.canvas.draw()
        except StopIteration:
            self.timer.stop()
            self.lidar.StopScanning()
            self.lidar.Disconnect()

    def process_scan_data(self, scan_data):
        x_coords = []
        y_coords = []
        for angle, distance in scan_data.items():
            if distance > 0:  # 유효한 거리 데이터만 처리
                angle_radians = math.radians(angle)
                y = (distance * math.cos(angle_radians)) / 10  # mm에서 cm로 변환
                x = (distance * math.sin(angle_radians)) / 10  # mm에서 cm로 변환
                x_coords.append(x)
                y_coords.append(y)
        return x_coords, y_coords

    def remove_noise(self, x_coords, y_coords):
        coords = np.column_stack((x_coords, y_coords))
        clustering = DBSCAN(eps=10, min_samples=5).fit(coords)  # 단위 변경에 따라 eps도 조정
        labels = clustering.labels_

        # 노이즈가 아닌 데이터만 필터링
        non_noise_coords = coords[labels != -1]
        return non_noise_coords[:, 0], non_noise_coords[:, 1]

    def update_map(self, x_coords, y_coords):
        # 새로운 데이터를 맵 데이터에 추가
        for x, y in zip(x_coords, y_coords):
            self.map_data.append((x, y))

    def plot_clusters(self, x_coords, y_coords):
        coords = np.column_stack((x_coords, y_coords))
        clustering = DBSCAN(eps=10, min_samples=5).fit(coords)  # 단위 변경에 따라 eps도 조정
        labels = clustering.labels_
        
        unique_labels = set(labels)
        colors = plt.cm.Spectral(np.linspace(0, 1, len(unique_labels)))
        
        for k, col in zip(unique_labels, colors):
            if k == -1:
                # 노이즈 포인트는 검은색으로 표시
                col = 'k'

            class_member_mask = (labels == k)
            xy = coords[class_member_mask]

            self.canvas.ax.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=col, markeredgecolor='k', markersize=6)

def main():
    port = input("LiDAR가 연결된 포트 이름을 입력하세요: ")  # 예: COM3 또는 /dev/ttyUSB0
    app = QtWidgets.QApplication(sys.argv)
    main_window = LidarWidget(port)
    main_window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()