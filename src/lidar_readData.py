import sys
import time
import math
import PyQt5.QtWidgets as QtWidgets
import PyQt5.QtCore as QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import PyLidar3

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

    def initUI(self):
        self.setWindowTitle('LiDAR 실시간 시각화')
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
            self.canvas.ax.clear()
            self.canvas.ax.scatter(x_coords, y_coords, s=1)
            self.canvas.ax.set_xlim(-10000, 10000)
            self.canvas.ax.set_ylim(-10000, 10000)
            self.canvas.ax.set_xlabel('X 좌표 (mm)')
            self.canvas.ax.set_ylabel('Y 좌표 (mm)')
            self.canvas.ax.set_title('LiDAR 실시간 스캔 데이터')
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
                y = distance * math.cos(angle_radians)
                x = distance * math.sin(angle_radians)
                x_coords.append(x)
                y_coords.append(y)
        return x_coords, y_coords

def main():
    port = input("LiDAR가 연결된 포트 이름을 입력하세요: ")  # 예: COM3 또는 /dev/ttyUSB0
    app = QtWidgets.QApplication(sys.argv)
    main_window = LidarWidget(port)
    main_window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()