#!/usr/bin/env python3
"""
ARECADA GCS
- Jetson에서는 detection_image ROS 토픽을 CAMERA 화면에 표시
- 필요하면 GCS_CAMERA_SOURCE=udp로 QGC용 RTP/H.264 직접 수신 가능
- PX4 텔레메트리와 QGC 화면 연동은 아직 목업
"""

import os
import select
import subprocess
import sys

from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QButtonGroup,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

rclpy = None
Image = None
ROS_IMPORT_ERROR = None


BG = "#0c1118"
PANEL = "#151d27"
PANEL_DARK = "#101720"
BORDER = "#41566c"
ACCENT = "#76d4ff"
ACCENT_DARK = "#199bd7"
TEXT = "#f2f6fa"
MUTED = "#91a4b7"


class ControlButton(QPushButton):
    """기능 이름만 표시하는 일반 버튼."""

    def __init__(self, text: str, parent=None):
        super().__init__(text, parent)
        self.setObjectName("controlButton")
        self.setCursor(Qt.PointingHandCursor)
        self.setMinimumHeight(38)


class EmptyDisplayPage(QFrame):
    """QGC 화면이 들어갈 빈 공간."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("emptyDisplay")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(500, 350)


class UdpVideoWorker(QThread):
    """QGC용 RTP/H.264 UDP 스트림을 GStreamer로 디코딩한다."""

    frame_ready = pyqtSignal(bytes, int, int)
    status_changed = pyqtSignal(str)

    def __init__(self, port=5600, width=1280, height=720, parent=None):
        super().__init__(parent)
        self.port = port
        self.width = width
        self.height = height
        self.process = None

    def run(self):
        frame_size = self.width * self.height * 3
        pipeline = [
            "gst-launch-1.0",
            "-q",
            "udpsrc",
            f"port={self.port}",
            (
                "caps=application/x-rtp,media=video,clock-rate=90000,"
                "encoding-name=H264,payload=96"
            ),
            "!",
            "rtpjitterbuffer",
            "latency=80",
            "drop-on-latency=true",
            "!",
            "rtph264depay",
            "!",
            "h264parse",
            "!",
            "avdec_h264",
            "!",
            "videoconvert",
            "!",
            "videoscale",
            "!",
            (
                f"video/x-raw,format=RGB,width={self.width},"
                f"height={self.height},pixel-aspect-ratio=1/1"
            ),
            "!",
            "fdsink",
            "fd=1",
            "sync=false",
        ]

        self.status_changed.emit(
            f"HM30 영상 대기 중...\nUDP {self.port}"
        )

        try:
            self.process = subprocess.Popen(
                pipeline,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except FileNotFoundError:
            self.status_changed.emit(
                "GStreamer를 찾을 수 없습니다.\n"
                "gst-launch-1.0 설치가 필요합니다."
            )
            return
        except Exception as exc:
            self.status_changed.emit(f"영상 수신기 실행 실패\n{exc}")
            return

        frame_buffer = bytearray()

        try:
            while not self.isInterruptionRequested():
                if self.process.poll() is not None:
                    error_message = self._read_process_error()
                    self.status_changed.emit(
                        "GStreamer 영상 수신이 종료되었습니다.\n"
                        f"{error_message}"
                    )
                    return

                stdout = self.process.stdout
                if stdout is None:
                    return

                readable, _, _ = select.select([stdout], [], [], 0.2)
                if not readable:
                    continue

                chunk = os.read(stdout.fileno(), 1024 * 1024)
                if not chunk:
                    continue

                frame_buffer.extend(chunk)
                complete_frame_count = len(frame_buffer) // frame_size
                if complete_frame_count == 0:
                    continue

                # 디코딩이 밀린 경우 오래된 프레임을 건너뛰고 최신 프레임만 표시한다.
                latest_start = (complete_frame_count - 1) * frame_size
                latest_end = latest_start + frame_size
                latest_frame = bytes(frame_buffer[latest_start:latest_end])
                del frame_buffer[:complete_frame_count * frame_size]

                self.frame_ready.emit(
                    latest_frame,
                    self.width,
                    self.height,
                )
        finally:
            self._stop_process()

    def stop(self):
        self.requestInterruption()
        self._stop_process()

    def _stop_process(self):
        process = self.process
        if process is None or process.poll() is not None:
            return

        process.terminate()
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.kill()

    def _read_process_error(self):
        if self.process is None or self.process.stderr is None:
            return "원인을 확인할 수 없습니다."

        try:
            message = self.process.stderr.read().decode(
                errors="replace"
            ).strip()
        except Exception:
            return "원인을 확인할 수 없습니다."

        return message or "RTP/H.264 스트림을 확인해 주세요."


class CameraDisplayPage(QFrame):
    """ROS Image 프레임을 화면 비율에 맞춰 보여주는 카메라 페이지."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("emptyDisplay")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(500, 350)

        self._source_pixmap = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.video_label = QLabel("카메라 연결 준비 중...")
        self.video_label.setObjectName("cameraView")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setSizePolicy(
            QSizePolicy.Ignored,
            QSizePolicy.Ignored,
        )
        layout.addWidget(self.video_label)

    def set_rgb_frame(self, frame):
        """RGB numpy 배열을 복사해 Qt 화면에 안전하게 표시한다."""
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            return

        height, width, channels = frame.shape
        image = QImage(
            frame.data,
            width,
            height,
            channels * width,
            QImage.Format_RGB888,
        ).copy()

        self._source_pixmap = QPixmap.fromImage(image)
        self._render_frame()

    def set_rgb_bytes(self, frame_data, width, height):
        image = QImage(
            frame_data,
            width,
            height,
            width * 3,
            QImage.Format_RGB888,
        ).copy()
        self.set_qimage(image)

    def set_qimage(self, image):
        self._source_pixmap = QPixmap.fromImage(image)
        self._render_frame()

    def show_status(self, message):
        if self._source_pixmap is None:
            self.video_label.clear()
            self.video_label.setText(message)

    def _render_frame(self):
        if self._source_pixmap is None:
            return

        scaled = self._source_pixmap.scaled(
            self.video_label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.video_label.setText("")
        self.video_label.setPixmap(scaled)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._render_frame()


class GroundStationMockup(QMainWindow):
    def __init__(
        self,
        camera_source="ros",
        camera_topic="/detection_image",
        video_port=5600,
        video_width=1280,
        video_height=720,
    ):
        super().__init__()

        self.camera_source = camera_source
        self.camera_topic = camera_topic
        self.video_port = video_port
        self.video_width = video_width
        self.video_height = video_height
        self.video_worker = None
        self.ros_node = None
        self.camera_subscription = None
        self.ros_spin_timer = None
        self.owns_ros_context = False

        self.setWindowTitle("GCS")
        self.resize(1280, 760)
        self.setMinimumSize(1000, 620)

        self._build_ui()
        self._apply_style()
        self._setup_camera()

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)

        outer_layout = QHBoxLayout(root)
        outer_layout.setContentsMargins(24, 20, 24, 20)
        outer_layout.setSpacing(16)

        outer_layout.addWidget(self._create_left_panel())
        outer_layout.addLayout(self._create_right_panel(), 1)

    def _create_left_panel(self):
        panel = QFrame()
        panel.setObjectName("leftPanel")
        panel.setFixedWidth(290)

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(12)

        layout.addWidget(self._section_title("FLIGHT MODE"))
        self.flight_mode_button = QPushButton("MISSION / OFFBOARD")
        self.flight_mode_button.setObjectName("largeButton")
        layout.addWidget(self.flight_mode_button)

        layout.addWidget(self._section_title("MISSION STATE"))
        self.state_button = QPushButton("STATE")
        self.state_button.setObjectName("largeButton")
        layout.addWidget(self.state_button)

        layout.addWidget(self._section_title("MODULE CONTROL"))

        control_names = [
            "cam_yolo",
            "coordinate",
            "yaw_align",
            "gripper",
            "mission_upload",
            "하부 개폐",
        ]

        self.control_buttons = {}
        for name in control_names:
            button = ControlButton(name)
            self.control_buttons[name] = button
            layout.addWidget(button)

        layout.addWidget(self._section_title("VTOL FLIGHT STATE"))
        self.vtol_state_button = QPushButton("고정익 / 천이 / 회전익")
        self.vtol_state_button.setObjectName("largeButton")
        layout.addWidget(self.vtol_state_button)

        layout.addStretch(1)
        return panel

    @staticmethod
    def _section_title(text: str):
        label = QLabel(text)
        label.setObjectName("sectionTitle")
        return label

    def _create_right_panel(self):
        right_layout = QVBoxLayout()
        right_layout.setSpacing(12)

        right_layout.addWidget(self._create_main_display(), 1)
        right_layout.addLayout(self._create_telemetry_panel())

        return right_layout

    def _create_main_display(self):
        frame = QFrame()
        frame.setObjectName("displayFrame")

        layout = QVBoxLayout(frame)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        toolbar = QHBoxLayout()
        toolbar.addStretch(1)

        self.camera_button = QPushButton("CAMERA")
        self.qgc_button = QPushButton("QGC")
        self.camera_button.setObjectName("viewButton")
        self.qgc_button.setObjectName("viewButton")

        self.camera_button.setCheckable(True)
        self.qgc_button.setCheckable(True)
        self.camera_button.setChecked(True)

        button_group = QButtonGroup(self)
        button_group.setExclusive(True)
        button_group.addButton(self.camera_button, 0)
        button_group.addButton(self.qgc_button, 1)

        toolbar.addWidget(self.camera_button)
        toolbar.addWidget(self.qgc_button)

        layout.addLayout(toolbar)

        self.display_stack = QStackedWidget()
        self.camera_page = CameraDisplayPage()
        self.qgc_page = EmptyDisplayPage()

        self.display_stack.addWidget(self.camera_page)
        self.display_stack.addWidget(self.qgc_page)

        self.camera_button.clicked.connect(
            lambda: self.display_stack.setCurrentWidget(self.camera_page)
        )
        self.qgc_button.clicked.connect(
            lambda: self.display_stack.setCurrentWidget(self.qgc_page)
        )

        layout.addWidget(self.display_stack, 1)
        return frame

    def _setup_camera(self):
        if self.camera_source == "ros":
            self._setup_ros_camera_subscription()
            return

        self.video_worker = UdpVideoWorker(
            port=self.video_port,
            width=self.video_width,
            height=self.video_height,
            parent=self,
        )
        self.video_worker.frame_ready.connect(
            self.camera_page.set_rgb_bytes
        )
        self.video_worker.status_changed.connect(
            self.camera_page.show_status
        )
        self.video_worker.start()

    def _setup_ros_camera_subscription(self):
        """Qt 이벤트 루프 안에서 ROS 2 카메라 토픽을 논블로킹 처리한다."""
        global rclpy, Image, ROS_IMPORT_ERROR

        try:
            import rclpy as rclpy_module
            from sensor_msgs.msg import Image as ImageMessage
        except ImportError as exc:
            ROS_IMPORT_ERROR = exc
        else:
            rclpy = rclpy_module
            Image = ImageMessage

        if ROS_IMPORT_ERROR is not None:
            self.camera_page.show_status(
                "ROS 2 카메라 모듈을 불러올 수 없습니다.\n"
                "ROS 환경을 source한 뒤 실행해 주세요.\n"
                f"({ROS_IMPORT_ERROR})"
            )
            return

        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                self.owns_ros_context = True

            self.ros_node = rclpy.create_node("arecada_gcs_camera")
            self.camera_subscription = self.ros_node.create_subscription(
                Image,
                self.camera_topic,
                self._camera_image_callback,
                1,
            )

            self.ros_spin_timer = QTimer(self)
            self.ros_spin_timer.timeout.connect(self._spin_ros_once)
            self.ros_spin_timer.start(15)

            self.camera_page.show_status(
                f"카메라 영상 대기 중...\n{self.camera_topic}"
            )
        except Exception as exc:
            self.camera_page.show_status(
                "카메라 토픽 연결에 실패했습니다.\n"
                f"{exc}"
            )

    def _spin_ros_once(self):
        if self.ros_node is None:
            return

        try:
            rclpy.spin_once(self.ros_node, timeout_sec=0.0)
        except Exception as exc:
            if self.ros_spin_timer is not None:
                self.ros_spin_timer.stop()
            self.camera_page.show_status(f"ROS 통신 오류\n{exc}")

    def _camera_image_callback(self, message):
        try:
            encoding = message.encoding.lower()
            frame_data = bytes(message.data)

            if encoding == "rgb8":
                image = QImage(
                    frame_data,
                    message.width,
                    message.height,
                    message.step,
                    QImage.Format_RGB888,
                ).copy()
            elif encoding == "bgr8":
                # tcam_siyi_yolo.py의 detection_image 인코딩
                image = QImage(
                    frame_data,
                    message.width,
                    message.height,
                    message.step,
                    QImage.Format_RGB888,
                ).rgbSwapped()
            elif encoding == "mono8":
                image = QImage(
                    frame_data,
                    message.width,
                    message.height,
                    message.step,
                    QImage.Format_Grayscale8,
                ).copy()
            else:
                raise ValueError(f"지원하지 않는 인코딩: {encoding}")

            self.camera_page.set_qimage(image)
        except Exception as exc:
            self.camera_page.show_status(f"영상 변환 오류\n{exc}")

    def _create_telemetry_panel(self):
        layout = QHBoxLayout()
        layout.setSpacing(10)

        for title in ["GPS", "TELEM", "THROTTLE", "ALTITUDE", "AIRSPEED"]:
            card = QFrame()
            card.setObjectName("telemetryCard")
            card.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

            card_layout = QVBoxLayout(card)
            card_layout.setContentsMargins(10, 12, 10, 12)

            label = QLabel(title)
            label.setObjectName("telemetryTitle")
            label.setAlignment(Qt.AlignCenter)

            blank = QLabel("")
            blank.setObjectName("telemetryBlank")
            blank.setAlignment(Qt.AlignCenter)
            blank.setMinimumHeight(28)

            card_layout.addWidget(label)
            card_layout.addWidget(blank)

            layout.addWidget(card)

        return layout

    def _apply_style(self):
        self.setStyleSheet(f"""
            * {{
                font-family: "DejaVu Sans", "Noto Sans CJK KR", sans-serif;
                color: {TEXT};
            }}

            QMainWindow,
            QWidget {{
                background-color: {BG};
            }}

            QFrame#leftPanel,
            QFrame#displayFrame {{
                background-color: {PANEL};
                border: 1px solid {BORDER};
                border-radius: 14px;
            }}

            QLabel#sectionTitle {{
                color: {ACCENT};
                font-size: 12px;
                font-weight: 800;
                letter-spacing: 1px;
                padding-top: 4px;
            }}

            QPushButton#largeButton {{
                min-height: 48px;
                background-color: {PANEL_DARK};
                border: 2px solid {ACCENT};
                border-radius: 10px;
                color: {TEXT};
                font-size: 18px;
                font-weight: 800;
            }}

            QPushButton#largeButton:hover {{
                background-color: #1c2d3a;
            }}

            QPushButton#largeButton:pressed {{
                background-color: #253c4d;
            }}

            QPushButton#controlButton {{
                min-height: 38px;
                background-color: #23303d;
                border: 1px solid {BORDER};
                border-radius: 8px;
                color: {TEXT};
                font-size: 14px;
                font-weight: 700;
                text-align: left;
                padding-left: 14px;
            }}

            QPushButton#controlButton:hover {{
                border: 1px solid {ACCENT};
                background-color: #2a3c4b;
            }}

            QPushButton#controlButton:pressed {{
                background-color: {ACCENT_DARK};
            }}

            QPushButton#viewButton {{
                min-width: 100px;
                min-height: 34px;
                background-color: {PANEL_DARK};
                border: 1px solid {BORDER};
                border-radius: 7px;
                color: {MUTED};
                font-weight: 700;
            }}

            QPushButton#viewButton:checked {{
                background-color: {ACCENT_DARK};
                border: 1px solid {ACCENT};
                color: white;
            }}

            QFrame#emptyDisplay {{
                background-color: #070b10;
                border: 1px solid {BORDER};
                border-radius: 10px;
            }}

            QFrame#telemetryCard {{
                background-color: {PANEL};
                border: 1px solid {BORDER};
                border-radius: 10px;
            }}

            QLabel#telemetryTitle {{
                color: {MUTED};
                font-size: 12px;
                font-weight: 800;
            }}

            QLabel#telemetryBlank {{
                background-color: {PANEL_DARK};
                border: 1px solid #2d4153;
                border-radius: 5px;
            }}

            QLabel#cameraView {{
                background-color: transparent;
                color: {MUTED};
                font-size: 16px;
                font-weight: 700;
            }}
        """)

    def closeEvent(self, event):
        if self.video_worker is not None:
            self.video_worker.stop()
            self.video_worker.wait(1500)

        if self.ros_spin_timer is not None:
            self.ros_spin_timer.stop()

        if self.ros_node is not None:
            self.ros_node.destroy_node()
            self.ros_node = None

        if (
            self.owns_ros_context
            and rclpy is not None
            and rclpy.ok()
        ):
            rclpy.shutdown()

        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("GCS")

    window = GroundStationMockup(
        camera_source=os.environ.get(
            "GCS_CAMERA_SOURCE",
            "ros",
        ).lower(),
        camera_topic=os.environ.get(
            "GCS_CAMERA_TOPIC",
            "/detection_image",
        ),
        video_port=int(os.environ.get("GCS_VIDEO_PORT", "5600")),
        video_width=int(os.environ.get("GCS_VIDEO_WIDTH", "1280")),
        video_height=int(os.environ.get("GCS_VIDEO_HEIGHT", "720")),
    )
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
