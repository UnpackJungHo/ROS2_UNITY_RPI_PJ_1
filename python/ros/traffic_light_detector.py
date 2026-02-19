#!/home/kjhz/miniconda3/envs/driving/bin/python
"""
Traffic Light Detector - YOLOv8n 기반 신호등 색상 인지 ROS2 노드

=== 실행 모드 ===
1. --simulation (기본): Unity CameraPublisher(/camera/image_raw) 구독 → PC에서 best.pt 추론
2. --real:              웹캠 직접 캡처 → RPi5에서 NCNN 추론

=== 발행 토픽 ===
- /traffic_light/state (std_msgs/String): 현재 신호등 상태
- /traffic_light/debug (sensor_msgs/Image): 디버그 시각화

=== 실행 ===
시뮬레이션 모드 (PC):
  python3 traffic_light_detector.py
  python3 traffic_light_detector.py --simulation
  python3 traffic_light_detector.py --simulation --model custom.pt

실제 웹캠 모드 (RPi5):
  python3 traffic_light_detector.py --real
  python3 traffic_light_detector.py --real --model best_ncnn_model

Author: KJH
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, FloatingPointRange, SetParametersResult
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import argparse
from pathlib import Path
from collections import deque, Counter
import threading


# 클래스 이름 매핑 (Roboflow 데이터셋 순서: 0=Green, 1=Red, 2=Yellow)
CLASS_NAMES = {0: 'green', 1: 'red', 2: 'yellow'}
STATE_TO_ID = {'none': 0, 'red': 1, 'yellow': 2, 'green': 3}


class CameraThread:
    """카메라 캡처 전용 스레드 - 추론과 분리하여 프레임 끊김 방지 (real 모드 전용)"""

    def __init__(self, device=0, width=640, height=480):
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.frame = None
        self.lock = threading.Lock()
        self.running = False

    def start(self):
        if not self.cap.isOpened():
            return False
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        return True

    def _capture_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.frame = frame

    def get_frame(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        if self.cap.isOpened():
            self.cap.release()


class TrafficLightDetector(Node):
    # ========== 슬라이딩 윈도우 디바운싱 상수 ==========
    WINDOW_SIZE = 15          # 최근 15프레임 기록 (30fps 기준 0.5초)
    THRESHOLD_COLOR = 0.4     # none → 색상: 40% 이상이면 전환 (빠른 감지)
    THRESHOLD_CHANGE = 0.5    # 색상 → 다른 색상: 50% 이상 (신중한 전환)
    THRESHOLD_NONE = 0.8      # 색상 → none: 80% 이상 (깜빡임 방지)

    def __init__(self, mode='simulation', model_path=None, camera_device=0, namespace=''):
        node_name = 'traffic_light_detector' if not namespace else f'traffic_light_detector_{namespace.strip("/").replace("/","_")}'
        super().__init__(node_name)
        self.ns = namespace.rstrip('/') if namespace else ''

        self.mode = mode
        self.bridge = CvBridge()

        # ========== 모드별 기본값 ==========
        if model_path is None:
            if mode == 'simulation':
                # PC: best.pt (GPU 추론)
                model_path = str(Path(__file__).parent / 'best.pt')
            else:
                # RPi5: NCNN 모델
                model_path = str(Path(__file__).parent.parent / 'best_ncnn_model')

        # simulation: 640 (PC GPU 여유), real: 480 (RPi5 속도/정확도 균형)
        self.infer_imgsz = 640 if mode == 'simulation' else 480

        # ========== 파라미터 선언 ==========
        self.declare_parameter('confidence_threshold', 0.15,
            ParameterDescriptor(
                description='YOLO 탐지 신뢰도 임계값',
                floating_point_range=[FloatingPointRange(
                    from_value=0.1, to_value=1.0, step=0.05
                )]
            ))

        self.declare_parameter('camera_device', camera_device,
            ParameterDescriptor(
                description='웹캠 디바이스 번호 (real 모드)',
                integer_range=[IntegerRange(from_value=0, to_value=10, step=1)]
            ))

        self.declare_parameter('model_path', model_path,
            ParameterDescriptor(description='YOLO 모델 경로 (.pt 또는 NCNN 폴더)'))

        # ========== YOLO 모델 로드 ==========
        actual_model_path = self.get_parameter('model_path').value
        self.get_logger().info(f'[{mode.upper()}] Loading YOLO model: {actual_model_path}')
        self.model = YOLO(actual_model_path, task='detect')
        self.get_logger().info('YOLO model loaded successfully')

        # ========== Publishers ==========
        self.state_pub = self.create_publisher(String, f'{self.ns}/traffic_light/state', 10)
        self.perception_pub = self.create_publisher(Float32MultiArray, f'{self.ns}/traffic_light/perception', 10)
        self.debug_pub = self.create_publisher(Image, f'{self.ns}/traffic_light/debug', 10)

        # ========== 슬라이딩 윈도우 디바운싱 상태 ==========
        self.current_state = 'none'
        self.history = deque(maxlen=self.WINDOW_SIZE)

        # ========== 모드별 입력 소스 설정 ==========
        self.camera_thread = None

        if mode == 'simulation':
            # Unity CameraPublisher 토픽 구독
            image_topic = f'{self.ns}/camera/image_raw'
            self.image_sub = self.create_subscription(
                Image, image_topic, self.image_callback, 10
            )
            self.get_logger().info(
                f'[Simulation] Subscribing {image_topic} (imgsz={self.infer_imgsz})')
        else:
            # 웹캠 직접 모드 (스레딩)
            dev = self.get_parameter('camera_device').value
            self.camera_thread = CameraThread(device=dev)
            if not self.camera_thread.start():
                self.get_logger().error(f'Failed to open camera device {dev}')
                return
            self.get_logger().info(
                f'[Real] Webcam device {dev} (imgsz={self.infer_imgsz})')
            self.timer = self.create_timer(0.1, self.timer_callback)

        # ========== 런타임 파라미터 변경 콜백 ==========
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info(f'=== Traffic Light Detector Started [{mode.upper()}] ===')

    def parameter_callback(self, params):
        """런타임 파라미터 변경 콜백 (rqt_reconfigure 지원)"""
        for param in params:
            self.get_logger().info(f'Parameter changed: {param.name} = {param.value}')
        return SetParametersResult(successful=True)

    def timer_callback(self):
        """웹캠 모드: 스레드에서 최신 프레임 가져와서 추론"""
        if self.camera_thread is None:
            return

        frame = self.camera_thread.get_frame()
        if frame is None:
            return

        self.process_frame(frame)

    def image_callback(self, msg: Image):
        """ROS 카메라 모드: 이미지 토픽 콜백"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        self.process_frame(frame)

    def process_frame(self, frame: np.ndarray):
        """프레임 추론 및 결과 발행"""
        conf_threshold = self.get_parameter('confidence_threshold').value

        # ========== 1. YOLO 추론 ==========
        results = self.model.predict(frame, conf=conf_threshold, imgsz=self.infer_imgsz, verbose=False)

        # ========== 2. 탐지 결과 분석 ==========
        detected_state = 'none'
        best_conf = 0.0
        best_box = None

        if len(results) > 0 and results[0].boxes is not None and len(results[0].boxes) > 0:
            boxes = results[0].boxes
            for i in range(len(boxes)):
                conf = float(boxes.conf[i])
                cls_id = int(boxes.cls[i])

                if cls_id in CLASS_NAMES and conf > best_conf:
                    best_conf = conf
                    detected_state = CLASS_NAMES[cls_id]
                    best_box = boxes.xyxy[i].cpu().numpy().astype(int)

        # ========== 3. 비대칭 디바운싱 ==========
        stable_state = self.update_debounce(detected_state)

        # ========== 4. 상태 변경 로깅 ==========
        if stable_state != self.current_state:
            self.get_logger().info(
                f'State changed: {self.current_state} -> {stable_state}'
            )
            self.current_state = stable_state

        # ========== 5. /traffic_light/state 발행 ==========
        state_msg = String()
        state_msg.data = self.current_state
        self.state_pub.publish(state_msg)

        # ========== 5-1. /traffic_light/perception 발행 ==========
        # data[0]: stable_state_id      (0:none, 1:red, 2:yellow, 3:green)
        # data[1]: stable_state_ratio   (윈도우 내 stable 비율 0~1)
        # data[2]: raw_state_id         (YOLO raw state)
        # data[3]: raw_confidence       (YOLO raw confidence 0~1)
        # data[4]: bbox_center_x_norm   (0~1, 없으면 -1)
        # data[5]: bbox_center_y_norm   (0~1, 없으면 -1)
        # data[6]: bbox_area_norm       (0~1, 없으면 0)
        perception_msg = Float32MultiArray()
        stable_ratio = self._state_ratio(self.current_state)
        raw_id = STATE_TO_ID.get(detected_state, 0)
        stable_id = STATE_TO_ID.get(self.current_state, 0)

        if best_box is not None:
            h, w = frame.shape[:2]
            x1, y1, x2, y2 = best_box
            cx = ((x1 + x2) * 0.5) / max(w, 1)
            cy = ((y1 + y2) * 0.5) / max(h, 1)
            area = max(0, x2 - x1) * max(0, y2 - y1)
            area_norm = area / max(w * h, 1)
            cx = float(np.clip(cx, 0.0, 1.0))
            cy = float(np.clip(cy, 0.0, 1.0))
            area_norm = float(np.clip(area_norm, 0.0, 1.0))
        else:
            cx, cy, area_norm = -1.0, -1.0, 0.0

        perception_msg.data = [
            float(stable_id),
            float(stable_ratio),
            float(raw_id),
            float(best_conf),
            float(cx),
            float(cy),
            float(area_norm),
        ]
        self.perception_pub.publish(perception_msg)

        # ========== 6. /traffic_light/debug 발행 ==========
        debug_frame = self.draw_debug(frame, detected_state, best_conf, best_box)
        self.publish_image(self.debug_pub, debug_frame)

    def _state_ratio(self, state: str) -> float:
        """현재 슬라이딩 윈도우에서 특정 상태 비율 반환"""
        if len(self.history) == 0:
            return 0.0
        counts = Counter(self.history)
        return counts.get(state, 0) / len(self.history)

    def update_debounce(self, detected_state: str) -> str:
        """
        슬라이딩 윈도우 디바운싱: 최근 N프레임에서 비율 기반 판단

        - none → 색상:       40% 이상 (빠른 감지, ~6/15프레임)
        - 색상 → 다른 색상:  50% 이상 (신중한 전환, ~8/15프레임)
        - 색상 → none:       80% 이상 (깜빡임 방지, ~12/15프레임)

        1프레임 미스에도 카운트가 리셋되지 않아 안정적
        """
        self.history.append(detected_state)

        if len(self.history) < 3:
            return self.current_state

        # 윈도우 내 각 상태 비율 계산
        counts = Counter(self.history)
        window_len = len(self.history)

        # 현재 상태 유지 비율 확인
        current_ratio = counts.get(self.current_state, 0) / window_len
        if current_ratio >= 0.4:
            return self.current_state

        # 가장 많이 감지된 상태 찾기
        best_state, best_count = counts.most_common(1)[0]
        best_ratio = best_count / window_len

        # 전환 방향에 따른 임계값 결정
        if best_state == 'none':
            threshold = self.THRESHOLD_NONE
        elif self.current_state == 'none':
            threshold = self.THRESHOLD_COLOR
        else:
            threshold = self.THRESHOLD_CHANGE

        if best_ratio >= threshold:
            self.history.clear()
            return best_state

        return self.current_state

    def draw_debug(self, frame: np.ndarray, detected_state: str,
                   confidence: float, box) -> np.ndarray:
        """디버그 이미지 생성: 바운딩 박스 + 라벨"""
        debug = frame.copy()

        # 색상 매핑
        color_map = {
            'red': (0, 0, 255),
            'yellow': (0, 255, 255),
            'green': (0, 255, 0),
            'none': (128, 128, 128),
        }

        # 바운딩 박스 그리기
        if box is not None:
            x1, y1, x2, y2 = box
            color = color_map.get(detected_state, (128, 128, 128))
            cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
            label = f"{detected_state} {confidence:.2f}"
            cv2.putText(debug, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # 상태 정보 패널
        cv2.rectangle(debug, (5, 5), (300, 75), (40, 40, 40), -1)
        state_color = color_map.get(self.current_state, (128, 128, 128))
        cv2.putText(debug, f"State: {self.current_state.upper()}", (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)
        cv2.putText(debug, f"Raw: {detected_state} ({confidence:.2f})", (15, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        # 슬라이딩 윈도우 현황
        counts = Counter(self.history)
        window_str = " ".join(f"{s[0].upper()}:{c}" for s, c in counts.most_common())
        cv2.putText(debug, f"Window[{len(self.history)}]: {window_str}", (15, 67),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

        return debug

    def publish_image(self, publisher, image, encoding='bgr8'):
        """이미지를 토픽으로 발행"""
        try:
            msg = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
            publisher.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Image publish failed: {e}')

    def destroy_node(self):
        """노드 종료 시 리소스 해제"""
        if self.camera_thread is not None:
            self.camera_thread.stop()
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description='Traffic Light Detector ROS2 Node')

    # 모드 선택 (기본: simulation)
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument('--simulation', action='store_true',
                            help='시뮬레이션 모드: Unity 카메라 토픽 구독 (기본)')
    mode_group.add_argument('--real', action='store_true',
                            help='실제 모드: 웹캠 직접 캡처 (RPi5)')

    parser.add_argument('--model', type=str, default=None,
                        help='YOLO 모델 경로 (미지정 시 모드별 기본값 사용)')
    parser.add_argument('--camera', type=int, default=0,
                        help='웹캠 디바이스 번호 (real 모드)')
    parser.add_argument('--namespace', type=str, default='',
                        help='토픽 네임스페이스 prefix (예: /amr0)')
    cli_args = parser.parse_args()

    mode = 'real' if cli_args.real else 'simulation'

    rclpy.init(args=args)
    ns = cli_args.namespace.rstrip('/')
    node = TrafficLightDetector(
        mode=mode,
        model_path=cli_args.model,
        camera_device=cli_args.camera,
        namespace=ns,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
