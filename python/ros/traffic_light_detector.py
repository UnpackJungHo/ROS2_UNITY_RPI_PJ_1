#!/usr/bin/env python3
"""
Traffic Light Detector - YOLOv8n 기반 신호등 색상 인지 ROS2 노드

=== 동작 모드 ===
1. 웹캠 직접 모드 (기본): cv2.VideoCapture(0)으로 직접 캡처
2. ROS 토픽 모드 (선택): /camera/image_raw 구독

=== 추론 파이프라인 ===
1. 프레임 캡처 (웹캠 or ROS 토픽)
2. YOLOv8n 추론 → 신호등 탐지 + 색상 분류
3. 가장 높은 confidence의 클래스 추출
4. 시간적 안정화 (3프레임 디바운싱)
5. /traffic_light/state 발행: "red", "yellow", "green", "none"
6. /traffic_light/debug 발행: 바운딩 박스 + 라벨 디버그 이미지

=== 발행 토픽 ===
- /traffic_light/state (std_msgs/String): 현재 신호등 상태
- /traffic_light/debug (sensor_msgs/Image): 디버그 시각화

=== 파라미터 (rqt_reconfigure) ===
- confidence_threshold: YOLO 탐지 임계값 (기본 0.5)
- camera_device: 웹캠 디바이스 번호 (기본 0)
- use_ros_camera: ROS 토픽 모드 사용 여부 (기본 False)
- model_path: YOLO 모델 경로 (기본 best.pt)

=== 실행 ===
RPi5에서:
  python3 traffic_light_detector.py
  python3 traffic_light_detector.py --model best_ncnn_model
  python3 traffic_light_detector.py --use_ros_camera

Author: KJH
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, FloatingPointRange, SetParametersResult
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from collections import deque
import argparse


# 클래스 이름 매핑 (Roboflow 데이터셋 순서: 0=Green, 1=Red, 2=Yellow)
CLASS_NAMES = {0: 'green', 1: 'red', 2: 'yellow'}


class TrafficLightDetector(Node):
    def __init__(self, model_path='best.pt', use_ros_camera=False, camera_device=0):
        super().__init__('traffic_light_detector')

        # CV Bridge
        self.bridge = CvBridge()

        # ========== 파라미터 선언 ==========
        self.declare_parameter('confidence_threshold', 0.5,
            ParameterDescriptor(
                description='YOLO 탐지 신뢰도 임계값',
                floating_point_range=[FloatingPointRange(
                    from_value=0.1, to_value=1.0, step=0.05
                )]
            ))

        self.declare_parameter('camera_device', camera_device,
            ParameterDescriptor(
                description='웹캠 디바이스 번호',
                integer_range=[IntegerRange(from_value=0, to_value=10, step=1)]
            ))

        self.declare_parameter('use_ros_camera', use_ros_camera,
            ParameterDescriptor(description='ROS 카메라 토픽 사용 여부'))

        self.declare_parameter('model_path', model_path,
            ParameterDescriptor(description='YOLO 모델 경로 (.pt 또는 NCNN 폴더)'))

        # ========== YOLO 모델 로드 ==========
        actual_model_path = self.get_parameter('model_path').value
        self.get_logger().info(f'Loading YOLO model: {actual_model_path}')
        self.model = YOLO(actual_model_path)
        self.get_logger().info('YOLO model loaded successfully')

        # ========== Publishers ==========
        self.state_pub = self.create_publisher(String, '/traffic_light/state', 10)
        self.debug_pub = self.create_publisher(Image, '/traffic_light/debug', 10)

        # ========== 안정화를 위한 이력 저장 (디바운싱) ==========
        self.state_history = deque(maxlen=3)  # 최근 3프레임 기록
        self.current_state = 'none'
        self.prev_state = 'none'

        # ========== 웹캠 or ROS 카메라 설정 ==========
        use_ros = self.get_parameter('use_ros_camera').value
        if not use_ros:
            # 웹캠 직접 모드
            dev = self.get_parameter('camera_device').value
            self.cap = cv2.VideoCapture(dev)
            if not self.cap.isOpened():
                self.get_logger().error(f'Failed to open camera device {dev}')
                return
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.get_logger().info(f'Webcam mode: device {dev}')

            # 10Hz 타이머
            self.timer = self.create_timer(0.1, self.timer_callback)
        else:
            # ROS 토픽 모드
            self.cap = None
            self.image_sub = self.create_subscription(
                Image, '/camera/image_raw', self.image_callback, 10
            )
            self.get_logger().info('ROS camera mode: subscribing to /camera/image_raw')

        # ========== 런타임 파라미터 변경 콜백 ==========
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info('=== Traffic Light Detector Started ===')

    def parameter_callback(self, params):
        """런타임 파라미터 변경 콜백 (rqt_reconfigure 지원)"""
        for param in params:
            self.get_logger().info(f'Parameter changed: {param.name} = {param.value}')
        return SetParametersResult(successful=True)

    def timer_callback(self):
        """웹캠 모드: 타이머 콜백으로 프레임 캡처 및 추론"""
        if self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
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
        results = self.model.predict(frame, conf=conf_threshold, verbose=False)

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

        # ========== 3. 시간적 안정화 (3프레임 디바운싱) ==========
        self.state_history.append(detected_state)
        stable_state = self.get_stable_state()

        # ========== 4. 상태 변경 로깅 ==========
        if stable_state != self.current_state:
            self.get_logger().info(
                f'State changed: {self.current_state} -> {stable_state}'
            )
            self.prev_state = self.current_state
            self.current_state = stable_state

        # ========== 5. /traffic_light/state 발행 ==========
        state_msg = String()
        state_msg.data = self.current_state
        self.state_pub.publish(state_msg)

        # ========== 6. /traffic_light/debug 발행 ==========
        debug_frame = self.draw_debug(frame, detected_state, best_conf, best_box)
        self.publish_image(self.debug_pub, debug_frame)

    def get_stable_state(self) -> str:
        """최근 3프레임 중 과반수 상태 반환 (디바운싱)"""
        if len(self.state_history) < 3:
            return self.current_state

        # 최빈값 계산
        states = list(self.state_history)
        state_counts = {}
        for s in states:
            state_counts[s] = state_counts.get(s, 0) + 1

        # 가장 많은 상태 반환
        majority_state = max(state_counts, key=state_counts.get)
        majority_count = state_counts[majority_state]

        # 과반수(2/3 이상)일 때만 상태 변경
        if majority_count >= 2:
            return majority_state

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
        cv2.rectangle(debug, (5, 5), (280, 60), (40, 40, 40), -1)
        state_color = color_map.get(self.current_state, (128, 128, 128))
        cv2.putText(debug, f"State: {self.current_state.upper()}", (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)
        cv2.putText(debug, f"Conf: {confidence:.2f}", (15, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

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
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    # CLI 인자 파싱
    parser = argparse.ArgumentParser(description='Traffic Light Detector ROS2 Node')
    parser.add_argument('--model', type=str, default='best.pt',
                        help='YOLO 모델 경로 (.pt 또는 NCNN 폴더)')
    parser.add_argument('--use_ros_camera', action='store_true',
                        help='ROS 카메라 토픽 모드 사용')
    parser.add_argument('--camera', type=int, default=0,
                        help='웹캠 디바이스 번호')
    cli_args = parser.parse_args()

    rclpy.init(args=args)
    node = TrafficLightDetector(
        model_path=cli_args.model,
        use_ros_camera=cli_args.use_ros_camera,
        camera_device=cli_args.camera,
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
