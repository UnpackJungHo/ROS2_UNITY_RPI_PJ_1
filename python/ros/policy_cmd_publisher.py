#!/home/kjhz/miniconda3/envs/driving/bin/python
"""
External policy publisher for Unity vehicle topic actuation.

Pipeline:
1) /camera/policy/image_raw + /odom -> regression(base) ONNX inference
2) (raw/topic/hybrid) collision input -> warning level/TTC evaluation
3) observation vector -> residual RL ONNX inference
4) final action(base + residual) -> /vehicle/cmd (geometry_msgs/Twist)

Run example:
  /home/kjhz/miniconda3/envs/driving/bin/python python/ros/policy_cmd_publisher.py
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray, String


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def clamp(value: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(vmax, value))


def resolve_topic(namespace_prefix: str, topic_name: str) -> str:
    topic = topic_name if topic_name.startswith("/") else f"/{topic_name}"
    ns = (namespace_prefix or "").strip()
    if not ns:
        return topic
    ns = ns if ns.startswith("/") else f"/{ns}"
    ns = ns.rstrip("/")
    if topic == ns or topic.startswith(f"{ns}/"):
        return topic
    return f"{ns}{topic}"


def to_inf_if_negative(value: float) -> float:
    if value < 0.0 or math.isnan(value):
        return math.inf
    return value


@dataclass
class TrafficPerception:
    state: str = "none"
    stable_ratio: float = 0.0
    bbox_center_x: float = -1.0
    bbox_area: float = 0.0


@dataclass
class StopLinePerception:
    stable_detected: bool = False
    confidence: float = 0.0
    distance_norm: float = -1.0


@dataclass
class CollisionState:
    min_distance: float = math.inf
    ttc: float = math.inf
    warning_level: int = 0
    ego_speed: float = 0.0
    closing_speed: float = 0.0
    source_id: int = 0
    closest_ultrasonic_id: int = 0
    closest_radar_id: int = 0
    ultrasonic_confidence: float = 0.0


@dataclass
class UltrasonicSample:
    distance: float = math.inf
    confidence: float = 0.0
    stamp_sec: float = -1e9


@dataclass
class RadarSample:
    distance: float = math.inf
    radial_velocity: float = 0.0
    stamp_sec: float = -1e9


ULTRASONIC_ORDER: Tuple[str, ...] = ("fl", "fr", "fc", "rl", "rr", "rc")
RADAR_ORDER: Tuple[str, ...] = ("front", "rear")
ULTRASONIC_ID_MAP: Dict[str, int] = {
    "fl": 0,
    "fr": 1,
    "rl": 2,
    "rr": 3,
    "fc": 4,
    "rc": 5,
}
ULTRASONIC_REAR_IDS = {2, 3, 5}
ULTRASONIC_SIDE_IDS = {0, 1, 2, 3}
ULTRASONIC_LEFT_IDS = {0, 2}
ULTRASONIC_RIGHT_IDS = {1, 3}


class PolicyCmdPublisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        node_name = "policy_cmd_publisher"
        if args.namespace:
            ns_name = args.namespace.strip("/").replace("/", "_")
            node_name = f"{node_name}_{ns_name}" if ns_name else node_name
        super().__init__(node_name)

        self.args = args
        self.bridge = CvBridge()

        self.regression_session = ort.InferenceSession(
            str(args.regression_model),
            providers=["CPUExecutionProvider"],
        )
        self.rl_session = ort.InferenceSession(
            str(args.rl_model),
            providers=["CPUExecutionProvider"],
        )

        self.reg_front_input_name, self.reg_speed_input_name = self._resolve_regression_inputs()
        self.reg_output_name = self.regression_session.get_outputs()[0].name
        self.rl_input_name = self.rl_session.get_inputs()[0].name
        self.rl_action_output_name = self._resolve_rl_action_output_name()

        self.current_image_rgb: Optional[np.ndarray] = None
        self.current_speed_mps: float = 0.0
        self.have_image: bool = False
        self.have_odom: bool = False

        self.collision_warning_level: int = 0
        self.current_ttc: float = math.inf
        self.collision_state = CollisionState()
        self.collision_selected_from_raw: bool = False
        self.topic_collision_state = CollisionState()
        self.ultrasonic_samples: Dict[str, UltrasonicSample] = {
            key: UltrasonicSample() for key in ULTRASONIC_ORDER
        }
        self.radar_samples: Dict[str, RadarSample] = {
            "front": RadarSample(),
            "rear": RadarSample(),
        }
        self.topic_ultrasonic_distances: Dict[str, float] = {
            key: math.inf for key in ULTRASONIC_ORDER
        }
        self.topic_radar_distances: Dict[str, float] = {
            key: math.inf for key in RADAR_ORDER
        }
        self.raw_prev_min_distance: float = math.inf
        self.raw_prev_min_stamp_sec: float = -1e9
        self.raw_smoothed_closing_speed: float = 0.0

        self.traffic = TrafficPerception()
        self.stop_line = StopLinePerception()

        self.obs_lateral_error_abs: float = 0.0
        self.obs_signed_lateral_error: float = 0.0
        self.obs_heading_error_deg: float = 0.0
        self.obs_progress_ratio: float = 0.0

        self.last_cmd_steer: float = 0.0
        self.last_cmd_throttle: float = 0.0
        self.last_cmd_brake: float = 0.0
        self.last_log_time_sec: float = 0.0

        self.front_mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        self.front_std = np.array([0.229, 0.224, 0.225], dtype=np.float32)

        self.reg_speed_normalize = args.reg_speed_normalize
        self._try_load_regression_meta_speed_norm()

        image_topic = resolve_topic(args.namespace, args.camera_topic)
        odom_topic = resolve_topic(args.namespace, args.odom_topic)
        cmd_topic = resolve_topic(args.namespace, args.cmd_topic)
        collision_topic = resolve_topic(args.namespace, args.collision_warning_topic)
        collision_ui_topic = resolve_topic(args.namespace, args.collision_ui_topic)
        ultrasonic_topics = {
            "fl": resolve_topic(args.namespace, args.ultrasonic_fl_topic),
            "fr": resolve_topic(args.namespace, args.ultrasonic_fr_topic),
            "fc": resolve_topic(args.namespace, args.ultrasonic_fc_topic),
            "rl": resolve_topic(args.namespace, args.ultrasonic_rl_topic),
            "rr": resolve_topic(args.namespace, args.ultrasonic_rr_topic),
            "rc": resolve_topic(args.namespace, args.ultrasonic_rc_topic),
        }
        radar_topics = {
            "front": resolve_topic(args.namespace, args.radar_front_topic),
            "rear": resolve_topic(args.namespace, args.radar_rear_topic),
        }
        traffic_state_topic = resolve_topic(args.namespace, args.traffic_light_state_topic)
        traffic_perception_topic = resolve_topic(args.namespace, args.traffic_light_perception_topic)
        stop_line_perception_topic = resolve_topic(args.namespace, args.stop_line_perception_topic)

        self.create_subscription(Image, image_topic, self._on_image, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        if args.collision_mode in ("topic", "hybrid"):
            self.create_subscription(Float32MultiArray, collision_topic, self._on_collision_warning, 10)
        if args.collision_mode in ("raw", "hybrid"):
            for key, topic in ultrasonic_topics.items():
                self.create_subscription(
                    Float32MultiArray,
                    topic,
                    lambda msg, sensor_key=key: self._on_ultrasonic_raw(sensor_key, msg),
                    10,
                )
            for key, topic in radar_topics.items():
                self.create_subscription(
                    Float32MultiArray,
                    topic,
                    lambda msg, radar_key=key: self._on_radar_raw(radar_key, msg),
                    10,
                )
        self.create_subscription(String, traffic_state_topic, self._on_traffic_state, 10)
        self.create_subscription(Float32MultiArray, traffic_perception_topic, self._on_traffic_perception, 10)
        self.create_subscription(Float32MultiArray, stop_line_perception_topic, self._on_stop_line_perception, 10)

        if args.lateral_error_topic:
            self.create_subscription(
                Float32,
                resolve_topic(args.namespace, args.lateral_error_topic),
                lambda msg: self._set_obs_value("lateral_abs", msg.data),
                10,
            )
        if args.signed_lateral_error_topic:
            self.create_subscription(
                Float32,
                resolve_topic(args.namespace, args.signed_lateral_error_topic),
                lambda msg: self._set_obs_value("lateral_signed", msg.data),
                10,
            )
        if args.heading_error_topic:
            self.create_subscription(
                Float32,
                resolve_topic(args.namespace, args.heading_error_topic),
                lambda msg: self._set_obs_value("heading_deg", msg.data),
                10,
            )
        if args.progress_ratio_topic:
            self.create_subscription(
                Float32,
                resolve_topic(args.namespace, args.progress_ratio_topic),
                lambda msg: self._set_obs_value("progress_ratio", msg.data),
                10,
            )

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 20)
        self.collision_ui_pub = self.create_publisher(Float32MultiArray, collision_ui_topic, 20)

        timer_period = 1.0 / max(1.0, float(args.control_hz))
        self.create_timer(timer_period, self._control_loop)

        self.get_logger().info(
            "PolicyCmdPublisher started | "
            f"ns={args.namespace or '(none)'} | image={image_topic} odom={odom_topic} cmd={cmd_topic} | "
            f"collision_mode={args.collision_mode} | "
            f"reg={args.regression_model} rl={args.rl_model} | "
            f"reg_speed_norm={self.reg_speed_normalize:.3f}"
        )
        if args.collision_mode in ("raw", "hybrid"):
            self.get_logger().info(
                "Raw collision topics | "
                f"ultra={ultrasonic_topics} radar={radar_topics} timeout={args.sensor_timeout_sec:.2f}s"
            )
        if args.collision_mode in ("topic", "hybrid"):
            self.get_logger().info(f"Collision warning topic | {collision_topic}")
        self.get_logger().info(f"Collision UI topic | {collision_ui_topic}")

    def _resolve_regression_inputs(self) -> Tuple[str, str]:
        input_names = [x.name for x in self.regression_session.get_inputs()]
        if "front_image" in input_names and "speed" in input_names:
            return "front_image", "speed"
        if len(input_names) != 2:
            raise RuntimeError(
                f"Regression ONNX input mismatch: expected 2 inputs(front_image,speed), got {input_names}"
            )
        return input_names[0], input_names[1]

    def _resolve_rl_action_output_name(self) -> str:
        outputs = [o.name for o in self.rl_session.get_outputs()]
        mode = (self.args.rl_action_mode or "deterministic").lower()

        if mode == "deterministic":
            if "deterministic_continuous_actions" in outputs:
                return "deterministic_continuous_actions"
            raise RuntimeError(
                "rl_action_mode=deterministic 이지만 ONNX에 deterministic_continuous_actions가 없습니다. "
                f"available={outputs}"
            )

        if mode == "stochastic":
            if "continuous_actions" in outputs:
                return "continuous_actions"
            raise RuntimeError(
                "rl_action_mode=stochastic 이지만 ONNX에 continuous_actions가 없습니다. "
                f"available={outputs}"
            )

        raise RuntimeError(f"Unknown rl_action_mode={self.args.rl_action_mode}")

    def _try_load_regression_meta_speed_norm(self) -> None:
        if self.args.reg_speed_norm_from_meta is False:
            return
        reg_path = Path(self.args.regression_model)
        meta_path = reg_path.with_name(f"{reg_path.stem}_meta.json")
        if not meta_path.exists():
            return
        try:
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
            speed_norm = float(meta.get("speed_normalize", self.reg_speed_normalize))
            if speed_norm > 0.0:
                self.reg_speed_normalize = speed_norm
        except Exception as exc:
            self.get_logger().warning(f"Failed to load regression meta ({meta_path}): {exc}")

    def _set_obs_value(self, key: str, value: float) -> None:
        v = float(value)
        if not math.isfinite(v):
            return
        if key == "lateral_abs":
            self.obs_lateral_error_abs = abs(v)
        elif key == "lateral_signed":
            self.obs_signed_lateral_error = v
        elif key == "heading_deg":
            self.obs_heading_error_deg = v
        elif key == "progress_ratio":
            self.obs_progress_ratio = clamp(v, 0.0, 1.0)

    def _on_image(self, msg: Image) -> None:
        try:
            image_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as exc:
            self.get_logger().warning(f"Image conversion failed: {exc}")
            return
        self.current_image_rgb = image_rgb
        self.have_image = True

    def _on_odom(self, msg: Odometry) -> None:
        try:
            vx = float(msg.twist.twist.linear.x)
            vy = float(msg.twist.twist.linear.y)
            vz = float(msg.twist.twist.linear.z)
        except Exception:
            return

        if self.args.use_speed_magnitude:
            speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        else:
            speed = vx
        if math.isfinite(speed):
            self.current_speed_mps = speed
            self.have_odom = True

    def _on_collision_warning(self, msg: Float32MultiArray) -> None:
        data = msg.data
        if data is None or len(data) < 3:
            return

        cs = self.topic_collision_state
        cs.min_distance = to_inf_if_negative(float(data[0])) if len(data) > 0 else math.inf
        cs.ttc = to_inf_if_negative(float(data[1])) if len(data) > 1 else math.inf
        cs.warning_level = int(clamp(int(round(float(data[2]))), 0, 6)) if len(data) > 2 else 0
        cs.ego_speed = float(data[3]) if len(data) > 3 and math.isfinite(float(data[3])) else 0.0
        cs.closing_speed = float(data[4]) if len(data) > 4 and math.isfinite(float(data[4])) else 0.0
        cs.source_id = int(round(float(data[13]))) if len(data) > 13 else 0
        cs.closest_ultrasonic_id = int(round(float(data[14]))) if len(data) > 14 else 0
        cs.closest_radar_id = int(round(float(data[15]))) if len(data) > 15 else 0
        cs.ultrasonic_confidence = clamp(float(data[16]), 0.0, 1.0) if len(data) > 16 else 0.0

        self.topic_ultrasonic_distances["fl"] = to_inf_if_negative(float(data[5])) if len(data) > 5 else math.inf
        self.topic_ultrasonic_distances["fr"] = to_inf_if_negative(float(data[6])) if len(data) > 6 else math.inf
        self.topic_ultrasonic_distances["fc"] = to_inf_if_negative(float(data[7])) if len(data) > 7 else math.inf
        self.topic_ultrasonic_distances["rl"] = to_inf_if_negative(float(data[8])) if len(data) > 8 else math.inf
        self.topic_ultrasonic_distances["rr"] = to_inf_if_negative(float(data[9])) if len(data) > 9 else math.inf
        self.topic_ultrasonic_distances["rc"] = to_inf_if_negative(float(data[10])) if len(data) > 10 else math.inf
        self.topic_radar_distances["front"] = to_inf_if_negative(float(data[11])) if len(data) > 11 else math.inf
        self.topic_radar_distances["rear"] = to_inf_if_negative(float(data[12])) if len(data) > 12 else math.inf

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _is_fresh(self, stamp_sec: float, now_sec: float) -> bool:
        return (now_sec - stamp_sec) <= max(0.05, self.args.sensor_timeout_sec)

    def _refresh_collision_state(self) -> None:
        mode = self.args.collision_mode
        if mode == "topic":
            selected = self.topic_collision_state
            selected_from_raw = False
        elif mode == "raw":
            selected, _ = self._compute_raw_collision_state()
            selected_from_raw = True
        else:
            raw_state, raw_ready = self._compute_raw_collision_state()
            selected = raw_state if raw_ready else self.topic_collision_state
            selected_from_raw = raw_ready

        self.collision_state = CollisionState(
            min_distance=selected.min_distance,
            ttc=selected.ttc,
            warning_level=selected.warning_level,
            ego_speed=selected.ego_speed,
            closing_speed=selected.closing_speed,
            source_id=selected.source_id,
            closest_ultrasonic_id=selected.closest_ultrasonic_id,
            closest_radar_id=selected.closest_radar_id,
            ultrasonic_confidence=selected.ultrasonic_confidence,
        )
        self.collision_selected_from_raw = selected_from_raw
        self.current_ttc = self.collision_state.ttc
        self.collision_warning_level = self.collision_state.warning_level

    def _on_ultrasonic_raw(self, key: str, msg: Float32MultiArray) -> None:
        sample = self.ultrasonic_samples.get(key)
        if sample is None:
            return
        now_sec = self._now_sec()
        data = msg.data if msg is not None else None
        if data is None or len(data) == 0:
            sample.distance = math.inf
            sample.confidence = 0.0
            sample.stamp_sec = now_sec
            return

        distance = to_inf_if_negative(float(data[0]))
        confidence = clamp(float(data[1]), 0.0, 1.0) if len(data) > 1 and math.isfinite(float(data[1])) else 1.0

        sample.distance = distance if math.isfinite(distance) and distance > 0.0 else math.inf
        sample.confidence = confidence
        sample.stamp_sec = now_sec

    def _on_radar_raw(self, key: str, msg: Float32MultiArray) -> None:
        sample = self.radar_samples.get(key)
        if sample is None:
            return
        now_sec = self._now_sec()
        distance, radial_velocity = self._parse_radar_closest(msg.data if msg is not None else None)
        sample.distance = distance
        sample.radial_velocity = radial_velocity
        sample.stamp_sec = now_sec

    def _parse_radar_closest(self, data: Optional[List[float]]) -> Tuple[float, float]:
        if data is None or len(data) == 0:
            return math.inf, 0.0

        candidates: List[Tuple[float, float]] = []

        if len(data) >= 8 and len(data) % 8 == 0:
            with_sensor_id = True
            for i in range(0, len(data), 8):
                sensor_id = float(data[i])
                if not math.isfinite(sensor_id) or sensor_id < -0.5 or sensor_id > 1.5:
                    with_sensor_id = False
                    break
            if with_sensor_id:
                for i in range(0, len(data), 8):
                    dist = to_inf_if_negative(float(data[i + 1]))
                    rv = float(data[i + 3]) if math.isfinite(float(data[i + 3])) else 0.0
                    if math.isfinite(dist) and dist > 0.0:
                        candidates.append((dist, rv))

        if not candidates and len(data) >= 7 and len(data) % 7 == 0:
            for i in range(0, len(data), 7):
                dist = to_inf_if_negative(float(data[i]))
                rv = float(data[i + 2]) if math.isfinite(float(data[i + 2])) else 0.0
                if math.isfinite(dist) and dist > 0.0:
                    candidates.append((dist, rv))

        if not candidates:
            dist = to_inf_if_negative(float(data[0]))
            rv = float(data[2]) if len(data) > 2 and math.isfinite(float(data[2])) else 0.0
            if math.isfinite(dist) and dist > 0.0:
                candidates.append((dist, rv))

        if not candidates:
            return math.inf, 0.0

        best_distance, best_rv = min(candidates, key=lambda x: x[0])
        return best_distance, best_rv

    def _estimate_closing_speed(self, min_distance: float, now_sec: float) -> float:
        if not math.isfinite(min_distance):
            self.raw_smoothed_closing_speed = 0.0
            self.raw_prev_min_distance = min_distance
            self.raw_prev_min_stamp_sec = now_sec
            return 0.0

        if not math.isfinite(self.raw_prev_min_distance):
            self.raw_prev_min_distance = min_distance
            self.raw_prev_min_stamp_sec = now_sec
            return self.raw_smoothed_closing_speed

        dt = now_sec - self.raw_prev_min_stamp_sec
        if dt <= 1e-3:
            self.raw_prev_min_distance = min_distance
            self.raw_prev_min_stamp_sec = now_sec
            return self.raw_smoothed_closing_speed

        raw_closing_speed = (self.raw_prev_min_distance - min_distance) / dt
        alpha = clamp(self.args.collision_closing_speed_smooth_factor, 0.05, 1.0)
        self.raw_smoothed_closing_speed = (
            (1.0 - alpha) * self.raw_smoothed_closing_speed + alpha * raw_closing_speed
        )
        if abs(self.raw_smoothed_closing_speed) < self.args.collision_min_valid_closing_speed:
            self.raw_smoothed_closing_speed = 0.0

        self.raw_prev_min_distance = min_distance
        self.raw_prev_min_stamp_sec = now_sec
        return self.raw_smoothed_closing_speed

    def _compute_ttc_value(self, min_distance: float, closing_speed: float, ego_speed_abs: float) -> float:
        if not math.isfinite(min_distance):
            return math.inf
        if closing_speed > self.args.collision_min_valid_closing_speed:
            return min_distance / max(1e-4, closing_speed)
        if ego_speed_abs > 0.01 and min_distance < self.args.collision_low_speed_caution_distance:
            return min_distance / max(1e-4, ego_speed_abs)
        return math.inf

    def _evaluate_ttc_warning(self, ttc: float) -> int:
        if not math.isfinite(ttc) or ttc < 0.0:
            return 0
        if ttc <= self.args.collision_ttc_brake:
            return 5
        if ttc <= self.args.collision_ttc_warning:
            return 4
        if ttc <= self.args.collision_ttc_slowdown:
            return 3
        if ttc <= self.args.collision_ttc_caution:
            return 2
        if ttc <= self.args.collision_ttc_awareness:
            return 1
        return 0

    def _evaluate_low_speed_warning(self, ultrasonic_min: float, radar_min: float) -> int:
        min_distance = min(ultrasonic_min, radar_min)
        if not math.isfinite(min_distance):
            return 0
        if min_distance <= self.args.collision_low_speed_warning_distance:
            return 4
        if min_distance <= self.args.collision_low_speed_caution_distance:
            return 2
        return 0

    @staticmethod
    def _compute_path_min_distance(
        ultra_dist: Dict[str, float],
        radar_front_dist: float,
    ) -> float:
        forward_core = min(ultra_dist["fc"], radar_front_dist)
        if not math.isfinite(forward_core):
            forward_core = min(ultra_dist["fl"], ultra_dist["fr"], ultra_dist["fc"])
        return forward_core

    @staticmethod
    def _compute_side_min_distance(ultra_dist: Dict[str, float]) -> float:
        left_min = min(ultra_dist["fl"], ultra_dist["rl"])
        right_min = min(ultra_dist["fr"], ultra_dist["rr"])
        return min(left_min, right_min)

    def _is_turning_toward_closest_side(self, closest_ultrasonic_id: int) -> bool:
        steering_angle_deg = -self.last_cmd_steer * self.args.collision_steering_full_angle_deg
        threshold = max(0.0, self.args.collision_side_turn_steering_angle_threshold_deg)
        turning_left = steering_angle_deg > threshold
        turning_right = steering_angle_deg < -threshold
        closest_is_left = closest_ultrasonic_id in ULTRASONIC_LEFT_IDS
        closest_is_right = closest_ultrasonic_id in ULTRASONIC_RIGHT_IDS
        return (closest_is_left and turning_left) or (closest_is_right and turning_right)

    def _compute_raw_collision_state(self) -> Tuple[CollisionState, bool]:
        now_sec = self._now_sec()

        ultra_dist: Dict[str, float] = {}
        ultra_conf: Dict[str, float] = {}
        for key in ULTRASONIC_ORDER:
            sample = self.ultrasonic_samples[key]
            if self._is_fresh(sample.stamp_sec, now_sec):
                ultra_dist[key] = sample.distance if sample.distance > 0.0 else math.inf
                ultra_conf[key] = sample.confidence
            else:
                ultra_dist[key] = math.inf
                ultra_conf[key] = 0.0

        radar_front = self.radar_samples["front"]
        radar_rear = self.radar_samples["rear"]
        front_dist = radar_front.distance if self._is_fresh(radar_front.stamp_sec, now_sec) else math.inf
        rear_dist = radar_rear.distance if self._is_fresh(radar_rear.stamp_sec, now_sec) else math.inf

        ultrasonic_min_key = min(ULTRASONIC_ORDER, key=lambda key: ultra_dist[key])
        ultrasonic_min = ultra_dist[ultrasonic_min_key]
        ultrasonic_confidence = ultra_conf[ultrasonic_min_key] if math.isfinite(ultrasonic_min) else 0.0
        radar_min = min(front_dist, rear_dist)
        radar_closest_id = 0 if front_dist <= rear_dist else 1

        has_raw_sensor = (
            any(math.isfinite(ultra_dist[key]) for key in ULTRASONIC_ORDER)
            or math.isfinite(front_dist)
            or math.isfinite(rear_dist)
        )
        if not has_raw_sensor:
            return CollisionState(), False

        path_min = self._compute_path_min_distance(ultra_dist, front_dist)
        side_min = self._compute_side_min_distance(ultra_dist)
        global_min = min(ultrasonic_min, radar_min)
        use_path_aware = bool(self.args.collision_enable_path_aware_risk_split)
        min_distance = path_min if (use_path_aware and math.isfinite(path_min)) else global_min

        ego_speed = abs(self.current_speed_mps)
        closing_speed = self._estimate_closing_speed(min_distance, now_sec)
        ttc = self._compute_ttc_value(min_distance, closing_speed, ego_speed)
        ttc_level = self._evaluate_ttc_warning(ttc)

        low_speed_level = 0
        if ego_speed <= self.args.collision_low_speed_threshold:
            low_speed_primary = min_distance if use_path_aware else ultrasonic_min
            low_speed_level = self._evaluate_low_speed_warning(low_speed_primary, radar_min)

        warning_level = max(ttc_level, low_speed_level)
        source_id = 0

        if (
            math.isfinite(ultrasonic_min)
            and ultrasonic_min <= self.args.collision_emergency_stop_distance
            and ultrasonic_confidence >= self.args.collision_min_emergency_confidence
        ):
            closest_ultrasonic_id = ULTRASONIC_ID_MAP.get(ultrasonic_min_key, 0)
            is_rear_closest = closest_ultrasonic_id in ULTRASONIC_REAR_IDS
            is_side_closest = closest_ultrasonic_id in ULTRASONIC_SIDE_IDS
            rear_relevant = (
                (not use_path_aware)
                or (not self.args.collision_suppress_rear_emergency_when_not_reversing)
                or (not is_rear_closest)
                or (ego_speed < -0.05)
            )
            allow_side_emergency = True
            if use_path_aware and is_side_closest:
                allow_side_emergency = ultrasonic_min <= max(0.01, self.args.collision_side_emergency_hard_distance)

            if rear_relevant and allow_side_emergency:
                return (
                    CollisionState(
                        min_distance=min_distance,
                        ttc=ttc,
                        warning_level=6,
                        ego_speed=ego_speed,
                        closing_speed=closing_speed,
                        source_id=1,
                        closest_ultrasonic_id=closest_ultrasonic_id,
                        closest_radar_id=radar_closest_id,
                        ultrasonic_confidence=ultrasonic_confidence,
                    ),
                    True,
                )

        if ttc_level >= low_speed_level:
            warning_level = ttc_level
            if math.isfinite(radar_min) and radar_min <= ultrasonic_min:
                source_id = 2
            elif math.isfinite(ultrasonic_min):
                source_id = 1
        else:
            warning_level = low_speed_level
            if ultrasonic_min <= radar_min:
                source_id = 1
            elif math.isfinite(radar_min):
                source_id = 2

        if (
            use_path_aware
            and side_min <= self.args.collision_side_turn_brake_distance
            and self._is_turning_toward_closest_side(ULTRASONIC_ID_MAP.get(ultrasonic_min_key, 0))
            and warning_level < 4
        ):
            warning_level = 4
            source_id = 1

        if ego_speed <= 0.01 and math.isfinite(radar_min) and radar_min <= self.args.collision_radar_min_safe_distance:
            if warning_level < 4:
                warning_level = 4
                source_id = 2

        if source_id == 0:
            if math.isfinite(ultrasonic_min) and (not math.isfinite(radar_min) or ultrasonic_min <= radar_min):
                source_id = 1
            elif math.isfinite(radar_min):
                source_id = 2

        return (
            CollisionState(
                min_distance=min_distance,
                ttc=ttc,
                warning_level=int(clamp(warning_level, 0, 6)),
                ego_speed=ego_speed,
                closing_speed=closing_speed,
                source_id=source_id,
                closest_ultrasonic_id=ULTRASONIC_ID_MAP.get(ultrasonic_min_key, 0),
                closest_radar_id=radar_closest_id,
                ultrasonic_confidence=ultrasonic_confidence,
            ),
            True,
        )

    @staticmethod
    def _encode_distance(value: float) -> float:
        return -1.0 if (not math.isfinite(value) or value < 0.0) else float(value)

    def _read_raw_distances_for_publish(self) -> Tuple[Dict[str, float], Dict[str, float]]:
        now_sec = self._now_sec()

        ultra: Dict[str, float] = {}
        for key in ULTRASONIC_ORDER:
            sample = self.ultrasonic_samples[key]
            if self._is_fresh(sample.stamp_sec, now_sec) and math.isfinite(sample.distance) and sample.distance > 0.0:
                ultra[key] = sample.distance
            else:
                ultra[key] = math.inf

        radar: Dict[str, float] = {}
        for key in RADAR_ORDER:
            sample = self.radar_samples[key]
            if self._is_fresh(sample.stamp_sec, now_sec) and math.isfinite(sample.distance) and sample.distance > 0.0:
                radar[key] = sample.distance
            else:
                radar[key] = math.inf

        return ultra, radar

    def _build_collision_warning_data(self) -> List[float]:
        if self.collision_selected_from_raw:
            ultra_dist, radar_dist = self._read_raw_distances_for_publish()
        else:
            ultra_dist = self.topic_ultrasonic_distances
            radar_dist = self.topic_radar_distances

        cs = self.collision_state
        return [
            self._encode_distance(cs.min_distance),                    # 0
            self._encode_distance(cs.ttc),                             # 1
            float(int(clamp(cs.warning_level, 0, 6))),                 # 2
            float(cs.ego_speed if math.isfinite(cs.ego_speed) else 0), # 3
            float(cs.closing_speed if math.isfinite(cs.closing_speed) else 0), # 4
            self._encode_distance(ultra_dist.get("fl", math.inf)),     # 5
            self._encode_distance(ultra_dist.get("fr", math.inf)),     # 6
            self._encode_distance(ultra_dist.get("fc", math.inf)),     # 7
            self._encode_distance(ultra_dist.get("rl", math.inf)),     # 8
            self._encode_distance(ultra_dist.get("rr", math.inf)),     # 9
            self._encode_distance(ultra_dist.get("rc", math.inf)),     # 10
            self._encode_distance(radar_dist.get("front", math.inf)),  # 11
            self._encode_distance(radar_dist.get("rear", math.inf)),   # 12
            float(cs.source_id),                                       # 13
            float(cs.closest_ultrasonic_id),                           # 14
            float(cs.closest_radar_id),                                # 15
            float(clamp(cs.ultrasonic_confidence, 0.0, 1.0)),          # 16
        ]

    def _publish_collision_warning(self) -> None:
        msg = Float32MultiArray()
        msg.data = self._build_collision_warning_data()
        self.collision_ui_pub.publish(msg)

    def _on_traffic_state(self, msg: String) -> None:
        state = (msg.data or "none").strip().lower()
        if state not in ("red", "yellow", "green", "none"):
            state = "none"
        self.traffic.state = state

    def _on_traffic_perception(self, msg: Float32MultiArray) -> None:
        data = msg.data
        if data is None or len(data) < 7:
            return
        self.traffic.stable_ratio = clamp(float(data[1]), 0.0, 1.0)
        self.traffic.bbox_center_x = float(data[4])
        self.traffic.bbox_area = max(0.0, float(data[6]))

    def _on_stop_line_perception(self, msg: Float32MultiArray) -> None:
        data = msg.data
        if data is None or len(data) < 10:
            return
        self.stop_line.stable_detected = float(data[1]) > 0.5
        self.stop_line.confidence = clamp(float(data[2]), 0.0, 1.0)
        self.stop_line.distance_norm = float(data[8])

    def _preprocess_front_image(self, image_rgb: np.ndarray) -> np.ndarray:
        resized = cv2.resize(
            image_rgb,
            (self.args.front_width, self.args.front_height),
            interpolation=cv2.INTER_LINEAR,
        )
        img = resized.astype(np.float32) / 255.0
        img = (img - self.front_mean[None, None, :]) / self.front_std[None, None, :]
        chw = np.transpose(img, (2, 0, 1))
        return chw[np.newaxis, ...].astype(np.float32)

    def _run_regression(self) -> Tuple[float, float]:
        if self.current_image_rgb is None:
            return 0.0, 0.0

        front = self._preprocess_front_image(self.current_image_rgb)
        speed_norm = self.current_speed_mps / max(1e-4, self.reg_speed_normalize)
        speed = np.array([[speed_norm]], dtype=np.float32)

        output = self.regression_session.run(
            [self.reg_output_name],
            {
                self.reg_front_input_name: front,
                self.reg_speed_input_name: speed,
            },
        )[0]
        steer = clamp(float(output[0][0]), -1.0, 1.0)
        throttle = clamp(float(output[0][1]), 0.0, 1.0)
        return steer, throttle

    def _normalize_signed(self, value: float, denom: float) -> float:
        if not math.isfinite(value):
            return 0.0
        return clamp(value / max(1e-4, denom), -1.0, 1.0)

    def _normalize_01(self, value: float, max_range: float, default_invalid: float) -> float:
        if not math.isfinite(value) or value < 0.0:
            return default_invalid
        return clamp(value / max(1e-4, max_range), 0.0, 1.0)

    def _compute_stop_line_distance_m(self) -> float:
        if not self.stop_line.stable_detected:
            return math.inf
        if self.stop_line.confidence < self.args.min_stop_line_confidence:
            return math.inf
        norm = self.stop_line.distance_norm
        if norm < 0.0 or not math.isfinite(norm):
            return math.inf

        near_norm = min(self.args.stop_line_near_norm, self.args.stop_line_far_norm - 1e-3)
        far_norm = max(self.args.stop_line_far_norm, near_norm + 1e-3)
        t = clamp((norm - near_norm) / (far_norm - near_norm), 0.0, 1.0)
        near_m = max(0.1, self.args.stop_line_near_distance_m)
        far_m = max(near_m, self.args.stop_line_far_distance_m)
        dist = near_m + (far_m - near_m) * t
        return clamp(dist, near_m, max(near_m, self.args.max_stop_line_distance_m))

    def _collision_source_name(self) -> str:
        source = self.collision_state.source_id
        if source == 1:
            return "Ultrasonic"
        if source == 2:
            return "Radar"
        return "None"

    def _closest_sensor_name(self) -> str:
        source = self.collision_state.source_id
        if source == 1:
            mapping = {
                0: "FrontLeft",
                1: "FrontRight",
                2: "RearLeft",
                3: "RearRight",
                4: "FrontCenter",
                5: "RearCenter",
            }
            return mapping.get(self.collision_state.closest_ultrasonic_id, "Unknown")
        if source == 2:
            return "Front" if self.collision_state.closest_radar_id == 0 else "Rear"
        return "None"

    def _compute_stopping_distance(self, speed: float) -> float:
        decel = max(0.1, self.args.comfortable_deceleration)
        return (speed * self.args.reaction_time) + ((speed * speed) / (2.0 * decel))

    def _set_stop_recommendation(self, speed: float, dist: float) -> Tuple[float, float]:
        rec_throttle_scale = 0.0
        if not math.isfinite(dist) or dist <= 0.0:
            return rec_throttle_scale, self.args.max_stop_brake

        required_decel = (speed * speed) / max(0.1, 2.0 * dist)
        rec_brake = clamp(required_decel / 3.0, 0.0, 1.0)
        rec_brake = max(rec_brake, 0.35)
        rec_brake = min(rec_brake, self.args.max_stop_brake)
        return rec_throttle_scale, rec_brake

    def _compute_traffic_decision(self, speed: float, stop_line_distance: float) -> Tuple[int, float, float]:
        state = self.traffic.state
        confidence_ok = self.traffic.stable_ratio >= self.args.min_state_confidence
        area_ok = self.traffic.bbox_area >= self.args.min_bbox_area
        lane_ok = (
            self.traffic.bbox_center_x >= 0.0
            and abs(self.traffic.bbox_center_x - self.args.lane_center_x) <= self.args.max_lane_center_offset
        )
        is_relevant = (state != "none") and confidence_ok and area_ok and lane_ok
        if not is_relevant:
            return 0, 1.0, 0.0

        stopping_distance = self._compute_stopping_distance(speed)
        can_stop = stopping_distance + self.args.stop_distance_buffer <= stop_line_distance

        if state == "red":
            if math.isfinite(stop_line_distance) and stop_line_distance <= self.args.max_red_reaction_distance:
                rec_th, rec_brake = self._set_stop_recommendation(speed, stop_line_distance)
                return 2, rec_th, rec_brake
            return 0, 1.0, 0.0

        if state == "yellow":
            if (not math.isfinite(stop_line_distance)) or stop_line_distance > self.args.max_yellow_reaction_distance:
                return 0, 1.0, 0.0
            if can_stop:
                rec_th, rec_brake = self._set_stop_recommendation(speed, stop_line_distance)
                return 2, rec_th, rec_brake
            if self.args.caution_on_unstoppable_yellow:
                return 1, self.args.caution_throttle_scale_traffic, self.args.caution_brake_traffic
            return 0, 1.0, 0.0

        return 0, 1.0, 0.0

    def _build_rl_observation(
        self,
        base_steer: float,
        base_throttle: float,
        stop_line_distance: float,
        traffic_decision: int,
    ) -> np.ndarray:
        obs = np.zeros((1, 16), dtype=np.float32)

        obs[0, 0] = self._normalize_signed(self.current_speed_mps, self.args.speed_normalize)
        obs[0, 1] = self._normalize_01(self.obs_lateral_error_abs, self.args.lateral_error_normalize, 0.0)
        obs[0, 2] = self._normalize_signed(self.obs_signed_lateral_error, self.args.lateral_error_normalize)
        obs[0, 3] = self._normalize_signed(self.obs_heading_error_deg, self.args.heading_error_normalize_deg)
        obs[0, 4] = clamp(self.obs_progress_ratio, 0.0, 1.0)
        obs[0, 5] = self._normalize_01(self.current_ttc, self.args.ttc_normalize_seconds, 1.0)
        obs[0, 6] = clamp(self.collision_warning_level / 6.0, 0.0, 1.0)
        obs[0, 7] = self._normalize_01(stop_line_distance, self.args.stop_line_distance_normalize, 1.0)

        obs[0, 8] = clamp(self.last_cmd_steer, -1.0, 1.0)
        obs[0, 9] = clamp(self.last_cmd_throttle, -1.0 if self.args.allow_reverse else 0.0, 1.0)
        obs[0, 10] = clamp(self.last_cmd_brake, 0.0, 1.0)

        obs[0, 11] = clamp(base_steer, -1.0, 1.0)
        obs[0, 12] = clamp(base_throttle, 0.0, 1.0)

        obs[0, 13] = 1.0 if traffic_decision == 0 else 0.0
        obs[0, 14] = 1.0 if traffic_decision == 1 else 0.0
        obs[0, 15] = 1.0 if traffic_decision == 2 else 0.0

        return obs

    def _run_rl_residual(self, observation: np.ndarray) -> Tuple[float, float]:
        actions = self.rl_session.run(
            [self.rl_action_output_name],
            {self.rl_input_name: observation},
        )[0]
        raw_delta_steer = clamp(float(actions[0][0]), -1.0, 1.0)
        raw_delta_accel = clamp(float(actions[0][1]), -1.0, 1.0)
        delta_steer = raw_delta_steer * self.args.residual_steer_scale
        delta_accel = raw_delta_accel * self.args.residual_accel_scale
        return delta_steer, delta_accel

    def _apply_safety_override(
        self,
        throttle: float,
        brake: float,
        speed: float,
        traffic_decision: int,
        rec_throttle_scale: float,
        rec_brake: float,
    ) -> Tuple[float, float]:
        if not self.args.enable_safety_override:
            return throttle, brake

        level = self.collision_warning_level
        if level >= 6:
            throttle = 0.0
            brake = max(brake, self.args.emergency_brake)
        elif level >= 5:
            throttle = 0.0
            brake = max(brake, self.args.brake_level_brake)
        elif level >= 4:
            throttle = min(throttle, 0.15)
            if abs(speed) >= max(0.0, self.args.warning_brake_min_speed):
                brake = max(brake, self.args.warning_brake)
        elif level >= 3:
            throttle = min(throttle, self.args.caution_throttle_scale_collision)

        if traffic_decision == 2:
            throttle = 0.0
            brake = max(brake, rec_brake)
        elif traffic_decision == 1:
            throttle = min(throttle, rec_throttle_scale)
            brake = max(brake, rec_brake)

        return throttle, brake

    def _action_to_twist(self, steer: float, throttle: float, brake: float) -> Tuple[float, float]:
        safe_forward = max(0.01, self.args.cmd_max_forward_speed)
        safe_reverse = max(0.01, self.args.cmd_max_reverse_speed)
        safe_yaw = max(0.01, self.args.cmd_max_yaw_rate)

        if self.args.allow_reverse and throttle < 0.0:
            linear_x = -clamp(-throttle, 0.0, 1.0) * safe_reverse
        elif brake > 0.001:
            linear_x = -clamp(brake, 0.0, 1.0) * safe_reverse
        else:
            linear_x = clamp(throttle, 0.0, 1.0) * safe_forward

        angular_z = clamp(steer, -1.0, 1.0) * safe_yaw
        return linear_x, angular_z

    def _publish_cmd(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def _control_loop(self) -> None:
        if not self.have_image or not self.have_odom:
            self._refresh_collision_state()
            self._publish_collision_warning()
            if self.args.publish_zero_when_not_ready:
                self._publish_cmd(0.0, 0.0)
            return

        try:
            self._refresh_collision_state()
            self._publish_collision_warning()
            base_steer, base_throttle = self._run_regression()
            stop_line_distance = self._compute_stop_line_distance_m()
            if not math.isfinite(stop_line_distance) and self.args.use_collision_distance_for_stopline_proxy:
                stop_line_distance = self.collision_state.min_distance
            speed_non_negative = max(0.0, abs(self.current_speed_mps))
            traffic_decision, rec_throttle_scale, rec_brake = self._compute_traffic_decision(
                speed_non_negative,
                stop_line_distance,
            )

            obs = self._build_rl_observation(
                base_steer=base_steer,
                base_throttle=base_throttle,
                stop_line_distance=stop_line_distance,
                traffic_decision=traffic_decision,
            )
            delta_steer, delta_accel = self._run_rl_residual(obs)

            final_steer = clamp(base_steer + delta_steer, -1.0, 1.0)
            accel_min = -1.0 if self.args.allow_reverse else 0.0
            final_accel = clamp(base_throttle + delta_accel, accel_min, 1.0)

            if self.args.allow_reverse:
                throttle = final_accel
                brake = 0.0
            else:
                if final_accel >= 0.0:
                    throttle = final_accel
                    brake = 0.0
                else:
                    throttle = 0.0
                    brake = -final_accel

            throttle, brake = self._apply_safety_override(
                throttle=throttle,
                brake=brake,
                speed=self.current_speed_mps,
                traffic_decision=traffic_decision,
                rec_throttle_scale=rec_throttle_scale,
                rec_brake=rec_brake,
            )

            throttle = clamp(throttle, -1.0 if self.args.allow_reverse else 0.0, 1.0)
            brake = clamp(brake, 0.0, 1.0)

            linear_x, angular_z = self._action_to_twist(final_steer, throttle, brake)
            self._publish_cmd(linear_x, angular_z)

            self.last_cmd_steer = final_steer
            self.last_cmd_throttle = throttle
            self.last_cmd_brake = brake

            now = self.get_clock().now().nanoseconds * 1e-9
            if self.args.log_interval_sec > 0.0 and (now - self.last_log_time_sec) >= self.args.log_interval_sec:
                self.last_log_time_sec = now
                closest_dist_str = (
                    "inf"
                    if not math.isfinite(self.collision_state.min_distance)
                    else f"{self.collision_state.min_distance:.2f}m"
                )
                self.get_logger().info(
                    f"base=({base_steer:+.3f},{base_throttle:+.3f}) "
                    f"delta=({delta_steer:+.3f},{delta_accel:+.3f}) "
                    f"final=({final_steer:+.3f},th={throttle:+.3f},br={brake:+.3f}) "
                    f"cmd=(vx={linear_x:+.3f},wz={angular_z:+.3f}) "
                    f"warn={self.collision_warning_level} "
                    f"ttc={'inf' if not math.isfinite(self.current_ttc) else f'{self.current_ttc:.2f}'} "
                    f"closest={self._collision_source_name()}-{self._closest_sensor_name()} "
                    f"dist={closest_dist_str}"
                )
        except Exception as exc:
            self.get_logger().error(f"control loop failed: {exc}")
            self._publish_cmd(0.0, 0.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish regression+residual policy cmd to /vehicle/cmd")

    parser.add_argument("--namespace", type=str, default="/amr0")
    parser.add_argument("--camera-topic", type=str, default="/camera/policy/image_raw")
    parser.add_argument("--odom-topic", type=str, default="/odom")
    parser.add_argument("--cmd-topic", type=str, default="/vehicle/cmd")
    parser.add_argument("--collision-mode", type=str, choices=["raw", "topic", "hybrid"], default="raw")
    parser.add_argument("--collision-warning-topic", type=str, default="/collision_warning")
    parser.add_argument("--collision-ui-topic", type=str, default="/policy/collision_warning")
    parser.add_argument("--ultrasonic-fl-topic", type=str, default="/ultrasonic/fl")
    parser.add_argument("--ultrasonic-fr-topic", type=str, default="/ultrasonic/fr")
    parser.add_argument("--ultrasonic-fc-topic", type=str, default="/ultrasonic/fc")
    parser.add_argument("--ultrasonic-rl-topic", type=str, default="/ultrasonic/rl")
    parser.add_argument("--ultrasonic-rr-topic", type=str, default="/ultrasonic/rr")
    parser.add_argument("--ultrasonic-rc-topic", type=str, default="/ultrasonic/rc")
    parser.add_argument("--radar-front-topic", type=str, default="/radar/front")
    parser.add_argument("--radar-rear-topic", type=str, default="/radar/rear")
    parser.add_argument("--sensor-timeout-sec", type=float, default=0.5)
    parser.add_argument("--traffic-light-state-topic", type=str, default="/traffic_light/state")
    parser.add_argument("--traffic-light-perception-topic", type=str, default="/traffic_light/perception")
    parser.add_argument("--stop-line-perception-topic", type=str, default="/stop_line/perception")
    parser.add_argument("--lateral-error-topic", type=str, default="")
    parser.add_argument("--signed-lateral-error-topic", type=str, default="")
    parser.add_argument("--heading-error-topic", type=str, default="")
    parser.add_argument("--progress-ratio-topic", type=str, default="")

    parser.add_argument(
        "--regression-model",
        type=Path,
        default=PROJECT_ROOT / "Assets" / "Models" / "ONNX" / "driving_regression.onnx",
    )
    parser.add_argument(
        "--rl-model",
        type=Path,
        default=PROJECT_ROOT / "results" / "autodriver_manual_001" / "AutoDriver.onnx",
    )
    parser.add_argument("--front-width", type=int, default=200)
    parser.add_argument("--front-height", type=int, default=66)
    parser.add_argument("--reg-speed-normalize", type=float, default=3.0)
    parser.add_argument("--reg-speed-norm-from-meta", action=argparse.BooleanOptionalAction, default=True)

    parser.add_argument("--control-hz", type=float, default=20.0)
    parser.add_argument("--residual-steer-scale", type=float, default=0.16)
    parser.add_argument("--residual-accel-scale", type=float, default=0.12)
    parser.add_argument("--rl-action-mode", type=str, choices=["deterministic", "stochastic"], default="deterministic")
    parser.add_argument("--allow-reverse", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--use-speed-magnitude", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--publish-zero-when-not-ready", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--log-interval-sec", type=float, default=1.0)

    parser.add_argument("--cmd-max-forward-speed", type=float, default=2.0)
    parser.add_argument("--cmd-max-reverse-speed", type=float, default=1.0)
    parser.add_argument("--cmd-max-yaw-rate", type=float, default=1.2)

    parser.add_argument("--enable-safety-override", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--caution-throttle-scale-collision", type=float, default=0.55)
    parser.add_argument("--warning-brake", type=float, default=0.15)
    parser.add_argument("--warning-brake-min-speed", type=float, default=0.6)
    parser.add_argument("--brake-level-brake", type=float, default=0.8)
    parser.add_argument("--emergency-brake", type=float, default=1.0)
    parser.add_argument("--collision-emergency-stop-distance", type=float, default=0.3)
    parser.add_argument("--collision-min-emergency-confidence", type=float, default=0.55)
    parser.add_argument("--collision-low-speed-threshold", type=float, default=0.5)
    parser.add_argument("--collision-low-speed-caution-distance", type=float, default=1.5)
    parser.add_argument("--collision-low-speed-warning-distance", type=float, default=0.8)
    parser.add_argument("--collision-radar-min-safe-distance", type=float, default=0.5)
    parser.add_argument("--collision-min-valid-closing-speed", type=float, default=0.05)
    parser.add_argument("--collision-closing-speed-smooth-factor", type=float, default=0.3)
    parser.add_argument("--collision-ttc-awareness", type=float, default=5.0)
    parser.add_argument("--collision-ttc-caution", type=float, default=3.0)
    parser.add_argument("--collision-ttc-slowdown", type=float, default=2.0)
    parser.add_argument("--collision-ttc-warning", type=float, default=1.5)
    parser.add_argument("--collision-ttc-brake", type=float, default=1.0)
    parser.add_argument("--collision-enable-path-aware-risk-split", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--collision-side-advisory-distance", type=float, default=0.45)
    parser.add_argument("--collision-side-turn-brake-distance", type=float, default=0.25)
    parser.add_argument("--collision-side-emergency-hard-distance", type=float, default=0.12)
    parser.add_argument("--collision-side-turn-steering-angle-threshold-deg", type=float, default=7.0)
    parser.add_argument("--collision-steering-full-angle-deg", type=float, default=30.0)
    parser.add_argument(
        "--collision-suppress-rear-emergency-when-not-reversing",
        action=argparse.BooleanOptionalAction,
        default=True,
    )

    parser.add_argument("--min-state-confidence", type=float, default=0.35)
    parser.add_argument("--min-bbox-area", type=float, default=0.0012)
    parser.add_argument("--lane-center-x", type=float, default=0.5)
    parser.add_argument("--max-lane-center-offset", type=float, default=0.22)

    parser.add_argument("--min-stop-line-confidence", type=float, default=0.45)
    parser.add_argument("--stop-line-near-distance-m", type=float, default=2.0)
    parser.add_argument("--stop-line-far-distance-m", type=float, default=18.0)
    parser.add_argument("--stop-line-near-norm", type=float, default=0.05)
    parser.add_argument("--stop-line-far-norm", type=float, default=0.70)
    parser.add_argument("--max-stop-line-distance-m", type=float, default=30.0)

    parser.add_argument("--max-red-reaction-distance", type=float, default=12.0)
    parser.add_argument("--max-yellow-reaction-distance", type=float, default=18.0)
    parser.add_argument("--stop-distance-buffer", type=float, default=0.8)
    parser.add_argument("--reaction-time", type=float, default=0.6)
    parser.add_argument("--comfortable-deceleration", type=float, default=1.2)
    parser.add_argument("--caution-on-unstoppable-yellow", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--caution-throttle-scale-traffic", type=float, default=0.35)
    parser.add_argument("--caution-brake-traffic", type=float, default=0.25)
    parser.add_argument("--max-stop-brake", type=float, default=1.0)

    parser.add_argument("--speed-normalize", type=float, default=3.0)
    parser.add_argument("--lateral-error-normalize", type=float, default=2.0)
    parser.add_argument("--heading-error-normalize-deg", type=float, default=45.0)
    parser.add_argument("--ttc-normalize-seconds", type=float, default=8.0)
    parser.add_argument("--stop-line-distance-normalize", type=float, default=30.0)
    parser.add_argument("--use-collision-distance-for-stopline-proxy", action=argparse.BooleanOptionalAction, default=True)

    args = parser.parse_args()
    args.regression_model = args.regression_model.resolve()
    args.rl_model = args.rl_model.resolve()
    return args


def main() -> None:
    args = parse_args()

    if not args.regression_model.exists():
        raise FileNotFoundError(f"Regression model not found: {args.regression_model}")
    if not args.rl_model.exists():
        raise FileNotFoundError(f"RL model not found: {args.rl_model}")

    rclpy.init()
    node: Optional[PolicyCmdPublisher] = None
    try:
        node = PolicyCmdPublisher(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node._publish_cmd(0.0, 0.0)
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
