#!/home/kjhz/miniconda3/envs/driving/bin/python
"""
Stop Line Detector - /camera/image_raw 기반 정지선(횡단보도 시작선) 검출 노드

핵심 아이디어:
1) ROI(하단 사다리꼴)로 탐색 영역 제한
2) 흰색 마킹 마스크(HLS/HSV) 추출
3) 유효 컨투어(횡단보도 stripe) 추출
4) 각 stripe의 "차량에 가장 가까운 엣지 포인트"를 모아 선형 피팅

출력 토픽:
- /stop_line/state (std_msgs/String): "detected" or "none"
- /stop_line/perception (std_msgs/Float32MultiArray)
- /stop_line/debug (sensor_msgs/Image): 디버그 오버레이

perception.data 매핑:
0: raw_detected (0/1)
1: stable_detected (0/1)
2: confidence (0~1)
3: x1_norm (0~1, 없으면 -1)
4: y1_norm (0~1, 없으면 -1)
5: x2_norm (0~1, 없으면 -1)
6: y2_norm (0~1, 없으면 -1)
7: y_center_norm (0~1, 없으면 -1)
8: distance_from_bottom_norm (0~1, 클수록 멀리)
9: stripe_count
"""

from collections import deque
from math import ceil

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import (
    FloatingPointRange,
    IntegerRange,
    ParameterDescriptor,
    SetParametersResult,
)
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String


class StopLineDetector(Node):
    def __init__(self):
        super().__init__("stop_line_detector")

        self.bridge = CvBridge()

        # ===== ROS topics =====
        self.declare_parameter(
            "image_topic",
            "/camera/image_raw",
            ParameterDescriptor(description="입력 카메라 이미지 토픽"),
        )
        self.declare_parameter(
            "state_topic",
            "/stop_line/state",
            ParameterDescriptor(description="정지선 상태 토픽"),
        )
        self.declare_parameter(
            "perception_topic",
            "/stop_line/perception",
            ParameterDescriptor(description="정지선 수치 정보 토픽"),
        )
        self.declare_parameter(
            "debug_topic",
            "/stop_line/debug",
            ParameterDescriptor(description="디버그 이미지 토픽"),
        )

        # ===== ROI (normalized coordinates) =====
        self.declare_parameter(
            "roi_top_y",
            0.52,
            ParameterDescriptor(
                description="ROI 상단 y (0~1)",
                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)],
            ),
        )
        self.declare_parameter(
            "roi_bottom_y",
            0.98,
            ParameterDescriptor(
                description="ROI 하단 y (0~1)",
                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)],
            ),
        )
        self.declare_parameter(
            "roi_top_left_x",
            0.10,
            ParameterDescriptor(
                description="ROI 좌상단 x (0~1)",
                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)],
            ),
        )
        self.declare_parameter(
            "roi_top_right_x",
            0.95,
            ParameterDescriptor(
                description="ROI 우상단 x (0~1)",
                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)],
            ),
        )
        self.declare_parameter(
            "roi_bottom_left_x",
            0.00,
            ParameterDescriptor(
                description="ROI 좌하단 x (0~1)",
                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)],
            ),
        )
        self.declare_parameter(
            "roi_bottom_right_x",
            1.00,
            ParameterDescriptor(
                description="ROI 우하단 x (0~1)",
                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)],
            ),
        )

        # ===== White mask thresholds =====
        self.declare_parameter(
            "white_l_min",
            155,
            ParameterDescriptor(
                description="HLS L 채널 최소값",
                integer_range=[IntegerRange(from_value=0, to_value=255, step=1)],
            ),
        )
        self.declare_parameter(
            "white_s_max",
            90,
            ParameterDescriptor(
                description="HLS S 채널 최대값",
                integer_range=[IntegerRange(from_value=0, to_value=255, step=1)],
            ),
        )
        self.declare_parameter(
            "white_v_min",
            170,
            ParameterDescriptor(
                description="HSV V 채널 최소값",
                integer_range=[IntegerRange(from_value=0, to_value=255, step=1)],
            ),
        )
        self.declare_parameter(
            "morph_kernel",
            5,
            ParameterDescriptor(
                description="모폴로지 커널 크기(홀수 권장)",
                integer_range=[IntegerRange(from_value=1, to_value=15, step=2)],
            ),
        )

        # ===== Contour / fitting =====
        self.declare_parameter(
            "min_contour_area",
            220.0,
            ParameterDescriptor(
                description="유효 stripe 최소 contour 면적",
                floating_point_range=[FloatingPointRange(from_value=10.0, to_value=50000.0, step=1.0)],
            ),
        )
        self.declare_parameter(
            "near_edge_band_px",
            6,
            ParameterDescriptor(
                description="stripe 근접엣지 포인트 추출 band(px)",
                integer_range=[IntegerRange(from_value=1, to_value=40, step=1)],
            ),
        )
        self.declare_parameter(
            "min_stripe_count",
            3,
            ParameterDescriptor(
                description="라인 피팅에 필요한 최소 stripe 수",
                integer_range=[IntegerRange(from_value=2, to_value=20, step=1)],
            ),
        )
        self.declare_parameter(
            "use_birds_eye",
            True,
            ParameterDescriptor(description="원근 보정(BEV) 기반 추정 사용"),
        )
        self.declare_parameter(
            "bev_column_step",
            4,
            ParameterDescriptor(
                description="BEV 하단 경계 탐색 x 스텝(px)",
                integer_range=[IntegerRange(from_value=1, to_value=20, step=1)],
            ),
        )
        self.declare_parameter(
            "bev_min_valid_columns",
            45,
            ParameterDescriptor(
                description="BEV 유효 컬럼 최소 개수",
                integer_range=[IntegerRange(from_value=5, to_value=500, step=1)],
            ),
        )
        self.declare_parameter(
            "bev_min_x_coverage",
            0.35,
            ParameterDescriptor(
                description="BEV 경계점 x coverage 최소 비율(0~1)",
                floating_point_range=[FloatingPointRange(from_value=0.05, to_value=1.0, step=0.01)],
            ),
        )

        # ===== Temporal stabilization =====
        self.declare_parameter(
            "line_history_size",
            6,
            ParameterDescriptor(
                description="라인 평균화 히스토리 길이",
                integer_range=[IntegerRange(from_value=1, to_value=50, step=1)],
            ),
        )
        self.declare_parameter(
            "detect_history_size",
            5,
            ParameterDescriptor(
                description="검출 여부 히스토리 길이",
                integer_range=[IntegerRange(from_value=1, to_value=50, step=1)],
            ),
        )
        self.declare_parameter(
            "stable_detect_ratio",
            0.6,
            ParameterDescriptor(
                description="stable 판정 비율(히스토리 내 검출 비율)",
                floating_point_range=[FloatingPointRange(from_value=0.1, to_value=1.0, step=0.05)],
            ),
        )

        # ===== Debug =====
        self.declare_parameter(
            "log_interval_sec",
            1.0,
            ParameterDescriptor(
                description="상태 로그 주기(초), 0 이하면 비활성",
                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=10.0, step=0.1)],
            ),
        )

        self._reset_histories()

        image_topic = self.get_parameter("image_topic").value
        state_topic = self.get_parameter("state_topic").value
        perception_topic = self.get_parameter("perception_topic").value
        debug_topic = self.get_parameter("debug_topic").value

        self.state_pub = self.create_publisher(String, state_topic, 10)
        self.perception_pub = self.create_publisher(Float32MultiArray, perception_topic, 10)
        self.debug_pub = self.create_publisher(Image, debug_topic, 10)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)

        self.last_log_time = self.get_clock().now()
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info(
            f"[StopLine] Subscribed image={image_topic}, publish state={state_topic}, "
            f"perception={perception_topic}, debug={debug_topic}"
        )

    def _reset_histories(self):
        line_size = int(self.get_parameter("line_history_size").value) if self.has_parameter("line_history_size") else 6
        det_size = int(self.get_parameter("detect_history_size").value) if self.has_parameter("detect_history_size") else 5
        self.line_history = deque(maxlen=max(1, line_size))
        self.detect_history = deque(maxlen=max(1, det_size))

    def parameter_callback(self, params):
        # 히스토리 길이 파라미터가 바뀌면 deque 재생성
        need_reset = any(p.name in ("line_history_size", "detect_history_size") for p in params)
        if need_reset:
            self._reset_histories()
        return SetParametersResult(successful=True)

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"[StopLine] image conversion failed: {exc}")
            return

        result = self.detect_stop_line(frame)

        self.detect_history.append(1 if result["raw_detected"] else 0)
        ratio = float(self.get_parameter("stable_detect_ratio").value)
        min_votes = max(1, ceil(len(self.detect_history) * ratio))
        stable_detected = sum(self.detect_history) >= min_votes

        if result["raw_detected"]:
            self.line_history.append(
                (
                    result["x1_norm"],
                    result["y1_norm"],
                    result["x2_norm"],
                    result["y2_norm"],
                    result["y_center_norm"],
                    result["dist_from_bottom_norm"],
                    result["confidence"],
                    float(result["stripe_count"]),
                )
            )

        stable_line = self._average_line_history() if (stable_detected and len(self.line_history) > 0) else None
        self.publish_outputs(result, stable_detected, stable_line)
        self.publish_debug_image(frame, result, stable_detected, stable_line)
        self.maybe_log(result, stable_detected)

    def _average_line_history(self):
        arr = np.array(self.line_history, dtype=np.float32)
        avg = np.mean(arr, axis=0)
        return {
            "x1_norm": float(avg[0]),
            "y1_norm": float(avg[1]),
            "x2_norm": float(avg[2]),
            "y2_norm": float(avg[3]),
            "y_center_norm": float(avg[4]),
            "dist_from_bottom_norm": float(avg[5]),
            "confidence": float(avg[6]),
            "stripe_count": float(avg[7]),
        }

    def detect_stop_line(self, frame):
        h, w = frame.shape[:2]
        roi_mask, roi_poly = self.build_roi_mask(w, h)

        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        l_channel = hls[:, :, 1]
        s_channel = hls[:, :, 2]
        v_channel = hsv[:, :, 2]

        l_min = int(self.get_parameter("white_l_min").value)
        s_max = int(self.get_parameter("white_s_max").value)
        v_min = int(self.get_parameter("white_v_min").value)

        white_mask = np.logical_and.reduce(
            (
                l_channel >= l_min,
                s_channel <= s_max,
                v_channel >= v_min,
            )
        ).astype(np.uint8) * 255
        white_mask = cv2.bitwise_and(white_mask, roi_mask)

        kernel_size = int(self.get_parameter("morph_kernel").value)
        kernel_size = max(1, kernel_size)
        if kernel_size % 2 == 0:
            kernel_size += 1
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_area = float(self.get_parameter("min_contour_area").value)
        near_band = int(self.get_parameter("near_edge_band_px").value)
        near_band = max(1, near_band)

        near_points = []
        valid_contours = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue

            pts = cnt.reshape(-1, 2)
            max_y = int(np.max(pts[:, 1]))
            band_pts = pts[pts[:, 1] >= (max_y - near_band)]
            if band_pts.shape[0] == 0:
                continue

            x_mean = float(np.mean(band_pts[:, 0]))
            y_mean = float(np.mean(band_pts[:, 1]))
            near_points.append((x_mean, y_mean))
            valid_contours.append(cnt)

        raw_detected = False
        confidence = 0.0
        x1_norm = y1_norm = x2_norm = y2_norm = -1.0
        y_center_norm = -1.0
        dist_from_bottom_norm = -1.0
        line_points_px = None
        detection_method = "none"

        # ===== Method A: BEV 기반 추정 (기본) =====
        bev_result = None
        if bool(self.get_parameter("use_birds_eye").value):
            bev_result = self.detect_line_by_bev(white_mask, roi_poly)

        if bev_result is not None:
            raw_detected = True
            line_points_px = bev_result["line_points_px"]
            p1, p2 = line_points_px

            x1_norm = float(np.clip(p1[0] / max(w - 1, 1), 0.0, 1.0))
            y1_norm = float(np.clip(p1[1] / max(h - 1, 1), 0.0, 1.0))
            x2_norm = float(np.clip(p2[0] / max(w - 1, 1), 0.0, 1.0))
            y2_norm = float(np.clip(p2[1] / max(h - 1, 1), 0.0, 1.0))
            y_center = 0.5 * (p1[1] + p2[1])
            y_center_norm = float(np.clip(y_center / max(h - 1, 1), 0.0, 1.0))
            dist_from_bottom_norm = float(np.clip((h - 1 - y_center) / max(h - 1, 1), 0.0, 1.0))
            confidence = float(bev_result["confidence"])
            detection_method = "bev"
        else:
            # ===== Method B: 기존 contour + fitLine fallback =====
            stripe_count = len(near_points)
            min_stripes = int(self.get_parameter("min_stripe_count").value)

            if stripe_count >= min_stripes:
                pts = np.array(near_points, dtype=np.float32)
                vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).flatten().tolist()

                # full image 경계 교차 대신, 실제 포인트 x 범위로 line segment를 구성해 기울기 과장을 완화
                x_min = float(np.min(pts[:, 0]))
                x_max = float(np.max(pts[:, 0]))
                if (x_max - x_min) >= max(5.0, 0.12 * w):
                    p1, p2 = self.compute_line_segment_from_x_range(vx, vy, x0, y0, x_min, x_max, w, h)
                else:
                    p1, p2 = self.compute_line_endpoints(vx, vy, x0, y0, w, h)

                if p1 is not None and p2 is not None:
                    raw_detected = True
                    line_points_px = (p1, p2)

                    x1_norm = float(np.clip(p1[0] / max(w - 1, 1), 0.0, 1.0))
                    y1_norm = float(np.clip(p1[1] / max(h - 1, 1), 0.0, 1.0))
                    x2_norm = float(np.clip(p2[0] / max(w - 1, 1), 0.0, 1.0))
                    y2_norm = float(np.clip(p2[1] / max(h - 1, 1), 0.0, 1.0))

                    if abs(vx) > 1e-6:
                        y_center = y0 + ((w * 0.5 - x0) * (vy / vx))
                    else:
                        y_center = y0

                    y_center = float(np.clip(y_center, 0.0, h - 1))
                    y_center_norm = float(np.clip(y_center / max(h - 1, 1), 0.0, 1.0))
                    dist_from_bottom_norm = float(np.clip((h - 1 - y_center) / max(h - 1, 1), 0.0, 1.0))

                    xs = np.array([p[0] for p in near_points], dtype=np.float32)
                    stripe_score = min(1.0, stripe_count / 8.0)
                    spread_score = min(1.0, float(np.std(xs)) / max(1.0, 0.18 * w))
                    confidence = 0.55 * stripe_score + 0.45 * spread_score
                    detection_method = "contour_fit"

        stripe_count = len(near_points)

        return {
            "raw_detected": raw_detected,
            "confidence": float(confidence),
            "x1_norm": x1_norm,
            "y1_norm": y1_norm,
            "x2_norm": x2_norm,
            "y2_norm": y2_norm,
            "y_center_norm": y_center_norm,
            "dist_from_bottom_norm": dist_from_bottom_norm,
            "stripe_count": stripe_count,
            "near_points": near_points,
            "valid_contours": valid_contours,
            "roi_poly": roi_poly,
            "line_points_px": line_points_px,
            "detection_method": detection_method,
            "bev_boundary_points": bev_result["boundary_points"] if bev_result is not None else [],
        }

    def build_roi_mask(self, width, height):
        tlx = float(self.get_parameter("roi_top_left_x").value)
        trx = float(self.get_parameter("roi_top_right_x").value)
        blx = float(self.get_parameter("roi_bottom_left_x").value)
        brx = float(self.get_parameter("roi_bottom_right_x").value)
        top_y = float(self.get_parameter("roi_top_y").value)
        bot_y = float(self.get_parameter("roi_bottom_y").value)

        top_y = float(np.clip(top_y, 0.0, 1.0))
        bot_y = float(np.clip(bot_y, 0.0, 1.0))
        if bot_y <= top_y:
            bot_y = min(1.0, top_y + 0.1)

        poly = np.array(
            [
                [int(tlx * width), int(top_y * height)],
                [int(trx * width), int(top_y * height)],
                [int(brx * width), int(bot_y * height)],
                [int(blx * width), int(bot_y * height)],
            ],
            dtype=np.int32,
        )

        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(mask, [poly], 255)
        return mask, poly

    @staticmethod
    def compute_line_endpoints(vx, vy, x0, y0, w, h):
        eps = 1e-6
        candidates = []

        if abs(vx) > eps:
            for x in (0.0, float(w - 1)):
                t = (x - x0) / vx
                y = y0 + t * vy
                if 0.0 <= y <= (h - 1):
                    candidates.append((x, y))

        if abs(vy) > eps:
            for y in (0.0, float(h - 1)):
                t = (y - y0) / vy
                x = x0 + t * vx
                if 0.0 <= x <= (w - 1):
                    candidates.append((x, y))

        if len(candidates) < 2:
            return None, None

        best_pair = None
        best_dist = -1.0
        for i in range(len(candidates)):
            for j in range(i + 1, len(candidates)):
                p1 = candidates[i]
                p2 = candidates[j]
                d = (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2
                if d > best_dist:
                    best_dist = d
                    best_pair = (p1, p2)

        if best_pair is None:
            return None, None

        p1 = (int(round(best_pair[0][0])), int(round(best_pair[0][1])))
        p2 = (int(round(best_pair[1][0])), int(round(best_pair[1][1])))
        return p1, p2

    @staticmethod
    def compute_line_segment_from_x_range(vx, vy, x0, y0, x_min, x_max, w, h):
        eps = 1e-6
        if abs(vx) <= eps:
            return None, None

        y1 = y0 + ((x_min - x0) * (vy / vx))
        y2 = y0 + ((x_max - x0) * (vy / vx))

        p1 = (int(np.clip(round(x_min), 0, w - 1)), int(np.clip(round(y1), 0, h - 1)))
        p2 = (int(np.clip(round(x_max), 0, w - 1)), int(np.clip(round(y2), 0, h - 1)))
        return p1, p2

    def detect_line_by_bev(self, white_mask, roi_poly):
        h, w = white_mask.shape[:2]
        src = np.array(roi_poly, dtype=np.float32)
        dst = np.array(
            [
                [0.0, 0.0],
                [float(w - 1), 0.0],
                [float(w - 1), float(h - 1)],
                [0.0, float(h - 1)],
            ],
            dtype=np.float32,
        )
        m = cv2.getPerspectiveTransform(src, dst)
        minv = cv2.getPerspectiveTransform(dst, src)
        bev_mask = cv2.warpPerspective(white_mask, m, (w, h), flags=cv2.INTER_NEAREST)

        col_step = max(1, int(self.get_parameter("bev_column_step").value))
        boundary_points = []
        for x in range(0, w, col_step):
            ys = np.where(bev_mask[:, x] > 0)[0]
            if ys.size == 0:
                continue
            y_max = float(np.max(ys))
            boundary_points.append((float(x), y_max))

        min_cols = int(self.get_parameter("bev_min_valid_columns").value)
        if len(boundary_points) < min_cols:
            return None

        xs = np.array([p[0] for p in boundary_points], dtype=np.float32)
        ys = np.array([p[1] for p in boundary_points], dtype=np.float32)
        x_cov = float((np.max(xs) - np.min(xs)) / max(1.0, (w - 1)))
        min_cov = float(self.get_parameter("bev_min_x_coverage").value)
        if x_cov < min_cov:
            return None

        # BEV에서 stop line은 거의 수평 -> robust 중앙값 사용
        y_line = float(np.median(ys))
        x_min = float(np.min(xs))
        x_max = float(np.max(xs))
        p1_bev = np.array([[[x_min, y_line]]], dtype=np.float32)
        p2_bev = np.array([[[x_max, y_line]]], dtype=np.float32)
        p1_img = cv2.perspectiveTransform(p1_bev, minv)[0][0]
        p2_img = cv2.perspectiveTransform(p2_bev, minv)[0][0]

        p1 = (
            int(np.clip(round(float(p1_img[0])), 0, w - 1)),
            int(np.clip(round(float(p1_img[1])), 0, h - 1)),
        )
        p2 = (
            int(np.clip(round(float(p2_img[0])), 0, w - 1)),
            int(np.clip(round(float(p2_img[1])), 0, h - 1)),
        )

        col_score = min(1.0, len(boundary_points) / max(1.0, (w / col_step) * 0.75))
        conf = 0.55 * x_cov + 0.45 * col_score

        return {
            "line_points_px": (p1, p2),
            "confidence": float(np.clip(conf, 0.0, 1.0)),
            "boundary_points": boundary_points,
        }

    def publish_outputs(self, result, stable_detected, stable_line):
        state_msg = String()
        state_msg.data = "detected" if stable_detected else "none"
        self.state_pub.publish(state_msg)

        if stable_line is None:
            x1 = y1 = x2 = y2 = yc = dist = -1.0
            conf = result["confidence"] if result["raw_detected"] else 0.0
            stripe_count = float(result["stripe_count"])
        else:
            x1 = stable_line["x1_norm"]
            y1 = stable_line["y1_norm"]
            x2 = stable_line["x2_norm"]
            y2 = stable_line["y2_norm"]
            yc = stable_line["y_center_norm"]
            dist = stable_line["dist_from_bottom_norm"]
            conf = stable_line["confidence"]
            stripe_count = stable_line["stripe_count"]

        perception = Float32MultiArray()
        perception.data = [
            float(1.0 if result["raw_detected"] else 0.0),
            float(1.0 if stable_detected else 0.0),
            float(np.clip(conf, 0.0, 1.0)),
            float(x1),
            float(y1),
            float(x2),
            float(y2),
            float(yc),
            float(dist),
            float(stripe_count),
        ]
        self.perception_pub.publish(perception)

    def publish_debug_image(self, frame, result, stable_detected, stable_line):
        dbg = frame.copy()

        roi_poly = result["roi_poly"]
        cv2.polylines(dbg, [roi_poly], True, (255, 200, 0), 2)

        cv2.drawContours(dbg, result["valid_contours"], -1, (0, 255, 0), 2)

        for pt in result["near_points"]:
            cv2.circle(dbg, (int(round(pt[0])), int(round(pt[1]))), 3, (0, 255, 255), -1)

        # BEV 경계 포인트(표시 과밀 방지를 위해 샘플링)
        bev_points = result.get("bev_boundary_points", [])
        if len(bev_points) > 0:
            step = max(1, len(bev_points) // 80)
            for pt in bev_points[::step]:
                cv2.circle(dbg, (int(round(pt[0])), int(round(pt[1]))), 1, (255, 0, 255), -1)

        if result["line_points_px"] is not None:
            p1, p2 = result["line_points_px"]
            cv2.line(dbg, p1, p2, (0, 140, 255), 2)

        if stable_detected and stable_line is not None:
            h, w = dbg.shape[:2]
            p1 = (
                int(np.clip(stable_line["x1_norm"] * (w - 1), 0, w - 1)),
                int(np.clip(stable_line["y1_norm"] * (h - 1), 0, h - 1)),
            )
            p2 = (
                int(np.clip(stable_line["x2_norm"] * (w - 1), 0, w - 1)),
                int(np.clip(stable_line["y2_norm"] * (h - 1), 0, h - 1)),
            )
            cv2.line(dbg, p1, p2, (0, 0, 255), 4)

        status = "DETECTED" if stable_detected else "NONE"
        conf = stable_line["confidence"] if (stable_detected and stable_line is not None) else result["confidence"]
        cv2.rectangle(dbg, (8, 8), (430, 95), (35, 35, 35), -1)
        cv2.putText(dbg, f"StopLine: {status}", (18, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(
            dbg,
            f"Raw:{int(result['raw_detected'])} Stable:{int(stable_detected)} Conf:{conf:.2f}",
            (18, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (220, 220, 220),
            1,
        )
        cv2.putText(
            dbg,
            f"Stripes:{result['stripe_count']} Method:{result.get('detection_method', 'none')}",
            (18, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (220, 220, 220),
            1,
        )

        try:
            msg = self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8")
            self.debug_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"[StopLine] debug publish failed: {exc}")

    def maybe_log(self, result, stable_detected):
        interval = float(self.get_parameter("log_interval_sec").value)
        if interval <= 0.0:
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_log_time).nanoseconds / 1e9
        if elapsed < interval:
            return

        self.last_log_time = now
        self.get_logger().info(
            f"[StopLine] stable={stable_detected}, raw={result['raw_detected']}, "
            f"stripes={result['stripe_count']}, conf={result['confidence']:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = StopLineDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
