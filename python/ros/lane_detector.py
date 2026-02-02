#!/usr/bin/env python3
"""
Lane Detector - Step 2: 선 그룹 밀도 분석으로 점선/실선 구분

=== 핵심 논리 ===
1. Hough Transform으로 모든 선 검출
2. 선들을 X 위치 기준으로 2개 그룹으로 클러스터링 (왼쪽/오른쪽)
3. 각 그룹의 "파편화 점수" 계산
   - 파편화 점수 = 선 개수 ÷ 평균 선 길이
   - 점수가 높으면 → 점선 (짧은 선이 많음 = 중앙선)
   - 점수가 낮으면 → 실선 (긴 선이 적음 = 도로 끝)
4. 회전 시에도 상대적 특성으로 판단하므로 위치에 무관하게 작동

발행 토픽:
- /lane_step/roi, gray, blur, white_mask, edges, combined
- /lane_debug : 최종 결과

파라미터 (rqt_reconfigure로 조절 가능):
- white_threshold, min_line_length, max_line_gap, hough_threshold
- canny_low, canny_high, size_threshold

Author: KJH
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, SetParametersResult
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # ========== 파라미터 선언 ==========
        self.declare_parameter('white_threshold', 150,
            ParameterDescriptor(
                description='흰색 검출 임계값 (낮을수록 민감)',
                integer_range=[IntegerRange(from_value=0, to_value=255, step=1)]
            ))
        
        self.declare_parameter('min_line_length', 10,
            ParameterDescriptor(
                description='Hough: 최소 선 길이 (픽셀)',
                integer_range=[IntegerRange(from_value=1, to_value=200, step=1)]
            ))
        
        self.declare_parameter('max_line_gap', 10,
            ParameterDescriptor(
                description='Hough: 최대 선 간격 (픽셀)',
                integer_range=[IntegerRange(from_value=1, to_value=200, step=1)]
            ))
        
        self.declare_parameter('hough_threshold', 50,
            ParameterDescriptor(
                description='Hough: 투표 임계값',
                integer_range=[IntegerRange(from_value=1, to_value=200, step=1)]
            ))
        
        self.declare_parameter('canny_low', 50,
            ParameterDescriptor(
                description='Canny: 하한값',
                integer_range=[IntegerRange(from_value=0, to_value=255, step=1)]
            ))
        
        self.declare_parameter('canny_high', 150,
            ParameterDescriptor(
                description='Canny: 상한값',
                integer_range=[IntegerRange(from_value=0, to_value=255, step=1)]
            ))

        self.declare_parameter('size_threshold', 100,
            ParameterDescriptor(
                description='연결 컴포넌트 크기 임계값 (이상이면 실선)',
                integer_range=[IntegerRange(from_value=1, to_value=500, step=1)]
            ))

        self.declare_parameter('height_ratio_threshold', 15, # Default lowered to 15% 이 값 낮추면 실선 판정 쉬워짐
            ParameterDescriptor(
                description='세로 길이 비율 임계값 (% 단위, ROI 높이 대비 이상이면 실선)',
                integer_range=[IntegerRange(from_value=10, to_value=100, step=1)]
            ))
            
        self.declare_parameter('morph_kernel_size', 13, # Increased to 13 # 이 값 높이면 더 강하게 연결
            ParameterDescriptor(
                description='모폴로지 연산 커널 크기 (끊어진 선 연결용 홀수)',
                integer_range=[IntegerRange(from_value=1, to_value=21, step=2)]
            ))
        
        # Removed score_threshold parameters
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        
        # Publishers
        self.pub_roi = self.create_publisher(Image, '/lane_step/roi', 10)
        self.pub_gray = self.create_publisher(Image, '/lane_step/gray', 10)
        self.pub_blur = self.create_publisher(Image, '/lane_step/blur', 10)
        self.pub_white_mask = self.create_publisher(Image, '/lane_step/white_mask', 10)
        self.pub_edges = self.create_publisher(Image, '/lane_step/edges', 10)
        self.pub_combined = self.create_publisher(Image, '/lane_step/combined', 10)
        self.pub_debug = self.create_publisher(Image, '/lane_debug', 10)
        
        # ========== 안정화를 위한 이력 저장 ==========
        self.prev_left_is_dashed = True   # 이전 판정 결과 (기본: 왼쪽=점선)
        self.prev_right_is_dashed = False  # 이전 판정 결과 (기본: 오른쪽=실선)

        # ========== 판정 연속성 안정화 ==========
        # 판정이 바뀌려고 할 때, 연속된 변화를 방지
        self.left_change_confirmation = 0  # 같은 판정이 연속된 프레임 수
        self.right_change_confirmation = 0
        self.MIN_CONFIRMATION_FRAMES = 5   # 판정이 확인되려면 최소 N프레임 연속
        
        # ========== 런타임 파라미터 변경 콜백 등록 ==========
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info('=== Lane Detector Started ===')
        self.get_logger().info('Step 2: 연결 요소 분석 (Connected Components)')
    
    def get_params(self):
        """현재 파라미터 값 가져오기"""
        return {
            'white_threshold': self.get_parameter('white_threshold').value,
            'min_line_length': self.get_parameter('min_line_length').value,
            'max_line_gap': self.get_parameter('max_line_gap').value,
            'hough_threshold': self.get_parameter('hough_threshold').value,
            'canny_low': self.get_parameter('canny_low').value,
            'canny_high': self.get_parameter('canny_high').value,
            'size_threshold': self.get_parameter('size_threshold').value,
            'height_ratio_threshold': self.get_parameter('height_ratio_threshold').value,
            'morph_kernel_size': self.get_parameter('morph_kernel_size').value,
        }

    def parameter_callback(self, params):
        """런타임 파라미터 변경 콜백 (rqt_reconfigure 지원)"""
        for param in params:
            self.get_logger().info(f'Parameter changed: {param.name} = {param.value}')
        return SetParametersResult(successful=True)
    
    def image_callback(self, msg: Image):
        """카메라 이미지 콜백"""
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return
        
        self.detect_lanes(image)
    
    def publish_image(self, publisher, image, encoding='bgr8'):
        """이미지를 토픽으로 발행"""
        try:
            if len(image.shape) == 2:
                encoding = 'mono8'
            msg = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
            publisher.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Image publish failed: {e}')
    
    def detect_lanes(self, image: np.ndarray):
        """
        차선 감지 메인 함수
        
        처리 흐름:
        1. 전처리 (ROI, 그레이스케일, 블러, 엣지)
        2. Hough Transform으로 선 검출
        3. 선 그룹화 (X 위치 기반 클러스터링)
        4. 각 그룹의 파편화 점수 계산
        5. 점선/실선 분류 및 시각화
        """
        params = self.get_params()
        height, width = image.shape[:2]
        
        # ========== 1. 전처리 ==========
        roi_top = int(height * 0.35)
        roi = image[roi_top:, :].copy()
        roi_height, roi_width = roi.shape[:2]
        self.publish_image(self.pub_roi, roi)
        
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        self.publish_image(self.pub_gray, gray)
        
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        self.publish_image(self.pub_blur, blur)
        
        _, white_mask = cv2.threshold(blur, params['white_threshold'], 255, cv2.THRESH_BINARY)
        self.publish_image(self.pub_white_mask, white_mask)
        
        edges = cv2.Canny(blur, params['canny_low'], params['canny_high'])
        self.publish_image(self.pub_edges, edges)
        
        combined = cv2.bitwise_or(edges, white_mask)
        self.publish_image(self.pub_combined, combined)
        
        # ========== 1.5. 모폴로지 연산 (끊어진 선 연결) ==========
        # Canny 등으로 인해 미세하게 끊어진 실선을 하나로 연결
        k_size = params['morph_kernel_size']
        if k_size > 1:
            # 세로 방향으로 긴 커널 사용 (차선은 주로 세로/대각선이므로)
            # (3, k_size) 형태의 커널 추천
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, k_size))
            edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        
        # ========== 2. Connected Component Analysis (핵심 로직 변경) ==========
        # 엣지 이미지에서 연결된 요소를 찾아, 각 요소의 "최대 높이"를 분석
        
        # ROI 절반으로 나누기
        center_x = int(roi_width / 2)
        edges_left = edges[:, :center_x]
        edges_right = edges[:, center_x:]
        
        # 왼쪽/오른쪽 각각 분석
        left_max_height = self.get_max_component_height(edges_left, params['size_threshold'])
        right_max_height = self.get_max_component_height(edges_right, params['size_threshold'])
        
        # 실선 판정 임계값 (ROI 높이의 N%)
        solid_height_threshold = roi_height * (params['height_ratio_threshold'] / 100.0)
        
        # ========== 3. 점선/실선 판정 ==========
        # 높이가 임계값보다 크면 실선(Solid), 작으면 점선(Dashed)
        
        # LEFT 판정
        left_is_dashed_new = left_max_height < solid_height_threshold
        
        # RIGHT 판정
        right_is_dashed_new = right_max_height < solid_height_threshold
        
        # ========== 4. 안정화 (Debouncing) ==========
        
        # LEFT 판정 안정화
        if left_is_dashed_new != self.prev_left_is_dashed:
            self.left_change_confirmation += 1
            if self.left_change_confirmation >= self.MIN_CONFIRMATION_FRAMES:
                left_is_dashed = left_is_dashed_new
                self.prev_left_is_dashed = left_is_dashed
                self.left_change_confirmation = 0
            else:
                left_is_dashed = self.prev_left_is_dashed
        else:
            left_is_dashed = left_is_dashed_new
            self.prev_left_is_dashed = left_is_dashed
            self.left_change_confirmation = 0

        # RIGHT 판정 안정화
        if right_is_dashed_new != self.prev_right_is_dashed:
            self.right_change_confirmation += 1
            if self.right_change_confirmation >= self.MIN_CONFIRMATION_FRAMES:
                right_is_dashed = right_is_dashed_new
                self.prev_right_is_dashed = right_is_dashed
                self.right_change_confirmation = 0
            else:
                right_is_dashed = self.prev_right_is_dashed
        else:
            right_is_dashed = right_is_dashed_new
            self.prev_right_is_dashed = right_is_dashed
            self.right_change_confirmation = 0
        
        # ========== 5. 디버그 이미지 생성 (선별 연속성 분석 시각화) ==========
        # 각 열(column)의 연속성을 분석하여 점선/실선 색칠
        debug = self.colorize_edges_by_continuity(edges, params)

        # 정보 패널 (영어만 사용 - 한글 깨짐 방지)
        cv2.rectangle(debug, (5, 5), (320, 80), (40, 40, 40), -1)

        # 왼쪽 정보
        left_type = "DASHED (center)" if left_is_dashed else "SOLID"
        left_color = (255, 0, 0) if left_is_dashed else (0, 255, 0)
        cv2.putText(debug, f"LEFT: {left_type} (H:{left_max_height})", (15, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, left_color, 1)

        # 오른쪽 정보
        right_type = "DASHED" if right_is_dashed else "SOLID (edge)"
        right_color = (255, 0, 0) if right_is_dashed else (0, 255, 0)
        cv2.putText(debug, f"RIGHT: {right_type} (H:{right_max_height})", (15, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, right_color, 1)

        # 범례
        cv2.putText(debug, "Blue=Dashed  Green=Solid", (15, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, (200, 200, 200), 1)

        self.publish_image(self.pub_debug, debug)
    
    def group_lines_by_position(self, lines: np.ndarray, image_width: int):
        """
        선들을 X 위치 기준으로 2개 그룹으로 나눔
        
        논리:
        - 각 선의 중심 X 좌표 계산
        - 이미지 중앙(image_width / 2)을 기준으로 왼쪽/오른쪽 분류
        
        Args:
            lines: Hough Transform 결과 (N, 1, 4) 형태
            image_width: 이미지 너비
            
        Returns:
            left_group: 왼쪽 그룹의 선 리스트 [(x1,y1,x2,y2), ...]
            right_group: 오른쪽 그룹의 선 리스트
        """
        left_group = []   # 이미지 왼쪽 절반의 선들
        right_group = []  # 이미지 오른쪽 절반의 선들
        
        center_x = image_width / 2
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # 선의 중심 X 좌표 계산
            line_center_x = (x1 + x2) / 2
            
            if line_center_x < center_x:
                left_group.append((x1, y1, x2, y2))
            else:
                right_group.append((x1, y1, x2, y2))
        
        return left_group, right_group
    
    def colorize_edges_by_continuity(self, edges: np.ndarray, params: dict) -> np.ndarray:
        """
        연결된 컴포넌트(connected components) 분석으로 점선/실선 색칠

        === 핵심 논리 ===
        1. 엣지 이미지에서 연결된 컴포넌트 찾기
        2. 각 컴포넌트의 크기(픽셀 수) + 세로 길이(height) 분석
        3. 크기 크고 AND 세로로 길면 = 실선 → 초록색
        4. 그 외 = 점선 → 파란색

        실선 판정 조건:
        - 픽셀 수 >= size_threshold AND
        - 세로 길이 >= ROI 높이의 height_ratio_threshold%

        Args:
            edges: Canny 엣지 이미지 (그레이스케일)
            params: 파라미터 딕셔너리 (size_threshold, height_ratio_threshold 포함)

        Returns:
            colored: 색칠된 BGR 이미지
        """
        img_height, img_width = edges.shape
        colored = np.zeros((img_height, img_width, 3), dtype=np.uint8)

        # 연결된 컴포넌트 찾기
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(edges, connectivity=8)

        # 임계값 설정
        size_threshold = params['size_threshold']
        # 세로 길이 임계값: ROI 높이의 N% 이상이어야 실선
        height_threshold = img_height * params['height_ratio_threshold'] / 100.0

        for label in range(1, num_labels):  # 0은 배경
            # 컴포넌트의 픽셀 수
            component_size = stats[label, cv2.CC_STAT_AREA]
            # 컴포넌트의 바운딩 박스 높이 (세로 길이)
            component_height = stats[label, cv2.CC_STAT_HEIGHT]

            # 컴포넌트의 마스크
            mask = (labels == label)

            # 실선 판정: 크기 크고 AND 세로로 길어야 함
            if component_size >= size_threshold and component_height >= height_threshold:
                color = (0, 255, 0)  # 초록색 = 실선
            else:
                color = (255, 0, 0)  # 파란색 = 점선

            colored[mask] = color

        return colored


    def get_max_component_height(self, roi_edges: np.ndarray, size_threshold: int) -> int:
        """
        ROI 내에서 가장 큰 세로 길이를 가진 연결 요소의 높이를 반환
        """
        # 연결 요소 찾기
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(roi_edges, connectivity=8)
        
        max_height = 0
        
        for label in range(1, num_labels): # 0은 배경
            size = stats[label, cv2.CC_STAT_AREA]
            height = stats[label, cv2.CC_STAT_HEIGHT]
            
            # 노이즈 필터링
            if size < size_threshold:
                continue
                
            if height > max_height:
                max_height = height
                
        return max_height


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
