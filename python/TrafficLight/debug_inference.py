#!/usr/bin/env python3
"""
디버그 스크립트: ROS 토픽으로 수신되는 이미지를 저장 + 오프라인 추론 비교

1단계: ROS 토픽에서 이미지 10장 저장
  python python/TrafficLight/debug_inference.py --save

2단계: 저장된 이미지에 오프라인 추론 (ROS 없이)
  python python/TrafficLight/debug_inference.py --infer

3단계: directlyDataset 이미지에 오프라인 추론 (학습 데이터와 비교)
  python python/TrafficLight/debug_inference.py --infer-dataset
"""

import argparse
import cv2
import numpy as np
from pathlib import Path
from ultralytics import YOLO

SCRIPT_DIR = Path(__file__).parent
MODEL_PATH = SCRIPT_DIR.parent / "ros" / "best.pt"
DEBUG_DIR = SCRIPT_DIR.parent / "debug_frames"
DATASET_DIR = SCRIPT_DIR.parent / "datasets" / "directlyDataset" / "images"

CLASS_NAMES = {0: "Green", 1: "Red", 2: "Yellow"}
COLORS = {0: (0, 255, 0), 1: (0, 0, 255), 2: (0, 255, 255)}


def save_ros_frames(count=10):
    """ROS 토픽에서 이미지를 N장 저장"""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    DEBUG_DIR.mkdir(exist_ok=True)
    bridge = CvBridge()
    saved = [0]

    class Saver(Node):
        def __init__(self):
            super().__init__("debug_frame_saver")
            self.sub = self.create_subscription(Image, "/camera/image_raw", self.cb, 10)
            self.get_logger().info(f"Waiting for /camera/image_raw ... (saving {count} frames)")

        def cb(self, msg):
            if saved[0] >= count:
                return
            frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            path = DEBUG_DIR / f"ros_frame_{saved[0]:04d}.jpg"
            cv2.imwrite(str(path), frame)
            h, w = frame.shape[:2]
            self.get_logger().info(f"Saved {path.name} ({w}x{h}, dtype={frame.dtype}, mean={frame.mean():.1f})")
            saved[0] += 1
            if saved[0] >= count:
                self.get_logger().info(f"Done! {count} frames saved to {DEBUG_DIR}")
                raise SystemExit

    rclpy.init()
    try:
        rclpy.spin(Saver())
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()


def run_inference(image_dir, tag, sample=10):
    """이미지 폴더에 대해 오프라인 추론 실행"""
    print(f"\n{'='*60}")
    print(f"Offline Inference: {tag}")
    print(f"Model: {MODEL_PATH}")
    print(f"{'='*60}")

    model = YOLO(str(MODEL_PATH), task="detect")

    images = sorted(image_dir.glob("*.jpg"))[:sample]
    if not images:
        print(f"No images found in {image_dir}")
        return

    output_dir = DEBUG_DIR / f"results_{tag}"
    output_dir.mkdir(parents=True, exist_ok=True)

    for img_path in images:
        frame = cv2.imread(str(img_path))
        h, w = frame.shape[:2]

        # conf=0.1로 낮춰서 모든 탐지 확인
        results = model.predict(frame, conf=0.1, imgsz=640, verbose=False)

        detections = []
        if len(results) > 0 and results[0].boxes is not None:
            for i in range(len(results[0].boxes)):
                cls = int(results[0].boxes.cls[i])
                conf = float(results[0].boxes.conf[i])
                box = results[0].boxes.xyxy[i].cpu().numpy().astype(int)
                detections.append((cls, conf, box))

                # 바운딩 박스 그리기
                x1, y1, x2, y2 = box
                color = COLORS.get(cls, (128, 128, 128))
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                label = f"{CLASS_NAMES.get(cls, '?')} {conf:.2f}"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # 결과 저장
        cv2.imwrite(str(output_dir / img_path.name), frame)

        det_str = ", ".join([f"{CLASS_NAMES[c]} {cf:.2f}" for c, cf, _ in detections]) if detections else "NONE"
        print(f"  {img_path.name} ({w}x{h}) -> [{det_str}]")

    print(f"\nResults saved to: {output_dir}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--save", action="store_true", help="ROS 토픽에서 이미지 저장")
    parser.add_argument("--infer", action="store_true", help="저장된 ROS 이미지에 오프라인 추론")
    parser.add_argument("--infer-dataset", action="store_true", help="directlyDataset 이미지에 오프라인 추론")
    parser.add_argument("-n", type=int, default=10, help="이미지 수")
    args = parser.parse_args()

    if args.save:
        save_ros_frames(args.n)
    if args.infer:
        run_inference(DEBUG_DIR, "ros_frames", args.n)
    if args.infer_dataset:
        run_inference(DATASET_DIR, "dataset", args.n)

    if not (args.save or args.infer or args.infer_dataset):
        parser.print_help()


if __name__ == "__main__":
    main()
