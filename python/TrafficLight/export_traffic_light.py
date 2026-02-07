#!/usr/bin/env python3
"""
YOLOv8n 모델 내보내기 - RPi5 배포용

=== 내보내기 포맷 ===
1. NCNN (추천): ARM 프로세서(RPi5) 최적화 추론 엔진
2. ONNX (범용): 다양한 런타임 호환

=== 사용법 ===
1. 학습 완료 후 실행:
   python python/TrafficLight/export_traffic_light.py

2. RPi5에 복사:
   scp -r runs/detect/traffic_light_yolov8n/weights/best_ncnn_model/ pi@<ip>:~/ros2_ws/

Author: KJH
"""

from ultralytics import YOLO
from pathlib import Path
import argparse


def export_model(weights_path: str, formats: list):
    """YOLOv8n 모델을 지정된 포맷으로 내보내기"""
    print("=" * 60)
    print("YOLOv8n Traffic Light Model Export")
    print("=" * 60)

    model = YOLO(weights_path)
    print(f"\n[Model] Loaded: {weights_path}")

    for fmt in formats:
        print(f"\n{'─' * 40}")
        print(f"[Export] Format: {fmt.upper()}")
        print(f"{'─' * 40}")

        if fmt == 'ncnn':
            # NCNN: RPi5 ARM 프로세서 최적화
            model.export(format='ncnn')
            ncnn_dir = Path(weights_path).parent / f"{Path(weights_path).stem}_ncnn_model"
            print(f"  → Output: {ncnn_dir}/")
            print(f"  → RPi5에 이 폴더를 통째로 복사하세요")

        elif fmt == 'onnx':
            # ONNX: 범용 추론 엔진
            model.export(format='onnx', simplify=True)
            onnx_path = Path(weights_path).with_suffix('.onnx')
            print(f"  → Output: {onnx_path}")

        else:
            print(f"  → Unsupported format: {fmt}")
            continue

    print(f"\n{'=' * 60}")
    print("Export Complete!")
    print("=" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="YOLOv8n Traffic Light Model Export")
    parser.add_argument("--weights", type=str,
                        default="runs/detect/traffic_light_yolov8n/weights/best.pt",
                        help="학습된 모델 가중치 경로")
    parser.add_argument("--formats", type=str, nargs='+',
                        default=['ncnn', 'onnx'],
                        choices=['ncnn', 'onnx'],
                        help="내보내기 포맷 (기본: ncnn onnx)")
    args = parser.parse_args()

    export_model(args.weights, args.formats)
