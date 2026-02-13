#!/usr/bin/env python3
"""
YOLOv8s 전이학습 - 신호등 색상 탐지 (Green / Red / Yellow)

=== 데이터셋 (Roboflow) ===
Roboflow에서 수집한 실제 신호등 이미지 데이터 사용.
(9,710장: train 8,493 / valid 811 / test 406)

datasets/traffic_light/
├── data.yaml
├── train/
│   ├── images/
│   └── labels/
├── valid/
│   ├── images/
│   └── labels/
└── test/
    ├── images/
    └── labels/

=== 클래스 ===
0: green
1: red
2: yellow

=== 실행 ===
conda activate driving
python python/TrafficLight/train_traffic_light.py

Author: KJH
"""

from ultralytics import YOLO
from pathlib import Path
import argparse
import yaml
import os


# 프로젝트 루트 (ros2_unity_autoDriver/)
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent


def prepare_dataset_yaml(dataset_path: str) -> str:
    """
    data.yaml의 상대경로를 절대경로로 보정한 YAML 생성
    원본 data.yaml은 수정하지 않음
    """
    original_yaml = os.path.join(dataset_path, 'data.yaml')
    fixed_yaml = os.path.join(dataset_path, 'data_abs.yaml')

    if not os.path.exists(original_yaml):
        raise FileNotFoundError(f"data.yaml not found: {original_yaml}")

    with open(original_yaml, 'r') as f:
        config = yaml.safe_load(f)

    abs_dataset = os.path.abspath(dataset_path)
    config['path'] = abs_dataset
    config['train'] = 'train/images'
    config['val'] = 'valid/images'
    config['test'] = 'test/images'

    with open(fixed_yaml, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)

    print(f"[Dataset] Fixed YAML:    {fixed_yaml}")
    print(f"[Dataset] Path:          {abs_dataset}")
    print(f"[Dataset] Classes:       {config['names']}")
    print(f"[Dataset] Train: {abs_dataset}/train/images/")
    print(f"[Dataset] Val:   {abs_dataset}/valid/images/")
    print(f"[Dataset] Test:  {abs_dataset}/test/images/")

    return fixed_yaml


def train(args):
    """YOLOv8s 전이학습 실행"""
    print("=" * 60)
    print("YOLOv8s Traffic Light Detection - Transfer Learning")
    print("  Dataset: Roboflow real-world traffic light images")
    print("=" * 60)

    yaml_path = prepare_dataset_yaml(args.dataset_path)

    model = YOLO('yolov8n.pt')
    print(f"\n[Model] YOLOv8n loaded (pretrained on COCO)")
    print(f"[Model] Parameters: ~11.2M, GFLOPs: ~28.6")

    # 결과 저장 경로: {PROJECT_ROOT}/runs/detect/
    output_project = str(PROJECT_ROOT / 'runs' / 'detect')

    results = model.train(
        data=yaml_path,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch_size,
        device=args.device,
        project=output_project,
        name='traffic_light_yolov8s',
        patience=args.patience,
        save=True,
        save_period=10,
        val=True,
        plots=True,
        # 전이학습 최적화
        lr0=args.lr,
        lrf=0.01,
        warmup_epochs=3,
        warmup_momentum=0.8,
        # 데이터 증강 (대규모 데이터 → 표준 증강, 색상 보존)
        hsv_h=0.0,       # 색상 변환 없음 (신호등 색상 보존 필수)
        hsv_s=0.3,
        hsv_v=0.4,
        degrees=5.0,
        translate=0.1,
        scale=0.5,
        flipud=0.0,
        fliplr=0.5,
        mosaic=1.0,
        mixup=0.1,
        copy_paste=0.0,
    )

    best_weights = os.path.join(output_project, 'traffic_light_yolov8s', 'weights', 'best.pt')
    print("\n" + "=" * 60)
    print("Training Complete!")
    print(f"  Best weights: {best_weights}")
    print("=" * 60)

    print("\n[Validation] Running final validation...")
    val_results = model.val()
    print(f"  mAP50: {val_results.box.map50:.4f}")
    print(f"  mAP50-95: {val_results.box.map:.4f}")

    return model


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="YOLOv8s Traffic Light Training")
    parser.add_argument("--dataset_path", type=str,
                        default=str(Path(__file__).parent.parent / "datasets" / "traffic_light"),
                        help="데이터셋 경로")
    parser.add_argument("--epochs", type=int, default=100,
                        help="학습 에폭 수")
    parser.add_argument("--imgsz", type=int, default=640,
                        help="입력 이미지 크기")
    parser.add_argument("--batch_size", type=int, default=16,
                        help="배치 크기")
    parser.add_argument("--device", type=str, default="0",
                        help="학습 디바이스 (0=GPU, cpu=CPU)")
    parser.add_argument("--lr", type=float, default=0.01,
                        help="초기 학습률")
    parser.add_argument("--patience", type=int, default=20,
                        help="Early stopping patience")
    args = parser.parse_args()

    train(args)
