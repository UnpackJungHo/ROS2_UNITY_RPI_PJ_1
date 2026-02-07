#!/usr/bin/env python3
"""
YOLOv8n 전이학습 - 신호등 색상 탐지 (Green / Red / Yellow)

=== 데이터셋 (Roboflow) ===
Roboflow에서 YOLOv8 포맷으로 다운로드한 데이터셋을 그대로 사용합니다.

datasets/traffic_light/
├── data.yaml           ← Roboflow 제공 YAML (경로 자동 보정)
├── train/
│   ├── images/
│   └── labels/
├── valid/
│   ├── images/
│   └── labels/
└── test/
    ├── images/
    └── labels/

=== 클래스 (Roboflow 순서) ===
0: Green
1: Red
2: Yellow

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


def prepare_dataset_yaml(dataset_path: str) -> str:
    """
    Roboflow data.yaml의 상대경로를 절대경로로 보정한 YAML 생성
    원본 data.yaml은 수정하지 않음
    """
    original_yaml = os.path.join(dataset_path, 'data.yaml')
    fixed_yaml = os.path.join(dataset_path, 'data_abs.yaml')

    if not os.path.exists(original_yaml):
        raise FileNotFoundError(f"data.yaml not found: {original_yaml}")

    with open(original_yaml, 'r') as f:
        config = yaml.safe_load(f)

    # Roboflow 상대경로 → 절대경로 변환
    abs_dataset = os.path.abspath(dataset_path)
    config['path'] = abs_dataset
    config['train'] = 'train/images'
    config['val'] = 'valid/images'
    config['test'] = 'test/images'

    with open(fixed_yaml, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)

    print(f"[Dataset] Original YAML: {original_yaml}")
    print(f"[Dataset] Fixed YAML:    {fixed_yaml}")
    print(f"[Dataset] Path:          {abs_dataset}")
    print(f"[Dataset] Classes:       {config['names']}")
    print(f"[Dataset] Train: {abs_dataset}/train/images/")
    print(f"[Dataset] Val:   {abs_dataset}/valid/images/")
    print(f"[Dataset] Test:  {abs_dataset}/test/images/")

    return fixed_yaml


def train(args):
    """YOLOv8n 전이학습 실행"""
    print("=" * 60)
    print("YOLOv8n Traffic Light Detection - Transfer Learning")
    print("=" * 60)

    # 데이터셋 YAML 준비 (상대경로 → 절대경로 보정)
    yaml_path = prepare_dataset_yaml(args.dataset_path)

    # YOLOv8n pretrained 모델 로드
    model = YOLO('yolov8n.pt')
    print(f"\n[Model] YOLOv8n loaded (pretrained on COCO)")
    print(f"[Model] Parameters: ~3.2M, GFLOPs: ~8.7")

    # 전이학습 실행
    results = model.train(
        data=yaml_path,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch_size,
        device=args.device,
        name='traffic_light_yolov8n',
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
        # 데이터 증강
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        degrees=5.0,
        translate=0.1,
        scale=0.5,
        flipud=0.0,  # 상하 반전 비활성화 (신호등은 위아래가 중요)
        fliplr=0.5,
        mosaic=1.0,
        mixup=0.1,
    )

    # 결과 출력
    print("\n" + "=" * 60)
    print("Training Complete!")
    print(f"  Best weights: runs/detect/traffic_light_yolov8n/weights/best.pt")
    print(f"  Last weights: runs/detect/traffic_light_yolov8n/weights/last.pt")
    print("=" * 60)

    # 검증
    print("\n[Validation] Running final validation...")
    val_results = model.val()
    print(f"  mAP50: {val_results.box.map50:.4f}")
    print(f"  mAP50-95: {val_results.box.map:.4f}")

    return model


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="YOLOv8n Traffic Light Training")
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
