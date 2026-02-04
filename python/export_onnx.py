# export_onnx.py
"""
Speed-Aware Single View 모델을 ONNX 형식으로 변환

입력: front_image + speed
출력: logits [1, 7]
"""
import torch
import torch.onnx
from pathlib import Path
import sys

# 프로젝트 경로 추가
sys.path.insert(0, str(Path(__file__).parent))


def export_speed_aware_to_onnx(
    checkpoint_path: str,
    output_path: str,
    front_size: tuple = (66, 200),  # (H, W)
):
    """Speed-Aware Single View 모델을 ONNX로 변환"""
    from train_classification import SpeedAwareSingleViewNet, CLASS_NAMES

    print(f"Loading checkpoint: {checkpoint_path}")

    checkpoint = torch.load(checkpoint_path, map_location='cpu')

    # 모델 버전 확인
    model_version = checkpoint.get('model_version', 'unknown')
    backbone = checkpoint.get('backbone', 'resnet18')
    speed_normalize = checkpoint.get('speed_normalize', 5.0)

    print(f"  Model version: {model_version}")
    print(f"  Backbone: {backbone}")
    print(f"  Speed normalize: {speed_normalize}")
    print(f"  Epoch: {checkpoint['epoch']}, Val acc: {checkpoint.get('val_acc', 0):.1f}%")

    # 모델 로드 (dropout=0 for inference)
    model = SpeedAwareSingleViewNet(
        backbone=backbone,
        pretrained=False,
        dropout=0.0
    )

    model.load_state_dict(checkpoint['model_state_dict'])
    model.eval()

    # 더미 입력 (mask 제거됨)
    dummy_front = torch.randn(1, 3, front_size[0], front_size[1])
    dummy_speed = torch.randn(1, 1)

    # ONNX 변환
    print(f"Exporting to ONNX: {output_path}")

    torch.onnx.export(
        model,
        (dummy_front, dummy_speed),
        output_path,
        export_params=True,
        opset_version=17,  # Unity Sentis 호환
        do_constant_folding=True,
        input_names=['front_image', 'speed'],
        output_names=['logits'],
        dynamic_axes={
            'front_image': {0: 'batch_size'},
            'speed': {0: 'batch_size'},
            'logits': {0: 'batch_size'}
        },
        dynamo=False  # 단일 파일 ONNX 내보내기 (weights 내장)
    )

    print(f"ONNX export complete!")
    print(f"  Inputs: front_image (1, 3, {front_size[0]}, {front_size[1]}), speed (1, 1)")
    print(f"  Output: logits [batch, 7] = {CLASS_NAMES}")
    print(f"")
    print(f"[중요] Unity에서 사용 시:")
    print(f"  - speedNormalize 값을 {speed_normalize}로 설정하세요")
    print(f"  - speed 입력 = currentSpeed / {speed_normalize}")

    # 검증
    try:
        import onnx
        onnx_model = onnx.load(output_path)
        onnx.checker.check_model(onnx_model)
        print("ONNX model validation: OK")
    except ImportError:
        print("(onnx package not installed, skipping validation)")
    except Exception as e:
        print(f"ONNX validation warning: {e}")

    # ONNX Runtime 테스트
    try:
        import onnxruntime as ort
        import numpy as np

        session = ort.InferenceSession(output_path)
        front_np = np.random.randn(1, 3, front_size[0], front_size[1]).astype(np.float32)
        speed_np = np.array([[0.5]], dtype=np.float32)  # 정규화된 속도

        outputs = session.run(None, {
            'front_image': front_np,
            'speed': speed_np
        })

        logits = outputs[0][0]
        predicted = np.argmax(logits)
        print(f"Test inference - Predicted: {CLASS_NAMES[predicted]} (logits: {logits.round(3)})")
        print("ONNX Runtime test: OK")
    except ImportError:
        print("(onnxruntime not installed, skipping runtime test)")
    except Exception as e:
        print(f"ONNX Runtime test warning: {e}")

    # 메타데이터 저장
    meta_path = output_path.replace('.onnx', '_meta.json')
    import json
    with open(meta_path, 'w') as f:
        json.dump({
            'model_version': model_version,
            'backbone': backbone,
            'speed_normalize': speed_normalize,
            'class_names': CLASS_NAMES,
            'front_size': front_size,
        }, f, indent=2)
    print(f"Metadata saved to: {meta_path}")

    return output_path


if __name__ == "__main__":
    checkpoint_dir = Path(__file__).parent / "checkpoints"
    unity_assets = Path(__file__).parent.parent / "Assets" / "Models" / "ONNX"
    unity_assets.mkdir(parents=True, exist_ok=True)

    # Speed-Aware Single View 모델 확인
    v2_checkpoint = checkpoint_dir / "driving_classifier.pth"

    if v2_checkpoint.exists():
        print("=== Speed-Aware Single View Model Export ===")
        output_path = unity_assets / "driving_classifier_single.onnx"

        export_speed_aware_to_onnx(
            checkpoint_path=str(v2_checkpoint),
            output_path=str(output_path)
        )

        print(f"\nONNX model saved to: {output_path}")
        print("Unity AutonomousDrivingController의 Model Asset에 이 파일을 할당하세요.")
    else:
        print(f"Error: No checkpoint found at {v2_checkpoint}")
        print("먼저 train_classification.py를 실행하여 학습하세요.")
        exit(1)
