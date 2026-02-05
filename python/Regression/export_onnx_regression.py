# export_onnx_regression.py
"""
Speed-Aware 회귀 모델을 ONNX 형식으로 변환

입력: front_image + speed
출력: output [1, 2] → [steering, throttle]
"""
import torch
import torch.onnx
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent))


def export_regression_to_onnx(
    checkpoint_path: str,
    output_path: str,
    front_size: tuple = (66, 200),
):
    """회귀 모델을 ONNX로 변환"""
    from python.Regression.train_regression import SpeedAwareRegressionNet

    print(f"Loading checkpoint: {checkpoint_path}")

    checkpoint = torch.load(checkpoint_path, map_location='cpu')

    model_version = checkpoint.get('model_version', 'unknown')
    backbone = checkpoint.get('backbone', 'resnet18')
    speed_normalize = checkpoint.get('speed_normalize', 5.0)
    steering_loss_weight = checkpoint.get('steering_loss_weight', 5.0)

    print(f"  Model version: {model_version}")
    print(f"  Backbone: {backbone}")
    print(f"  Speed normalize: {speed_normalize}")
    print(f"  Steering loss weight: {steering_loss_weight}")
    print(f"  Epoch: {checkpoint['epoch']}, Val steer MAE: {checkpoint.get('val_steer_mae', 0):.4f}")

    model = SpeedAwareRegressionNet(
        backbone=backbone,
        pretrained=False,
        dropout=0.0
    )

    model.load_state_dict(checkpoint['model_state_dict'])
    model.eval()

    dummy_front = torch.randn(1, 3, front_size[0], front_size[1])
    dummy_speed = torch.randn(1, 1)

    print(f"Exporting to ONNX: {output_path}")

    torch.onnx.export(
        model,
        (dummy_front, dummy_speed),
        output_path,
        export_params=True,
        opset_version=17,
        do_constant_folding=True,
        input_names=['front_image', 'speed'],
        output_names=['output'],
        dynamic_axes={
            'front_image': {0: 'batch_size'},
            'speed': {0: 'batch_size'},
            'output': {0: 'batch_size'}
        },
        dynamo=False
    )

    print(f"ONNX export complete!")
    print(f"  Inputs: front_image (1, 3, {front_size[0]}, {front_size[1]}), speed (1, 1)")
    print(f"  Output: output [batch, 2] = [steering, throttle]")
    print(f"")
    print(f"[Unity 설정]")
    print(f"  - speedNormalize = {speed_normalize}")
    print(f"  - output[0] = steering [-1, 1] -> SetSteering()")
    print(f"  - output[1] = throttle [0, 1]  -> SetThrottle()")

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
        speed_np = np.array([[0.5]], dtype=np.float32)

        outputs = session.run(None, {
            'front_image': front_np,
            'speed': speed_np
        })

        result = outputs[0][0]
        print(f"Test inference - steering: {result[0]:.4f}, throttle: {result[1]:.4f}")
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
            'model_type': 'regression',
            'backbone': backbone,
            'speed_normalize': speed_normalize,
            'steering_loss_weight': steering_loss_weight,
            'output_names': ['steering', 'throttle'],
            'output_ranges': [[-1.0, 1.0], [0.0, 1.0]],
            'front_size': front_size,
        }, f, indent=2)
    print(f"Metadata saved to: {meta_path}")

    return output_path


if __name__ == "__main__":
    checkpoint_dir = Path(__file__).parent / "checkpoints"
    unity_assets = Path(__file__).parent.parent / "Assets" / "Models" / "ONNX"
    unity_assets.mkdir(parents=True, exist_ok=True)

    checkpoint = checkpoint_dir / "driving_regression.pth"

    if checkpoint.exists():
        print("=== Speed-Aware Regression Model Export ===")
        output_path = unity_assets / "driving_regression.onnx"

        export_regression_to_onnx(
            checkpoint_path=str(checkpoint),
            output_path=str(output_path)
        )

        print(f"\nONNX model saved to: {output_path}")
        print("Unity RegressionDrivingController의 Model Asset에 이 파일을 할당하세요.")
    else:
        print(f"Error: No checkpoint found at {checkpoint}")
        print("먼저 train_regression.py를 실행하여 학습하세요.")
        exit(1)
