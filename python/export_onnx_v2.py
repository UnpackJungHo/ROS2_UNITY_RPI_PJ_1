# export_onnx_v2.py
"""
분류 모델 또는 회귀 모델을 ONNX 형식으로 변환
"""
import torch
import torch.onnx
from pathlib import Path
import sys

# 프로젝트 경로 추가
sys.path.insert(0, str(Path(__file__).parent))


def export_classifier_to_onnx(
    checkpoint_path: str,
    output_path: str,
    front_size: tuple = (66, 200),
    top_size: tuple = (128, 128),
):
    """분류 모델을 ONNX로 변환"""
    from train_classification import DualViewClassificationNet, CLASS_NAMES

    print(f"Loading classifier checkpoint: {checkpoint_path}")

    checkpoint = torch.load(checkpoint_path, map_location='cpu')
    print(f"Model loaded (epoch {checkpoint['epoch']}, val_acc {checkpoint.get('val_acc', 0):.1f}%)")

    # 모델 로드 (dropout=0 for inference)
    model = DualViewClassificationNet(
        backbone="resnet18",
        pretrained=False,
        dropout=0.0
    )

    model.load_state_dict(checkpoint['model_state_dict'])
    model.eval()

    # 더미 입력
    dummy_front = torch.randn(1, 3, front_size[0], front_size[1])
    dummy_top = torch.randn(1, 3, top_size[0], top_size[1])

    # ONNX 변환 (출력: logits [1, 7])
    print(f"Exporting to ONNX: {output_path}")

    torch.onnx.export(
        model,
        (dummy_front, dummy_top),
        output_path,
        export_params=True,
        opset_version=12,
        do_constant_folding=True,
        input_names=['front_image', 'top_image'],
        output_names=['logits'],
        dynamic_axes={
            'front_image': {0: 'batch_size'},
            'top_image': {0: 'batch_size'},
            'logits': {0: 'batch_size'}
        }
    )

    print(f"ONNX export complete!")
    print(f"  Output: logits [batch, 7] = {CLASS_NAMES}")

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
        top_np = np.random.randn(1, 3, top_size[0], top_size[1]).astype(np.float32)

        outputs = session.run(None, {
            'front_image': front_np,
            'top_image': top_np
        })

        logits = outputs[0][0]
        predicted = np.argmax(logits)
        print(f"Test inference - Predicted: {CLASS_NAMES[predicted]} (logits: {logits.round(3)})")
        print("ONNX Runtime test: OK")
    except ImportError:
        print("(onnxruntime not installed, skipping runtime test)")
    except Exception as e:
        print(f"ONNX Runtime test warning: {e}")

    return output_path


if __name__ == "__main__":
    checkpoint_dir = Path(__file__).parent / "checkpoints"
    unity_assets = Path(__file__).parent.parent / "Assets" / "Models" / "ONNX"
    unity_assets.mkdir(parents=True, exist_ok=True)

    # 분류 모델 우선 확인
    classifier_checkpoint = checkpoint_dir / "driving_classifier.pth"

    if classifier_checkpoint.exists():
        print("=== Classification Model Export ===")
        output_path = unity_assets / "driving_classifier.onnx"

        export_classifier_to_onnx(
            checkpoint_path=str(classifier_checkpoint),
            output_path=str(output_path)
        )

        print(f"\nONNX model saved to: {output_path}")
        print("Unity AutonomousDrivingController의 Model Asset에 이 파일을 할당하세요.")
    else:
        print(f"Error: No classifier checkpoint found at {classifier_checkpoint}")
        print("먼저 train_classification.py를 실행하여 학습하세요.")
        exit(1)
