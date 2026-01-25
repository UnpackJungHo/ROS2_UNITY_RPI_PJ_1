# export_onnx.py
"""
학습된 듀얼 뷰 모델을 ONNX 형식으로 변환
Unity Barracuda/Sentis에서 사용 가능
"""
import torch
import torch.onnx
from pathlib import Path
from models.driving_model import DualViewDrivingNet


def export_to_onnx(
    checkpoint_path: str,
    output_path: str,
    front_size: tuple = (66, 200),   # DrivingDataCollector 설정과 동일
    top_size: tuple = (128, 128),    # DrivingDataCollector 설정과 동일
):
    """
    PyTorch 모델을 ONNX로 변환

    Args:
        checkpoint_path: 학습된 체크포인트 경로 (.pth)
        output_path: ONNX 출력 경로 (.onnx)
        front_size: Front View 이미지 크기 (H, W)
        top_size: Top View 이미지 크기 (H, W)
    """
    print(f"Loading checkpoint: {checkpoint_path}")

    # 모델 로드
    model = DualViewDrivingNet(
        backbone="resnet18",
        pretrained=False,
        dropout=0.0  # 추론 시 dropout 비활성화
    )

    checkpoint = torch.load(checkpoint_path, map_location='cpu')
    model.load_state_dict(checkpoint['model_state_dict'])
    model.eval()

    print(f"Model loaded (epoch {checkpoint['epoch']}, val_loss {checkpoint['val_loss']:.4f})")

    # 더미 입력 생성
    dummy_front = torch.randn(1, 3, front_size[0], front_size[1])
    dummy_top = torch.randn(1, 3, top_size[0], top_size[1])

    # ONNX 변환
    print(f"Exporting to ONNX: {output_path}")

    torch.onnx.export(
        model,
        (dummy_front, dummy_top),
        output_path,
        export_params=True,
        opset_version=12,
        do_constant_folding=True,
        input_names=['front_image', 'top_image'],
        output_names=['steering', 'throttle'],
        dynamic_axes={
            'front_image': {0: 'batch_size'},
            'top_image': {0: 'batch_size'},
            'steering': {0: 'batch_size'},
            'throttle': {0: 'batch_size'}
        }
    )

    print(f"ONNX export complete!")

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

    # ONNX Runtime으로 테스트
    try:
        import onnxruntime as ort
        import numpy as np

        session = ort.InferenceSession(output_path)

        # 테스트 추론
        front_np = np.random.randn(1, 3, front_size[0], front_size[1]).astype(np.float32)
        top_np = np.random.randn(1, 3, top_size[0], top_size[1]).astype(np.float32)

        outputs = session.run(None, {
            'front_image': front_np,
            'top_image': top_np
        })

        print(f"Test inference - Steering: {outputs[0][0]:.4f}, Throttle: {outputs[1][0]:.4f}")
        print("ONNX Runtime test: OK")
    except ImportError:
        print("(onnxruntime not installed, skipping runtime test)")
    except Exception as e:
        print(f"ONNX Runtime test warning: {e}")

    return output_path


if __name__ == "__main__":
    # 경로 설정
    checkpoint_dir = Path(__file__).parent / "checkpoints"
    checkpoint_path = checkpoint_dir / "driving_dual_bc.pth"

    # Unity Assets 폴더에 직접 저장
    unity_assets = Path(__file__).parent.parent / "Assets" / "Models" / "ONNX"
    unity_assets.mkdir(parents=True, exist_ok=True)
    output_path = unity_assets / "driving_dual_bc.onnx"

    if not checkpoint_path.exists():
        print(f"Error: Checkpoint not found: {checkpoint_path}")
        exit(1)

    export_to_onnx(
        checkpoint_path=str(checkpoint_path),
        output_path=str(output_path)
    )

    print(f"\nONNX model saved to: {output_path}")
    print("Unity에서 이 파일을 Barracuda/Sentis로 로드하여 사용하세요.")
