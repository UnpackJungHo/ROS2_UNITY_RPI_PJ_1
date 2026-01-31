# python_test_2.py
"""
ONNX 모델 추론 테스트 스크립트

실제 학습 이미지로 ONNX 모델의 예측 결과를 확인합니다.
"""

import onnxruntime as ort
import numpy as np
from PIL import Image
import glob
from pathlib import Path

# 클래스 이름
CLASS_NAMES = ['FORWARD', 'FORWARD_LEFT', 'FORWARD_RIGHT', 'LEFT', 'RIGHT', 'BACKWARD', 'NONE']

# ImageNet 정규화 값
MEAN = np.array([0.485, 0.456, 0.406])
STD = np.array([0.229, 0.224, 0.225])


def preprocess_image(image_path, size):
    """이미지를 로드하고 전처리"""
    img = Image.open(image_path).convert('RGB').resize((size[1], size[0]))  # (width, height)
    img_np = np.array(img) / 255.0
    img_norm = (img_np - MEAN) / STD
    img_tensor = img_norm.transpose(2, 0, 1).astype(np.float32)  # CHW 형식
    return img_tensor[np.newaxis]  # 배치 차원 추가


def test_onnx_model():
    # ONNX 모델 경로
    onnx_path = Path(__file__).parent.parent / "Assets" / "Models" / "ONNX" / "driving_classifier_v2_single.onnx"
    
    if not onnx_path.exists():
        print(f"Error: ONNX 파일을 찾을 수 없습니다: {onnx_path}")
        return
    
    print(f"=== ONNX 모델 테스트 ===")
    print(f"모델: {onnx_path.name}")
    print()
    
    # ONNX Runtime 세션 생성
    session = ort.InferenceSession(str(onnx_path))
    
    # 입력 정보 출력
    print("입력 텐서:")
    for inp in session.get_inputs():
        print(f"  {inp.name}: {inp.shape}")
    print()
    
    # 세션 폴더 찾기
    data_base = Path(__file__).parent.parent / "TrainingDataV2"
    sessions = sorted(glob.glob(str(data_base / "session_*")))
    
    if not sessions:
        print("Error: 세션 데이터를 찾을 수 없습니다!")
        return
    
    # 첫 번째 세션에서 테스트
    test_session = Path(sessions[0])
    front_files = sorted(glob.glob(str(test_session / "front" / "*.jpg")))[:10]
    
    print(f"테스트 세션: {test_session.name}")
    print(f"테스트 이미지: {len(front_files)}개")
    print("-" * 50)
    
    predictions = {}
    
    for fp in front_files:
        # 이미지 전처리
        front_np = preprocess_image(fp, (66, 200))
        
        top_path = fp.replace('/front/', '/top/')
        top_np = preprocess_image(top_path, (128, 128))
        
        # 속도 (정규화된 값)
        speed_np = np.array([[0.6]], dtype=np.float32)  # 3m/s / 5.0
        
        # 추론
        outputs = session.run(None, {
            'front_image': front_np,
            'top_image': top_np,
            'speed': speed_np
        })
        
        logits = outputs[0][0]
        pred_class = np.argmax(logits)
        pred_name = CLASS_NAMES[pred_class]
        
        predictions[pred_name] = predictions.get(pred_name, 0) + 1
        
        # 개별 결과 출력
        frame_name = Path(fp).stem
        print(f"{frame_name}: {pred_name:15s} | logits: {logits.round(2)}")
    
    # 요약
    print("-" * 50)
    print("예측 분포:")
    for name, count in sorted(predictions.items(), key=lambda x: -x[1]):
        print(f"  {name}: {count}")


def test_random_input():
    """랜덤 입력으로 모델 테스트"""
    onnx_path = Path(__file__).parent.parent / "Assets" / "Models" / "ONNX" / "driving_classifier_v2_single.onnx"
    
    if not onnx_path.exists():
        print(f"Error: ONNX 파일을 찾을 수 없습니다!")
        return
    
    print("=== 랜덤 입력 테스트 ===")
    
    session = ort.InferenceSession(str(onnx_path))
    
    for i in range(5):
        front_np = np.random.randn(1, 3, 66, 200).astype(np.float32)
        top_np = np.random.randn(1, 3, 128, 128).astype(np.float32)
        speed_np = np.array([[0.5]], dtype=np.float32)
        
        outputs = session.run(None, {
            'front_image': front_np,
            'top_image': top_np,
            'speed': speed_np
        })
        
        logits = outputs[0][0]
        pred = CLASS_NAMES[np.argmax(logits)]
        print(f"Test {i+1}: {pred:15s} | logits: {logits.round(2)}")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="ONNX 모델 추론 테스트")
    parser.add_argument("--random", action="store_true", help="랜덤 입력으로 테스트")
    args = parser.parse_args()
    
    if args.random:
        test_random_input()
    else:
        test_onnx_model()
