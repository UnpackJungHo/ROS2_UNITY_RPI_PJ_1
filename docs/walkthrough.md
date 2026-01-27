# Speed-Aware DAgger 디버깅 보고서

## 1. 초기 증상

**문제:** AI 자율주행 모드에서 **FWD+RIGHT 100%** 예측만 반복

![Unity 스크린샷](/home/kjhz/.gemini/antigravity/brain/e360f2d0-a3bb-4b46-a583-39ba61622ff7/uploaded_media_1769520980363.png)

---

## 2. 가설 1: 클래스 불균형 문제

### 2.1 데이터 분석

```bash
cd TrainingDataV2 && find . -name "driving_log.csv" -exec sh -c \
'cut -d, -f3 {} | sort | uniq -c | sort -rn' \;
```

**결과:**
```
FORWARD(0): ~40%
RIGHT(4): ~17%
NONE(6): ~38%
FORWARD_RIGHT(2): ~0.5%  ← 매우 희귀한 클래스
```

### 2.2 시도한 수정

- `use_weighted_sampling=False` → 희귀 클래스 과도 샘플링 방지
- `use_class_weights=False` → 손실 함수 가중치 제거

### 2.3 결과

학습 결과는 개선됨 (val_loss: 0.95), 하지만 Unity에서 **여전히 FWD+RIGHT 100%**

---

## 3. 가설 2: Python 모델 자체 문제

### 3.1 랜덤 입력 테스트

```python
# 랜덤 텐서로 모델 테스트
for i in range(10):
    front = torch.randn(1, 3, 66, 200)
    top = torch.randn(1, 3, 128, 128)
    speed = torch.tensor([[0.6]])
    logits = model(front, top, speed)
    pred = torch.argmax(logits, dim=1).item()
    print(CLASS_NAMES[pred])
```

**결과:** 모두 NONE 예측 ← **랜덤 입력에서는 작동**

### 3.2 실제 이미지 테스트

```python
# 학습 데이터 이미지로 테스트
front_img = Image.open('session_xxx/front/frame_000000.jpg')
# 전처리 후 모델 추론
```

**결과:** **FORWARD 20개 정확 예측!** ← Python 모델은 정상

---

## 4. 가설 3: ONNX 변환 문제

### 4.1 ONNX Runtime 테스트

```python
session = ort.InferenceSession('driving_classifier_v2.onnx')
outputs = session.run(None, {
    'front_image': front_np,
    'top_image': top_np,
    'speed': np.array([[0.6]], dtype=np.float32)
})
logits = outputs[0][0]
print(f'Logits: {logits.round(2)}')
```

**결과:**
```
Logits: [7.22, -5.75, -6.62, -5.36, -7.16, -24.24, 5.16]
Predicted: FORWARD  ← ONNX 모델도 정상!
```

---

## 5. 가설 4: Unity Sentis 입력 문제

### 5.1 Unity에서 logits 디버깅

```csharp
// C# 코드 추가
string logitsStr = "";
for (int i = 0; i < 7; i++)
    logitsStr += $"{cpuTensor[i]:F2}, ";
Debug.Log($"[AI] Logits: [{logitsStr}]");
```

**결과:**
```
Unity:  [-154, 229, 792, 576, -367, -193, 242]   ← 100배 큼!
Python: [7.22, -5.75, -6.62, ...]               ← 정상
```

### 5.2 픽셀 값 확인

```csharp
Debug.Log($"First pixel RGB: ({pixels[0].r:F3}, {pixels[0].g:F3}, {pixels[0].b:F3})");
```

**결과:** `(0.357, 0.361, 0.357)` ← 0-1 범위로 정상

### 5.3 정규화된 텐서 값 확인

```csharp
var frontData = frontInputTensor.DownloadToArray();
Debug.Log($"Normalized: R={frontData[0]:F3}, G={frontData[66*200]:F3}");
```

**결과:** `R=-0.080, G=0.520, B=1.403` ← 정규화도 정상

---

## 6. 근본 원인 발견

### 6.1 ONNX 파일 구조 확인

```bash
ls -la Assets/Models/ONNX/
```

**결과:**
```
driving_classifier_v2.onnx       247 KB  ← 메인 파일
driving_classifier_v2.onnx.data  87 MB   ← 외부 weights 파일!
```

**문제:** PyTorch의 새 ONNX exporter가 weights를 별도 `.data` 파일로 분리함. **Unity Sentis가 이 외부 파일을 제대로 읽지 못함!**

---

## 7. 최종 해결책

### 7.1 단일 파일 ONNX 내보내기

```python
torch.onnx.export(
    model,
    (dummy_front, dummy_top, dummy_speed),
    output_path,
    export_params=True,
    opset_version=14,
    do_constant_folding=True,
    input_names=['front_image', 'top_image', 'speed'],
    output_names=['logits'],
    dynamo=False,  # 기존 방식 사용 → 단일 파일 생성
)
```

**결과:**
```
driving_classifier_v2_single.onnx  87.6 MB  ← 단일 파일, weights 내장
Test logits: [3.72, -4.28, -5.26, 0.45, -2.74, -13.8, 4.47]  ← 정상!
```

### 7.2 Unity 설정 변경

Model Asset을 `driving_classifier_v2_single.onnx`로 변경

---

## 8. 최종 결과

- **급커브 구간 제외** 정상 주행 성공
- Python과 Unity의 logits 값이 일치

---

## 9. 요약

| 단계 | 가설 | 테스트 방법 | 결과 |
|------|------|-------------|------|
| 1 | 클래스 불균형 | weighted sampling 비활성화 | ❌ 해결 안됨 |
| 2 | Python 모델 문제 | PyTorch 직접 추론 | ✅ 정상 |
| 3 | ONNX 변환 문제 | ONNX Runtime 테스트 | ✅ 정상 |
| 4 | Unity 입력 문제 | 픽셀/텐서 값 로깅 | ✅ 정상 |
| 5 | **ONNX 외부 데이터** | 단일 파일로 재변환 | ✅ **해결!** |

**핵심 교훈:** PyTorch 2.x의 새 ONNX exporter는 큰 모델을 외부 `.data` 파일로 분리하는데, Unity Sentis는 이를 제대로 지원하지 않음. `dynamo=False` 옵션으로 기존 방식 사용 필요.
