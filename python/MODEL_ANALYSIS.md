# Speed-Aware Dual-View Model 심층 분석 (Deep Dive)

이 문서는 `train_classification.py`에 구현된 `SpeedAwareDualViewNet` 모델의 아키텍처, 데이터 처리 흐름, 그리고 학습 알고리즘을 상세하게 설명합니다.

---

## 1. 모델 아키텍처 명 (Model Architecture Name)

이 모델은 학술적으로 널리 알려진 단일 모델(예: ResNet, VGG)이 아니라, 특수 목적을 위해 커스텀된 **"Speed-Aware Dual-View Network" (속도 인지형 이중 시점 네트워크)** 입니다.

기반이 되는 핵심 백본(Backbone) 네트워크는 **ResNet18** (또는 MobileNetV3)을 사용합니다. 이를 두 개 병렬로 배치하고, 속도 정보를 결합하는 **Late Fusion(후반 결합)** 구조를 가지고 있습니다.

---

## 2. 전체 데이터 흐름도 (Data Flow)

아래 다이어그램은 데이터가 입력되어 최종 예측(7개 클래스)이 나오기까지의 과정을 시각화한 것입니다.

```mermaid
graph TD
    subgraph INPUTS [입력 데이터]
        Front[전방 카메라 이미지<br/>(3x66x200)]
        Mask[차선 마스크 이미지<br/>(3x66x200)]
        Speed[현재 속도<br/>(Scalar)]
    end

    subgraph ENCODERS [특징 추출기 (Backbones)]
        Front -->|ResNet18| FrontFeat[전방 특징 벡터<br/>(512차원)]
        Mask -->|ResNet18| MaskFeat[마스크 특징 벡터<br/>(512차원)]
        
        Speed -->|Norm & FC| SpeedFeat[속도 특징 벡터<br/>(64차원)]
    end

    subgraph FUSION [특징 결합]
        FrontFeat & MaskFeat & SpeedFeat --> Concat[D = 512 + 512 + 64<br/>= 1088차원]
    end

    subgraph CLASSIFIER [분류기 (MLP Head)]
        Concat --> FC1[Linear (1088->512)]
        FC1 --> ReLU1[ReLU]
        ReLU1 --> Drop1[Dropout]
        Drop1 --> FC2[Linear (512->128)]
        FC2 --> ReLU2[ReLU]
        ReLU2 --> Drop2[Dropout]
        Drop2 --> FC3[Linear (128->7)]
    end

    subgraph OUTPUT [출력]
        FC3 --> Logits[Logits (7개 점수)]
        Logits --> Softmax[Softmax<br/>(확률 변환)]
        Softmax --> Final[최종 행동<br/>(Forward, Left, Right...)]
    end
```

---

## 3. 단계별 상세 알고리즘 분석

### Step 1: 입력 및 전처리 (Preprocessing)

모델에 들어가기 전, 데이터는 다음과 같은 전처리 과정을 거칩니다.

1.  **이미지 정규화 (Normalization)**
    *   **알고리즘**: `(Pixel - Mean) / Std`
    *   이미지 픽셀(0~255)을 0~1로 바꾼 뒤, ImageNet 데이터셋의 평균(Mean)과 표준편차(Std)를 이용해 정규화합니다. 이는 모델이 밝기나 대조에 덜 민감하게 하고 학습 수렴 속도를 높입니다.
2.  **속도 정규화 (Speed Normalization)**
    *   **코드**: `speed = current_speed / speed_normalize (5.0)`
    *   속도 값(예: 3.0 m/s)을 0~1 사이의 작은 값(0.6)으로 변환합니다. 신경망은 입력 값의 스케일이 다르면(이미지는 0~1인데 속도는 30이면) 학습이 불안정해지기 때문입니다.

### Step 2: 특징 추출 (Feature Extraction) - ResNet18

`front_encoder`와 `mask_encoder`는 각각 **ResNet18** 구조를 사용합니다.

*   **Convolution (합성곱)**: 이미지에서 선, 질감, 패턴 같은 특징을 찾아내는 필터입니다.
*   **Batch Normalization (BN)**: 레이어 간 데이터 분포를 일정하게 맞춰주어 "기울기 소실" 문제를 방지하고 학습 속도를 비약적으로 높입니다.
*   **ReLU (Rectified Linear Unit)**: `max(0, x)` 함수입니다. 음수를 0으로 만들어 비선형성을 추가합니다. 이것이 있어야 신경망이 복잡한 문제(휘어진 차선 등)를 해결할 수 있습니다.
*   **Residual Connection (잔차 연결)**: ResNet의 핵심입니다. `Output = Conv(Input) + Input` 형태로, 입력값을 출력에 더해줍니다. 층이 깊어져도 학습 정보(Gradient)가 잘 전달되게 합니다.
*   **Global Average Pooling**: 마지막에 이미지의 공간 정보(가로x세로)를 평균내어 하나의 1차원 벡터(512개 숫자)로 만듭니다.

### Step 3: 속도 임베딩 (Speed Embedding)

단순한 숫자 하나인 '속도'를 이미지 특징(512개)과 대등하게 결합하기 위해 뻥튀기(Embedding) 합니다.

*   **구조**: `Linear(1->32)` → `ReLU` → `Linear(32->64)` → `ReLU`
*   **이유**: 단순히 숫자 하나를 붙이는 것보다, 비선형 변환을 거쳐 "빠름", "적당함", "느림" 같은 추상적 특징으로 변환한 뒤 결합하는 것이 효과적입니다.

### Step 4: 분류기 (Classifier Head) - MLP

모든 특징을 합친 1088개의 숫자를 보고 최종 판단을 내리는 부분입니다.

1.  **Linear (행렬 곱)**: 입력 특징들에 가중치(Line Learning)를 곱해 정보를 압축/변환합니다.
2.  **Dropout (드롭아웃)**: 학습 시에 랜덤하게 뉴런의 50%(0.5)를 꺼버립니다. 모델이 특정 뉴런에만 의존하는 것을 막아 **과적합(Overfitting)**을 강력하게 방지합니다.
3.  **최종 출력**: 마지막에 7개의 숫자(Logits)가 나옵니다. 이 숫자가 클수록 해당 행동일 확률이 높다는 뜻입니다.

---

## 4. 학습 과정 (Training Process)

### 최적화 알고리즘: AdamW (Adam + Weight Decay)

이 코드는 **AdamW** 옵티마이저를 사용합니다.

*   **Adam (Adaptive Moment Estimation)**:
    *   Gradient(기울기)의 "방향(Momentum)"과 "보폭(Adaptive Learning Rate)"을 동시에 고려하는 매우 똑똑한 알고리즘입니다.
    *   SGD(확률적 경사 하강법)보다 훨씬 빠르고 안정적으로 정답을 찾아갑니다.
*   **Weight Decay (가중치 감쇠)**:
    *   학습된 가중치(Weight) 값들이 너무 커지지 않도록 강제로 조금씩 줄입니다.
    *   이는 모델이 너무 복잡해지는 것을 막아, 본 적 없는 길(Test Data)에서도 잘 주행하게 만듭니다.

### 손실 함수: CrossEntropyLoss

모델이 얼마나 틀렸는지 채점하는 함수입니다.

1.  **Softmax**: 모델의 출력(Logits)을 확률(총합 1)로 변환합니다. 예: `[2.0, 1.0, 0.1]` -> `[0.7, 0.2, 0.1]`
2.  **NLLLoss (Negative Log Likelihood)**: 정답 클래스의 확률에 로그를 취하고 부호를 바꿉니다. 정답 확률이 1에 가까울수록 Loss는 0이 되고, 0에 가까울수록 Loss는 무한대로 커집니다.
3.  **Class Weights (클래스 가중치)**:
    *   직진 데이터는 많고 좌회전 데이터는 적을 경우, 모델은 무조건 "직진"이라고 찍으려 합니다.
    *   이를 막기 위해 **데이터가 적은 클래스를 맞췄을 때 더 큰 점수(상)를 주고, 틀렸을 때 더 큰 벌점(Loss)**을 줍니다. (`use_class_weights=True` 옵션)
