# **가장 자세한 프로젝트 문서는 [https://unpackjungho.github.io/unity-autocar-doc/index.html](https://unpackjungho.github.io/unity-autocar-doc/index.html) 에 있습니다. 먼저 이 링크를 타고 자세하게 알아보세요.**

# ros2_unity_autoDriver

Unity 기반 AMR(자율주행 모바일 로봇) 시뮬레이터에 ROS2를 연결하고, 모방학습과 Residual RL을 결합해 perception부터 vehicle control까지 검증하는 하이브리드 자율주행 실험 프로젝트입니다.

이 저장소는 단순 주행 데모가 아니라 다음 흐름을 하나의 프로젝트 안에 묶습니다.

- Unity 물리 기반 차량 시뮬레이션
- ROS2 센서/제어 브리지
- 카메라 기반 모방학습 회귀 정책
- PPO 기반 residual reinforcement learning
- 신호등, 정지선, 충돌 위험을 포함한 안전 로직

## 프로젝트 목적

핵심 목표는 현실과 유사한 환경에서 자율주행 파이프라인을 단계적으로 검증하는 것입니다.

- 수동 주행 데이터를 수집해 base policy를 만든다.
- base policy 위에 residual RL을 얹어 환경 적응력을 높인다.
- Unity 센서와 ROS2 토픽을 통해 perception-control loop를 검증한다.
- Train 모드와 Test 모드를 분리해 학습 파이프라인과 외부 추론 파이프라인을 명확히 관리한다.

## 한눈에 보는 구조

### Train 모드

학습 중에는 Unity 내부 제어를 사용합니다.

1. `TrainTestModeSwitcher`가 Train 모드로 전환합니다.
2. `AutoDriverRLAgent`가 관측값을 수집합니다.
3. base 회귀 정책과 RL delta를 결합합니다.
4. 보상 계산과 종료 판정을 통해 PPO 학습을 진행합니다.

중요: 학습 중에는 `python/ros/policy_cmd_publisher.py`를 사용하지 않습니다.

### Test 모드

추론 시에는 외부 ROS2 정책 노드를 사용합니다.

1. Unity가 카메라, odom, ultrasonic, radar, traffic 관련 토픽을 발행합니다.
2. `python/ros/policy_cmd_publisher.py`가 base regression + residual RL 추론을 수행합니다.
3. 최종 제어 명령을 `/vehicle/cmd`로 발행합니다.
4. Unity `VehicleCmdSubscriber`가 이를 받아 차량에 적용합니다.

중요: 추론 시에는 `policy_cmd_publisher.py`가 핵심 제어 노드입니다.

## 시스템 구성

### Unity

- 차량 물리 및 구동 제어
- ROS-TCP-Connector 기반 ROS2 브리지
- Policy 전용 카메라와 일반 perception 카메라 발행
- Odometry, TF, IMU, LiDAR, Ultrasonic, Radar 센서 시뮬레이션
- RL observation publisher, reward provider, episode evaluator

### Python / ROS2

- `policy_cmd_publisher.py`: 외부 정책 추론 노드
- `traffic_light_detector.py`: 신호등 인지 노드
- `stop_line_detector.py`: 정지선 인지 노드
- `train_regression.py`: 모방학습 회귀 모델 학습
- `export_onnx_regression.py`: Unity/ROS 추론용 ONNX 변환

### Learning Pipeline

- 수동 주행 데이터 수집
- 이미지 기반 회귀 정책 학습
- ONNX export
- Unity 또는 ROS2 경로에서 정책 적용
- PPO 기반 residual RL로 추가 보정 학습

## 주요 기술 스택

- Unity `2022.3.62f3`
- ML-Agents `3.0.0-exp.1`
- ROS-TCP-Connector
- URDF Importer
- Unity Sentis `1.2.0-exp.2`
- ROS2 Jazzy 기반 실행 워크플로
- ONNX Runtime
- OpenCV

## 주요 토픽

이 프로젝트는 ROS2 토픽 중심으로 perception과 control을 연결합니다.

- 입력
  - `/camera/image_raw`
  - `/camera/policy/image_raw`
  - `/odom`
  - `/tf`
  - `/imu`
  - `/velodyne_points`
- 센서
  - `/ultrasonic`
  - `/ultrasonic/fl`, `/ultrasonic/fr`, `/ultrasonic/fc`, `/ultrasonic/rl`, `/ultrasonic/rr`, `/ultrasonic/rc`
  - `/radar`
  - `/radar/front`, `/radar/rear`
- 인지
  - `/traffic_light/state`
  - `/traffic_light/perception`
  - `/stop_line/state`
  - `/stop_line/perception`
- 제어/출력
  - `/vehicle/cmd`
  - `/policy/collision_warning`

전체 토픽 설명은 `TOPIC_LIST.md`와 외부 문서를 참고하세요.

## 빠른 실행

### 1. 추론 실행

ROS2 연결 후 `driving` 환경에서 perception과 policy 노드를 실행합니다.

```bash
jazzy && conda activate driving && cd /home/kjhz/UnityProjects/ros2_unity_rpi/ros2_unity_autoDriver && python python/ros/traffic_light_detector.py --namespace /amr0
```

```bash
jazzy && conda activate driving && cd /home/kjhz/UnityProjects/ros2_unity_rpi/ros2_unity_autoDriver && python python/ros/stop_line_detector.py --namespace /amr0
```

```bash
jazzy && conda activate driving && cd /home/kjhz/UnityProjects/ros2_unity_rpi/ros2_unity_autoDriver && python python/ros/policy_cmd_publisher.py --namespace /amr0
```

### 2. 강화학습 실행

강화학습은 `mlagents_r21` 환경에서 진행합니다.

```bash
cd /home/kjhz/UnityProjects/ros2_unity_rpi/ros2_unity_autoDriver && conda activate mlagents_r21 && jazzy
```

```bash
mlagents-learn config/AutoDriver.yaml --run-id autodriver_manual_001 --resume --time-scale 3 --timeout-wait 86400
```

```bash
mlagents-learn config/AutoDriver.yaml --run-id autodriver_manual_003 --time-scale 5 --timeout-wait 86400
```

주의:

- 새 학습 시작 시 `run-id`를 중복하지 마세요.
- 학습 중에는 외부 정책 노드를 붙이지 말고 Unity 내부 제어를 사용해야 합니다.

### 3. 모방학습 실행

```bash
conda activate driving
python python/Regression/train_regression.py --image_type front_3 --lr 3e-4 --backbone_lr 3e-5
```

```bash
python python/Regression/export_onnx_regression.py \
  --checkpoint python/Regression/checkpoints/driving_regression_3_best_steer_mae.pth \
  --output Assets/Models/ONNX/driving_regression_3_best_steer_mae.onnx
```

현재 프로젝트 기준으로는 `front_3` 입력 구성이 대표 이미지 타입입니다.

## 저장소 구조

```text
Assets/
  Scripts/
    Camera/          Policy/Perception 카메라 발행
    Learning/        데이터 수집 및 보조 학습 로직
    Reinforcement/   RL 에이전트, 보상, 관측, Train/Test 스위칭
    ROS/             ROS2 제어 명령 수신 및 네임스페이스 처리
    Sensor/          LiDAR, IMU, Radar, Ultrasonic, Odometry
    Testing/         회귀 정책 기반 주행 테스트
    View/            차량 시각화 및 선택 UI
python/
  ros/               외부 정책 및 perception 노드
  Regression/        모방학습 학습/배포 스크립트
  TrafficLight/      신호등 모델 학습 및 검증
  rl_setup/          ML-Agents 환경 준비 스크립트
config/
  AutoDriver.yaml    PPO 하이퍼파라미터
TrainingData/        수동 주행 수집 데이터
results/             RL 학습 결과와 체크포인트
```

## 운영 규칙

- Python 실행은 목적에 맞는 conda 환경에서 수행합니다.
  - 추론/인지/모방학습: `driving`
  - 강화학습: `mlagents_r21`
- 임의 보정값을 먼저 넣지 말고, 원본 로직 구현 후 테스트 결과를 보고 조정합니다.
- Train/Test 모드를 혼용하지 않는 것이 가장 중요합니다.
- 자세한 실행 절차, 토픽 맵, 센서 설정, 학습 절차는 상단 외부 문서를 우선 참고하세요.

## 참고 문서

- 외부 프로젝트 문서: https://unpackjungho.github.io/unity-autocar-doc/index.html
- 프로젝트 컨텍스트: `PROJECT_CONTEXT.md`
- 프로젝트 규칙: `PROJECT_RULES.md`
- ROS2 토픽 설명: `TOPIC_LIST.md`
- RL 설정: `config/AutoDriver.yaml`

## 문서 안내

이 README는 신규 참여자가 빠르게 구조를 파악하도록 만든 요약 문서입니다. 센서 스펙, 차량 세팅, Train/Test 운영 절차, 토픽 맵, 3D SLAM/맵핑, 향후 R&D 항목까지 포함한 상세 설명은 상단 링크의 공식 문서를 기준으로 보시면 됩니다.
