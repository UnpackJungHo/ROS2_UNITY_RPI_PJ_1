# CLAUDE.md

## 프로젝트 개요

Unity 기반 AMR(자율주행 모바일 로봇) 시뮬레이터 + ROS2 브리지 + 하이브리드 ML 파이프라인.

- **언어**: C# (Unity), Python (ML/ROS2)
- **학습 방식**: Imitation Learning (DAgger) + Residual RL (PPO)
- **핵심 의존성**: ML-Agents 3.0, ROS-TCP-Connector, Unity Sentis, YOLO

---

## 디렉토리 구조

```
Assets/Scripts/
  ROS/            - ROS2 토픽 Pub/Sub (VehicleCmdSubscriber, PolicyCameraPublisher 등)
  Reinforcement/  - ML-Agents 연동 (AutoDriverRLAgent, TrainTestModeSwitcher 등)
  Sensor/         - 센서 시뮬레이션 (Ultrasonic, Radar, CollisionWarning)
  Camera/         - 카메라 스트리밍 및 Policy 입력
  View/           - UI/시각화 (DrivingStatusUIController, AMRViewController)
  Learning/       - 데이터 수집 및 모방학습
python/
  ros/            - ROS2 Policy/Perception 노드 (policy_cmd_publisher.py 등)
  Regression/     - 모방학습 모델 (train_regression.py, export_onnx_regression.py)
  models/         - YOLO 등 사전학습 모델
  checkpoints/    - PyTorch 체크포인트 (130MB+)
config/
  AutoDriver.yaml - ML-Agents PPO 하이퍼파라미터
```

---

### 코드 작성 규칙

임의 보정 금지: 명시적 요청이 없는 한, 원본 데이터에 임의의 가중치나 보정치(예: 0.3, 20% 등)를 적용하지 말 것.
단계적 구현: 모든 기능을 한 번에 구현하지 말고 [원본 로직 구현 → 테스트 → 결과에 따른 보정] 순서로 진행할 것.
superpowers 관련 스킬은 사용을 명시해야만 실행함.

## 학습 vs 추론 구분 (핵심)

| 구분 | 모드 | policy_cmd_publisher.py |
|------|------|------------------------|
| **학습 중** | Train (Unity 내부 제어) | **사용 X** |
| **추론 시** | Test (외부 ROS 토픽) | **사용 O** |

---

### 유니티 환경 점검 규칙
유니티 mcp를 통해 직접 확인 후 대답할 것.


## 주요 명령어

### 추론 (conda activate driving)
```bash
# ROS2 연결: ros2unitytcp

# 신호등 인지
jazzy && conda activate driving && cd UnityProjects/ros2_unity_rpi/ros2_unity_autoDriver && python python/ros/traffic_light_detector.py --namespace /amr0

# 정지선 인지
jazzy && conda activate driving && cd UnityProjects/ros2_unity_rpi/ros2_unity_autoDriver && python python/ros/stop_line_detector.py --namespace /amr0

# 주행/Policy 노드 실행
jazzy && conda activate driving && cd UnityProjects/ros2_unity_rpi/ros2_unity_autoDriver && python python/ros/policy_cmd_publisher.py --namespace /amr0
```

### 강화학습 (conda activate mlagents_r21)
```bash
# 환경 준비 (기존 이어서)
cd /home/kjhz/UnityProjects/ros2_unity_rpi/ros2_unity_autoDriver && conda activate mlagents_r21 && jazzy

# 이어서 학습
mlagents-learn config/AutoDriver.yaml --run-id autodriver_manual_001 --resume --time-scale 3 --timeout-wait 86400

# 새 run 시작 (run-id 숫자 중복 금지)
mlagents-learn config/AutoDriver.yaml --run-id autodriver_manual_003 --time-scale 5 --timeout-wait 86400
```

### 모방학습 (conda activate driving)
```bash
conda activate driving

# 학습 (front_3 확정)
python python/Regression/train_regression.py --image_type front_3 --lr 3e-4 --backbone_lr 3e-5

# ONNX 내보내기
python python/Regression/export_onnx_regression.py \
  --checkpoint python/Regression/checkpoints/driving_regression_3_best_steer_mae.pth \
  --output Assets/Models/ONNX/driving_regression_3_best_steer_mae.onnx
```

### ROS2 토픽 확인
```bash
ros2 topic info /amr0/vehicle/cmd
ros2 topic pub /amr0/vehicle/cmd geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"
```

---

## Git 규칙

- **커밋 전 반드시 사용자 확인** 후 add/commit/push
- Conventional Commits 형식: `type(scope): summary`
- 커밋 본문은 bullet 최대 3개
- Unity Editor.log / Player.log에 Error/Exception 있으면 push 금지

---

## Python 환경

- 모든 Python 실행은 `conda activate driving` 이후에
- 테스트 시 환경 미활성화 상태 금지

---

## 참고 문서

- `PROJECT_CONTEXT.md` — 전체 아키텍처 및 런타임 설정 (한국어)
- `PROJECT_RULES.md` — Git 워크플로, 테스트 게이트, Python 환경 규칙
- `TOPIC_LIST.md` — UNITY-ROS2 관련 모든 토픽 리스트
- `docs/task.md` — 구현 태스크 체크리스트
- `docs/implementation_plan.md` — DAgger + 속도 인식 학습 계획

