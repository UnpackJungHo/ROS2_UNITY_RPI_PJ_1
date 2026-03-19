# RL 리팩토링 계획: Observation 정리 + SafetyOverride 분리

> 작성일: 2026-03-19
> 대상 파일: `AutoDriverRLAgent.cs`, `config/AutoDriver.yaml`
> 기준: RL 씬 Inspector 실측값

---

## 배경

### 1. Observation 17D 중 9개가 상수/중복

현재 17D 관측 공간에서 IL 비활성, 신호등 null, stub 함수 등으로 인해
다수 차원이 **항상 동일한 값(상수)** 을 출력하여 정보량이 0이다.

### 2. SafetyOverride ↔ Terminal 충돌 루프

학습 중 `ApplySafetyOverride`가 에이전트의 action을 강제 덮어씀으로써
에이전트가 아닌 하드코딩 로직이 terminal을 유발.
PPO가 잘못된 action에 blame을 귀속하여 credit assignment가 파괴된다.

```
에이전트 action: [steer=0.3, accel=0.8]  "전진하겠다"
       ↓
ApplySafetyOverride: throttle=0, brake=1.0 (강제 덮어씀)
       ↓
차량 정지 0.7초 유지
       ↓
FailRiskStop → AddReward(-30)
```

PPO가 blame하는 것: 마지막 N스텝의 "탈출 시도" action
실제 원인: 그 이전에 벽에 접근한 action

---

## 현재 Inspector 실측값

### AutoDriverRLAgent

| 파라미터 | 값 |
|----------|-----|
| residualSteerScale | 1.0 |
| residualAccelScale | 1.0 |
| applyTrainingStabilityTuning | false |
| regressionDrivingController | null |
| enableSafetyOverride | **true** |
| successTerminalReward | **+30** |
| failureTerminalPenalty | **-30** |
| includeTrafficDecisionOneHot | true |
| allowReverse | true |

### RLEpisodeEvaluator

| 파라미터 | 값 |
|----------|-----|
| enableFinishLineTerminal | **true** |
| maxEpisodeSeconds | 0 (비활성) |
| dangerLevelThreshold | 5 (Brake) |
| dangerStopHoldSeconds | 0.7s |
| stuckTimeWindow | 20s |
| stuckGracePeriod | 5s |
| finishBonus | 20 (episodeScore 진단용) |
| baseFailurePenalty | 10 (episodeScore 진단용) |

### ProgressRewardProvider

| 파라미터 | 값 |
|----------|-----|
| progressRewardScale | 1.0 |
| reverseProgressPenaltyScale | 1.4 |
| zoneRewardWeight | **0.5** |
| headingRewardWeight | 0.3 |
| lateralRewardWeight | 0.2 |
| emergencyPenaltyPerSec | 1.2 |

---

## 리팩토링 1: Observation 공간 정리 (17D → 8D)

### 제거 대상 (코드에서 AddObservation 호출 삭제)

| 인덱스 | 항목 | 제거 사유 |
|--------|------|-----------|
| #1 | lateralErrorAbs | #2 signedLateralError에서 파생 가능 (중복) |
| #4 | progressRatio | `GetPathProgressRatio()` → 항상 0f 반환 (stub) |

### 비활성화 대상 (bool 플래그로 제어, 향후 재활성화 가능)

| 인덱스 | 항목 | 비활성화 사유 |
|--------|------|-------------|
| #7 | stopLineDistance | 현재 맵에 정지선 학습 미포함 |
| #11 | baseSteering | regressionDrivingController=null → 항상 0f |
| #12 | baseThrottle | regressionDrivingController=null → 항상 0f |
| #13 | trafficLightGo | 신호등 null → 항상 1f |
| #14 | trafficLightCaution | 신호등 null → 항상 0f |
| #15 | trafficLightStop | 신호등 null → 항상 0f |
| #16 | safetyOverrideLevel | #6 warningLevel과 동일 소스, 학습 중 Override 비활성이면 의미 없음 |

### 정리 후 활성 관측 (8D)

| 새 인덱스 | 원래 | 항목 | 범위 |
|-----------|------|------|------|
| 0 | #0 | speed | [-1, 1] |
| 1 | #2 | signedLateralError | [-1, 1] |
| 2 | #3 | headingError | [-1, 1] |
| 3 | #5 | ttc | [0, 1] |
| 4 | #6 | warningLevel | [0, 1] |
| 5 | #8 | currentSteer | [-1, 1] |
| 6 | #9 | currentThrottle | [-1, 1] |
| 7 | #10 | currentBrake | [0, 1] |

### 구현 방법

**AutoDriverRLAgent.cs:**

```csharp
// 제거: lateralErrorAbs, progressRatio의 AddObservation 호출 삭제

// 비활성화: 기존 includeTrafficDecisionOneHot과 동일 패턴의 bool 플래그 추가
[Header("Optional Observations")]
public bool includeStopLineDistance = false;       // #7
public bool includeBasePredictions = false;        // #11, #12
// includeTrafficDecisionOneHot 기존 플래그 재활용  // #13~#15
public bool includeSafetyOverrideLevel = false;    // #16
```

**config/AutoDriver.yaml:**

BehaviorParameters.VectorObservationSize를 코드에서 동적 계산하거나,
Inspector에서 수동으로 17 → 8로 변경. (ML-Agents는 VectorObservationSize를
코드의 AddObservation 호출 횟수와 일치시켜야 함)

**검증:**

- [ ] 학습 시작 시 ML-Agents 콘솔에 observation size mismatch 에러 없음 확인
- [ ] `read_console`로 컴파일 에러 없음 확인

---

## 리팩토링 2: SafetyOverride 학습/테스트 분리

### 현재 문제

```
학습 중 enableSafetyOverride=true
  → ApplySafetyOverride가 agent action 강제 덮어씀
  → 0.7초 후 FailRiskStop terminal(-30)
  → PPO가 Override 중의 무의미한 action에 blame 귀속
  → credit assignment 파괴
```

### 해결: 학습 시 SafetyOverride 비활성화

**AutoDriverRLAgent.cs — OnEpisodeBegin() 에서 자동 전환:**

```csharp
void AutoConfigureSafetyOverride()
{
    bool isTraining = Academy.Instance != null && Academy.Instance.IsCommunicatorOn;
    enableSafetyOverride = !isTraining;
}
```

- 학습 (Communicator 연결) → `enableSafetyOverride = false`
- 테스트/추론 (Communicator 없음) → `enableSafetyOverride = true`

### SafetyOverride 비활성 시 에이전트가 위험을 학습하는 경로

| 신호 | 역할 |
|------|------|
| obs[4] warningLevel (0~1) | 위험 상태를 직접 관측 |
| safetyPenalty step reward (-0.05 ~ -1.2/s) | 위험 접근 시 즉각 음수 보상 |
| FailCollision terminal (-30) | 실제 충돌 시 강한 패널티 |
| FailRiskStop terminal (-30) | 에이전트 자신이 정지한 경우에만 발동 (Override 강제 정지 아님) |

Override가 없어도 step reward(safetyPenalty)와 warningLevel 관측이
충분한 위험 회피 학습 신호를 제공한다.

### 부수 효과

- FailRiskStop: 더 이상 Override가 유발하지 않으므로 빈도 감소
- FailCollision: 빈도 약간 증가 가능 (시뮬 내에서 허용)
- FailStuck: Override 강제 정지로 인한 Stuck 경로 완전 제거

### 검증

- [ ] 학습 시작 시 콘솔에서 `enableSafetyOverride=false` 로그 확인
- [ ] 테스트 모드 전환 시 `enableSafetyOverride=true` 로그 확인
- [ ] FailRiskStop 빈도 감소, FailCollision 빈도가 비정상적으로 폭증하지 않는지 TensorBoard 확인

---

## 작업 순서

| 단계 | 작업 | 파일 |
|------|------|------|
| 1 | Observation 제거/비활성화 플래그 추가 | AutoDriverRLAgent.cs |
| 2 | CollectObservations()에서 제거/플래그 적용 | AutoDriverRLAgent.cs |
| 3 | Inspector VectorObservationSize 17→8 반영 | RL 씬 Inspector |
| 4 | SafetyOverride 자동 전환 로직 추가 | AutoDriverRLAgent.cs |
| 5 | 컴파일 확인 + 학습 시작 테스트 | Unity Console |
| 6 | TensorBoard로 reward/terminal 분포 확인 | 학습 10k steps 후 |

---

## 변경하지 않는 것

- `ApplySafetyOverride()` 로직 자체 (테스트/배포용으로 유지)
- `ProgressRewardProvider` reward 가중치 (이번 리팩토링 범위 밖)
- `RLEpisodeEvaluator` terminal 조건 (SafetyOverride 분리로 자연 해결)
- `config/AutoDriver.yaml` reward 스케일 (별도 튜닝 단계에서 진행)
