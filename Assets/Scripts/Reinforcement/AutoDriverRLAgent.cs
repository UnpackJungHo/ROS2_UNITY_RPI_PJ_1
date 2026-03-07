using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

/// <summary>
/// Residual RL Agent: 모방학습(Regression) 출력 위에 보정값(delta)을 학습.
///
/// 최종 제어 = clamp(base + delta)
///   base: RegressionDrivingController의 ONNX 추론 결과
///   delta: RL 정책이 출력하는 보정값
///
/// Continuous action space (2):
///   0: delta_steering [-1, 1] (실제 효과는 residualScale로 제한)
///   1: delta_accel    [-1, 1]
///
/// Observation space (16, includeTrafficDecisionOneHot=true):
///   0-7:   환경 상태 (speed, lateral, heading, progress, ttc, warning, stopLine)
///   8-10:  현재 차량 입력 (steer, throttle, brake)
///   11-12: 모방학습 base 예측 (baseSteering, baseThrottle)
///   13-15: 신호등 one-hot (Go, Caution, Stop)
/// </summary>
public class AutoDriverRLAgent : Agent
{
    [Header("References")]
    public WheelTest wheelController;
    public ProgressRewardProvider progressRewardProvider;
    public RLEpisodeEvaluator episodeEvaluator;
    public CollisionWarningPublisher collisionWarningPublisher;
    public TrafficLightDecisionEngine trafficLightDecisionEngine;
    public RegressionDrivingController regressionDrivingController;
    public VehicleCmdSubscriber vehicleCmdSubscriber;
    public DecisionRequester decisionRequester;

    [Header("Setup")]
    public bool autoFindReferences = true;
    [Tooltip("DecisionRequester가 없을 때 FixedUpdate마다 RequestDecision 호출")]
    public bool requestDecisionInFixedUpdateWithoutRequester = false;
    [Tooltip("할당하면 Agent Transform이 이 타겟(base_link 등)을 추적")]
    public Transform followTargetTransform;
    public bool followTargetPosition = true;
    public bool followTargetRotation = true;
    public bool resetVehicleTransformOnEpisodeBegin = true;
    public bool resetRootArticulationVelocity = true;
    [Tooltip("미할당 시 Agent 현재 위치/회전을 시작 포즈로 사용")]
    public Transform episodeStartTransform;
    [Tooltip("episodeStartTransform 사용 시 Y축 높이도 그대로 쓸지 여부 (false면 차량 현재 높이 유지)")]
    public bool useEpisodeStartTransformHeight = false;
    [Tooltip("episodeStartTransform 사용 시 회전도 그대로 쓸지 여부 (false면 차량 현재 회전 유지)")]
    public bool useEpisodeStartTransformRotation = true;

    [Header("Residual RL")]
    [Tooltip("RL 보정값의 최대 크기 (steering)")]
    [Range(0.01f, 1f)] public float residualSteerScale = 0.3f;
    [Tooltip("RL 보정값의 최대 크기 (accel)")]
    [Range(0.01f, 1f)] public float residualAccelScale = 0.3f;

    [Header("Training Stability Tuning")]
    [Tooltip("학습 안정화를 위해 residual/safety 파라미터를 권장 범위로 자동 보정")]
    public bool applyTrainingStabilityTuning = true;
    [Range(0.12f, 0.2f)] public float tunedResidualSteerScale = 0.16f;
    [Range(0.1f, 0.15f)] public float tunedResidualAccelScale = 0.12f;
    [Range(0.1f, 0.2f)] public float tunedWarningBrake = 0.15f;

    [Header("Action Mapping")]
    [Tooltip("false면 음수 accel은 브레이크로 처리")]
    public bool allowReverse = false;
    public bool enableSafetyOverride = true;
    [Range(0f, 1f)] public float cautionThrottleScale = 0.55f;
    [Range(0f, 1f)] public float warningBrake = 0.35f;
    [Range(0f, 1f)] public float brakeLevelBrake = 0.8f;
    [Range(0f, 1f)] public float emergencyBrake = 1f;

    [Tooltip("Warning 단계에서 brake를 적용할 최소 속도 (m/s). 저속에서는 크리핑을 허용")]
    public float warningBrakeMinSpeed = 0.6f;


    [Header("Control Source")]
    [Tooltip("true면 외부 ROS cmd 토픽만 사용하고, Agent/Regression의 내부 제어 출력을 차량에 적용하지 않음")]
    public bool externalRosCmdInputOnly = false;
    [Tooltip("학습 Communicator가 연결되면 externalRosCmdInputOnly를 무시하고 내부 Residual RL 제어를 강제 사용")]
    public bool forceInternalControlWhenTraining = true;

    [Header("Observation Normalization")]
    public float speedNormalize = 3f;
    public float lateralErrorNormalize = 2f;
    public float headingErrorNormalizeDeg = 45f;
    public float ttcNormalizeSeconds = 8f;
    public float stopLineDistanceNormalize = 30f;
    public bool includeTrafficDecisionOneHot = true;

    [Header("Terminal Reward Shaping")]
    public float successTerminalReward = 1f;
    public float failureTerminalPenalty = -1f;

    [Header("Debug (Read Only)")]
    [SerializeField] private float lastBaseSteering = 0f;
    [SerializeField] private float lastBaseThrottle = 0f;
    [SerializeField] private float lastDeltaSteering = 0f;
    [SerializeField] private float lastDeltaAccel = 0f;
    [SerializeField] private float lastAppliedSteer = 0f;
    [SerializeField] private float lastAppliedThrottle = 0f;
    [SerializeField] private float lastAppliedBrake = 0f;
    [SerializeField] private float lastConsumedStepReward = 0f;
    [SerializeField] private float lastHeadingErrorDeg = 0f;
    [SerializeField] private float lastSignedLateralError = 0f;
    [SerializeField] private string lastTerminalReason = "None";
    [SerializeField] private bool runtimeForceInternalControlActive = false;

    private Vector3 startPosition;           // 에피소드 시작 위치 캐시
    private Quaternion startRotation;        // 에피소드 시작 회전 캐시
    private bool hasStartPose = false;       // 시작 포즈가 캐시되었는지 여부
    private ArticulationBody rootArticulation; // 차량 루트 ArticulationBody (물리 리셋용)
    private RLEpisodeEvaluator subscribedEvaluator; // 현재 구독 중인 에피소드 평가기 (중복 구독 방지)
    private bool agentInitialized = false;   // Initialize() 완료 여부 플래그

    // ──────────────────────────────────────────────
    //  ML-Agents 라이프사이클 오버라이드
    // ──────────────────────────────────────────────

    /// <summary>
    /// Agent 최초 초기화. 레퍼런스 탐색, 학습 안정화 튜닝, 시작 포즈 캐시,
    /// 에피소드 종료 이벤트 구독, Residual 모드 활성화를 순서대로 수행한다.
    /// </summary>
    public override void Initialize()
    {
        // 1) Inspector에 미할당된 컴포넌트를 자동으로 탐색하여 연결
        if (autoFindReferences)
            AutoFindReferences();

        // 2) 학습 안정화를 위해 residual/safety 파라미터를 권장 범위로 클램프
        ApplyTrainingStabilityTuning();

        // 3) 에피소드 시작 위치/회전을 캐시해 두어 리셋 시 사용
        CacheStartPose();

        // 4) EpisodeEvaluator의 종료 이벤트에 구독 (에피소드 종료 시 보상 부여 + EndEpisode 호출)
        RefreshTerminalSubscription();

        // 5) 모방학습 컨트롤러를 predictionOnly 모드로 전환하여 RL이 제어 우선권을 가짐
        EnableResidualMode();

        // 6) 초기화 완료 플래그 설정 (종료 콜백에서 초기화 전 호출 방지용)
        agentInitialized = true;
    }

    /// <summary>
    /// GameObject가 활성화될 때 호출. 에피소드 종료 이벤트 구독을 갱신한다.
    /// </summary>
    protected override void OnEnable()
    {
        base.OnEnable();
        RefreshTerminalSubscription();
    }

    /// <summary>
    /// GameObject가 비활성화될 때 호출. 에피소드 종료 이벤트 구독을 해제한다.
    /// </summary>
    protected override void OnDisable()
    {
        UnsubscribeTerminalEvent();
        base.OnDisable();
    }

    /// <summary>
    /// GameObject가 파괴될 때 호출. 이벤트 구독을 해제하여 메모리 누수를 방지한다.
    /// </summary>
    void OnDestroy()
    {
        UnsubscribeTerminalEvent();
    }

    /// <summary>
    /// 매 물리 프레임마다 호출.
    /// - Agent Transform을 followTarget에 동기화
    /// - 외부 ROS 입력 모드일 때 관련 플래그를 강제 설정
    /// - DecisionRequester 없이 매 프레임 결정 요청이 필요한 경우 RequestDecision 호출
    /// </summary>
    void FixedUpdate()
    {
        // Agent Transform을 followTarget(base_link 등)의 위치/회전으로 동기화
        SyncToFollowTarget();

        // 외부 ROS cmd 모드가 활성화된 경우: 모방학습을 예측 전용으로, 휠 컨트롤러를 외부 제어 모드로 설정
        if (IsExternalRosCmdInputActive())
        {
            if (regressionDrivingController != null && !regressionDrivingController.predictionOnlyMode)
                regressionDrivingController.predictionOnlyMode = true;

            if (wheelController != null && !wheelController.externalControlEnabled)
                wheelController.externalControlEnabled = true;
        }

        // 외부 입력 비활성 & DecisionRequester 없을 때: 수동으로 매 FixedUpdate 결정 요청
        if (!IsExternalRosCmdInputActive() &&
            requestDecisionInFixedUpdateWithoutRequester &&
            decisionRequester == null)
            RequestDecision();
    }

    /// <summary>
    /// 에피소드 시작 시 호출. 차량 상태를 리셋하고, 모방학습 컨트롤러와
    /// 에피소드 평가기를 초기화하여 새 에피소드를 준비한다.
    /// </summary>
    public override void OnEpisodeBegin()
    {
        // 1) 레퍼런스 자동 탐색 (런타임에 컴포넌트가 변경되었을 수 있으므로 재탐색)
        if (autoFindReferences)
            AutoFindReferences();

        // 2) 학습 안정화 파라미터 재적용
        ApplyTrainingStabilityTuning();

        // 3) 에피소드 종료 이벤트 구독 갱신
        RefreshTerminalSubscription();

        // 4) Agent Transform 동기화
        SyncToFollowTarget();

        // 5) 차량 위치/회전/속도를 시작 포즈로 리셋
        if (resetVehicleTransformOnEpisodeBegin)
        {
            // Residual RL 모드에서는 시작 회전을 강제 적용해야 모방학습이 올바르게 동작
            bool forceStartRotationForResidualRl =
                episodeStartTransform != null &&
                regressionDrivingController != null &&
                regressionDrivingController.predictionOnlyMode;

            // 회전 리셋 설정에 대한 경고 로그 출력
            if (episodeStartTransform != null && !useEpisodeStartTransformRotation)
            {
                if (forceStartRotationForResidualRl)
                {
                    Debug.Log(
                        "[AutoDriverRLAgent] useEpisodeStartTransformRotation=False 이지만 " +
                        "Residual RL 모드에서는 시작 회전을 강제 적용합니다."
                    );
                }
                else
                {
                    Debug.LogWarning(
                        "[AutoDriverRLAgent] episodeStartTransform은 지정되었지만 회전 리셋이 꺼져 있습니다. " +
                        "재시작 시 실패 시점 헤딩이 유지되어 시작 직후 교착이 발생할 수 있습니다."
                    );
                }
            }
            // 실제 차량 위치/회전/속도 리셋 수행
            ResetVehicleState();
        }

        // 6) 모방학습을 predictionOnly 모드로, RL이 제어 우선권을 갖도록 설정
        EnableResidualMode();

        // 7) 모방학습 컨트롤러의 내부 상태를 리셋
        //    forceInference=false: TeleportRoot 직후 카메라가 아직 이전 위치를 가리킬 수 있으므로
        //    즉시 추론하지 않고 다음 Update()에서 물리 정착 후 추론
        if (regressionDrivingController != null)
        {
            Debug.Log($"[RLAgent.OnEpisodeBegin] BEFORE ResetForEpisodeRestart: base steer={regressionDrivingController.GetPredictedSteering():F4}, throttle={regressionDrivingController.GetPredictedThrottle():F4}");
            regressionDrivingController.ResetForEpisodeRestart(forceInference: false);
            Debug.Log($"[RLAgent.OnEpisodeBegin] AFTER  ResetForEpisodeRestart: base steer={regressionDrivingController.GetPredictedSteering():F4}, throttle={regressionDrivingController.GetPredictedThrottle():F4}");
        }

        // 8) 에피소드 평가기 또는 보상 제공자의 상태를 리셋
        //    에피소드 리셋의 소유권을 단일화하여 이중 리셋과 인덱스 드리프트를 방지
        if (episodeEvaluator != null)
        {
            // 에피소드가 비활성이거나 이미 종료 상태면 새 에피소드 시작
            if (!episodeEvaluator.IsEpisodeActive() || episodeEvaluator.IsTerminalReached())
                episodeEvaluator.BeginEpisode();
        }
        else if (progressRewardProvider != null)
        {
            // 평가기가 없으면 보상 제공자만 직접 리셋
            progressRewardProvider.ResetRewardState();
        }

        // 9) 디버그 상태 초기화
        lastTerminalReason = "None";
        lastConsumedStepReward = 0f;
    }

    /// <summary>
    /// 매 스텝마다 환경 관측값(observation)을 수집하여 정책 네트워크에 전달한다.
    /// 총 16개의 연속 관측값: 환경 상태(8) + 현재 차량 입력(3) + base 예측(2) + 신호등 one-hot(3)
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        // Agent Transform을 물리 객체에 동기화 (관측 전 최신 위치 보장)
        SyncToFollowTarget();

        // ─── 환경 상태 관측 (인덱스 0~7) ───

        // 현재 차량 속도 (m/s)
        float speed = wheelController != null ? wheelController.GetSpeedMS() : 0f;
        // 경로 대비 횡방향 오차 (절대값)
        float lateralErrorAbs = progressRewardProvider != null ? progressRewardProvider.GetCurrentLateralError() : 0f;
        // 경로 진행률 (0~1)
        float progressRatio = progressRewardProvider != null ? progressRewardProvider.GetPathProgressRatio() : 0f;
        // 충돌까지 남은 시간 (TTC, seconds)
        float ttc = collisionWarningPublisher != null
            ? collisionWarningPublisher.GetTimeToCollision()
            : float.PositiveInfinity;
        // 충돌 경고 레벨 정규화 (0~1, 6단계 기준)
        float warningLevelNorm = collisionWarningPublisher != null
            ? Mathf.Clamp01((int)collisionWarningPublisher.GetWarningLevel() / 6f)
            : 0f;
        // 정지선까지 거리
        float stopLineDistance = trafficLightDecisionEngine != null
            ? trafficLightDecisionEngine.GetDecisionDistance()
            : float.PositiveInfinity;

        // 경로 대비 헤딩 오차(도)와 부호 있는 횡방향 오차 계산
        float headingErrorDeg = ComputeHeadingErrorDeg(out float signedLateralError);
        lastHeadingErrorDeg = headingErrorDeg;
        lastSignedLateralError = signedLateralError;

        // 관측값을 정규화하여 센서에 추가
        sensor.AddObservation(NormalizeSigned(speed, speedNormalize));                    // 0: 속도 (부호 있는 정규화)
        sensor.AddObservation(Normalize01(lateralErrorAbs, lateralErrorNormalize, 0f));   // 1: 횡방향 오차 절대값 (0~1)
        sensor.AddObservation(NormalizeSigned(signedLateralError, lateralErrorNormalize)); // 2: 횡방향 오차 부호 포함 (-1~1)
        sensor.AddObservation(NormalizeSigned(headingErrorDeg, headingErrorNormalizeDeg)); // 3: 헤딩 오차 (-1~1)
        sensor.AddObservation(Mathf.Clamp01(progressRatio));                              // 4: 경로 진행률 (0~1)
        sensor.AddObservation(Normalize01(ttc, ttcNormalizeSeconds, 1f));                  // 5: TTC (0~1, 무한대→1)
        sensor.AddObservation(warningLevelNorm);                                           // 6: 충돌 경고 레벨 (0~1)
        sensor.AddObservation(Normalize01(stopLineDistance, stopLineDistanceNormalize, 1f)); // 7: 정지선 거리 (0~1)

        // ─── 현재 차량 입력 관측 (인덱스 8~10) ───

        float currentSteer = wheelController != null ? wheelController.GetSteeringInput() : 0f;
        float currentThrottle = wheelController != null ? wheelController.GetThrottleInput() : 0f;
        float currentBrake = wheelController != null ? wheelController.GetBrakeInput() : 0f;
        sensor.AddObservation(Mathf.Clamp(currentSteer, -1f, 1f));                        // 8: 현재 조향 입력
        sensor.AddObservation(Mathf.Clamp(currentThrottle, -1f, 1f));                     // 9: 현재 스로틀 입력
        sensor.AddObservation(Mathf.Clamp01(currentBrake));                                // 10: 현재 브레이크 입력

        // ─── 모방학습 base 예측값 관측 (인덱스 11~12) ───

        float baseSteering = regressionDrivingController != null ? regressionDrivingController.GetPredictedSteering() : 0f;
        float baseThrottle = regressionDrivingController != null ? regressionDrivingController.GetPredictedThrottle() : 0f;
        sensor.AddObservation(Mathf.Clamp(baseSteering, -1f, 1f));                        // 11: base 조향 예측
        sensor.AddObservation(Mathf.Clamp(baseThrottle, 0f, 1f));                         // 12: base 스로틀 예측

        // 디버그용 마지막 base 값 기록
        lastBaseSteering = baseSteering;
        lastBaseThrottle = baseThrottle;

        // ─── 신호등 결정 one-hot 관측 (인덱스 13~15) ───

        if (includeTrafficDecisionOneHot)
        {
            int decision = trafficLightDecisionEngine != null ? (int)trafficLightDecisionEngine.GetDecision() : 0;
            sensor.AddObservation(decision == 0 ? 1f : 0f);                               // 13: Go (진행)
            sensor.AddObservation(decision == 1 ? 1f : 0f);                               // 14: Caution (주의)
            sensor.AddObservation(decision == 2 ? 1f : 0f);                               // 15: Stop (정지)
        }
    }

    /// <summary>
    /// 정책 네트워크로부터 행동을 받아 실행하는 콜백.
    /// Residual 행동(delta)을 base에 합산하여 차량에 적용하고, 스텝 보상을 소비한다.
    /// </summary>
    public override void OnActionReceived(ActionBuffers actions)
    {
        // 1) Residual 행동을 모방학습 base에 합산하여 차량 제어에 적용
        ApplyResidualAction(actions);

        // 2) ProgressRewardProvider에 누적된 스텝 보상을 소비하여 Agent에 부여
        lastConsumedStepReward = ConsumeAndApplyStepReward();
    }

    /// <summary>
    /// Heuristic 모드(사람 제어 또는 테스트) 시 호출.
    /// delta를 0으로 설정하여 모방학습 출력을 그대로 사용한다.
    /// </summary>
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // delta=0 → base 예측만으로 주행 (보정 없음)
        var continuous = actionsOut.ContinuousActions;
        if (continuous.Length > 0)
            continuous[0] = 0f; // delta_steering = 0
        if (continuous.Length > 1)
            continuous[1] = 0f; // delta_accel = 0
    }

    // ──────────────────────────────────────────────
    //  레퍼런스 탐색 & 이벤트 구독
    // ──────────────────────────────────────────────

    /// <summary>
    /// Inspector에 미할당된 컴포넌트 레퍼런스를 자동으로 탐색하여 연결한다.
    /// 자기 자신 → 부모 계층 순으로 검색하며, followTarget과 ArticulationBody 루트도 탐색한다.
    /// </summary>
    void AutoFindReferences()
    {
        // 각 컴포넌트가 null이면 자신 → 부모 순으로 탐색
        if (wheelController == null)
            wheelController = GetComponent<WheelTest>() ?? GetComponentInParent<WheelTest>();

        if (progressRewardProvider == null)
            progressRewardProvider = GetComponent<ProgressRewardProvider>() ?? GetComponentInParent<ProgressRewardProvider>();

        if (episodeEvaluator == null)
            episodeEvaluator = GetComponent<RLEpisodeEvaluator>() ?? GetComponentInParent<RLEpisodeEvaluator>();

        if (collisionWarningPublisher == null)
            collisionWarningPublisher = GetComponent<CollisionWarningPublisher>() ?? GetComponentInParent<CollisionWarningPublisher>();

        if (trafficLightDecisionEngine == null)
            trafficLightDecisionEngine = GetComponent<TrafficLightDecisionEngine>() ?? GetComponentInParent<TrafficLightDecisionEngine>();

        if (regressionDrivingController == null)
            regressionDrivingController = GetComponent<RegressionDrivingController>() ?? GetComponentInParent<RegressionDrivingController>();

        if (vehicleCmdSubscriber == null)
            vehicleCmdSubscriber = GetComponent<VehicleCmdSubscriber>() ?? GetComponentInParent<VehicleCmdSubscriber>();

        // DecisionRequester는 자기 자신에게만 붙어야 하므로 부모 탐색 없음
        if (decisionRequester == null)
            decisionRequester = GetComponent<DecisionRequester>();

        // followTarget이 미할당이면 wheelController의 Transform을 기본값으로 사용
        if (followTargetTransform == null && wheelController != null)
            followTargetTransform = wheelController.transform;

        // ArticulationBody 루트를 wheelController → followTarget → self 순으로 탐색
        if (rootArticulation == null)
            rootArticulation = ResolveArticulationRoot(wheelController != null ? wheelController.transform : null);

        if (rootArticulation == null)
            rootArticulation = ResolveArticulationRoot(followTargetTransform);

        if (rootArticulation == null)
            rootArticulation = ResolveArticulationRoot(transform);
    }

    /// <summary>
    /// EpisodeEvaluator의 종료 이벤트 구독을 갱신한다.
    /// 이전 구독을 해제하고 현재 episodeEvaluator에 새로 구독하여 중복 구독을 방지한다.
    /// </summary>
    void RefreshTerminalSubscription()
    {
        // 이미 같은 평가기에 구독 중이면 스킵
        if (subscribedEvaluator == episodeEvaluator)
            return;

        // 이전 평가기의 이벤트 구독 해제
        if (subscribedEvaluator != null)
            subscribedEvaluator.OnEpisodeTerminated -= OnEpisodeTerminatedByEvaluator;

        // 현재 평가기로 교체 후 구독
        subscribedEvaluator = episodeEvaluator;

        if (subscribedEvaluator != null)
            subscribedEvaluator.OnEpisodeTerminated += OnEpisodeTerminatedByEvaluator;
    }

    /// <summary>
    /// 에피소드 종료 이벤트 구독을 완전히 해제한다. OnDisable/OnDestroy에서 호출.
    /// </summary>
    void UnsubscribeTerminalEvent()
    {
        if (subscribedEvaluator != null)
            subscribedEvaluator.OnEpisodeTerminated -= OnEpisodeTerminatedByEvaluator;

        subscribedEvaluator = null;
    }

    /// <summary>
    /// EpisodeEvaluator로부터 에피소드 종료 이벤트를 받는 콜백.
    /// 남은 스텝 보상을 소비하고, 성공/실패 터미널 보상을 부여한 뒤 에피소드를 종료한다.
    /// </summary>
    void OnEpisodeTerminatedByEvaluator(RLEpisodeEvaluator evaluator)
    {
        // 현재 할당된 평가기와 다른 평가기의 이벤트는 무시
        if (episodeEvaluator != null && evaluator != episodeEvaluator)
            return;

        // 아직 Initialize()가 완료되지 않았으면 무시 (초기화 전 콜백 방지)
        if (!agentInitialized)
            return;

        // 남은 스텝 보상을 모두 소비하여 Agent에 부여
        ConsumeAndApplyStepReward();

        // 성공/실패에 따라 터미널 보상 부여
        if (evaluator != null && evaluator.IsEpisodeSuccess())
            AddReward(successTerminalReward);
        else
            AddReward(failureTerminalPenalty);

        // 종료 사유를 기록하고 에피소드 종료
        lastTerminalReason = evaluator != null ? evaluator.GetTerminalReason() : "Unknown";
        EndEpisode();
    }

    // ──────────────────────────────────────────────
    //  제어 모드 & 학습 안정화
    // ──────────────────────────────────────────────

    /// <summary>
    /// 모방학습 컨트롤러를 predictionOnly 모드로 설정하고,
    /// RL Agent가 차량 제어 우선권을 가지도록 외부 제어 모드를 활성화한다.
    /// 외부 ROS cmd 모드일 때는 모방학습을 예측 전용으로만 두고 휠 컨트롤러를 외부 제어 모드로 설정.
    /// </summary>
    void EnableResidualMode()
    {
        // 외부 ROS cmd 입력 모드인 경우: 모방학습은 예측만, 휠은 외부 제어
        if (IsExternalRosCmdInputActive())
        {
            if (regressionDrivingController != null)
            {
                regressionDrivingController.predictionOnlyMode = true;
            }

            if (wheelController != null)
                wheelController.externalControlEnabled = true;

            return;
        }

        // 내부 Residual RL 모드: 모방학습 자율주행 ON + 예측 전용 모드 ON
        if (regressionDrivingController != null)
        {
            regressionDrivingController.isAutonomousMode = true;     // 자율주행 모드 활성화
            regressionDrivingController.predictionOnlyMode = true;   // 직접 제어하지 않고 예측값만 제공
        }

        // 휠 컨트롤러를 외부 제어 모드로 설정 (RL Agent가 직접 조향/스로틀/브레이크 제어)
        if (wheelController != null)
            wheelController.externalControlEnabled = true;
    }

    /// <summary>
    /// 학습 안정화를 위해 residualScale과 warningBrake를 튜닝된 범위로 클램프한다.
    /// applyTrainingStabilityTuning이 true일 때만 동작한다.
    /// </summary>
    void ApplyTrainingStabilityTuning()
    {
        if (!applyTrainingStabilityTuning)
            return;

        // 조향 보정 스케일을 0.12~0.2 범위로 제한
        residualSteerScale = Mathf.Clamp(tunedResidualSteerScale, 0.12f, 0.2f);
        // 가속 보정 스케일을 0.1~0.15 범위로 제한
        residualAccelScale = Mathf.Clamp(tunedResidualAccelScale, 0.1f, 0.15f);
        // 경고 시 브레이크 강도를 0.1~0.2 범위로 제한
        warningBrake = Mathf.Clamp(tunedWarningBrake, 0.1f, 0.2f);
    }

    // ──────────────────────────────────────────────
    //  차량 상태 리셋 & 포즈 관리
    // ──────────────────────────────────────────────

    /// <summary>
    /// 에피소드 시작 위치/회전을 캐시한다.
    /// episodeStartTransform이 지정되어 있으면 그 위치/회전을, 아니면 현재 위치/회전을 사용한다.
    /// Residual RL 모드에서는 시작 회전을 강제 적용한다.
    /// </summary>
    void CacheStartPose()
    {
        // 추적 대상이 있으면 그것을 기준으로, 없으면 자기 자신을 기준으로
        Transform follow = followTargetTransform != null ? followTargetTransform : transform;

        // Residual RL 모드에서 시작 회전을 강제할지 결정
        bool forceStartRotationForResidualRl =
            episodeStartTransform != null &&
            regressionDrivingController != null &&
            regressionDrivingController.predictionOnlyMode;

        if (episodeStartTransform != null)
        {
            // 지정된 시작 위치 사용
            startPosition = episodeStartTransform.position;

            // 높이(Y)를 시작 Transform에서 쓸지 현재 차량 높이를 유지할지 결정
            if (!useEpisodeStartTransformHeight && follow != null)
                startPosition.y = follow.position.y;

            // 회전 결정: Residual RL 강제 or 설정에 의한 사용 or 현재 회전 유지
            if (forceStartRotationForResidualRl || useEpisodeStartTransformRotation || follow == null)
                startRotation = episodeStartTransform.rotation;
            else
                startRotation = follow.rotation;
        }
        else
        {
            // episodeStartTransform이 없으면 현재 위치/회전을 시작 포즈로 사용
            startPosition = follow.position;
            startRotation = follow.rotation;
        }

        hasStartPose = true;
    }

    /// <summary>
    /// 차량의 물리 상태를 완전히 리셋한다.
    /// 위치/회전을 시작 포즈로 텔레포트하고, 모든 속도를 0으로 초기화하며,
    /// 조향/스로틀/브레이크 입력도 초기화한다.
    /// </summary>
    void ResetVehicleState()
    {
        // 시작 포즈가 캐시되지 않았으면 먼저 캐시
        if (!hasStartPose)
            CacheStartPose();

        // 리셋 대상 Transform 결정 (followTarget → wheelController → self)
        Transform resetTarget = followTargetTransform != null
            ? followTargetTransform
            : (wheelController != null ? wheelController.transform : transform);

        // 물리 텔레포트로 위치/회전 리셋
        ArticulationBody articulationRoot = ResolveArticulationRoot(resetTarget);
        if (hasStartPose)
        {
            if (articulationRoot != null)
                // ArticulationBody가 있으면 물리 엔진의 TeleportRoot 사용
                articulationRoot.TeleportRoot(startPosition, startRotation);
            else
                // 없으면 Transform 직접 설정
                resetTarget.SetPositionAndRotation(startPosition, startRotation);

            // TeleportRoot 후 카메라 등 자식 Transform이 즉시 반영되도록 물리 동기화
            // 이 호출 없이는 즉시 RunInference()할 때 카메라가 이전 위치(사고 장면)를 볼 수 있음
            Physics.SyncTransforms();
        }

        // 모든 ArticulationBody의 선속도/각속도를 0으로 초기화
        if (resetRootArticulationVelocity)
        {
            ArticulationBody velocityRoot = articulationRoot != null ? articulationRoot : rootArticulation;
            if (velocityRoot == null)
                velocityRoot = ResolveArticulationRoot(resetTarget);

            if (velocityRoot != null)
            {
                // 루트 포함 모든 자식 ArticulationBody의 속도를 0으로 리셋
                ArticulationBody[] bodies = velocityRoot.GetComponentsInChildren<ArticulationBody>(true);
                for (int i = 0; i < bodies.Length; i++)
                {
                    if (bodies[i] == null)
                        continue;

                    bodies[i].velocity = Vector3.zero;
                    bodies[i].angularVelocity = Vector3.zero;
                }
            }
        }

        // 휠 컨트롤러 입력 초기화 (조향/스로틀/브레이크 모두 0)
        if (wheelController != null)
        {
            wheelController.SetSteering(0f);
            wheelController.SetThrottle(0f);
            wheelController.SetBrake(0f);
        }

        // 리셋 후 Agent Transform을 다시 동기화
        SyncToFollowTarget();
    }

    /// <summary>
    /// 주어진 Transform에서 시작하여 ArticulationBody 계층을 따라 올라가
    /// 루트(최상위) ArticulationBody를 찾아 반환한다.
    /// </summary>
    ArticulationBody ResolveArticulationRoot(Transform target)
    {
        if (target == null)
            return null;

        // 자기 자신 → 부모 순으로 ArticulationBody 탐색
        ArticulationBody node = target.GetComponent<ArticulationBody>() ?? target.GetComponentInParent<ArticulationBody>();

        // isRoot가 될 때까지 부모 방향으로 계속 올라감
        while (node != null && !node.isRoot)
            node = node.transform.parent != null ? node.transform.parent.GetComponent<ArticulationBody>() : null;

        return node;
    }

    // ──────────────────────────────────────────────
    //  Residual 행동 적용 & 안전 오버라이드
    // ──────────────────────────────────────────────

    /// <summary>
    /// RL 정책의 delta 행동을 모방학습 base 예측에 합산하여 차량에 적용한다.
    /// 최종 제어값 = clamp(base + delta * residualScale)
    /// 안전 오버라이드가 활성화되면 충돌 경고/신호등에 따라 throttle/brake를 보정한다.
    /// </summary>
    void ApplyResidualAction(ActionBuffers actions)
    {
        if (wheelController == null)
            return;

        // 외부 ROS cmd 모드면 차량 제어를 건드리지 않음
        if (IsExternalRosCmdInputActive())
        {
            wheelController.externalControlEnabled = true;
            return;
        }

        // 1) RL 정책의 delta 값 추출 및 스케일 적용
        var continuous = actions.ContinuousActions;
        float deltaSteer = continuous.Length > 0 ? Mathf.Clamp(continuous[0], -1f, 1f) * residualSteerScale : 0f;
        float deltaAccel = continuous.Length > 1 ? Mathf.Clamp(continuous[1], -1f, 1f) * residualAccelScale : 0f;

        // 2) 모방학습 base 예측값 가져오기
        float baseSteering = regressionDrivingController != null ? regressionDrivingController.GetPredictedSteering() : 0f;
        float baseThrottle = regressionDrivingController != null ? regressionDrivingController.GetPredictedThrottle() : 0f;

        // [DEBUG] base값이 특정 고정값에 멈춰 있으면 경고 (stale prediction 감지)
        if (Mathf.Abs(baseSteering - (-0.384f)) < 0.005f && Mathf.Abs(baseThrottle - 0.185f) < 0.005f)
            Debug.LogWarning($"[RLAgent.Action] ⚠️ STALE BASE DETECTED! steer={baseSteering:F4}, throttle={baseThrottle:F4} | delta=({deltaSteer:F4},{deltaAccel:F4})");

        // 3) Residual 합산: base + delta → 최종 제어값
        float finalSteer = Mathf.Clamp(baseSteering + deltaSteer, -1f, 1f);
        float accelMin = allowReverse ? -1f : 0f;
        float finalAccel = Mathf.Clamp(baseThrottle + deltaAccel, accelMin, 1f);

        // 4) accel 값을 throttle/brake로 분리
        float throttle;
        float brake;

        if (allowReverse)
        {
            // 후진 허용: 음수 accel은 후진 스로틀
            throttle = finalAccel;
            brake = 0f;
        }
        else
        {
            // 후진 비허용: 양수→스로틀, 음수→브레이크로 변환
            if (finalAccel >= 0f)
            {
                throttle = finalAccel;
                brake = 0f;
            }
            else
            {
                throttle = 0f;
                brake = -finalAccel; // 음수를 양수 브레이크로 변환
            }
        }

        // 5) 안전 오버라이드 적용 (충돌 경고/신호등에 따른 throttle/brake 보정)
        if (enableSafetyOverride)
            ApplySafetyOverride(ref throttle, ref brake);

        // 6) 최종 클램프 후 차량에 적용
        float clampedThrottle = Mathf.Clamp(throttle, allowReverse ? -1f : 0f, 1f);
        float clampedBrake = Mathf.Clamp01(brake);

        wheelController.externalControlEnabled = true;
        wheelController.SetSteering(finalSteer);
        wheelController.SetThrottle(clampedThrottle);
        wheelController.SetBrake(clampedBrake);

        // 7) 디버그용 마지막 적용값 기록
        lastDeltaSteering = deltaSteer;
        lastDeltaAccel = deltaAccel;
        lastAppliedSteer = finalSteer;
        lastAppliedThrottle = clampedThrottle;
        lastAppliedBrake = clampedBrake;
    }

    /// <summary>
    /// 외부 ROS cmd 입력 모드가 활성 상태인지 판단한다.
    /// externalRosCmdInputOnly가 true이되, 학습 Communicator 연결 시
    /// forceInternalControlWhenTraining에 의해 내부 제어가 강제될 수 있다.
    /// </summary>
    bool IsExternalRosCmdInputActive()
    {
        // ML-Agents Communicator(학습 파이썬 프로세스)가 연결되어 있는지 확인
        bool communicatorOn = Academy.Instance != null && Academy.Instance.IsCommunicatorOn;

        // 학습 중이면 내부 제어를 강제 사용 (외부 ROS cmd 무시)
        runtimeForceInternalControlActive = forceInternalControlWhenTraining && communicatorOn;

        // 외부 입력이 활성 = externalRosCmdInputOnly가 켜져 있고 && 강제 내부 제어가 아닐 때
        return externalRosCmdInputOnly && !runtimeForceInternalControlActive;
    }

    /// <summary>
    /// 충돌 경고 레벨과 신호등 결정에 따라 throttle/brake를 안전하게 오버라이드한다.
    /// 경고 레벨이 높을수록 더 강한 제동을 적용하며, 신호등 Stop/Caution에도 반응한다.
    /// </summary>
    void ApplySafetyOverride(ref float throttle, ref float brake)
    {
        // ─── 충돌 경고 기반 안전 오버라이드 ───
        if (collisionWarningPublisher != null)
        {
            CollisionWarningPublisher.WarningLevel level = collisionWarningPublisher.GetWarningLevel();

            if (level >= CollisionWarningPublisher.WarningLevel.EmergencyStop)
            {
                // 비상 정지: 스로틀 0, 최대 브레이크
                throttle = 0f;
                brake = Mathf.Max(brake, emergencyBrake);
            }
            else if (level >= CollisionWarningPublisher.WarningLevel.Brake)
            {
                // 브레이크 단계: 스로틀 0, 강한 브레이크
                throttle = 0f;
                brake = Mathf.Max(brake, brakeLevelBrake);
            }
            else if (level >= CollisionWarningPublisher.WarningLevel.Warning)
            {
                // 경고 단계: 스로틀 제한, 최소 속도 이상이면 브레이크 적용
                throttle = Mathf.Min(throttle, 0.15f);
                float speed = wheelController != null ? wheelController.GetSpeedMS() : 0f;
                if (Mathf.Abs(speed) >= Mathf.Max(0f, warningBrakeMinSpeed))
                    brake = Mathf.Max(brake, warningBrake);
            }
            else if (level >= CollisionWarningPublisher.WarningLevel.SlowDown)
            {
                // 감속 단계: 스로틀만 제한 (부드러운 감속)
                throttle = Mathf.Min(throttle, cautionThrottleScale);
            }
        }

        // ─── 신호등 기반 안전 오버라이드 ───
        if (trafficLightDecisionEngine != null)
        {
            TrafficLightDecisionEngine.TrafficDecision decision = trafficLightDecisionEngine.GetDecision();

            if (decision == TrafficLightDecisionEngine.TrafficDecision.Stop)
            {
                // 정지 신호: 스로틀 0, 권장 브레이크 적용
                throttle = 0f;
                brake = Mathf.Max(brake, trafficLightDecisionEngine.GetRecommendedBrake());
            }
            else if (decision == TrafficLightDecisionEngine.TrafficDecision.Caution)
            {
                // 주의 신호: 스로틀 제한 + 권장 브레이크 적용
                throttle = Mathf.Min(throttle, trafficLightDecisionEngine.GetRecommendedThrottleScale());
                brake = Mathf.Max(brake, trafficLightDecisionEngine.GetRecommendedBrake());
            }
        }
    }

    // ──────────────────────────────────────────────
    //  보상 & 관측 유틸리티
    // ──────────────────────────────────────────────

    /// <summary>
    /// ProgressRewardProvider에 누적된 스텝 보상을 소비(consume)하여 Agent에 부여한다.
    /// 유효하고 0이 아닌 보상만 AddReward로 전달한다.
    /// </summary>
    float ConsumeAndApplyStepReward()
    {
        if (progressRewardProvider == null)
            return 0f;

        // 보상 제공자로부터 누적 보상을 소비 (이후 제공자 내부에서 0으로 리셋됨)
        float reward = progressRewardProvider.ConsumeStepReward();

        // 유효한(NaN/Infinity가 아닌) 비zero 보상만 Agent에 부여
        if (IsFinite(reward) && !Mathf.Approximately(reward, 0f))
            AddReward(reward);

        return IsFinite(reward) ? reward : 0f;
    }

    /// <summary>
    /// 차량의 현재 위치에서 가장 가까운 경로 세그먼트를 찾아
    /// 헤딩 오차(도)와 부호 있는 횡방향 오차를 계산한다.
    /// 양의 횡방향 오차 = 경로 오른쪽, 음의 횡방향 오차 = 경로 왼쪽.
    /// </summary>
    float ComputeHeadingErrorDeg(out float signedLateralError)
    {
        signedLateralError = 0f;

        // 웨이포인트가 2개 미만이면 계산 불가
        if (progressRewardProvider == null ||
            progressRewardProvider.progressWaypoints == null ||
            progressRewardProvider.progressWaypoints.Length < 2)
        {
            return 0f;
        }

        Transform[] waypoints = progressRewardProvider.progressWaypoints;
        bool looped = progressRewardProvider.isLoopedPath;
        Vector3 position = transform.position;

        // 가장 가까운 세그먼트 추적 변수
        float bestDistSq = float.PositiveInfinity;
        Vector3 bestTangent = transform.forward;
        float bestSignedLateral = 0f;

        // 모든 경로 세그먼트를 순회하며 최근접 세그먼트 탐색
        int segCount = looped ? waypoints.Length : waypoints.Length - 1;
        for (int i = 0; i < segCount; i++)
        {
            int j = (i + 1) % waypoints.Length;
            if (waypoints[i] == null || waypoints[j] == null)
                continue;

            Vector3 a = waypoints[i].position;
            Vector3 b = waypoints[j].position;
            Vector3 ab = b - a;
            float abLenSq = ab.sqrMagnitude;

            // 길이가 거의 0인 세그먼트는 스킵
            if (abLenSq < 1e-6f)
                continue;

            // 차량 위치를 세그먼트 AB 위에 투영 (t: 0~1로 클램프)
            float t = Mathf.Clamp01(Vector3.Dot(position - a, ab) / abLenSq);
            Vector3 projected = a + (ab * t);
            Vector3 offset = position - projected;
            float distSq = offset.sqrMagnitude;

            // 더 가까운 세그먼트를 찾으면 갱신
            if (distSq < bestDistSq)
            {
                bestDistSq = distSq;
                bestTangent = ab.normalized; // 세그먼트 진행 방향

                // 외적으로 좌/우 방향 판별 (up 축 기준)
                float sideSign = Mathf.Sign(Vector3.Dot(Vector3.Cross(bestTangent, offset), Vector3.up));
                if (Mathf.Approximately(sideSign, 0f))
                    sideSign = 1f;

                // 부호 있는 횡방향 오차 (양수=오른쪽, 음수=왼쪽)
                bestSignedLateral = Mathf.Sqrt(distSq) * sideSign;
            }
        }

        signedLateralError = bestSignedLateral;

        // 세그먼트 접선 방향과 차량 전방 벡터 사이의 부호 있는 각도(도) 반환
        return Vector3.SignedAngle(bestTangent, transform.forward, Vector3.up);
    }

    // ──────────────────────────────────────────────
    //  정규화 & 유틸리티 헬퍼
    // ──────────────────────────────────────────────

    /// <summary>
    /// float 값이 유한한지(NaN이나 Infinity가 아닌지) 검사한다.
    /// </summary>
    static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    /// <summary>
    /// 값을 [-1, 1] 범위로 정규화한다. 비유한 값은 0을 반환한다.
    /// </summary>
    float NormalizeSigned(float value, float denom)
    {
        if (!IsFinite(value))
            return 0f;

        // 분모가 0에 가까울 때 나눗셈 오류 방지
        float safeDenom = Mathf.Max(1e-4f, denom);
        return Mathf.Clamp(value / safeDenom, -1f, 1f);
    }

    /// <summary>
    /// 값을 [0, 1] 범위로 정규화한다. 비유한 값이나 음수는 defaultWhenInvalid를 반환한다.
    /// </summary>
    float Normalize01(float value, float maxRange, float defaultWhenInvalid)
    {
        // NaN, Infinity, 음수는 기본값으로 처리
        if (!IsFinite(value) || value < 0f)
            return defaultWhenInvalid;

        float safeRange = Mathf.Max(1e-4f, maxRange);
        return Mathf.Clamp01(value / safeRange);
    }

    // ──────────────────────────────────────────────
    //  외부 접근용 Getter (디버그/UI용)
    // ──────────────────────────────────────────────

    /// <summary>마지막 모방학습 base 조향 예측값을 반환한다.</summary>
    public float GetLastBaseSteering() => lastBaseSteering;
    /// <summary>마지막 모방학습 base 스로틀 예측값을 반환한다.</summary>
    public float GetLastBaseThrottle() => lastBaseThrottle;
    /// <summary>마지막 RL delta 조향 보정값을 반환한다.</summary>
    public float GetLastDeltaSteering() => lastDeltaSteering;
    /// <summary>마지막 RL delta 가속 보정값을 반환한다.</summary>
    public float GetLastDeltaAccel() => lastDeltaAccel;
    /// <summary>마지막으로 차량에 적용된 최종 조향값을 반환한다.</summary>
    public float GetLastAppliedSteer() => lastAppliedSteer;
    /// <summary>마지막으로 차량에 적용된 최종 스로틀값을 반환한다.</summary>
    public float GetLastAppliedThrottle() => lastAppliedThrottle;
    /// <summary>마지막으로 차량에 적용된 최종 브레이크값을 반환한다.</summary>
    public float GetLastAppliedBrake() => lastAppliedBrake;
    /// <summary>마지막으로 계산된 헤딩 오차(도)를 반환한다.</summary>
    public float GetLastHeadingErrorDeg() => lastHeadingErrorDeg;
    /// <summary>마지막으로 계산된 부호 있는 횡방향 오차를 반환한다.</summary>
    public float GetLastSignedLateralError() => lastSignedLateralError;

    /// <summary>
    /// 현재 시점의 헤딩 오차(도)와 부호 있는 횡방향 오차를 즉시 계산하여 반환한다.
    /// Transform을 먼저 동기화한 뒤 계산하므로 최신 값을 보장한다.
    /// </summary>
    public float GetCurrentHeadingErrorDeg(out float signedLateralError)
    {
        SyncToFollowTarget();
        float headingErrorDeg = ComputeHeadingErrorDeg(out signedLateralError);
        lastHeadingErrorDeg = headingErrorDeg;
        lastSignedLateralError = signedLateralError;
        return headingErrorDeg;
    }

    /// <summary>
    /// 디버그용 행동 요약 문자열을 반환한다.
    /// base, delta, final 제어값과 스텝 보상을 포함한다.
    /// </summary>
    public string GetActionDebugSummary()
    {
        return
            $"base=({lastBaseSteering:F3},{lastBaseThrottle:F3}) " +
            $"delta=({lastDeltaSteering:F3},{lastDeltaAccel:F3}) " +
            $"final=({lastAppliedSteer:F3},{lastAppliedThrottle:F3},{lastAppliedBrake:F3}) " +
            $"stepReward={lastConsumedStepReward:F3}";
    }

    /// <summary>
    /// Agent의 Transform을 followTargetTransform(예: base_link)의 위치/회전에 동기화한다.
    /// Agent가 물리 객체와 별도 계층에 있을 때 관측값이 올바른 위치를 기준으로 수집되도록 보장.
    /// </summary>
    void SyncToFollowTarget()
    {
        if (followTargetTransform == null)
            return;

        if (followTargetPosition)
            transform.position = followTargetTransform.position;

        if (followTargetRotation)
            transform.rotation = followTargetTransform.rotation;
    }
}
