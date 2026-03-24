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
/// Observation space (8, 기본 활성):
///   0: speed (속도)
///   1: signedLateralError (부호 있는 횡방향 오차)
///   2: headingError (헤딩 오차)
///   3: ttc (충돌까지 남은 시간)
///   4: warningLevel (충돌 경고 레벨)
///   5: currentSteer (현재 조향 입력)
///   6: currentThrottle (현재 스로틀 입력)
///   7: currentBrake (현재 브레이크 입력)
///
///   비활성 (Inspector 플래그로 재활성화 가능):
///   includeStopLineDistance      → 정지선 거리 (+1D)
///   includeBasePredictions       → 모방학습 base 예측 steer/throttle (+2D)
///   includeTrafficDecisionOneHot → 신호등 one-hot Go/Caution/Stop (+3D)
/// </summary>
public class AutoDriverRLAgent : Agent
{
    [Header("Setup")]
    public bool autoFindReferences = true;
    [Tooltip("DecisionRequester가 없을 때 FixedUpdate마다 RequestDecision 호출")]
    public bool requestDecisionInFixedUpdateWithoutRequester = false;

    [Header("References")]
    public VehicleMotionController vehicleMotionController;
    public ProgressRewardProvider progressRewardProvider;
    public RLEpisodeEvaluator episodeEvaluator;
    public CollisionWarningEngine collisionWarningEngine;
    public TrafficLightDecisionEngine trafficLightDecisionEngine;
    public RegressionDrivingController regressionDrivingController;
    public VehicleCmdSubscriber vehicleCmdSubscriber;
    public DecisionRequester decisionRequester;

    [Header("Sensor Observations (+10D)")]
    [Tooltip("true면 초음파 8D + 레이더 2D 를 관측에 포함 (총 +10D). BehaviorParameters도 동일하게 변경 필요")]
    public bool includeSensorObservations = false;
    [Tooltip("초음파 거리 정규화 기준 (m) — rangeMax=4m 에 맞춤")]
    public float ultrasonicNormalizeM = 4f;
    [Tooltip("레이더 거리 정규화 기준 (m)")]
    public float radarNormalizeM = 10f;
    [Tooltip("초음파 FL — ultrasonic_fl_link의 SingleUltrasonicSensor")]
    public SingleUltrasonicSensor sensorFL;
    [Tooltip("초음파 FR — ultrasonic_fr_link의 SingleUltrasonicSensor")]
    public SingleUltrasonicSensor sensorFR;
    [Tooltip("초음파 FC — ultrasonic_fc_link의 SingleUltrasonicSensor")]
    public SingleUltrasonicSensor sensorFC;
    [Tooltip("초음파 RL — ultrasonic_rl_link의 SingleUltrasonicSensor")]
    public SingleUltrasonicSensor sensorRL;
    [Tooltip("초음파 RR — ultrasonic_rr_link의 SingleUltrasonicSensor")]
    public SingleUltrasonicSensor sensorRR;
    [Tooltip("초음파 RC — ultrasonic_rc_link의 SingleUltrasonicSensor")]
    public SingleUltrasonicSensor sensorRC;
    [Tooltip("초음파 SL — ultrasonic_sl_link의 SingleUltrasonicSensor")]
    public SingleUltrasonicSensor sensorSL;
    [Tooltip("초음파 SR — ultrasonic_sr_link의 SingleUltrasonicSensor")]
    public SingleUltrasonicSensor sensorSR;
    [Tooltip("레이더 전방 — SingleRadarSensor (Front)")]
    public SingleRadarSensor radarFront;
    [Tooltip("레이더 후방 — SingleRadarSensor (Rear)")]
    public SingleRadarSensor radarRear;

    [Header("Random Road Start")]
    [Tooltip("true면 에피소드마다 ProgressRewardProvider 중심선 위 랜덤 위치로 스폰")]
    public bool useRandomRoadStart = false;
    [Tooltip("도로 횡방향 최대 오프셋 (m). 도로 폭 내를 벗어나지 않도록 설정")]
    public float randomLateralOffsetM = 0.3f;

    [Header("Decision Staggering")]
    [Tooltip("true: DecisionRequester.DecisionStep을 에이전트별로 자동 배정\n" +
             "→ 8대 동시 BehaviourUpdate 스파이크를 프레임별 1~2회로 분산\n" +
             "DecisionRequester가 반드시 붙어 있어야 함")]
    public bool useStaggeredDecision = false;
    [Tooltip("-1이면 Play 진입 시 자동 배정 (에이전트 등록 순서 0~N, mod DecisionPeriod)")]
    [SerializeField] private int staggerOffset = -1;

    // Play 진입마다 정적 카운터를 0으로 초기화 (도메인 리로드 안전)
    private static int s_staggerCounter;
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    static void ResetStaggerCounter() => s_staggerCounter = 0;
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

    [Header("Action Mapping")]
    [Tooltip("false면 음수 accel은 브레이크로 처리")]
    public bool allowReverse = false;

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

    [Header("Optional Observations")]
    [Tooltip("true면 정지선 거리를 관측에 포함 (+1D). 현재 맵에 정지선 학습 미포함 시 비활성 권장.")]
    public bool includeStopLineDistance = false;
    [Tooltip("true면 모방학습 base 예측 (steer, throttle)을 관측에 포함 (+2D). regressionDrivingController=null이면 항상 0.")]
    public bool includeBasePredictions = false;
    [Tooltip("true면 신호등 one-hot (Go, Caution, Stop)을 관측에 포함 (+3D). 신호등 null이면 항상 Go=1.")]
    public bool includeTrafficDecisionOneHot = false;

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
    /// Agent 최초 초기화. 레퍼런스 탐색, 시작 포즈 캐시,
    /// 에피소드 종료 이벤트 구독, Residual 모드 활성화를 순서대로 수행한다.
    /// </summary>
    public override void Initialize()
    {
        // 1) Inspector에 미할당된 컴포넌트를 자동으로 탐색하여 연결
        if (autoFindReferences)
            AutoFindReferences();

        // 2) 에피소드 시작 위치/회전을 캐시해 두어 리셋 시 사용
        CacheStartPose();

        // 3) EpisodeEvaluator의 종료 이벤트에 구독 (에피소드 종료 시 보상 부여 + EndEpisode 호출)
        RefreshTerminalSubscription();

        // 4) 모방학습 컨트롤러를 predictionOnly 모드로 전환하여 RL이 제어 우선권을 가짐
        EnableResidualMode();

        // 5) 초기화 완료 플래그 설정 (종료 콜백에서 초기화 전 호출 방지용)
        agentInitialized = true;

        // 6) Stagger 활성화: DecisionRequester.DecisionStep에 에이전트별 오프셋 배정
        //    DecisionRequester는 그대로 유지 — 공식 ML-Agents API로 분산 처리
        if (useStaggeredDecision && decisionRequester != null)
        {
            if (staggerOffset < 0)
                staggerOffset = s_staggerCounter++;
            decisionRequester.DecisionStep = staggerOffset % decisionRequester.DecisionPeriod;
        }
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

        // IsExternalRosCmdInputActive()를 한 번만 호출하여 캐시 (FixedUpdate 핫패스)
        bool isExternalCmd = IsExternalRosCmdInputActive();

        // 외부 ROS cmd 모드: 모방학습을 예측 전용으로, 휠 컨트롤러를 외부 제어 모드로 설정
        if (isExternalCmd)
        {
            if (regressionDrivingController != null && !regressionDrivingController.predictionOnlyMode)
                regressionDrivingController.predictionOnlyMode = true;

            if (vehicleMotionController != null && !vehicleMotionController.externalControlEnabled)
                vehicleMotionController.externalControlEnabled = true;
        }
        // DecisionRequester 없이 수동 결정 요청 (fallback)
        else if (requestDecisionInFixedUpdateWithoutRequester && decisionRequester == null)
        {
            RequestDecision();
        }
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

        // 2) 에피소드 종료 이벤트 구독 갱신
        RefreshTerminalSubscription();

        // 3) Agent Transform 동기화
        SyncToFollowTarget();

        // 4) 랜덤 도로 스폰 활성 시 시작 포즈를 매 에피소드 새로 샘플링
        if (useRandomRoadStart)
            ApplyRandomRoadStartPose();

        // 5) 차량 위치/회전/속도를 시작 포즈로 리셋
        if (resetVehicleTransformOnEpisodeBegin)
        {
            // Residual RL 모드에서는 시작 회전을 강제 적용해야 모방학습이 올바르게 동작
            bool forceStartRotationForResidualRl =
                episodeStartTransform != null &&
                regressionDrivingController != null &&
                regressionDrivingController.predictionOnlyMode;

            // 회전 리셋 설정에 대한 경고 로그 출력 (학습 성능을 위해 비활성화)
            // if (episodeStartTransform != null && !useEpisodeStartTransformRotation)
            // {
            //     if (forceStartRotationForResidualRl)
            //         Debug.Log("[AutoDriverRLAgent] useEpisodeStartTransformRotation=False 이지만 Residual RL 모드에서는 시작 회전을 강제 적용합니다.");
            //     else
            //         Debug.LogWarning("[AutoDriverRLAgent] episodeStartTransform은 지정되었지만 회전 리셋이 꺼져 있습니다.");
            // }
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
            regressionDrivingController.ResetForEpisodeRestart(forceInference: false);
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
    /// 기본 8D: speed, signedLateralError, headingError, ttc, warningLevel, steer, throttle, brake
    /// Optional 플래그로 stopLineDistance(+1), basePredictions(+2), trafficOneHot(+3) 추가 가능.
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        // SyncToFollowTarget() ← 제거: FixedUpdate()에서 이미 수행됨

        // ─── 환경 상태 관측 (인덱스 0~4, 필수) ───

        // 현재 차량 속도 (m/s)
        float speed = vehicleMotionController != null ? vehicleMotionController.GetSpeedMS() : 0f;
        // 충돌 경고 존 최소 거리 (TTC 제거 후 대체)
        float ttc = collisionWarningEngine != null
            ? collisionWarningEngine.GetDistanceToObstacle()
            : float.PositiveInfinity;
        // 충돌 경고 레벨 정규화 (0~1, 6단계 기준)
        float warningLevelNorm = collisionWarningEngine != null
            ? Mathf.Clamp01((int)collisionWarningEngine.GetWarningLevel() / 6f)
            : 0f;
        // 정지선까지 거리
        float stopLineDistance = trafficLightDecisionEngine != null
            ? trafficLightDecisionEngine.GetDecisionDistance()
            : float.PositiveInfinity;

        // ProgressRewardProvider 캐시에서 헤딩/횡오차 읽기 (O(N) → O(1))
        // ProgressRewardProvider.FixedUpdate()가 CollectObservations()보다 먼저 실행됨이 보장된다.
        float headingErrorDeg;
        float signedLateralError;
        if (progressRewardProvider != null)
        {
            headingErrorDeg = progressRewardProvider.GetCachedHeadingErrorDeg();
            signedLateralError = progressRewardProvider.GetCachedSignedLateralError();
        }
        else
        {
            headingErrorDeg = ComputeHeadingErrorDeg(out signedLateralError);
        }
        lastHeadingErrorDeg = headingErrorDeg;
        lastSignedLateralError = signedLateralError;

        // 관측값을 정규화하여 센서에 추가 (기본 8D)
        sensor.AddObservation(NormalizeSigned(speed, speedNormalize));                    // 0: 속도 (부호 있는 정규화)
        sensor.AddObservation(NormalizeSigned(signedLateralError, lateralErrorNormalize)); // 1: 횡방향 오차 부호 포함 (-1~1)
        sensor.AddObservation(NormalizeSigned(headingErrorDeg, headingErrorNormalizeDeg)); // 2: 헤딩 오차 (-1~1)
        sensor.AddObservation(Normalize01(ttc, ttcNormalizeSeconds, 1f));                  // 3: TTC (0~1, 무한대→1)
        sensor.AddObservation(warningLevelNorm);                                           // 4: 충돌 경고 레벨 (0~1)

        if (includeStopLineDistance)
            sensor.AddObservation(Normalize01(stopLineDistance, stopLineDistanceNormalize, 1f)); // opt: 정지선 거리 (0~1)

        // ─── 현재 차량 입력 관측 (인덱스 5~7) ───

        float currentSteer = vehicleMotionController != null ? vehicleMotionController.GetSteeringInput() : 0f;
        float currentThrottle = vehicleMotionController != null ? vehicleMotionController.GetThrottleInput() : 0f;
        float currentBrake = vehicleMotionController != null ? vehicleMotionController.GetBrakeInput() : 0f;
        sensor.AddObservation(Mathf.Clamp(currentSteer, -1f, 1f));                        // 5: 현재 조향 입력
        sensor.AddObservation(Mathf.Clamp(currentThrottle, -1f, 1f));                     // 6: 현재 스로틀 입력
        sensor.AddObservation(Mathf.Clamp01(currentBrake));                                // 7: 현재 브레이크 입력

        // ─── 모방학습 base 예측값 관측 (Optional, includeBasePredictions=true 시 +2D) ───

        float baseSteering = regressionDrivingController != null ? regressionDrivingController.GetPredictedSteering() : 0f;
        float baseThrottle = regressionDrivingController != null ? regressionDrivingController.GetPredictedThrottle() : 0f;

        if (includeBasePredictions)
        {
            sensor.AddObservation(Mathf.Clamp(baseSteering, -1f, 1f));                    // opt: base 조향 예측
            sensor.AddObservation(Mathf.Clamp(baseThrottle, 0f, 1f));                     // opt: base 스로틀 예측
        }

        // 디버그용 마지막 base 값 기록
        lastBaseSteering = baseSteering;
        lastBaseThrottle = baseThrottle;

        // ─── 신호등 결정 one-hot 관측 (Optional, includeTrafficDecisionOneHot=true 시 +3D) ───

        if (includeTrafficDecisionOneHot)
        {
            int decision = trafficLightDecisionEngine != null ? (int)trafficLightDecisionEngine.GetDecision() : 0;
            sensor.AddObservation(decision == 0 ? 1f : 0f);                               // opt: Go (진행)
            sensor.AddObservation(decision == 1 ? 1f : 0f);                               // opt: Caution (주의)
            sensor.AddObservation(decision == 2 ? 1f : 0f);                               // opt: Stop (정지)
        }

        // ─── 초음파 + 레이더 센서 관측 (Optional, includeSensorObservations=true 시 +10D) ───
        if (includeSensorObservations)
        {
            float uMax = Mathf.Max(1f, ultrasonicNormalizeM);
            // 초음파 8개: FL/FR/FC/RL/RR/RC/SL/SR 순서 고정 (미할당 시 1.0=장애물 없음)
            sensor.AddObservation(NormalizeDist(sensorFL  != null ? sensorFL.Distance  : float.PositiveInfinity, uMax)); // 8: 초음파 FL
            sensor.AddObservation(NormalizeDist(sensorFR  != null ? sensorFR.Distance  : float.PositiveInfinity, uMax)); // 9: 초음파 FR
            sensor.AddObservation(NormalizeDist(sensorFC  != null ? sensorFC.Distance  : float.PositiveInfinity, uMax)); // 10: 초음파 FC
            sensor.AddObservation(NormalizeDist(sensorRL  != null ? sensorRL.Distance  : float.PositiveInfinity, uMax)); // 11: 초음파 RL
            sensor.AddObservation(NormalizeDist(sensorRR  != null ? sensorRR.Distance  : float.PositiveInfinity, uMax)); // 12: 초음파 RR
            sensor.AddObservation(NormalizeDist(sensorRC  != null ? sensorRC.Distance  : float.PositiveInfinity, uMax)); // 13: 초음파 RC
            sensor.AddObservation(NormalizeDist(sensorSL  != null ? sensorSL.Distance  : float.PositiveInfinity, uMax)); // 14: 초음파 SL
            sensor.AddObservation(NormalizeDist(sensorSR  != null ? sensorSR.Distance  : float.PositiveInfinity, uMax)); // 15: 초음파 SR

            float rMax = Mathf.Max(1f, radarNormalizeM);
            // 레이더 2개: Front/Rear (미할당 시 1.0=감지 없음)
            sensor.AddObservation(NormalizeDist(radarFront != null ? radarFront.Distance : float.PositiveInfinity, rMax)); // 16: 레이더 전방
            sensor.AddObservation(NormalizeDist(radarRear  != null ? radarRear.Distance  : float.PositiveInfinity, rMax)); // 17: 레이더 후방
        }
    }

    /// <summary>거리 값을 0~1로 정규화. 무한대(감지 없음)→1, 0m→0.</summary>
    static float NormalizeDist(float distance, float maxRange)
    {
        if (float.IsInfinity(distance) || float.IsNaN(distance)) return 1f;
        return Mathf.Clamp01(distance / maxRange);
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
        if (vehicleMotionController == null)
            vehicleMotionController = GetComponent<VehicleMotionController>() ?? GetComponentInParent<VehicleMotionController>();

        if (progressRewardProvider == null)
            progressRewardProvider = GetComponent<ProgressRewardProvider>() ?? GetComponentInParent<ProgressRewardProvider>();

        if (episodeEvaluator == null)
            episodeEvaluator = GetComponent<RLEpisodeEvaluator>() ?? GetComponentInParent<RLEpisodeEvaluator>();

        if (collisionWarningEngine == null)
            collisionWarningEngine = GetComponent<CollisionWarningEngine>() ?? GetComponentInParent<CollisionWarningEngine>();

        if (trafficLightDecisionEngine == null)
            trafficLightDecisionEngine = GetComponent<TrafficLightDecisionEngine>() ?? GetComponentInParent<TrafficLightDecisionEngine>();

        if (regressionDrivingController == null)
            regressionDrivingController = GetComponent<RegressionDrivingController>() ?? GetComponentInParent<RegressionDrivingController>();

        if (vehicleCmdSubscriber == null)
            vehicleCmdSubscriber = GetComponent<VehicleCmdSubscriber>() ?? GetComponentInParent<VehicleCmdSubscriber>();

        // DecisionRequester는 자기 자신에게만 붙어야 하므로 부모 탐색 없음
        if (decisionRequester == null)
            decisionRequester = GetComponent<DecisionRequester>();

        // followTarget이 미할당이면 vehicleMotionController의 Transform을 기본값으로 사용
        if (followTargetTransform == null && vehicleMotionController != null)
            followTargetTransform = vehicleMotionController.transform;

        // ArticulationBody 루트를 vehicleMotionController → followTarget → self 순으로 탐색
        if (rootArticulation == null)
            rootArticulation = ResolveArticulationRoot(vehicleMotionController != null ? vehicleMotionController.transform : null);

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
    //  제어 모드
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

            if (vehicleMotionController != null)
                vehicleMotionController.externalControlEnabled = true;

            return;
        }

        // 내부 Residual RL 모드: 모방학습 자율주행 ON + 예측 전용 모드 ON
        if (regressionDrivingController != null)
        {
            regressionDrivingController.isAutonomousMode = true;     // 자율주행 모드 활성화
            regressionDrivingController.predictionOnlyMode = true;   // 직접 제어하지 않고 예측값만 제공
        }

        // 휠 컨트롤러를 외부 제어 모드로 설정 (RL Agent가 직접 조향/스로틀/브레이크 제어)
        if (vehicleMotionController != null)
            vehicleMotionController.externalControlEnabled = true;
    }

    // ──────────────────────────────────────────────
    //  차량 상태 리셋 & 포즈 관리
    // ──────────────────────────────────────────────

    /// <summary>
    /// useRandomRoadStart=true 시 ProgressRewardProvider 중심선에서 랜덤 위치를 샘플링하여
    /// startPosition/startRotation을 덮어쓴다. 헤딩은 0~360° 균일 랜덤, 횡방향은 ±randomLateralOffsetM.
    /// ResetVehicleState() 전에 호출해야 한다.
    /// </summary>
    void ApplyRandomRoadStartPose()
    {
        if (progressRewardProvider == null) return;

        if (!progressRewardProvider.TryGetRandomCenterlinePoint(out Vector3 pos, out Vector3 tangent))
        {
            Debug.LogWarning("[AutoDriverRLAgent] 랜덤 스폰: 중심선 데이터 없음 → 기본 스폰 사용");
            return;
        }

        // 횡방향 랜덤 오프셋 (도로 내)
        Vector3 right = Vector3.Cross(Vector3.up, tangent).normalized;
        float lateralOffset = UnityEngine.Random.Range(-randomLateralOffsetM, randomLateralOffsetM);
        pos += right * lateralOffset;

        // Y 높이: 차량 현재 높이 유지 (지면 높이가 다를 수 있으므로)
        Transform follow = followTargetTransform != null ? followTargetTransform : transform;
        pos.y = follow.position.y;

        // 0~360° 완전 랜덤 헤딩 (역주행, 측면 포함)
        float randomYaw = UnityEngine.Random.Range(0f, 360f);
        Quaternion rot = Quaternion.Euler(0f, randomYaw, 0f);

        // 직접 캐시 덮어쓰기 → ResetVehicleState()가 이 값으로 텔레포트
        startPosition = pos;
        startRotation = rot;
        hasStartPose = true;
    }

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

        // 리셋 대상 Transform 결정 (followTarget → vehicleMotionController → self)
        Transform resetTarget = followTargetTransform != null
            ? followTargetTransform
            : (vehicleMotionController != null ? vehicleMotionController.transform : transform);

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
        if (vehicleMotionController != null)
        {
            vehicleMotionController.SetSteering(0f);
            vehicleMotionController.SetThrottle(0f);
            vehicleMotionController.SetBrake(0f);
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
    //  Residual 행동 적용
    // ──────────────────────────────────────────────

    /// <summary>
    /// RL 정책의 delta 행동을 모방학습 base 예측에 합산하여 차량에 적용한다.
    /// 최종 제어값 = clamp(base + delta * residualScale)
    /// </summary>
    void ApplyResidualAction(ActionBuffers actions)
    {
        if (vehicleMotionController == null)
            return;

        // 외부 ROS cmd 모드면 차량 제어를 건드리지 않음
        if (IsExternalRosCmdInputActive())
        {
            vehicleMotionController.externalControlEnabled = true;
            return;
        }

        // 1) RL 정책의 delta 값 추출 및 스케일 적용
        var continuous = actions.ContinuousActions;
        float deltaSteer = continuous.Length > 0 ? Mathf.Clamp(continuous[0], -1f, 1f) * residualSteerScale : 0f;
        float deltaAccel = continuous.Length > 1 ? Mathf.Clamp(continuous[1], -1f, 1f) * residualAccelScale : 0f;

        // 2) 모방학습 base 예측값 가져오기
        float baseSteering = regressionDrivingController != null ? regressionDrivingController.GetPredictedSteering() : 0f;
        float baseThrottle = regressionDrivingController != null ? regressionDrivingController.GetPredictedThrottle() : 0f;

        // [DEBUG] stale prediction 감지 (학습 성능을 위해 비활성화)
        // if (Mathf.Abs(baseSteering - (-0.384f)) < 0.005f && Mathf.Abs(baseThrottle - 0.185f) < 0.005f)
        //     Debug.LogWarning($"[RLAgent.Action] STALE BASE DETECTED!");

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

        // 5) 최종 클램프 후 차량에 적용
        float clampedThrottle = Mathf.Clamp(throttle, allowReverse ? -1f : 0f, 1f);
        float clampedBrake = Mathf.Clamp01(brake);

        vehicleMotionController.externalControlEnabled = true;
        vehicleMotionController.SetSteering(finalSteer);
        vehicleMotionController.SetThrottle(clampedThrottle);
        vehicleMotionController.SetBrake(clampedBrake);

        // 6) 디버그용 마지막 적용값 기록
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
        bool communicatorOn = IsTrainingMode();

        // 학습 중이면 내부 제어를 강제 사용 (외부 ROS cmd 무시)
        runtimeForceInternalControlActive = forceInternalControlWhenTraining && communicatorOn;

        // 외부 입력이 활성 = externalRosCmdInputOnly가 켜져 있고 && 강제 내부 제어가 아닐 때
        return externalRosCmdInputOnly && !runtimeForceInternalControlActive;
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
    /// ML-Agents Communicator(학습 파이썬 프로세스)가 연결된 학습 모드인지 반환한다.
    /// IsExternalRosCmdInputActive 등에서 공유 사용.
    /// </summary>
    static bool IsTrainingMode() => Academy.Instance != null && Academy.Instance.IsCommunicatorOn;

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
        // 캐시 우선 사용 (SyncToFollowTarget + ComputeHeadingErrorDeg 제거)
        if (progressRewardProvider != null)
        {
            signedLateralError = progressRewardProvider.GetCachedSignedLateralError();
            float heading = progressRewardProvider.GetCachedHeadingErrorDeg();
            lastHeadingErrorDeg = heading;
            lastSignedLateralError = signedLateralError;
            return heading;
        }

        // fallback: progressRewardProvider 없을 때만 직접 계산
        SyncToFollowTarget();
        float h = ComputeHeadingErrorDeg(out signedLateralError);
        lastHeadingErrorDeg = h;
        lastSignedLateralError = signedLateralError;
        return h;
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
