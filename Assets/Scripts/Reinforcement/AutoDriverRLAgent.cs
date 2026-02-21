using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using RosMessageTypes.Geometry;

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
    [Tooltip("Warning 단계에서 brake를 적용할 최소 속도 (m/s). 저속에서는 크리핑을 허용")]
    public float warningBrakeMinSpeed = 0.6f;
    [Range(0f, 1f)] public float brakeLevelBrake = 0.8f;
    [Range(0f, 1f)] public float emergencyBrake = 1f;

    [Header("ROS Command Output (Topic-Driven Actuation)")]
    [Tooltip("true면 외부 ROS cmd 토픽만 사용하고, Agent/Regression의 내부 제어 출력을 차량에 적용하지 않음")]
    public bool externalRosCmdInputOnly = false;
    [Tooltip("true면 RL 최종 제어를 /vehicle/cmd(Twist)로 발행")]
    public bool outputViaVehicleCmdTopic = true;
    [Tooltip("VehicleCmdSubscriber가 있으면 direct 제어 대신 토픽 경유 제어를 우선 사용")]
    public bool preferTopicActuationWhenSubscriberPresent = true;
    [Tooltip("RL 출력 command 토픽 이름 (RosTopicNamespace prefix가 자동 적용됨)")]
    public string vehicleCmdTopicName = "/vehicle/cmd";
    [Tooltip("throttle=1일 때 선속도(m/s)")]
    public float cmdTopicMaxForwardSpeed = 2.0f;
    [Tooltip("brake=1 또는 reverse=1일 때 대응 선속도 크기(m/s)")]
    public float cmdTopicMaxReverseSpeed = 1.0f;
    [Tooltip("steer=1일 때 대응 yaw rate(rad/s)")]
    public float cmdTopicMaxYawRateForFullSteer = 1.2f;
    [Tooltip("OnDisable 시 1회 정지 명령 발행")]
    public bool publishZeroCmdOnDisable = true;

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
    [SerializeField] private string resolvedCmdTopicName = "";
    [SerializeField] private float lastPublishedCmdLinearX = 0f;
    [SerializeField] private float lastPublishedCmdAngularZ = 0f;
    [SerializeField] private bool lastActionPublishedToRos = false;

    private Vector3 startPosition;
    private Quaternion startRotation;
    private bool hasStartPose = false;
    private ArticulationBody rootArticulation;
    private RLEpisodeEvaluator subscribedEvaluator;
    private bool agentInitialized = false;
    private ROSConnection ros;
    private bool rosCmdPublisherReady = false;

    public override void Initialize()
    {
        if (autoFindReferences)
            AutoFindReferences();

        ApplyTrainingStabilityTuning();
        CacheStartPose();
        RefreshTerminalSubscription();
        EnableResidualMode();
        agentInitialized = true;
    }

    protected override void OnEnable()
    {
        base.OnEnable();
        RefreshTerminalSubscription();
    }

    protected override void OnDisable()
    {
        if (publishZeroCmdOnDisable && !externalRosCmdInputOnly)
            PublishVehicleCmdFromAction(0f, 0f, 1f);

        UnsubscribeTerminalEvent();
        base.OnDisable();
    }

    void OnDestroy()
    {
        UnsubscribeTerminalEvent();
    }

    void FixedUpdate()
    {
        SyncToFollowTarget();

        if (externalRosCmdInputOnly)
        {
            if (regressionDrivingController != null && !regressionDrivingController.predictionOnlyMode)
                regressionDrivingController.predictionOnlyMode = true;

            if (wheelController != null && !wheelController.externalControlEnabled)
                wheelController.externalControlEnabled = true;
        }

        if (!externalRosCmdInputOnly &&
            requestDecisionInFixedUpdateWithoutRequester &&
            decisionRequester == null)
            RequestDecision();
    }

    public override void OnEpisodeBegin()
    {
        if (autoFindReferences)
            AutoFindReferences();

        ApplyTrainingStabilityTuning();
        RefreshTerminalSubscription();
        SyncToFollowTarget();

        if (resetVehicleTransformOnEpisodeBegin)
        {
            bool forceStartRotationForResidualRl =
                episodeStartTransform != null &&
                regressionDrivingController != null &&
                regressionDrivingController.predictionOnlyMode;

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
            ResetVehicleState();
        }

        EnableResidualMode();

        // [DEBUG] ResetForEpisodeRestart 전후 base값 확인
        // forceInference: false → TeleportRoot 직후 카메라 Transform이 아직 이전 위치(사고 장면)를
        // 가리킬 수 있으므로 즉시 추론하지 않고, 다음 Update()에서 물리 정착 후 추론하게 함.
        if (regressionDrivingController != null)
        {
            Debug.Log($"[RLAgent.OnEpisodeBegin] BEFORE ResetForEpisodeRestart: base steer={regressionDrivingController.GetPredictedSteering():F4}, throttle={regressionDrivingController.GetPredictedThrottle():F4}");
            regressionDrivingController.ResetForEpisodeRestart(forceInference: false);
            Debug.Log($"[RLAgent.OnEpisodeBegin] AFTER  ResetForEpisodeRestart: base steer={regressionDrivingController.GetPredictedSteering():F4}, throttle={regressionDrivingController.GetPredictedThrottle():F4}");
        }

        // Episode reset must have a single owner to avoid double-reset and index drift.
        if (episodeEvaluator != null)
        {
            if (!episodeEvaluator.IsEpisodeActive() || episodeEvaluator.IsTerminalReached())
                episodeEvaluator.BeginEpisode();
        }
        else if (progressRewardProvider != null)
        {
            progressRewardProvider.ResetRewardState();
        }

        lastTerminalReason = "None";
        lastConsumedStepReward = 0f;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        SyncToFollowTarget();

        // --- 환경 상태 (8) ---
        float speed = wheelController != null ? wheelController.GetSpeedMS() : 0f;
        float lateralErrorAbs = progressRewardProvider != null ? progressRewardProvider.GetCurrentLateralError() : 0f;
        float progressRatio = progressRewardProvider != null ? progressRewardProvider.GetPathProgressRatio() : 0f;
        float ttc = collisionWarningPublisher != null
            ? collisionWarningPublisher.GetTimeToCollision()
            : float.PositiveInfinity;
        float warningLevelNorm = collisionWarningPublisher != null
            ? Mathf.Clamp01((int)collisionWarningPublisher.GetWarningLevel() / 6f)
            : 0f;
        float stopLineDistance = trafficLightDecisionEngine != null
            ? trafficLightDecisionEngine.GetDecisionDistance()
            : float.PositiveInfinity;

        float headingErrorDeg = ComputeHeadingErrorDeg(out float signedLateralError);
        lastHeadingErrorDeg = headingErrorDeg;
        lastSignedLateralError = signedLateralError;

        sensor.AddObservation(NormalizeSigned(speed, speedNormalize));                    // 0
        sensor.AddObservation(Normalize01(lateralErrorAbs, lateralErrorNormalize, 0f));   // 1
        sensor.AddObservation(NormalizeSigned(signedLateralError, lateralErrorNormalize)); // 2
        sensor.AddObservation(NormalizeSigned(headingErrorDeg, headingErrorNormalizeDeg)); // 3
        sensor.AddObservation(Mathf.Clamp01(progressRatio));                              // 4
        sensor.AddObservation(Normalize01(ttc, ttcNormalizeSeconds, 1f));                  // 5
        sensor.AddObservation(warningLevelNorm);                                           // 6
        sensor.AddObservation(Normalize01(stopLineDistance, stopLineDistanceNormalize, 1f)); // 7

        // --- 현재 차량 입력 (3) ---
        float currentSteer = wheelController != null ? wheelController.GetSteeringInput() : 0f;
        float currentThrottle = wheelController != null ? wheelController.GetThrottleInput() : 0f;
        float currentBrake = wheelController != null ? wheelController.GetBrakeInput() : 0f;
        sensor.AddObservation(Mathf.Clamp(currentSteer, -1f, 1f));                        // 8
        sensor.AddObservation(Mathf.Clamp(currentThrottle, -1f, 1f));                     // 9
        sensor.AddObservation(Mathf.Clamp01(currentBrake));                                // 10

        // --- 모방학습 base 예측값 (2) ---
        float baseSteering = regressionDrivingController != null ? regressionDrivingController.GetPredictedSteering() : 0f;
        float baseThrottle = regressionDrivingController != null ? regressionDrivingController.GetPredictedThrottle() : 0f;
        sensor.AddObservation(Mathf.Clamp(baseSteering, -1f, 1f));                        // 11
        sensor.AddObservation(Mathf.Clamp(baseThrottle, 0f, 1f));                         // 12

        lastBaseSteering = baseSteering;
        lastBaseThrottle = baseThrottle;

        // --- 신호등 one-hot (3) ---
        if (includeTrafficDecisionOneHot)
        {
            int decision = trafficLightDecisionEngine != null ? (int)trafficLightDecisionEngine.GetDecision() : 0;
            sensor.AddObservation(decision == 0 ? 1f : 0f);                               // 13
            sensor.AddObservation(decision == 1 ? 1f : 0f);                               // 14
            sensor.AddObservation(decision == 2 ? 1f : 0f);                               // 15
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        ApplyResidualAction(actions);
        lastConsumedStepReward = ConsumeAndApplyStepReward();
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Heuristic 모드: delta=0 → 모방학습 출력 그대로 사용
        var continuous = actionsOut.ContinuousActions;
        if (continuous.Length > 0)
            continuous[0] = 0f;
        if (continuous.Length > 1)
            continuous[1] = 0f;
    }

    void AutoFindReferences()
    {
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

        if (decisionRequester == null)
            decisionRequester = GetComponent<DecisionRequester>();

        if (followTargetTransform == null && wheelController != null)
            followTargetTransform = wheelController.transform;

        if (rootArticulation == null)
            rootArticulation = ResolveArticulationRoot(wheelController != null ? wheelController.transform : null);

        if (rootArticulation == null)
            rootArticulation = ResolveArticulationRoot(followTargetTransform);

        if (rootArticulation == null)
            rootArticulation = ResolveArticulationRoot(transform);
    }

    void RefreshTerminalSubscription()
    {
        if (subscribedEvaluator == episodeEvaluator)
            return;

        if (subscribedEvaluator != null)
            subscribedEvaluator.OnEpisodeTerminated -= OnEpisodeTerminatedByEvaluator;

        subscribedEvaluator = episodeEvaluator;

        if (subscribedEvaluator != null)
            subscribedEvaluator.OnEpisodeTerminated += OnEpisodeTerminatedByEvaluator;
    }

    void UnsubscribeTerminalEvent()
    {
        if (subscribedEvaluator != null)
            subscribedEvaluator.OnEpisodeTerminated -= OnEpisodeTerminatedByEvaluator;

        subscribedEvaluator = null;
    }

    void OnEpisodeTerminatedByEvaluator(RLEpisodeEvaluator evaluator)
    {
        if (episodeEvaluator != null && evaluator != episodeEvaluator)
            return;

        if (!agentInitialized)
            return;

        ConsumeAndApplyStepReward();

        if (evaluator != null && evaluator.IsEpisodeSuccess())
            AddReward(successTerminalReward);
        else
            AddReward(failureTerminalPenalty);

        lastTerminalReason = evaluator != null ? evaluator.GetTerminalReason() : "Unknown";
        EndEpisode();
    }

    /// <summary>
    /// 모방학습을 predictionOnly 모드로 켜고, RL 출력이 제어 우선권을 갖도록 설정.
    /// </summary>
    void EnableResidualMode()
    {
        if (externalRosCmdInputOnly)
        {
            if (regressionDrivingController != null)
            {
                regressionDrivingController.predictionOnlyMode = true;
            }

            if (wheelController != null)
                wheelController.externalControlEnabled = true;

            return;
        }

        if (regressionDrivingController != null)
        {
            regressionDrivingController.isAutonomousMode = true;
            regressionDrivingController.predictionOnlyMode = true;
        }

        if (wheelController != null && !ShouldUseTopicActuation())
            wheelController.externalControlEnabled = true;
    }

    void ApplyTrainingStabilityTuning()
    {
        if (!applyTrainingStabilityTuning)
            return;

        residualSteerScale = Mathf.Clamp(tunedResidualSteerScale, 0.12f, 0.2f);
        residualAccelScale = Mathf.Clamp(tunedResidualAccelScale, 0.1f, 0.15f);
        warningBrake = Mathf.Clamp(tunedWarningBrake, 0.1f, 0.2f);
    }

    void CacheStartPose()
    {
        Transform follow = followTargetTransform != null ? followTargetTransform : transform;
        bool forceStartRotationForResidualRl =
            episodeStartTransform != null &&
            regressionDrivingController != null &&
            regressionDrivingController.predictionOnlyMode;

        if (episodeStartTransform != null)
        {
            startPosition = episodeStartTransform.position;
            if (!useEpisodeStartTransformHeight && follow != null)
                startPosition.y = follow.position.y;

            if (forceStartRotationForResidualRl || useEpisodeStartTransformRotation || follow == null)
                startRotation = episodeStartTransform.rotation;
            else
                startRotation = follow.rotation;
        }
        else
        {
            startPosition = follow.position;
            startRotation = follow.rotation;
        }

        hasStartPose = true;
    }

    void ResetVehicleState()
    {
        if (!hasStartPose)
            CacheStartPose();

        Transform resetTarget = followTargetTransform != null
            ? followTargetTransform
            : (wheelController != null ? wheelController.transform : transform);

        ArticulationBody articulationRoot = ResolveArticulationRoot(resetTarget);
        if (hasStartPose)
        {
            if (articulationRoot != null)
                articulationRoot.TeleportRoot(startPosition, startRotation);
            else
                resetTarget.SetPositionAndRotation(startPosition, startRotation);

            // TeleportRoot 후 카메라 등 자식 Transform이 즉시 반영되도록 동기화.
            // 이 호출 없이는 즉시 RunInference()할 때 카메라가 이전 위치(사고 장면)를 볼 수 있음.
            Physics.SyncTransforms();
        }

        if (resetRootArticulationVelocity)
        {
            ArticulationBody velocityRoot = articulationRoot != null ? articulationRoot : rootArticulation;
            if (velocityRoot == null)
                velocityRoot = ResolveArticulationRoot(resetTarget);

            if (velocityRoot != null)
            {
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

        if (wheelController != null)
        {
            wheelController.SetSteering(0f);
            wheelController.SetThrottle(0f);
            wheelController.SetBrake(0f);
        }

        SyncToFollowTarget();
    }

    ArticulationBody ResolveArticulationRoot(Transform target)
    {
        if (target == null)
            return null;

        ArticulationBody node = target.GetComponent<ArticulationBody>() ?? target.GetComponentInParent<ArticulationBody>();
        while (node != null && !node.isRoot)
            node = node.transform.parent != null ? node.transform.parent.GetComponent<ArticulationBody>() : null;

        return node;
    }

    void ApplyResidualAction(ActionBuffers actions)
    {
        if (wheelController == null)
            return;

        if (externalRosCmdInputOnly)
        {
            wheelController.externalControlEnabled = true;
            lastActionPublishedToRos = false;
            return;
        }

        var continuous = actions.ContinuousActions;
        float deltaSteer = continuous.Length > 0 ? Mathf.Clamp(continuous[0], -1f, 1f) * residualSteerScale : 0f;
        float deltaAccel = continuous.Length > 1 ? Mathf.Clamp(continuous[1], -1f, 1f) * residualAccelScale : 0f;

        // 모방학습 base 예측값
        float baseSteering = regressionDrivingController != null ? regressionDrivingController.GetPredictedSteering() : 0f;
        float baseThrottle = regressionDrivingController != null ? regressionDrivingController.GetPredictedThrottle() : 0f;

        // [DEBUG] base값이 고정값인지 확인 (매 스텝 출력은 과도하므로 의심 범위에서만)
        if (Mathf.Abs(baseSteering - (-0.384f)) < 0.005f && Mathf.Abs(baseThrottle - 0.185f) < 0.005f)
            Debug.LogWarning($"[RLAgent.Action] ⚠️ STALE BASE DETECTED! steer={baseSteering:F4}, throttle={baseThrottle:F4} | delta=({deltaSteer:F4},{deltaAccel:F4})");

        // Residual: base + delta
        float finalSteer = Mathf.Clamp(baseSteering + deltaSteer, -1f, 1f);
        float accelMin = allowReverse ? -1f : 0f;
        float finalAccel = Mathf.Clamp(baseThrottle + deltaAccel, accelMin, 1f);

        float throttle;
        float brake;

        if (allowReverse)
        {
            throttle = finalAccel;
            brake = 0f;
        }
        else
        {
            if (finalAccel >= 0f)
            {
                throttle = finalAccel;
                brake = 0f;
            }
            else
            {
                throttle = 0f;
                brake = -finalAccel;
            }
        }

        if (enableSafetyOverride)
            ApplySafetyOverride(ref throttle, ref brake);

        float clampedThrottle = Mathf.Clamp(throttle, allowReverse ? -1f : 0f, 1f);
        float clampedBrake = Mathf.Clamp01(brake);

        bool publishedToRos = PublishVehicleCmdFromAction(finalSteer, clampedThrottle, clampedBrake);
        bool useTopicActuation = ShouldUseTopicActuation() && publishedToRos;

        if (!useTopicActuation)
        {
            wheelController.externalControlEnabled = true;
            wheelController.SetSteering(finalSteer);
            wheelController.SetThrottle(clampedThrottle);
            wheelController.SetBrake(clampedBrake);
        }

        lastDeltaSteering = deltaSteer;
        lastDeltaAccel = deltaAccel;
        lastAppliedSteer = finalSteer;
        lastAppliedThrottle = clampedThrottle;
        lastAppliedBrake = clampedBrake;
        lastActionPublishedToRos = publishedToRos;
    }

    bool ShouldUseTopicActuation()
    {
        return outputViaVehicleCmdTopic &&
               preferTopicActuationWhenSubscriberPresent &&
               vehicleCmdSubscriber != null;
    }

    bool EnsureRosCmdPublisherReady()
    {
        if (!outputViaVehicleCmdTopic)
            return false;

        if (rosCmdPublisherReady && ros != null)
            return true;

        try
        {
            ros = ROSConnection.GetOrCreateInstance();
            resolvedCmdTopicName = RosTopicNamespace.Resolve(gameObject, vehicleCmdTopicName);
            ros.RegisterPublisher<TwistMsg>(resolvedCmdTopicName);
            rosCmdPublisherReady = true;
            return true;
        }
        catch (System.Exception e)
        {
            rosCmdPublisherReady = false;
            Debug.LogWarning($"[AutoDriverRLAgent] ROS cmd publisher init 실패: {e.Message}");
            return false;
        }
    }

    bool PublishVehicleCmdFromAction(float steerInput, float throttleInput, float brakeInput)
    {
        if (!EnsureRosCmdPublisherReady())
            return false;

        float safeForward = Mathf.Max(0.01f, cmdTopicMaxForwardSpeed);
        float safeReverse = Mathf.Max(0.01f, cmdTopicMaxReverseSpeed);
        float safeYaw = Mathf.Max(0.01f, cmdTopicMaxYawRateForFullSteer);

        float linearX;
        if (allowReverse && throttleInput < 0f)
        {
            linearX = -Mathf.Clamp01(-throttleInput) * safeReverse;
        }
        else if (brakeInput > 0.001f)
        {
            linearX = -Mathf.Clamp01(brakeInput) * safeReverse;
        }
        else
        {
            linearX = Mathf.Clamp01(throttleInput) * safeForward;
        }

        float angularZ = Mathf.Clamp(steerInput, -1f, 1f) * safeYaw;

        var twist = new TwistMsg
        {
            linear = new Vector3Msg(linearX, 0.0, 0.0),
            angular = new Vector3Msg(0.0, 0.0, angularZ)
        };

        try
        {
            ros.Publish(resolvedCmdTopicName, twist);
            lastPublishedCmdLinearX = linearX;
            lastPublishedCmdAngularZ = angularZ;
            return true;
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"[AutoDriverRLAgent] ROS cmd publish 실패: {e.Message}");
            rosCmdPublisherReady = false;
            return false;
        }
    }

    void ApplySafetyOverride(ref float throttle, ref float brake)
    {
        if (collisionWarningPublisher != null)
        {
            CollisionWarningPublisher.WarningLevel level = collisionWarningPublisher.GetWarningLevel();
            if (level >= CollisionWarningPublisher.WarningLevel.EmergencyStop)
            {
                throttle = 0f;
                brake = Mathf.Max(brake, emergencyBrake);
            }
            else if (level >= CollisionWarningPublisher.WarningLevel.Brake)
            {
                throttle = 0f;
                brake = Mathf.Max(brake, brakeLevelBrake);
            }
            else if (level >= CollisionWarningPublisher.WarningLevel.Warning)
            {
                throttle = Mathf.Min(throttle, 0.15f);
                float speed = wheelController != null ? wheelController.GetSpeedMS() : 0f;
                if (Mathf.Abs(speed) >= Mathf.Max(0f, warningBrakeMinSpeed))
                    brake = Mathf.Max(brake, warningBrake);
            }
            else if (level >= CollisionWarningPublisher.WarningLevel.SlowDown)
            {
                throttle = Mathf.Min(throttle, cautionThrottleScale);
            }
        }

        if (trafficLightDecisionEngine != null)
        {
            TrafficLightDecisionEngine.TrafficDecision decision = trafficLightDecisionEngine.GetDecision();
            if (decision == TrafficLightDecisionEngine.TrafficDecision.Stop)
            {
                throttle = 0f;
                brake = Mathf.Max(brake, trafficLightDecisionEngine.GetRecommendedBrake());
            }
            else if (decision == TrafficLightDecisionEngine.TrafficDecision.Caution)
            {
                throttle = Mathf.Min(throttle, trafficLightDecisionEngine.GetRecommendedThrottleScale());
                brake = Mathf.Max(brake, trafficLightDecisionEngine.GetRecommendedBrake());
            }
        }
    }

    float ConsumeAndApplyStepReward()
    {
        if (progressRewardProvider == null)
            return 0f;

        float reward = progressRewardProvider.ConsumeStepReward();
        if (IsFinite(reward) && !Mathf.Approximately(reward, 0f))
            AddReward(reward);

        return IsFinite(reward) ? reward : 0f;
    }

    float ComputeHeadingErrorDeg(out float signedLateralError)
    {
        signedLateralError = 0f;

        if (progressRewardProvider == null ||
            progressRewardProvider.progressWaypoints == null ||
            progressRewardProvider.progressWaypoints.Length < 2)
        {
            return 0f;
        }

        Transform[] waypoints = progressRewardProvider.progressWaypoints;
        bool looped = progressRewardProvider.isLoopedPath;
        Vector3 position = transform.position;

        float bestDistSq = float.PositiveInfinity;
        Vector3 bestTangent = transform.forward;
        float bestSignedLateral = 0f;

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
            if (abLenSq < 1e-6f)
                continue;

            float t = Mathf.Clamp01(Vector3.Dot(position - a, ab) / abLenSq);
            Vector3 projected = a + (ab * t);
            Vector3 offset = position - projected;
            float distSq = offset.sqrMagnitude;

            if (distSq < bestDistSq)
            {
                bestDistSq = distSq;
                bestTangent = ab.normalized;

                float sideSign = Mathf.Sign(Vector3.Dot(Vector3.Cross(bestTangent, offset), Vector3.up));
                if (Mathf.Approximately(sideSign, 0f))
                    sideSign = 1f;

                bestSignedLateral = Mathf.Sqrt(distSq) * sideSign;
            }
        }

        signedLateralError = bestSignedLateral;
        return Vector3.SignedAngle(bestTangent, transform.forward, Vector3.up);
    }

    static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    float NormalizeSigned(float value, float denom)
    {
        if (!IsFinite(value))
            return 0f;

        float safeDenom = Mathf.Max(1e-4f, denom);
        return Mathf.Clamp(value / safeDenom, -1f, 1f);
    }

    float Normalize01(float value, float maxRange, float defaultWhenInvalid)
    {
        if (!IsFinite(value) || value < 0f)
            return defaultWhenInvalid;

        float safeRange = Mathf.Max(1e-4f, maxRange);
        return Mathf.Clamp01(value / safeRange);
    }

    public float GetLastBaseSteering() => lastBaseSteering;
    public float GetLastBaseThrottle() => lastBaseThrottle;
    public float GetLastDeltaSteering() => lastDeltaSteering;
    public float GetLastDeltaAccel() => lastDeltaAccel;
    public float GetLastAppliedSteer() => lastAppliedSteer;
    public float GetLastAppliedThrottle() => lastAppliedThrottle;
    public float GetLastAppliedBrake() => lastAppliedBrake;
    public string GetActionDebugSummary()
    {
        return
            $"base=({lastBaseSteering:F3},{lastBaseThrottle:F3}) " +
            $"delta=({lastDeltaSteering:F3},{lastDeltaAccel:F3}) " +
            $"final=({lastAppliedSteer:F3},{lastAppliedThrottle:F3},{lastAppliedBrake:F3}) " +
            $"stepReward={lastConsumedStepReward:F3}";
    }

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
