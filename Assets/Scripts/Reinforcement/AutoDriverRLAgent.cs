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

    [Header("Residual RL")]
    [Tooltip("RL 보정값의 최대 크기 (steering)")]
    [Range(0.01f, 1f)] public float residualSteerScale = 0.3f;
    [Tooltip("RL 보정값의 최대 크기 (accel)")]
    [Range(0.01f, 1f)] public float residualAccelScale = 0.3f;

    [Header("Action Mapping")]
    [Tooltip("false면 음수 accel은 브레이크로 처리")]
    public bool allowReverse = false;
    public bool enableSafetyOverride = true;
    [Range(0f, 1f)] public float cautionThrottleScale = 0.55f;
    [Range(0f, 1f)] public float warningBrake = 0.35f;
    [Range(0f, 1f)] public float brakeLevelBrake = 0.8f;
    [Range(0f, 1f)] public float emergencyBrake = 1f;

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

    private Vector3 startPosition;
    private Quaternion startRotation;
    private bool hasStartPose = false;
    private ArticulationBody rootArticulation;
    private RLEpisodeEvaluator subscribedEvaluator;
    private bool agentInitialized = false;

    public override void Initialize()
    {
        if (autoFindReferences)
            AutoFindReferences();

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

        if (requestDecisionInFixedUpdateWithoutRequester && decisionRequester == null)
            RequestDecision();
    }

    public override void OnEpisodeBegin()
    {
        if (autoFindReferences)
            AutoFindReferences();

        RefreshTerminalSubscription();
        SyncToFollowTarget();

        if (resetVehicleTransformOnEpisodeBegin)
            ResetVehicleState();

        EnableResidualMode();

        if (progressRewardProvider != null)
            progressRewardProvider.ResetRewardState();

        if (episodeEvaluator != null)
            episodeEvaluator.BeginEpisode();

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

        if (decisionRequester == null)
            decisionRequester = GetComponent<DecisionRequester>();

        if (rootArticulation == null)
            rootArticulation = GetComponentInParent<ArticulationBody>();

        if (followTargetTransform == null && wheelController != null)
            followTargetTransform = wheelController.transform;
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
    /// 모방학습을 predictionOnly 모드로 켜고, RL이 차량을 직접 제어하도록 설정.
    /// </summary>
    void EnableResidualMode()
    {
        if (regressionDrivingController != null)
        {
            regressionDrivingController.isAutonomousMode = true;
            regressionDrivingController.predictionOnlyMode = true;
        }

        if (wheelController != null)
            wheelController.externalControlEnabled = true;
    }

    void CacheStartPose()
    {
        Transform pose = episodeStartTransform != null
            ? episodeStartTransform
            : (followTargetTransform != null ? followTargetTransform : transform);
        startPosition = pose.position;
        startRotation = pose.rotation;
        hasStartPose = true;
    }

    void ResetVehicleState()
    {
        if (!hasStartPose)
            CacheStartPose();

        if (hasStartPose)
            transform.SetPositionAndRotation(startPosition, startRotation);

        if (resetRootArticulationVelocity && rootArticulation != null)
        {
            rootArticulation.velocity = Vector3.zero;
            rootArticulation.angularVelocity = Vector3.zero;
        }

        if (wheelController != null)
        {
            wheelController.SetSteering(0f);
            wheelController.SetThrottle(0f);
            wheelController.SetBrake(0f);
        }
    }

    void ApplyResidualAction(ActionBuffers actions)
    {
        if (wheelController == null)
            return;

        var continuous = actions.ContinuousActions;
        float deltaSteer = continuous.Length > 0 ? Mathf.Clamp(continuous[0], -1f, 1f) * residualSteerScale : 0f;
        float deltaAccel = continuous.Length > 1 ? Mathf.Clamp(continuous[1], -1f, 1f) * residualAccelScale : 0f;

        // 모방학습 base 예측값
        float baseSteering = regressionDrivingController != null ? regressionDrivingController.GetPredictedSteering() : 0f;
        float baseThrottle = regressionDrivingController != null ? regressionDrivingController.GetPredictedThrottle() : 0f;

        // Residual: base + delta
        float finalSteer = Mathf.Clamp(baseSteering + deltaSteer, -1f, 1f);
        float finalAccel = Mathf.Clamp(baseThrottle + deltaAccel, allowReverse ? -1f : -1f, 1f);

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

        wheelController.externalControlEnabled = true;
        wheelController.SetSteering(finalSteer);
        wheelController.SetThrottle(Mathf.Clamp(throttle, allowReverse ? -1f : 0f, 1f));
        wheelController.SetBrake(Mathf.Clamp01(brake));

        lastDeltaSteering = deltaSteer;
        lastDeltaAccel = deltaAccel;
        lastAppliedSteer = finalSteer;
        lastAppliedThrottle = throttle;
        lastAppliedBrake = brake;
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
