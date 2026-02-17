using UnityEngine;

/// <summary>
/// 신호등 인지 결과(색/신뢰도/박스)와 정지선 거리를 결합해 주행 결정을 생성합니다.
/// Autoware식 파이프라인(인지 -> relevance gate -> rule decision)을 단순화한 버전입니다.
/// </summary>
public class TrafficLightDecisionEngine : MonoBehaviour
{
    public enum TrafficDecision
    {
        Go = 0,
        Caution = 1,
        Stop = 2
    }

    [Header("References")]
    public TrafficLightStateSubscriber trafficLightSubscriber;
    public StopLineStateSubscriber stopLineSubscriber;
    public WheelTest wheelController;
    [Tooltip("차량 기준 Transform (미할당 시 현재 오브젝트)")]
    public Transform egoTransform;
    [Tooltip("정지선 Transform (있으면 최우선 사용)")]
    public Transform stopLineTransform;
    public bool autoFindReferences = true;

    [Header("Perception Relevance Gate")]
    [Range(0f, 1f)] public float minStateConfidence = 0.35f;
    [Range(0f, 1f)] public float minBBoxAreaNormalized = 0.0012f;
    [Range(0f, 1f)] public float laneCenterX = 0.5f;
    [Range(0.01f, 0.5f)] public float maxLaneCenterOffset = 0.22f;

    [Header("Stop Line Perception Distance (fallback)")]
    [Tooltip("true면 /stop_line/perception을 정지선 거리 소스로 사용")]
    public bool useStopLinePerception = true;
    [Range(0f, 1f)] public float minStopLineConfidence = 0.45f;
    [Tooltip("distance_from_bottom_norm가 nearNorm일 때 거리(m)")]
    public float stopLineNearDistanceMeters = 2.0f;
    [Tooltip("distance_from_bottom_norm가 farNorm일 때 거리(m)")]
    public float stopLineFarDistanceMeters = 18.0f;
    [Range(0f, 1f)] public float stopLineNearNorm = 0.05f;
    [Range(0f, 1f)] public float stopLineFarNorm = 0.70f;
    [Tooltip("StopLine Perception 거리 상한(m)")]
    public float maxStopLinePerceptionDistance = 30f;

    [Header("Rule Thresholds")]
    public float maxRedReactionDistance = 12f;
    public float maxYellowReactionDistance = 18f;
    [Tooltip("정지판단 안전 버퍼 (m)")]
    public float stopDistanceBuffer = 0.8f;
    [Tooltip("반응시간 (s)")]
    public float reactionTime = 0.6f;
    [Tooltip("편안한 감속도 (m/s^2)")]
    public float comfortableDeceleration = 1.2f;
    [Tooltip("노란불에서 정지 불가 시 주의통과 모드")]
    public bool cautionOnUnstoppableYellow = true;

    [Header("Output (Read Only)")]
    [SerializeField] private bool isSignalRelevant = false;
    [SerializeField] private float gateStateConfidence = 0f;
    [SerializeField] private float gateBBoxArea = 0f;
    [SerializeField] private bool gateLaneOk = false;
    [SerializeField] private float stopLineDistance = float.PositiveInfinity;
    [SerializeField] private float stopLineDistancePerception = float.PositiveInfinity;
    [SerializeField] private string stopLineDistanceSource = "none";
    [SerializeField] private float fusedDecisionDistance = float.PositiveInfinity;
    [SerializeField] private TrafficDecision currentDecision = TrafficDecision.Go;
    [SerializeField] private string decisionReason = "NoSignal";

    [Header("Recommended Control")]
    [Range(0f, 1f)] public float cautionThrottleScale = 0.35f;
    [Range(0f, 1f)] public float cautionBrake = 0.25f;
    [Range(0f, 1f)] public float maxStopBrake = 1.0f;
    [SerializeField] private float recommendedThrottleScale = 1f;
    [SerializeField] private float recommendedBrake = 0f;

    void Start()
    {
        if (autoFindReferences)
            AutoFindReferences();
    }

    void Update()
    {
        EvaluateDecision();
    }

    void AutoFindReferences()
    {
        if (trafficLightSubscriber == null)
            trafficLightSubscriber = FindObjectOfType<TrafficLightStateSubscriber>();

        if (stopLineSubscriber == null)
            stopLineSubscriber = FindObjectOfType<StopLineStateSubscriber>();

        if (wheelController == null)
            wheelController = FindObjectOfType<WheelTest>();

        if (egoTransform == null)
            egoTransform = transform;
    }

    void EvaluateDecision()
    {
        if (trafficLightSubscriber == null)
        {
            currentDecision = TrafficDecision.Go;
            isSignalRelevant = false;
            decisionReason = "NoSubscriber";
            recommendedThrottleScale = 1f;
            recommendedBrake = 0f;
            return;
        }

        string state = trafficLightSubscriber.GetCurrentState();
        float stateConf = trafficLightSubscriber.GetStateConfidence();
        Vector2 center = trafficLightSubscriber.GetBBoxCenterNormalized();
        float area = trafficLightSubscriber.GetBBoxAreaNormalized();
        gateStateConfidence = stateConf;
        gateBBoxArea = area;

        bool confidenceOk = stateConf >= minStateConfidence;
        bool areaOk = area >= minBBoxAreaNormalized;
        bool laneOk = center.x >= 0f && Mathf.Abs(center.x - laneCenterX) <= maxLaneCenterOffset;
        gateLaneOk = laneOk;
        bool hasSignalState = state != "none";

        isSignalRelevant = hasSignalState && confidenceOk && areaOk && laneOk;

        stopLineDistance = ComputeStopLineDistanceFromTransform();
        stopLineDistancePerception = ComputeStopLineDistanceFromPerception();
        stopLineDistanceSource = "none";
        if (!float.IsInfinity(stopLineDistance))
            stopLineDistanceSource = "transform";
        else if (!float.IsInfinity(stopLineDistancePerception))
            stopLineDistanceSource = "perception";

        float effectiveStopLineDistance = !float.IsInfinity(stopLineDistance)
            ? stopLineDistance
            : stopLineDistancePerception;
        fusedDecisionDistance = effectiveStopLineDistance;

        currentDecision = TrafficDecision.Go;
        decisionReason = "DefaultGo";
        recommendedThrottleScale = 1f;
        recommendedBrake = 0f;

        if (!isSignalRelevant)
        {
            decisionReason = "Irrelevant";
            return;
        }

        float speed = wheelController != null ? Mathf.Max(0f, wheelController.GetSpeedMS()) : 0f;
        float stoppingDistance = ComputeStoppingDistance(speed);
        bool canComfortablyStop = stoppingDistance + stopDistanceBuffer <= fusedDecisionDistance;

        if (state == "red")
        {
            if (!float.IsInfinity(fusedDecisionDistance) && fusedDecisionDistance <= maxRedReactionDistance)
            {
                currentDecision = TrafficDecision.Stop;
                decisionReason = $"RedStop(dist={fusedDecisionDistance:F2}m <= {maxRedReactionDistance:F2}m)";
                SetStopRecommendation(speed, fusedDecisionDistance);
            }
            else
            {
                decisionReason = $"RedTooFar(dist={fusedDecisionDistance:F2}m, src={stopLineDistanceSource})";
            }
            return;
        }

        if (state == "yellow")
        {
            if (float.IsInfinity(fusedDecisionDistance) || fusedDecisionDistance > maxYellowReactionDistance)
            {
                decisionReason = $"YellowTooFar(dist={fusedDecisionDistance:F2}m)";
                return;
            }

            if (canComfortablyStop)
            {
                currentDecision = TrafficDecision.Stop;
                decisionReason = $"YellowStop(canStop, stopDist={stoppingDistance:F2}m)";
                SetStopRecommendation(speed, fusedDecisionDistance);
            }
            else
            {
                currentDecision = cautionOnUnstoppableYellow ? TrafficDecision.Caution : TrafficDecision.Go;
                decisionReason = $"YellowPass(cannotStop, stopDist={stoppingDistance:F2}m)";
                if (currentDecision == TrafficDecision.Caution)
                {
                    recommendedThrottleScale = cautionThrottleScale;
                    recommendedBrake = cautionBrake;
                }
            }
            return;
        }

        if (state == "green")
        {
            decisionReason = "GreenGo";
        }
    }

    float ComputeStopLineDistanceFromTransform()
    {
        if (stopLineTransform == null || egoTransform == null)
            return float.PositiveInfinity;

        Vector3 toStopLine = stopLineTransform.position - egoTransform.position;
        float longitudinal = Vector3.Dot(toStopLine, egoTransform.forward);
        if (longitudinal <= 0f)
            return float.PositiveInfinity;

        return longitudinal;
    }

    float ComputeStopLineDistanceFromPerception()
    {
        if (!useStopLinePerception || stopLineSubscriber == null)
            return float.PositiveInfinity;

        if (!stopLineSubscriber.IsStableDetected())
            return float.PositiveInfinity;

        float conf = stopLineSubscriber.GetConfidence();
        if (conf < minStopLineConfidence)
            return float.PositiveInfinity;

        float norm = stopLineSubscriber.GetDistanceFromBottomNormalized();
        if (norm < 0f)
            return float.PositiveInfinity;

        float nearNorm = Mathf.Min(stopLineNearNorm, stopLineFarNorm - 0.001f);
        float farNorm = Mathf.Max(stopLineFarNorm, nearNorm + 0.001f);
        float t = Mathf.InverseLerp(nearNorm, farNorm, norm);

        float nearMeters = Mathf.Max(0.1f, stopLineNearDistanceMeters);
        float farMeters = Mathf.Max(nearMeters, stopLineFarDistanceMeters);
        float dist = Mathf.Lerp(nearMeters, farMeters, t);
        dist = Mathf.Clamp(dist, nearMeters, Mathf.Max(nearMeters, maxStopLinePerceptionDistance));
        return dist;
    }

    float ComputeStoppingDistance(float speed)
    {
        float decel = Mathf.Max(0.1f, comfortableDeceleration);
        return (speed * reactionTime) + ((speed * speed) / (2f * decel));
    }

    void SetStopRecommendation(float speed, float dist)
    {
        recommendedThrottleScale = 0f;
        if (float.IsInfinity(dist) || dist <= 0f)
        {
            recommendedBrake = maxStopBrake;
            return;
        }

        float requiredDecel = (speed * speed) / Mathf.Max(0.1f, 2f * dist);
        // 0~3m/s^2 범위를 0~1 brake로 단순 맵핑
        recommendedBrake = Mathf.Clamp01(requiredDecel / 3f);
        recommendedBrake = Mathf.Max(recommendedBrake, 0.35f);
        recommendedBrake = Mathf.Min(recommendedBrake, maxStopBrake);
    }

    public TrafficDecision GetDecision() => currentDecision;
    public string GetDecisionReason() => decisionReason;
    public bool IsSignalRelevant() => isSignalRelevant;
    public float GetGateStateConfidence() => gateStateConfidence;
    public float GetGateBBoxArea() => gateBBoxArea;
    public bool GetGateLaneOk() => gateLaneOk;
    public float GetStopLineDistance() => stopLineDistance;
    public float GetStopLineDistancePerception() => stopLineDistancePerception;
    public string GetStopLineDistanceSource() => stopLineDistanceSource;
    public float GetEffectiveStopLineDistance() =>
        !float.IsInfinity(stopLineDistance) ? stopLineDistance : stopLineDistancePerception;
    public float GetDecisionDistance() => fusedDecisionDistance;
    public float GetRecommendedThrottleScale() => recommendedThrottleScale;
    public float GetRecommendedBrake() => recommendedBrake;
    public bool HasStopLineTransform() => stopLineTransform != null;
    public bool HasStopLinePerception() => stopLineSubscriber != null;
    public bool ShouldStop() => currentDecision == TrafficDecision.Stop;
    public bool ShouldCaution() => currentDecision == TrafficDecision.Caution;
}
