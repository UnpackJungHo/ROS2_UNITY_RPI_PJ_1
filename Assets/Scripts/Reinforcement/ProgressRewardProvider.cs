using System.Collections.Generic;
using System.Linq;
using UnityEngine;

/// <summary>
/// 강화학습 보상 신호 제공기 (Zone 세그먼트 기반).
/// - 주 보상: 전진 진행도 (velocity dot segment direction)
/// - 보조 보상: RewardZone 점수
/// - 보조 패널티: 충돌 위험도/신호 위반/헤딩 오차/횡오차
///
/// 기존 waypoint path-s 투영 방식을 제거하고,
/// Zone 세그먼트 MeshFilter 정점에서 도로 진행 방향을 직접 추출한다.
/// </summary>
public class ProgressRewardProvider : MonoBehaviour
{
    [Header("Trigger Target")]
    [Tooltip("트리거 감지할 대상 (base_link 등). 미할당 시 자기 자신의 OnTrigger 사용")]
    public GameObject triggerTarget;

    [Header("Progress Weights")]
    [Tooltip("전진 진행 보상 스케일 (velocity·segDirection * scale * dt)")]
    public float progressRewardScale = 1.0f;
    [Tooltip("역주행(음의 progress)에 대한 추가 패널티 배율")]
    public float reverseProgressPenaltyScale = 1.4f;
    [Header("Zone Weights")]
    [Tooltip("Zone 점수 가중치 (초당)")]
    public float zoneRewardWeight = 0.25f;
    [Tooltip("전진 진행이 없을 때 양의 Zone 보상 스케일. 0이면 전진 없이 zone 보상 0")]
    [Range(0f, 1f)] public float zoneRewardNoProgressScale = 0.0f;
    [Tooltip("이 값(m/s) 이상의 전진 속도일 때 Zone 보상을 100% 반영")]
    public float zoneRewardFullScaleProgressSpeed = 0.5f;

    [Header("Heading/Lateral Shaping")]
    [Tooltip("헤딩 오차 패널티 가중치 (초당)")]
    public float headingRewardWeight = 0.3f;
    [Tooltip("헤딩 오차 정규화 기준각(도)")]
    public float headingErrorNormalizeDeg = 45f;
    [Tooltip("횡오차 패널티 가중치 (초당)\n" +
             "lateral error는 targetZone(+zone) 중심 기준으로 측정됨.\n" +
             "Zone_m에 있으면 Zone_R1 중심까지의 거리가 그대로 패널티로 반영.")]
    public float lateralRewardWeight = 0.2f;
    [Tooltip("횡오차 정규화 기준(m) - 이 거리에서 패널티가 최대(1.0)에 도달\n" +
             "targetZone 중심 기준이므로 2~3m 수준이 적절.")]
    public float lateralErrorNormalizeM = 2f;

    [Header("Safety Penalty")]
    public CollisionWarningEngine collisionWarningEngine;
    public float cautionPenaltyPerSec = 0.05f;
    public float warningPenaltyPerSec = 0.15f;
    public float brakePenaltyPerSec = 0.35f;
    public float emergencyPenaltyPerSec = 1.2f;

    [Header("Traffic Penalty")]
    public TrafficLightDecisionEngine trafficLightDecisionEngine;
    public WheelTest wheelController;
    [Tooltip("정지 지시인데 이 속도 이상이면 위반 패널티 적용")]
    public float redViolationSpeedThreshold = 0.2f;
    public float redViolationPenaltyPerSec = 0.6f;

    [Header("Episode Guard")]
    [Tooltip("RLEpisodeEvaluator가 활성 상태일 때만 보상을 누적")]
    public bool accumulateOnlyWhenEpisodeActive = true;
    public RLEpisodeEvaluator episodeEvaluator;

    [Header("Debug (Read Only)")]
    [SerializeField] private float cumulativeReward = 0f;
    [SerializeField] private float cumulativeProgressReward = 0f;
    [SerializeField] private float cumulativeZoneReward = 0f;
    [SerializeField] private float currentLateralError = 0f;
    [SerializeField] private float cumulativeHeadingReward = 0f;
    [SerializeField] private float cumulativeLateralReward = 0f;
    [SerializeField] private float cumulativeSafetyPenalty = 0f;
    [SerializeField] private float cumulativeTrafficPenalty = 0f;
    [SerializeField] private float currentZoneScore = 0f;
    [SerializeField] private string currentZoneName = "None";
    [SerializeField] private int activeZoneCount = 0;
    [SerializeField] private float lastProgressReward = 0f;
    [SerializeField] private float lastZoneReward = 0f;
    [SerializeField] private float lastHeadingReward = 0f;
    [SerializeField] private float lastLateralReward = 0f;
    [SerializeField] private string targetZoneName = "None";
    [SerializeField] private float lastSafetyPenalty = 0f;
    [SerializeField] private float lastTrafficPenalty = 0f;
    [SerializeField] private float lastStepReward = 0f;
    [SerializeField] private float lastForwardProgress = 0f;
    [SerializeField] private float cachedHeadingErrorDegDebug = 0f;
    [SerializeField] private float cachedSignedLateralErrorDebug = 0f;

    // ── 호출처 호환용 stub (더 이상 내부 사용 안 함) ──
    [HideInInspector] public Transform[] progressWaypoints = new Transform[0];
    [HideInInspector] public bool isLoopedPath = false;

    // ── 세그먼트 방향 캐시 ──
    private struct SegmentInfo
    {
        public Vector3 direction;  // 시작→끝 정규화 벡터
        public Vector3 center;     // 시작·끝 중점
        public Vector3 right;      // Cross(up, direction)
    }
    private readonly Dictionary<RewardZone, SegmentInfo> segCache = new();

    // ── Zone 상태 ──
    private readonly HashSet<RewardZone> activeZones = new();
    private RewardZone primaryZone;
    private SegmentInfo primarySegInfo;
    private bool hasPrimaryZone = false;

    // ── Positive Zone 캐시: score > 0인 모든 zone (lateral error 기준 후보) ──
    // lateral error는 매 프레임 차량에서 횡방향 거리가 가장 가까운 +zone 중심선 기준으로 측정됨.
    // +zone이 1개면 항상 그 zone 기준, 여러 개면 현재 위치에서 가장 가까운 +zone 기준.
    private struct PositiveZoneEntry
    {
        public RewardZone zone;
        public SegmentInfo seg;
    }
    private readonly List<PositiveZoneEntry> positiveZones = new List<PositiveZoneEntry>();

    // ── 캐시 (외부에서 읽는 값) ──
    private float cachedHeadingErrorDeg = 0f;
    private float cachedSignedLateralError = 0f;

    // ── 보상 누적 ──
    private float unconsumedStepReward = 0f;

    void Start()
    {
        if (wheelController == null)
            wheelController = FindObjectOfType<WheelTest>();
        if (episodeEvaluator == null)
            episodeEvaluator = FindObjectOfType<RLEpisodeEvaluator>();

        if (triggerTarget != null && triggerTarget != gameObject)
        {
            var proxy = triggerTarget.GetComponent<ProgressRewardProxy>();
            if (proxy == null)
                proxy = triggerTarget.AddComponent<ProgressRewardProxy>();
            proxy.owner = this;
        }

        PreCachePositiveZones();
    }

    /// <summary>
    /// 씬 내 모든 RewardZone 중 score > 0인 zone을 전부 캐시한다.
    /// lateral error는 매 프레임 차량에서 가장 가까운 +zone 중심선 기준으로 동적 측정됨.
    /// +zone이 1개여도, 여러 개여도 동일하게 동작한다.
    /// </summary>
    void PreCachePositiveZones()
    {
        positiveZones.Clear();
        var allZones = FindObjectsOfType<RewardZone>(true);

        foreach (var zone in allZones)
        {
            if (zone.score <= 0f) continue;

            var col = zone.GetComponent<Collider>();
            if (col != null)
                TryCacheSegmentInfo(zone, col);

            if (segCache.TryGetValue(zone, out var info))
                positiveZones.Add(new PositiveZoneEntry { zone = zone, seg = info });
        }

        targetZoneName = positiveZones.Count > 0 ? positiveZones[0].zone.zoneName : "None";
        if (positiveZones.Count > 0)
            Debug.Log($"[ProgressRewardProvider] +zone 캐시: {positiveZones.Count}개");
    }

    /// <summary>
    /// 차량 위치에서 횡방향 거리가 가장 가까운 +zone의 SegmentInfo를 반환한다.
    /// +zone이 1개면 항상 그 zone 반환. 없으면 primarySegInfo fallback.
    /// </summary>
    SegmentInfo GetNearestPositiveSegInfo(Vector3 vehiclePos, out string zoneName)
    {
        if (positiveZones.Count == 1)
        {
            zoneName = positiveZones[0].zone.zoneName;
            return positiveZones[0].seg;
        }

        float minLateralDist = float.MaxValue;
        int nearestIdx = 0;

        for (int i = 0; i < positiveZones.Count; i++)
        {
            Vector3 offset = vehiclePos - positiveZones[i].seg.center;
            float lateralDist = Mathf.Abs(Vector3.Dot(offset, positiveZones[i].seg.right));
            if (lateralDist < minLateralDist)
            {
                minLateralDist = lateralDist;
                nearestIdx = i;
            }
        }

        zoneName = positiveZones[nearestIdx].zone.zoneName;
        return positiveZones[nearestIdx].seg;
    }

    void FixedUpdate()
    {
        if (accumulateOnlyWhenEpisodeActive &&
            episodeEvaluator != null &&
            !episodeEvaluator.IsEpisodeActive())
        {
            return;
        }

        if (!hasPrimaryZone)
            return;

        float dt = Time.fixedDeltaTime;
        UpdateZoneState();
        UpdateLateralAndHeading();
        float forwardProgress = ComputeForwardProgress();
        CalculateStepReward(dt, forwardProgress);
    }

    // ── Zone 상태 갱신 ──
    void UpdateZoneState()
    {
        activeZones.RemoveWhere(z => z == null);
        activeZoneCount = activeZones.Count;

        if (activeZoneCount == 0)
        {
            hasPrimaryZone = false;
            currentZoneName = "None";
            currentZoneScore = 0f;
            return;
        }

        if (activeZoneCount == 1)
        {
            // 1개: 그대로 primary
            foreach (var zone in activeZones)
            {
                primaryZone = zone;
                break;
            }
            currentZoneScore = primaryZone.score;
            currentZoneName = primaryZone.zoneName;
        }
        else
        {
            // 2개 이상: min(score) 보수적, bounds 큰 쪽이 primary
            float minScore = float.MaxValue;
            RewardZone largestZone = null;
            float largestVolume = -1f;

            foreach (var zone in activeZones)
            {
                if (zone.score < minScore)
                    minScore = zone.score;

                var col = zone.GetComponent<Collider>();
                if (col != null)
                {
                    Vector3 size = col.bounds.size;
                    float volume = size.x * size.y * size.z;
                    if (volume > largestVolume)
                    {
                        largestVolume = volume;
                        largestZone = zone;
                    }
                }
            }

            primaryZone = largestZone != null ? largestZone : primaryZone;
            currentZoneScore = minScore;
            currentZoneName = primaryZone.zoneName;
        }

        if (primaryZone != null && segCache.TryGetValue(primaryZone, out var info))
        {
            primarySegInfo = info;
            hasPrimaryZone = true;
        }
    }

    // ── 횡오차 + 헤딩 계산 (세그먼트 메쉬 기반) ──
    void UpdateLateralAndHeading()
    {
        if (!hasPrimaryZone) return;

        Vector3 vehiclePos = GetTrackedPosition();
        Transform vehicleT = GetTrackedTransform();

        // 횡오차: 현재 위치에서 가장 가까운 +zone 중심선 기준.
        // +zone이 없으면 primaryZone fallback.
        // → lateral=0은 오직 해당 +zone 중앙에 있을 때만 성립.
        SegmentInfo lateralRef = positiveZones.Count > 0
            ? GetNearestPositiveSegInfo(vehiclePos, out targetZoneName)
            : primarySegInfo;
        Vector3 lateralOffset = vehiclePos - lateralRef.center;
        cachedSignedLateralError = Vector3.Dot(lateralOffset, lateralRef.right);
        currentLateralError = Mathf.Abs(cachedSignedLateralError);

        // 헤딩오차: primaryZone(현재 주행 중인 zone)의 진행 방향 기준 유지.
        cachedHeadingErrorDeg = Vector3.SignedAngle(
            primarySegInfo.direction, vehicleT.forward, Vector3.up);

        // Inspector 디버그 표시
        cachedHeadingErrorDegDebug = cachedHeadingErrorDeg;
        cachedSignedLateralErrorDebug = cachedSignedLateralError;
    }

    // ── 전진/역주행 판단 (velocity dot segment direction) ──
    float ComputeForwardProgress()
    {
        if (!hasPrimaryZone) return 0f;

        Vector3 velocity = GetVehicleVelocity();
        float forwardProgress = Vector3.Dot(velocity, primarySegInfo.direction);
        lastForwardProgress = forwardProgress;
        return forwardProgress;
    }

    Vector3 GetVehicleVelocity()
    {
        if (wheelController == null) return Vector3.zero;

        // ArticulationBody → Rigidbody → wheel speed fallback
        var artBody = wheelController.GetComponent<ArticulationBody>();
        if (artBody != null) return artBody.velocity;

        var rb = wheelController.GetComponent<Rigidbody>();
        if (rb != null) return rb.velocity;

        // Fallback: 휠 속도 × 차량 전방
        Transform t = GetTrackedTransform();
        return t.forward * wheelController.GetSpeedMS();
    }

    // ── 보상 합산 ──
    void CalculateStepReward(float dt, float forwardProgress)
    {
        // Progress: 전진 속도 기반
        float progressReward = progressRewardScale * forwardProgress * dt;
        if (forwardProgress < 0f)
            progressReward *= reverseProgressPenaltyScale;

        // Zone 보상
        float zoneReward = zoneRewardWeight * currentZoneScore * dt;
        if (zoneReward > 0f)
        {
            float safeFullSpeed = Mathf.Max(1e-4f, zoneRewardFullScaleProgressSpeed);
            float forwardRatio = Mathf.Clamp01(forwardProgress / safeFullSpeed);
            zoneReward *= Mathf.Lerp(zoneRewardNoProgressScale, 1f, forwardRatio);
        }

        // Heading shaping
        float headingErrorNorm = Mathf.Clamp01(
            Mathf.Abs(cachedHeadingErrorDeg) / Mathf.Max(1f, headingErrorNormalizeDeg));
        float headingReward = -headingRewardWeight * headingErrorNorm * dt;

        // Lateral shaping: targetZone(+zone) 중심 기준 횡오차 패널티.
        // currentLateralError는 UpdateLateralAndHeading에서 targetZone 기준으로 이미 계산됨.
        // Zone_m에 있으면 Zone_R1 중심까지의 거리가 lateral error로 들어옴.
        float lateralErrorNorm = Mathf.Clamp01(
            currentLateralError / Mathf.Max(0.01f, lateralErrorNormalizeM));
        float lateralReward = -lateralRewardWeight * lateralErrorNorm * dt;

        float safetyPenalty = ComputeSafetyPenalty(dt);
        float trafficPenalty = ComputeTrafficPenalty(dt);

        lastProgressReward = progressReward;
        lastZoneReward = zoneReward;
        lastHeadingReward = headingReward;
        lastLateralReward = lateralReward;
        lastSafetyPenalty = safetyPenalty;
        lastTrafficPenalty = trafficPenalty;

        cumulativeProgressReward += progressReward;
        cumulativeZoneReward += zoneReward;
        cumulativeHeadingReward += headingReward;
        cumulativeLateralReward += lateralReward;
        cumulativeSafetyPenalty += safetyPenalty;
        cumulativeTrafficPenalty += trafficPenalty;

        lastStepReward = progressReward + zoneReward + headingReward + lateralReward
                         - safetyPenalty - trafficPenalty;
        unconsumedStepReward += lastStepReward;
        cumulativeReward += lastStepReward;
    }

    float ComputeSafetyPenalty(float dt)
    {
        if (collisionWarningEngine == null) return 0f;

        return collisionWarningEngine.GetWarningLevel() switch
        {
            CollisionWarningEngine.WarningLevel.Caution => cautionPenaltyPerSec * dt,
            CollisionWarningEngine.WarningLevel.SlowDown => warningPenaltyPerSec * dt,
            CollisionWarningEngine.WarningLevel.Warning => warningPenaltyPerSec * dt,
            CollisionWarningEngine.WarningLevel.Brake => brakePenaltyPerSec * dt,
            CollisionWarningEngine.WarningLevel.EmergencyStop => emergencyPenaltyPerSec * dt,
            _ => 0f
        };
    }

    float ComputeTrafficPenalty(float dt)
    {
        if (trafficLightDecisionEngine == null || wheelController == null) return 0f;
        if (!trafficLightDecisionEngine.ShouldStop()) return 0f;

        float speed = Mathf.Abs(wheelController.GetSpeedMS());
        if (speed > redViolationSpeedThreshold)
            return redViolationPenaltyPerSec * dt;

        return 0f;
    }

    // ── 세그먼트 방향 캐시 (메쉬 정점에서 1회 추출) ──
    SegmentInfo ComputeSegmentInfo(MeshFilter mf)
    {
        Vector3[] v = mf.sharedMesh.vertices;
        // 시작 에지 중심 (4개 정점 평균)
        Vector3 start = (v[0] + v[1] + v[2] + v[3]) * 0.25f;
        // 끝 에지 중심 (마지막 4개 정점 평균)
        int last = v.Length - 4;
        Vector3 end = (v[last] + v[last + 1] + v[last + 2] + v[last + 3]) * 0.25f;
        Vector3 dir = (end - start).normalized;

        return new SegmentInfo
        {
            direction = dir,
            center = (start + end) * 0.5f,
            right = Vector3.Cross(Vector3.up, dir).normalized
        };
    }

    void TryCacheSegmentInfo(RewardZone zone, Collider col)
    {
        if (segCache.ContainsKey(zone)) return;

        MeshFilter mf = col.GetComponent<MeshFilter>();
        if (mf != null && mf.sharedMesh != null && mf.sharedMesh.vertexCount >= 8)
            segCache[zone] = ComputeSegmentInfo(mf);
    }

    // ── 유틸 ──
    Vector3 GetTrackedPosition()
    {
        return (triggerTarget != null) ? triggerTarget.transform.position : transform.position;
    }

    Transform GetTrackedTransform()
    {
        return (triggerTarget != null) ? triggerTarget.transform : transform;
    }

    // ── 트리거 ──
    public void NotifyTriggerEnter(Collider other)
    {
        RewardZone zone = other.GetComponent<RewardZone>();
        if (zone == null) return;

        activeZones.Add(zone);
        TryCacheSegmentInfo(zone, other);

        // 첫 zone 진입 시 즉시 primary 설정
        if (!hasPrimaryZone && segCache.ContainsKey(zone))
        {
            primaryZone = zone;
            primarySegInfo = segCache[zone];
            hasPrimaryZone = true;
            currentZoneName = zone.zoneName;
            currentZoneScore = zone.score;
            activeZoneCount = activeZones.Count;
        }
    }

    public void NotifyTriggerStay(Collider other)
    {
        RewardZone zone = other.GetComponent<RewardZone>();
        if (zone == null) return;

        activeZones.Add(zone);
        // 에피소드 리셋 후 OnTriggerEnter가 누락될 수 있으므로 Stay에서도 캐시
        TryCacheSegmentInfo(zone, other);

        if (!hasPrimaryZone && segCache.ContainsKey(zone))
        {
            primaryZone = zone;
            primarySegInfo = segCache[zone];
            hasPrimaryZone = true;
            currentZoneName = zone.zoneName;
            currentZoneScore = zone.score;
            activeZoneCount = activeZones.Count;
        }
    }

    public void NotifyTriggerExit(Collider other)
    {
        RewardZone zone = other.GetComponent<RewardZone>();
        if (zone != null)
            activeZones.Remove(zone);
    }

    // triggerTarget이 자기 자신일 때 직접 수신
    void OnTriggerEnter(Collider other) { if (triggerTarget == null || triggerTarget == gameObject) NotifyTriggerEnter(other); }
    void OnTriggerStay(Collider other) { if (triggerTarget == null || triggerTarget == gameObject) NotifyTriggerStay(other); }
    void OnTriggerExit(Collider other) { if (triggerTarget == null || triggerTarget == gameObject) NotifyTriggerExit(other); }

    // ═══════════════════════════════════════════
    //  외부 인터페이스 (시그니처 유지)
    // ═══════════════════════════════════════════

    public float ConsumeStepReward()
    {
        float reward = unconsumedStepReward;
        unconsumedStepReward = 0f;
        return reward;
    }

    public float PeekStepReward() => unconsumedStepReward;
    public float GetLastStepReward() => lastStepReward;
    public float GetCumulativeReward() => cumulativeReward;
    public float GetLastProgressReward() => lastProgressReward;
    public float GetLastZoneReward() => lastZoneReward;
    public float GetLastSafetyPenalty() => lastSafetyPenalty;
    public float GetLastTrafficPenalty() => lastTrafficPenalty;
    public float GetCumulativeProgressReward() => cumulativeProgressReward;
    public float GetCumulativeZoneReward() => cumulativeZoneReward;
    public float GetCumulativeSafetyPenalty() => cumulativeSafetyPenalty;
    public float GetCumulativeTrafficPenalty() => cumulativeTrafficPenalty;
    public float GetCumulativeHeadingReward() => cumulativeHeadingReward;
    public float GetCumulativeLateralReward() => cumulativeLateralReward;
    public float GetCurrentLateralError() => currentLateralError;
    public string GetCurrentZoneName() => string.IsNullOrEmpty(currentZoneName) ? "None" : currentZoneName;
    public int GetActiveZoneCount() => activeZoneCount;
    public float GetCurrentZoneScore() => currentZoneScore;
    public float GetCachedHeadingErrorDeg() => cachedHeadingErrorDeg;
    public float GetCachedSignedLateralError() => cachedSignedLateralError;

    // ── 제거 대상이지만 호출처 컴파일 호환을 위해 stub 유지 ──
    // waypoint path-s 기반은 제거됨. -1 반환으로 RLEpisodeEvaluator가 XZ fallback 사용.
    public float GetCurrentPathS() => -1f;
    public float GetTotalPathLength() => 0f;
    public float GetPathProgressRatio() => 0f;
    public float GetLastZoneProgressScale() => 1f;
    public float GetLastRawDeltaS() => 0f;
    public float GetLastUsedDeltaS() => 0f;
    public bool WasLastDeltaSClamped() => false;
    public int GetLastNearestSegmentIndex() => 0;

    public void RefreshTrackingState()
    {
        // Zone 기반에서는 트리거에 의해 자동 갱신됨 — no-op
    }

    public void ResetRewardState()
    {
        activeZones.Clear();
        hasPrimaryZone = false;
        primaryZone = null;

        currentZoneScore = 0f;
        currentZoneName = "None";
        activeZoneCount = 0;
        currentLateralError = 0f;
        cachedHeadingErrorDeg = 0f;
        cachedSignedLateralError = 0f;
        cachedHeadingErrorDegDebug = 0f;
        cachedSignedLateralErrorDebug = 0f;
        lastForwardProgress = 0f;

        lastProgressReward = 0f;
        lastZoneReward = 0f;
        lastHeadingReward = 0f;
        lastLateralReward = 0f;
        lastSafetyPenalty = 0f;
        lastTrafficPenalty = 0f;
        lastStepReward = 0f;
        unconsumedStepReward = 0f;

        cumulativeProgressReward = 0f;
        cumulativeZoneReward = 0f;
        cumulativeHeadingReward = 0f;
        cumulativeLateralReward = 0f;
        cumulativeSafetyPenalty = 0f;
        cumulativeTrafficPenalty = 0f;
        cumulativeReward = 0f;

        // segCache는 초기화하지 않음 — 메쉬 정점은 런타임에 불변
    }
}
