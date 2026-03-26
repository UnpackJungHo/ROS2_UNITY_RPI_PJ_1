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
    // 레거시 필드 (cos 기반 progressMultiplier로 대체, 사용하지 않음)
    [HideInInspector] public float reverseGearPenaltyPerSec = 0f;

    [Header("Speed Bonus")]
    [Tooltip("목표 순항 속도 (m/s). 이 속도에서 speedBonus=1.0")]
    public float targetSpeedMs = 2.0f;
    [Tooltip("속도 0일 때 최소 보너스 배율 (progress 보상의 하한)")]
    public float speedBonusMin = 0.5f;
    [Tooltip("목표 속도 초과 시 최대 보너스 배율")]
    public float speedBonusMax = 1.2f;

    [Header("Heading/Lateral Shaping")]
    [Tooltip("cos 기반 헤딩 패널티 가중치 (초당). reward = weight × (cos(heading) - 1) × dt\n" +
             "0°→0, 90°→-weight, 180°→-2×weight. 전 구간 연속 gradient.")]
    public float headingRewardWeight = 0.3f;
    [Tooltip("횡오차 패널티 가중치 (m당, 초당). 1m 이탈 시 -weight×dt/step 패널티.")]
    public float lateralRewardWeight = 0.5f;
    [Tooltip("이 거리(m) 초과 시 패널티를 선형 증가 대신 클램프. 0이면 순수 선형(무제한).")]
    public float lateralErrorMaxM = 5f;

    [Header("Safety Penalty")]
    public CollisionWarningEngine collisionWarningEngine;
    public float cautionPenaltyPerSec = 0.05f;
    public float warningPenaltyPerSec = 0.15f;
    public float brakePenaltyPerSec = 0.35f;
    public float emergencyPenaltyPerSec = 1.2f;

    [Header("Traffic Penalty")]
    public TrafficLightDecisionEngine trafficLightDecisionEngine;
    public VehicleMotionController vehicleMotionController;
    [Tooltip("정지 지시인데 이 속도 이상이면 위반 패널티 적용")]
    public float redViolationSpeedThreshold = 0.2f;
    public float redViolationPenaltyPerSec = 0.6f;

    // ── 레거시 Heading Recovery 필드 (cos 기반 통합으로 더 이상 사용하지 않음) ──
    // 인스펙터 직렬화 호환을 위해 [HideInInspector]로 유지
    [HideInInspector] public float headingRecoveryThresholdDeg = 90f;
    [HideInInspector] public float headingRecoveryPenaltyWeight = 0.5f;
    [HideInInspector] public float headingFlatPenaltyThresholdDeg = 120f;
    [HideInInspector] public float headingFlatPenaltyPerSec = 1.5f;
    [HideInInspector] public float headingErrorNormalizeDeg = 45f;

    [Header("Episode Guard")]
    [Tooltip("RLEpisodeEvaluator가 활성 상태일 때만 보상을 누적")]
    public bool accumulateOnlyWhenEpisodeActive = true;
    public RLEpisodeEvaluator episodeEvaluator;

    [Header("Gizmos")]
    [Tooltip("씬 뷰에서 세그먼트 방향 화살표 시각화 (플레이 중 동작). 초록=+zone, 빨강=-zone, 노랑=현재 primaryZone")]
    public bool showSegmentGizmos = false;

    [Header("Debug (Read Only)")]
    [SerializeField] private float cumulativeReward = 0f;
    [SerializeField] private float cumulativeProgressReward = 0f;
[SerializeField] private float currentLateralError = 0f;
    [SerializeField] private float cumulativeHeadingReward = 0f;
    [SerializeField] private float cumulativeLateralReward = 0f;
    [SerializeField] private float cumulativeSafetyPenalty = 0f;
    [SerializeField] private float cumulativeTrafficPenalty = 0f;
    [SerializeField] private float currentZoneScore = 0f;
    [SerializeField] private string currentZoneName = "None";
    [SerializeField] private int activeZoneCount = 0;
    [SerializeField] private float lastProgressReward = 0f;
[SerializeField] private float lastHeadingReward = 0f;
    [SerializeField] private float lastLateralReward = 0f;
    [SerializeField] private string targetZoneName = "None";
    [SerializeField] private float lastSafetyPenalty = 0f;
    [SerializeField] private float lastTrafficPenalty = 0f;
    [SerializeField] private float lastHeadingRecoveryPenalty = 0f;
    [SerializeField] private float cumulativeHeadingRecoveryPenalty = 0f;
    [SerializeField] private float lastHeadingFlatPenalty = 0f;
    [SerializeField] private float cumulativeHeadingFlatPenalty = 0f;
    [SerializeField] private float lastReverseGearPenalty = 0f;
    [SerializeField] private float cumulativeReverseGearPenalty = 0f;
    [SerializeField] private float lastStepReward = 0f;
    [SerializeField] private float lastForwardProgress = 0f;
    [SerializeField] private float lastLookaheadHeadingDeg = 0f;

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

    // ── Positive Zone 캐시 ──
    private struct PositiveZoneEntry
    {
        public RewardZone zone;
        public SegmentInfo seg;
    }
    private readonly List<PositiveZoneEntry> positiveZones = new List<PositiveZoneEntry>();

    // ── +Zone 중심선 폴리라인 (Cross-Track Error 기준) ──
    // 설계 원칙:
    //  - +Zone이 1개: 그 Zone의 중심선 기준으로 CTE 측정
    //  - +Zone이 N개: 가장 가까운 +Zone 중심선 기준 CTE (= 현재 위치에서 자연스럽게 가까운 쪽 유도)
    //  - Zone 점수 차이(예: score=4 vs score=2)는 ZoneReward가 담당 → lateral은 centering만 담당
    //  - 도로가 여러 개: 도로(부모 그룹)별 독립 폴리라인, FindNearest가 자동으로 가장 가까운 도로 선택
    private struct CenterlinePoint
    {
        public Vector3 position;
        public Vector3 tangent;  // 다음 점 방향 (normalized)
    }

    private struct CenterlineGroup
    {
        public List<CenterlinePoint> points;
        public string zoneName;
        public float score;          // 해당 +Zone의 score (Gizmo 색상 구분용)
    }
    private readonly List<CenterlineGroup> allPositiveCenterlines = new List<CenterlineGroup>();

    // ── 캐시 (외부에서 읽는 값) ──
    [SerializeField] private float cachedHeadingErrorDeg = 0f;
    [SerializeField] private float cachedSignedLateralError = 0f;

    // ── 컴포넌트 캐시 (매 FixedUpdate GetComponent 방지) ──
    private ArticulationBody _cachedArtBody;
    private Rigidbody _cachedRigidbody;
    private readonly Dictionary<RewardZone, float> _zoneVolumeCache = new Dictionary<RewardZone, float>();

    // ── Seg 순서 캐시 (부모 그룹별 Seg 번호순 정렬) ──
    private readonly Dictionary<RewardZone, int> zoneToGroupIndex = new();  // zone → group 내 index
    private readonly Dictionary<RewardZone, List<RewardZone>> zoneToGroupList = new();  // zone → 소속 group 리스트

    // ── 보상 누적 ──
    private float unconsumedStepReward = 0f;

    void Start()
    {
        if (vehicleMotionController == null)
            vehicleMotionController = FindObjectOfType<VehicleMotionController>();
        if (episodeEvaluator == null)
            episodeEvaluator = FindObjectOfType<RLEpisodeEvaluator>();

        if (triggerTarget != null && triggerTarget != gameObject)
        {
            var proxy = triggerTarget.GetComponent<ProgressRewardProxy>();
            if (proxy == null)
                proxy = triggerTarget.AddComponent<ProgressRewardProxy>();
            proxy.owner = this;
        }

        PreCacheAllZonesAndDirections();

        if (vehicleMotionController != null)
        {
            _cachedArtBody = vehicleMotionController.GetComponent<ArticulationBody>();
            if (_cachedArtBody == null)
                _cachedRigidbody = vehicleMotionController.GetComponent<Rigidbody>();
        }
    }

    /// <summary>
    /// 씬 내 모든 RewardZone을 캐시하고, 같은 부모 아래 Seg 번호(Zone_*_Seg0 → SegN) 순서로
    /// 방향을 자동 계산한다. center(Seg_N+1) - center(Seg_N) 방향이 도로 진행 방향.
    /// 메쉬 버텍스 순서에 의존하지 않으므로 방향 역전 버그가 없다.
    /// </summary>
    void PreCacheAllZonesAndDirections()
    {
        positiveZones.Clear();
        var allZones = FindObjectsOfType<RewardZone>(true);

        // 1단계: 모든 zone의 mesh center 캐시 (mesh vertex 기반, 방향은 임시)
        foreach (var zone in allZones)
        {
            var col = zone.GetComponent<Collider>();
            if (col != null) TryCacheSegmentInfo(zone, col);
        }

        // 2단계: 부모 기준으로 그룹화 → Seg 번호 순서로 방향 재계산
        var groups = new Dictionary<Transform, List<RewardZone>>();
        foreach (var zone in allZones)
        {
            if (!segCache.ContainsKey(zone)) continue;
            var parent = zone.transform.parent;
            if (parent == null) continue;
            if (!groups.ContainsKey(parent)) groups[parent] = new List<RewardZone>();
            groups[parent].Add(zone);
        }

        foreach (var kvp in groups)
        {
            var segs = kvp.Value;
            segs.Sort((a, b) => ParseSegIndex(a.name).CompareTo(ParseSegIndex(b.name)));

            var centers = new Vector3[segs.Count];
            for (int i = 0; i < segs.Count; i++)
                centers[i] = segCache[segs[i]].center;

            for (int i = 0; i < segs.Count; i++)
            {
                Vector3 dir;
                if (i < segs.Count - 1)
                    dir = (centers[i + 1] - centers[i]).normalized;
                else
                    dir = i > 0 ? (centers[i] - centers[i - 1]).normalized : segCache[segs[i]].direction;

                var info = segCache[segs[i]];
                segCache[segs[i]] = new SegmentInfo
                {
                    direction = dir,
                    center    = info.center,
                    right     = Vector3.Cross(Vector3.up, dir).normalized
                };
            }
        }

        // 2.5단계: Seg 순서 캐시 구축 (곡률 lookahead용)
        zoneToGroupIndex.Clear();
        zoneToGroupList.Clear();
        foreach (var kvp in groups)
        {
            var segs = kvp.Value; // 이미 Seg 번호순 정렬됨
            for (int i = 0; i < segs.Count; i++)
            {
                zoneToGroupIndex[segs[i]] = i;
                zoneToGroupList[segs[i]] = segs;
            }
        }

        // 3단계: positiveZones 목록 재구성 (방향 재계산 이후에 실행해야 최신 값 반영)
        foreach (var zone in allZones)
        {
            if (zone.score <= 0f) continue;
            if (segCache.TryGetValue(zone, out var info))
                positiveZones.Add(new PositiveZoneEntry { zone = zone, seg = info });
        }

        // 4단계: +Zone 중심선 폴리라인 구성
        // 부모 기준으로 그룹화 → 가장 Seg가 많은 그룹(= 주 +Zone)을 Seg 번호 순 정렬 후 경로선 생성.
        BuildPositiveCenterline();

        targetZoneName = allPositiveCenterlines.Count > 0 ? positiveZones[0].zone.zoneName : "None";
        Debug.Log($"[ProgressRewardProvider] 전체 zone 캐시: {allZones.Length}개, +zone: {positiveZones.Count}개, 중심선 그룹: {allPositiveCenterlines.Count}개");
    }

    void BuildPositiveCenterline()
    {
        allPositiveCenterlines.Clear();
        if (positiveZones.Count == 0) return;

        // 부모 Transform 기준으로 그룹화 → 도로(RoadCreator 인스턴스)별로 독립 처리
        var posGroups = new Dictionary<Transform, List<PositiveZoneEntry>>();
        foreach (var entry in positiveZones)
        {
            var parent = entry.zone.transform.parent;
            if (parent == null) continue;
            if (!posGroups.ContainsKey(parent)) posGroups[parent] = new List<PositiveZoneEntry>();
            posGroups[parent].Add(entry);
        }

        // 모든 그룹(= 도로별, Zone별)을 각각 폴리라인으로 구성. score 내림차순 정렬.
        foreach (var group in posGroups.Values)
        {
            if (group.Count < 2) continue;

            group.Sort((a, b) => ParseSegIndex(a.zone.name).CompareTo(ParseSegIndex(b.zone.name)));

            float avgScore = group.Average(e => e.zone.score);
            string zName   = group[0].zone.zoneName;

            var line = new List<CenterlinePoint>(group.Count);
            int n = group.Count;
            for (int i = 0; i < n; i++)
            {
                Vector3 pos     = group[i].seg.center;
                Vector3 nextPos = group[(i + 1) % n].seg.center;
                Vector3 tangent = (nextPos - pos).normalized;
                line.Add(new CenterlinePoint { position = pos, tangent = tangent });
            }
            allPositiveCenterlines.Add(new CenterlineGroup
            {
                points   = line,
                zoneName = zName,
                score    = avgScore
            });
        }

        // score 높은 순으로 정렬 (Gizmo에서 가장 진한 파란색이 최우선 Zone)
        allPositiveCenterlines.Sort((a, b) => b.score.CompareTo(a.score));

        Debug.Log($"[ProgressRewardProvider] +Zone 중심선: {allPositiveCenterlines.Count}개 Zone그룹 " +
                  $"({string.Join(", ", allPositiveCenterlines.Select(g => $"{g.zoneName}({g.score:F1})"))})");
    }

    /// <summary>
    /// vehiclePos에서 모든 도로의 +Zone 중심선 폴리라인을 통틀어 가장 가까운 점과 접선을 반환.
    /// 도로가 여러 개여도 현재 위치에서 가장 가까운 도로의 중심선이 자동 선택됨.
    /// </summary>
    void FindNearestOnCenterline(Vector3 pos, out Vector3 nearestPt, out Vector3 tangent)
    {
        nearestPt = allPositiveCenterlines[0].points[0].position;
        tangent   = allPositiveCenterlines[0].points[0].tangent;
        float minSqDist = float.MaxValue;

        foreach (var group in allPositiveCenterlines)
        {
            var line  = group.points;
            int count = line.Count;
            for (int i = 0; i < count - 1; i++)
            {
                Vector3 a  = line[i].position;
                Vector3 b  = line[i + 1].position;
                Vector3 ab = b - a;
                if (ab.sqrMagnitude < 1e-8f) continue;
                float   t  = Mathf.Clamp01(Vector3.Dot(pos - a, ab) / ab.sqrMagnitude);
                Vector3 closest = a + t * ab;
                float sqDist = (pos - closest).sqrMagnitude;
                if (sqDist < minSqDist)
                {
                    minSqDist = sqDist;
                    nearestPt = closest;
                    tangent   = line[i].tangent;
                }
            }
        }
    }

    /// <summary>
    /// primaryZone(현재 속한 Zone)과 바로 다음 Seg 사이의 부호 있는 방향 변화(도)를 반환.
    /// 반환값: 양수=우회전, 음수=좌회전, 0=직선.
    /// </summary>
    public float GetLookaheadHeadingDeg()
    {
        if (!hasPrimaryZone || primaryZone == null) return 0f;
        if (!zoneToGroupIndex.TryGetValue(primaryZone, out int currentIdx)) return 0f;
        if (!zoneToGroupList.TryGetValue(primaryZone, out var groupList)) return 0f;

        if (currentIdx + 1 >= groupList.Count) return 0f;

        var zoneCurr = groupList[currentIdx];
        var zoneNext = groupList[currentIdx + 1];

        if (!segCache.TryGetValue(zoneCurr, out var infoCurr)) return 0f;
        if (!segCache.TryGetValue(zoneNext, out var infoNext)) return 0f;

        float angle = Vector3.SignedAngle(infoCurr.direction, infoNext.direction, Vector3.up);
        lastLookaheadHeadingDeg = angle;
        return angle;
    }

    /// <summary>"Zone_L2_Seg12" → 12 파싱. Seg 접미사가 없으면 int.MaxValue 반환.</summary>
    static int ParseSegIndex(string name)
    {
        int segPos = name.LastIndexOf("Seg", System.StringComparison.OrdinalIgnoreCase);
        if (segPos < 0) return int.MaxValue;
        string num = name.Substring(segPos + 3);
        return int.TryParse(num, out int n) ? n : int.MaxValue;
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
        GetLookaheadHeadingDeg();
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

                    if (_zoneVolumeCache.TryGetValue(zone, out float volume) && volume > largestVolume)
                    {
                        largestVolume = volume;
                        largestZone = zone;
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

    // ── 횡오차 + 헤딩 계산 ──
    void UpdateLateralAndHeading()
    {
        if (!hasPrimaryZone) return;

        Vector3 vehiclePos = GetTrackedPosition();
        Transform vehicleT = GetTrackedTransform();

        // 횡오차 (Cross-Track Error): +Zone 중심선 폴리라인에 수선의 발을 내려 수직 거리 측정.
        // 어느 Zone에 있든 항상 +Zone 중심선 기준으로 측정 → lateral=0은 +Zone 중앙일 때만 성립.
        if (allPositiveCenterlines.Count > 0)
        {
            FindNearestOnCenterline(vehiclePos, out Vector3 nearestPt, out Vector3 clTangent);
            Vector3 right = Vector3.Cross(Vector3.up, clTangent).normalized;
            cachedSignedLateralError = Vector3.Dot(vehiclePos - nearestPt, right);
        }
        else
        {
            // fallback: 폴리라인 미구성 시 primaryZone 기준
            Vector3 lateralOffset = vehiclePos - primarySegInfo.center;
            cachedSignedLateralError = Vector3.Dot(lateralOffset, primarySegInfo.right);
        }
        currentLateralError = Mathf.Abs(cachedSignedLateralError);

        // 헤딩오차: primaryZone(현재 주행 중인 zone)의 진행 방향 기준.
        cachedHeadingErrorDeg = Vector3.SignedAngle(
            primarySegInfo.direction, vehicleT.forward, Vector3.up);

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
        if (vehicleMotionController == null) return Vector3.zero;

        // ArticulationBody → Rigidbody → wheel speed fallback (컴포넌트는 Start에서 캐시됨)
        if (_cachedArtBody != null) return _cachedArtBody.velocity;
        if (_cachedRigidbody != null) return _cachedRigidbody.velocity;

        return GetTrackedTransform().forward * vehicleMotionController.GetSpeedMS();
    }

    // ── 보상 합산 ──
    void CalculateStepReward(float dt, float forwardProgress)
    {
        // Heading cos (보상 및 progress 가중치 공유)
        float headingCos = Mathf.Cos(cachedHeadingErrorDeg * Mathf.Deg2Rad);

        // Progress: 전진 속도 × 방향 정렬 가중치
        // headingCos * 0.75 + 0.25: 완전 차단 대신 완화된 감쇠
        //   0°→1.0, 60°→0.625, 90°→0.25, 120°→0, 180°→0
        // 초기 탐색 시 약간의 progress 신호를 허용하여 bootstrapping 지원.
        float progressMultiplier = Mathf.Max(0f, headingCos * 0.75f + 0.25f);

        // Speed bonus: 목표 속도에 가까울수록 progress 보상 증폭
        float currentSpeed = vehicleMotionController != null
            ? Mathf.Abs(vehicleMotionController.GetSpeedMS()) : 0f;
        float speedRatio = targetSpeedMs > 0f ? currentSpeed / targetSpeedMs : 1f;
        float speedBonus = Mathf.Clamp(speedRatio, speedBonusMin, speedBonusMax);

        float progressReward = progressRewardScale * forwardProgress * progressMultiplier * speedBonus * dt;
        if (forwardProgress < 0f)
            progressReward *= reverseProgressPenaltyScale;

        float reverseGearPenalty = 0f; // 레거시 호환 (0 고정)

        // Heading shaping (cos 기반):
        // (cosθ - 1): 0(정렬) → -1(직각) → -2(역주행)
        // 전 구간 연속 gradient, clamp/불연속 없음.
        float headingReward = headingRewardWeight * (headingCos - 1f) * dt;

        // recovery/flat 항은 cos에 이미 포함되어 불필요 → 0으로 고정
        float headingRecoveryPenalty = 0f;
        float absHeadingErr = Mathf.Abs(cachedHeadingErrorDeg);

        // Lateral shaping: targetZone(+zone) 중심 기준 횡오차 패널티.
        float cte = lateralErrorMaxM > 0f
            ? Mathf.Min(currentLateralError, lateralErrorMaxM)
            : currentLateralError;
        float lateralReward = -lateralRewardWeight * cte * dt;

        float safetyPenalty = ComputeSafetyPenalty(dt);
        float trafficPenalty = ComputeTrafficPenalty(dt);

        float headingFlatPenalty = 0f;

        lastProgressReward = progressReward;
        lastHeadingReward = headingReward;
        lastHeadingRecoveryPenalty = headingRecoveryPenalty;
        lastHeadingFlatPenalty = headingFlatPenalty;
        lastReverseGearPenalty = reverseGearPenalty;
        lastLateralReward = lateralReward;
        lastSafetyPenalty = safetyPenalty;
        lastTrafficPenalty = trafficPenalty;

        cumulativeProgressReward += progressReward;
        cumulativeHeadingReward += headingReward;
        cumulativeHeadingRecoveryPenalty += headingRecoveryPenalty;
        cumulativeHeadingFlatPenalty += headingFlatPenalty;
        cumulativeReverseGearPenalty += reverseGearPenalty;
        cumulativeLateralReward += lateralReward;
        cumulativeSafetyPenalty += safetyPenalty;
        cumulativeTrafficPenalty += trafficPenalty;

        lastStepReward = progressReward + headingReward - headingRecoveryPenalty
                         - headingFlatPenalty - reverseGearPenalty
                         + lateralReward - safetyPenalty - trafficPenalty;
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
        if (trafficLightDecisionEngine == null || vehicleMotionController == null) return 0f;
        if (!trafficLightDecisionEngine.ShouldStop()) return 0f;

        float speed = Mathf.Abs(vehicleMotionController.GetSpeedMS());
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

        Vector3 size = col.bounds.size;
        _zoneVolumeCache[zone] = size.x * size.y * size.z;

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
    void HandleZoneContact(RewardZone zone, Collider col)
    {
        activeZones.Add(zone);
        TryCacheSegmentInfo(zone, col);
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

    public void NotifyTriggerEnter(Collider other)
    {
        RewardZone zone = other.GetComponent<RewardZone>();
        if (zone == null) return;
        HandleZoneContact(zone, other);
    }

    public void NotifyTriggerStay(Collider other)
    {
        // 에피소드 리셋 후 OnTriggerEnter가 누락될 수 있으므로 Stay에서도 캐시
        RewardZone zone = other.GetComponent<RewardZone>();
        if (zone == null) return;
        HandleZoneContact(zone, other);
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
    public float GetLastSafetyPenalty() => lastSafetyPenalty;
    public float GetLastTrafficPenalty() => lastTrafficPenalty;
    public float GetCumulativeProgressReward() => cumulativeProgressReward;
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
        lastForwardProgress = 0f;

        lastProgressReward = 0f;
        lastHeadingReward = 0f;
        lastHeadingRecoveryPenalty = 0f;
        lastHeadingFlatPenalty = 0f;
        lastLateralReward = 0f;
        lastSafetyPenalty = 0f;
        lastTrafficPenalty = 0f;
        lastStepReward = 0f;
        unconsumedStepReward = 0f;

        cumulativeProgressReward = 0f;
        cumulativeHeadingReward = 0f;
        cumulativeHeadingRecoveryPenalty = 0f;
        cumulativeHeadingFlatPenalty = 0f;
        cumulativeReverseGearPenalty = 0f;
        cumulativeLateralReward = 0f;
        cumulativeSafetyPenalty = 0f;
        cumulativeTrafficPenalty = 0f;
        cumulativeReward = 0f;

        // segCache는 초기화하지 않음 — 메쉬 정점은 런타임에 불변
    }

    /// <summary>
    /// +Zone 중심선에서 랜덤 포인트를 샘플링하여 위치와 접선(tangent)을 반환한다.
    /// 랜덤 스폰 시 Agent의 시작 위치를 도로 위로 제한하기 위해 사용.
    /// </summary>
    public bool TryGetRandomCenterlinePoint(out Vector3 position, out Vector3 tangent)
    {
        if (allPositiveCenterlines.Count == 0)
        {
            position = Vector3.zero;
            tangent = Vector3.forward;
            return false;
        }

        int groupIdx = UnityEngine.Random.Range(0, allPositiveCenterlines.Count);
        var group = allPositiveCenterlines[groupIdx];

        if (group.points == null || group.points.Count == 0)
        {
            position = Vector3.zero;
            tangent = Vector3.forward;
            return false;
        }

        int ptIdx = UnityEngine.Random.Range(0, group.points.Count);
        position = group.points[ptIdx].position;
        tangent = group.points[ptIdx].tangent;
        return true;
    }

    // ═══════════════════════════════════════════
    //  Gizmos (Scene View 시각화)
    // ═══════════════════════════════════════════

    void OnDrawGizmos()
    {
        if (!showSegmentGizmos) return;

        // ── +Zone 중심선 폴리라인 (score 순 색상 구분, 도로 위 0.15m 띄워서 표시) ──
        // score 가장 높은 Zone = 진한 파란색, 낮을수록 하늘색에 가까워짐
        const float lineY = 0.15f;
        if (allPositiveCenterlines.Count > 0)
        {
            int totalGroups = allPositiveCenterlines.Count;
            for (int g = 0; g < totalGroups; g++)
            {
                var group = allPositiveCenterlines[g];
                // score 높을수록 진한 파란색(0,0,1), 낮을수록 하늘색(0,0.7,1)
                float t     = totalGroups > 1 ? (float)g / (totalGroups - 1) : 0f;
                Color color = Color.Lerp(Color.blue, new Color(0f, 0.7f, 1f), t);
                Gizmos.color = color;

                var line    = group.points;
                int clCount = line.Count;
                for (int i = 0; i < clCount; i++)
                {
                    Vector3 a = line[i].position + Vector3.up * lineY;
                    Gizmos.DrawSphere(a, 0.08f);
                    {
                        int nextIdx = (i + 1) % clCount;
                        Vector3 b = line[nextIdx].position + Vector3.up * lineY;
                        Gizmos.DrawLine(a, b);
                    }
                }

#if UNITY_EDITOR
                // Zone 이름 + score 라벨 (첫 번째 점에 표시)
                if (line.Count > 0)
                {
                    UnityEditor.Handles.color = color;
                    UnityEditor.Handles.Label(
                        line[0].position + Vector3.up * (lineY + 0.5f),
                        $"{group.zoneName} (score:{group.score:F1})");
                }
#endif
            }

            // 현재 수선의 발 + 헤딩 시각화 (플레이 중)
            if (Application.isPlaying && hasPrimaryZone)
            {
                Vector3 vehiclePos = GetTrackedPosition();
                Transform vehicleT  = GetTrackedTransform();
                FindNearestOnCenterline(vehiclePos, out Vector3 nearestPt, out Vector3 clTangent);
                Vector3 nearestPtUp = nearestPt + Vector3.up * lineY;

                // ── CTE (횡오차) ──
                Gizmos.color = Color.magenta;
                Gizmos.DrawLine(vehiclePos, nearestPtUp);
                Gizmos.DrawSphere(nearestPtUp, 0.12f);

                // ── 헤딩 시각화 ──
                const float arrowLen = 2.0f;
                Vector3 basePos = vehiclePos + Vector3.up * 0.1f;

                // 차량 전진 방향 (흰색)
                Vector3 fwdTip = basePos + vehicleT.forward * arrowLen;
                Gizmos.color = Color.white;
                Gizmos.DrawLine(basePos, fwdTip);
                Gizmos.DrawSphere(fwdTip, 0.08f);

                // 목표 방향 = 현재 primaryZone 세그먼트 방향 (초록)
                Vector3 segTip = basePos + primarySegInfo.direction * arrowLen;
                Gizmos.color = Color.green;
                Gizmos.DrawLine(basePos, segTip);
                Gizmos.DrawSphere(segTip, 0.08f);

                // 헤딩 오차 크기에 따라 색상: 0°=초록 → 90°=노랑 → 180°=빨강
                float absErr    = Mathf.Abs(cachedHeadingErrorDeg);
                float errRatio  = Mathf.Clamp01(absErr / 180f);
                Color errColor  = Color.Lerp(Color.green, Color.red, errRatio);
                if (absErr > headingRecoveryThresholdDeg) errColor = Color.red;

#if UNITY_EDITOR
                UnityEditor.Handles.color = Color.magenta;
                UnityEditor.Handles.Label(nearestPtUp + Vector3.up * 0.4f,
                    $"CTE: {Mathf.Abs(cachedSignedLateralError):F2}m");

                // 헤딩 정보 라벨 (차량 위)
                UnityEditor.Handles.color = errColor;
                string recoveryTag = absErr > headingRecoveryThresholdDeg ? " [RECOVERY]" : "";
                UnityEditor.Handles.Label(vehiclePos + Vector3.up * 1.2f,
                    $"Heading Err: {cachedHeadingErrorDeg:+0.0;-0.0}°{recoveryTag}\n" +
                    $"Safety: {collisionWarningEngine?.GetWarningLevel().ToString() ?? "N/A"}");

                // 흰=차량방향 라벨
                UnityEditor.Handles.color = Color.white;
                UnityEditor.Handles.Label(fwdTip + Vector3.up * 0.15f, "FWD");

                // 초록=세그먼트 방향 라벨
                UnityEditor.Handles.color = Color.green;
                UnityEditor.Handles.Label(segTip + Vector3.up * 0.15f, "SEG");
#endif
            }
        }

        if (segCache == null || segCache.Count == 0) return;

        foreach (var kvp in segCache)
        {
            RewardZone zone = kvp.Key;
            SegmentInfo info = kvp.Value;
            if (zone == null) continue;

            bool isPrimary  = hasPrimaryZone && zone == primaryZone;
            bool isPositive = zone.score > 0f;

            // 색상: 현재 primaryZone=노랑, +zone=초록, -zone=빨강
            if (isPrimary)
                Gizmos.color = Color.yellow;
            else if (isPositive)
                Gizmos.color = new Color(0.2f, 1f, 0.2f, 0.85f);
            else
                Gizmos.color = new Color(1f, 0.3f, 0.3f, 0.85f);

            // 중심 구체
            float sphereR = isPrimary ? 0.2f : 0.1f;
            Gizmos.DrawSphere(info.center, sphereR);

            // 방향 화살표 (center → center + direction)
            float arrowLen  = isPrimary ? 1.5f : 0.8f;
            Vector3 tip     = info.center + info.direction * arrowLen;
            Gizmos.DrawLine(info.center, tip);

            // 화살촉 (tip에서 양쪽으로 벌어지는 선 2개)
            float headLen   = arrowLen * 0.2f;
            Vector3 headDir = -info.direction * headLen;
            Vector3 side    = info.right * (headLen * 0.5f);
            Gizmos.DrawLine(tip, tip + headDir + side);
            Gizmos.DrawLine(tip, tip + headDir - side);

            // 횡방향(right) 표시: 가는 흰색 선
            Gizmos.color = new Color(1f, 1f, 1f, 0.3f);
            Gizmos.DrawLine(info.center - info.right * 0.4f, info.center + info.right * 0.4f);

#if UNITY_EDITOR
            // 이름 라벨 (primaryZone과 +zone만 표시, 너무 많으면 지저분)
            if (isPrimary || isPositive)
            {
                UnityEditor.Handles.color = isPrimary ? Color.yellow : Color.green;
                UnityEditor.Handles.Label(info.center + Vector3.up * 0.3f, zone.name);
            }
#endif
        }
    }
}
