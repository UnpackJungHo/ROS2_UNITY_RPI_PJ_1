using System.Collections.Generic;
using System.Collections;
using System.Reflection;
using UnityEngine;

/// <summary>
/// 강화학습 보상 신호 제공기.
/// - 주 보상: 경로 진행도(progress)
/// - 보조 보상: RewardZone 점수
/// - 보조 패널티: 충돌 위험도/신호 위반
/// </summary>
public class ProgressRewardProvider : MonoBehaviour
{
    [Header("Path Progress")]
    [Tooltip("도로 중심선 순서대로 waypoint를 할당하세요")]
    public Transform[] progressWaypoints;
    public bool isLoopedPath = false;
    public bool autoInitOnStart = true;

    [Header("Auto Waypoint Build (RoadCreator)")]
    [Tooltip("progressWaypoints가 비어있으면 RoadData에서 자동 생성")]
    public bool autoBuildWaypointsFromRoadData = true;
    [Tooltip("Road 오브젝트 직접 할당 (비어있으면 이름으로 탐색)")]
    public GameObject roadObject;
    [Tooltip("roadObject가 비어있을 때 찾을 도로 오브젝트 이름")]
    public string roadObjectName = "Road";
    [Tooltip("생성된 스플라인 포인트를 N개 간격으로 샘플링")]
    [Range(1, 20)] public int waypointStride = 3;
    [Tooltip("RoadData의 curveResolution을 사용해 스플라인 포인트 생성")]
    public bool useRoadCurveResolution = true;
    [SerializeField] private Transform generatedWaypointRoot;
    [SerializeField] private int generatedWaypointCount = 0;

    [Header("Reward Weights")]
    [Tooltip("진행도 1m당 보상")]
    public float progressRewardPerMeter = 1.0f;
    [Tooltip("Zone 점수 가중치 (초당)")]
    public float zoneRewardWeight = 0.25f;
    [Tooltip("역주행(음의 progress)에 대한 추가 패널티 배율")]
    public float reverseProgressPenaltyScale = 1.4f;

    [Header("Safety Penalty")]
    public CollisionWarningPublisher collisionWarningPublisher;
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

    [Header("Debug (Read Only)")]
    [SerializeField] private float currentPathS = 0f;
    [SerializeField] private float currentLateralError = 0f;
    [SerializeField] private string currentZoneName = "None";
    [SerializeField] private int activeZoneCount = 0;
    [SerializeField] private float lastProgressReward = 0f;
    [SerializeField] private float lastZoneReward = 0f;
    [SerializeField] private float lastSafetyPenalty = 0f;
    [SerializeField] private float lastTrafficPenalty = 0f;
    [SerializeField] private float cumulativeProgressReward = 0f;
    [SerializeField] private float cumulativeZoneReward = 0f;
    [SerializeField] private float cumulativeSafetyPenalty = 0f;
    [SerializeField] private float cumulativeTrafficPenalty = 0f;
    [SerializeField] private float currentZoneScore = 0f;
    [SerializeField] private float lastStepReward = 0f;
    [SerializeField] private float cumulativeReward = 0f;

    private readonly List<float> cumulativeLengths = new List<float>();
    private readonly HashSet<RewardZone> activeZones = new HashSet<RewardZone>();
    private float totalPathLength = 0f;
    private float previousPathS = 0f;
    private bool initialized = false;
    private float unconsumedStepReward = 0f;

    void Start()
    {
        if (wheelController == null)
            wheelController = FindObjectOfType<WheelTest>();

        if (autoBuildWaypointsFromRoadData && (progressWaypoints == null || progressWaypoints.Length < 2))
        {
            TryBuildWaypointsFromRoadData();
        }

        if (autoInitOnStart)
            InitializePath();
    }

    void FixedUpdate()
    {
        if (!initialized)
            return;

        float dt = Time.fixedDeltaTime;
        UpdateZoneScore();
        UpdatePathProgress();
        CalculateStepReward(dt);
    }

    public void InitializePath()
    {
        cumulativeLengths.Clear();
        totalPathLength = 0f;
        initialized = false;

        if (progressWaypoints == null || progressWaypoints.Length < 2)
        {
            Debug.LogWarning("[ProgressReward] progressWaypoints가 부족합니다. (최소 2개 필요)");
            return;
        }

        cumulativeLengths.Add(0f);
        for (int i = 1; i < progressWaypoints.Length; i++)
        {
            float segLen = Vector3.Distance(progressWaypoints[i - 1].position, progressWaypoints[i].position);
            totalPathLength += segLen;
            cumulativeLengths.Add(totalPathLength);
        }

        if (isLoopedPath)
        {
            totalPathLength += Vector3.Distance(
                progressWaypoints[progressWaypoints.Length - 1].position,
                progressWaypoints[0].position
            );
        }

        currentPathS = ProjectOnPath(transform.position, out currentLateralError);
        previousPathS = currentPathS;
        initialized = true;
    }

    [ContextMenu("Rebuild Waypoints From RoadData")]
    public void RebuildWaypointsFromRoadData()
    {
        if (TryBuildWaypointsFromRoadData())
            InitializePath();
    }

    bool TryBuildWaypointsFromRoadData()
    {
        GameObject sourceRoad = roadObject != null ? roadObject : GameObject.Find(roadObjectName);
        if (sourceRoad == null)
        {
            Debug.LogWarning($"[ProgressReward] Road 오브젝트를 찾지 못했습니다. name={roadObjectName}");
            return false;
        }

        Component roadData = sourceRoad.GetComponent("RoadData");
        if (roadData == null)
        {
            Debug.LogWarning("[ProgressReward] RoadData 컴포넌트가 없어 자동 waypoint 생성을 건너뜁니다.");
            return false;
        }

        List<Vector3> controlPoints = ReadVector3ListField(roadData, "controlPoints");
        if (controlPoints == null || controlPoints.Count < 2)
        {
            Debug.LogWarning("[ProgressReward] RoadData.controlPoints가 부족합니다. (최소 2개)");
            return false;
        }

        bool looped = ReadBoolField(roadData, "isLooped", false);
        int curveResolution = Mathf.Max(2, ReadIntField(roadData, "curveResolution", 10));
        isLoopedPath = looped;

        List<Vector3> sampledPathPoints;
        if (useRoadCurveResolution)
            sampledPathPoints = GenerateSplinePoints(controlPoints, curveResolution, looped);
        else
            sampledPathPoints = new List<Vector3>(controlPoints);

        List<Vector3> waypointPositions = SampleByStride(sampledPathPoints, waypointStride, looped);
        if (waypointPositions.Count < 2)
        {
            Debug.LogWarning("[ProgressReward] 자동 생성된 waypoint가 부족합니다. stride를 줄여보세요.");
            return false;
        }

        BuildWaypointTransforms(waypointPositions);
        Debug.Log($"[ProgressReward] Waypoint 자동생성 완료: {generatedWaypointCount}개 (looped={isLoopedPath})");
        return true;
    }

    void BuildWaypointTransforms(List<Vector3> worldPositions)
    {
        ClearGeneratedWaypointRoot();

        GameObject root = new GameObject("ProgressWaypoints_Generated");
        root.transform.SetParent(transform, false);
        generatedWaypointRoot = root.transform;

        Transform[] built = new Transform[worldPositions.Count];
        for (int i = 0; i < worldPositions.Count; i++)
        {
            GameObject wp = new GameObject($"WP_{i:000}");
            wp.transform.SetParent(generatedWaypointRoot, false);
            wp.transform.position = worldPositions[i];
            built[i] = wp.transform;
        }

        progressWaypoints = built;
        generatedWaypointCount = built.Length;
    }

    void ClearGeneratedWaypointRoot()
    {
        if (generatedWaypointRoot == null)
            return;

        if (Application.isPlaying)
            Destroy(generatedWaypointRoot.gameObject);
        else
            DestroyImmediate(generatedWaypointRoot.gameObject);

        generatedWaypointRoot = null;
        generatedWaypointCount = 0;
    }

    List<Vector3> ReadVector3ListField(Component component, string fieldName)
    {
        FieldInfo field = component.GetType().GetField(fieldName);
        if (field == null)
            return null;

        IList list = field.GetValue(component) as IList;
        if (list == null)
            return null;

        List<Vector3> result = new List<Vector3>(list.Count);
        for (int i = 0; i < list.Count; i++)
        {
            if (list[i] is Vector3 v)
                result.Add(v);
        }
        return result;
    }

    bool ReadBoolField(Component component, string fieldName, bool fallback)
    {
        FieldInfo field = component.GetType().GetField(fieldName);
        if (field == null) return fallback;
        object value = field.GetValue(component);
        return value is bool b ? b : fallback;
    }

    int ReadIntField(Component component, string fieldName, int fallback)
    {
        FieldInfo field = component.GetType().GetField(fieldName);
        if (field == null) return fallback;
        object value = field.GetValue(component);
        return value is int i ? i : fallback;
    }

    List<Vector3> GenerateSplinePoints(List<Vector3> points, int resolution, bool looped)
    {
        List<Vector3> result = new List<Vector3>();
        if (points == null || points.Count < 2)
            return result;

        if (points.Count == 2 && !looped)
        {
            int segments = Mathf.Max(2, resolution);
            for (int i = 0; i <= segments; i++)
            {
                float t = i / (float)segments;
                result.Add(Vector3.Lerp(points[0], points[1], t));
            }
            return result;
        }

        int pointCount = points.Count;
        int loopCount = looped ? pointCount : pointCount - 1;

        for (int i = 0; i < loopCount; i++)
        {
            Vector3 p0, p1, p2, p3;
            if (looped)
            {
                p0 = points[((i - 1) % pointCount + pointCount) % pointCount];
                p1 = points[i % pointCount];
                p2 = points[(i + 1) % pointCount];
                p3 = points[(i + 2) % pointCount];
            }
            else
            {
                p0 = points[Mathf.Max(i - 1, 0)];
                p1 = points[i];
                p2 = points[Mathf.Min(i + 1, pointCount - 1)];
                p3 = points[Mathf.Min(i + 2, pointCount - 1)];
            }

            for (int j = 0; j < resolution; j++)
            {
                float t = j / (float)resolution;
                result.Add(CatmullRom(p0, p1, p2, p3, t));
            }
        }

        if (!looped)
            result.Add(points[pointCount - 1]);
        else if (result.Count > 0)
            result.Add(result[0]);

        return result;
    }

    Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        float t2 = t * t;
        float t3 = t2 * t;
        return 0.5f * (
            (2f * p1) +
            (-p0 + p2) * t +
            (2f * p0 - 5f * p1 + 4f * p2 - p3) * t2 +
            (-p0 + 3f * p1 - 3f * p2 + p3) * t3
        );
    }

    List<Vector3> SampleByStride(List<Vector3> input, int stride, bool looped)
    {
        List<Vector3> sampled = new List<Vector3>();
        if (input == null || input.Count == 0)
            return sampled;

        int step = Mathf.Max(1, stride);
        for (int i = 0; i < input.Count; i += step)
            sampled.Add(input[i]);

        Vector3 last = input[input.Count - 1];
        if (!looped)
        {
            if (sampled.Count == 0 || Vector3.Distance(sampled[sampled.Count - 1], last) > 0.02f)
                sampled.Add(last);
        }
        else
        {
            // 루프의 경우 마지막 포인트(=시작점 복제)가 들어있으면 제거
            if (sampled.Count >= 2 && Vector3.Distance(sampled[0], sampled[sampled.Count - 1]) < 0.02f)
                sampled.RemoveAt(sampled.Count - 1);
        }

        return sampled;
    }

    void UpdatePathProgress()
    {
        currentPathS = ProjectOnPath(transform.position, out currentLateralError);
    }

    float ProjectOnPath(Vector3 position, out float lateralError)
    {
        lateralError = 0f;
        if (progressWaypoints == null || progressWaypoints.Length < 2)
            return 0f;

        float bestS = 0f;
        float bestDistSq = float.PositiveInfinity;

        int segCount = isLoopedPath ? progressWaypoints.Length : progressWaypoints.Length - 1;
        for (int i = 0; i < segCount; i++)
        {
            int j = (i + 1) % progressWaypoints.Length;
            Vector3 a = progressWaypoints[i].position;
            Vector3 b = progressWaypoints[j].position;
            Vector3 ab = b - a;
            float abLenSq = ab.sqrMagnitude;
            if (abLenSq < 1e-6f)
                continue;

            float t = Mathf.Clamp01(Vector3.Dot(position - a, ab) / abLenSq);
            Vector3 proj = a + t * ab;
            float distSq = (position - proj).sqrMagnitude;
            if (distSq < bestDistSq)
            {
                bestDistSq = distSq;
                float segStart = (i < cumulativeLengths.Count) ? cumulativeLengths[i] : 0f;
                float segLen = Mathf.Sqrt(abLenSq);
                bestS = segStart + (t * segLen);
            }
        }

        lateralError = Mathf.Sqrt(bestDistSq);
        return bestS;
    }

    void CalculateStepReward(float dt)
    {
        float deltaS = currentPathS - previousPathS;
        previousPathS = currentPathS;

        if (isLoopedPath && totalPathLength > 0.1f)
        {
            if (deltaS > totalPathLength * 0.5f)
                deltaS -= totalPathLength;
            else if (deltaS < -totalPathLength * 0.5f)
                deltaS += totalPathLength;
        }

        float progressReward = progressRewardPerMeter * deltaS;
        if (deltaS < 0f)
            progressReward *= reverseProgressPenaltyScale;

        float zoneReward = zoneRewardWeight * currentZoneScore * dt;
        float safetyPenalty = ComputeSafetyPenalty(dt);
        float trafficPenalty = ComputeTrafficPenalty(dt);

        lastProgressReward = progressReward;
        lastZoneReward = zoneReward;
        lastSafetyPenalty = safetyPenalty;
        lastTrafficPenalty = trafficPenalty;

        cumulativeProgressReward += progressReward;
        cumulativeZoneReward += zoneReward;
        cumulativeSafetyPenalty += safetyPenalty;
        cumulativeTrafficPenalty += trafficPenalty;

        lastStepReward = progressReward + zoneReward - safetyPenalty - trafficPenalty;
        unconsumedStepReward += lastStepReward;
        cumulativeReward += lastStepReward;
    }

    float ComputeSafetyPenalty(float dt)
    {
        if (collisionWarningPublisher == null)
            return 0f;

        return collisionWarningPublisher.GetWarningLevel() switch
        {
            CollisionWarningPublisher.WarningLevel.Caution => cautionPenaltyPerSec * dt,
            CollisionWarningPublisher.WarningLevel.SlowDown => warningPenaltyPerSec * dt,
            CollisionWarningPublisher.WarningLevel.Warning => warningPenaltyPerSec * dt,
            CollisionWarningPublisher.WarningLevel.Brake => brakePenaltyPerSec * dt,
            CollisionWarningPublisher.WarningLevel.EmergencyStop => emergencyPenaltyPerSec * dt,
            _ => 0f
        };
    }

    float ComputeTrafficPenalty(float dt)
    {
        if (trafficLightDecisionEngine == null || wheelController == null)
            return 0f;

        if (!trafficLightDecisionEngine.ShouldStop())
            return 0f;

        float speed = Mathf.Abs(wheelController.GetSpeedMS());
        if (speed > redViolationSpeedThreshold)
            return redViolationPenaltyPerSec * dt;

        return 0f;
    }

    void UpdateZoneScore()
    {
        float maxScore = 0f;
        string selectedZoneName = "None";
        int zoneCount = 0;
        bool hasZone = false;
        foreach (var zone in activeZones)
        {
            if (zone == null) continue;
            zoneCount++;
            if (!hasZone || zone.score > maxScore)
            {
                maxScore = zone.score;
                selectedZoneName = zone.zoneName;
                hasZone = true;
            }
        }

        activeZoneCount = zoneCount;
        currentZoneName = selectedZoneName;
        currentZoneScore = hasZone ? maxScore : 0f;
    }

    private void OnTriggerEnter(Collider other)
    {
        RewardZone zone = other.GetComponent<RewardZone>();
        if (zone != null)
            activeZones.Add(zone);
    }

    private void OnTriggerStay(Collider other)
    {
        RewardZone zone = other.GetComponent<RewardZone>();
        if (zone != null)
            activeZones.Add(zone);
    }

    private void OnTriggerExit(Collider other)
    {
        RewardZone zone = other.GetComponent<RewardZone>();
        if (zone != null)
            activeZones.Remove(zone);
    }

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
    public float GetCurrentPathS() => currentPathS;
    public float GetTotalPathLength() => totalPathLength;
    public float GetPathProgressRatio() => totalPathLength > 0.01f ? Mathf.Clamp01(currentPathS / totalPathLength) : 0f;
    public float GetCurrentLateralError() => currentLateralError;
    public string GetCurrentZoneName() => string.IsNullOrEmpty(currentZoneName) ? "None" : currentZoneName;
    public int GetActiveZoneCount() => activeZoneCount;
    public float GetCurrentZoneScore() => currentZoneScore;

    public void ResetRewardState()
    {
        if (!initialized)
            InitializePath();

        currentPathS = ProjectOnPath(transform.position, out currentLateralError);
        previousPathS = currentPathS;

        currentZoneScore = 0f;
        currentZoneName = "None";
        activeZoneCount = 0;
        activeZones.Clear();

        lastProgressReward = 0f;
        lastZoneReward = 0f;
        lastSafetyPenalty = 0f;
        lastTrafficPenalty = 0f;
        lastStepReward = 0f;
        unconsumedStepReward = 0f;

        cumulativeProgressReward = 0f;
        cumulativeZoneReward = 0f;
        cumulativeSafetyPenalty = 0f;
        cumulativeTrafficPenalty = 0f;
        cumulativeReward = 0f;
    }
}
