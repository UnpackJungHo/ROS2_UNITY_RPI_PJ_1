using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 단일 레이더 센서 시뮬레이터
/// - 멀티 타깃 감지
/// - 근사 RCS 기반 신뢰도
/// - 클러터/고스트 합성
/// - 도플러 직접값(상대속도 투영) 기반 방사속도 추정
/// </summary>
public class SingleRadarSensor : MonoBehaviour
{
    public enum SensorPosition
    {
        Front, // 전방 센서
        Rear   // 후방 센서
    }

    [Serializable]
    public struct RadarDetection
    {
        public float distance;       // m
        public float angle;          // deg (horizontal)
        public Vector3 point;        // world point
        public float radialVelocity; // m/s, +접근 -이탈
        public float rcs;            // dBsm 유사 지표(근사)
        public float confidence;     // 0~1
        public bool isGhost;
        public bool isClutter;
        public string colliderName;
    }

    [Header("Sensor Identity")]
    public SensorPosition sensorPosition = SensorPosition.Front;

    [Header("Radar Specifications (레이더 사양)")]
    public float rangeMin = 0.2f;
    public float rangeMax = 50f;
    public float horizontalFOV = 60f;
    public float verticalFOV = 10f;
    public int horizontalRayNum = 15;
    public int verticalRayNum = 3;
    [Tooltip("한 레이에서 반사점으로 채택할 최대 hit 수, 한 레이가 여러 물체를 통과할 때 몇 개까지 기록할지")]
    public int maxHitsPerRay = 2;
    [Tooltip("한 스캔에서 유지할 최대 타깃 수")]
    public int maxDetections = 24;

    [Header("Detection Quality (검출 품질)")]
    [Range(0f, 1f)]
    public float minDetectionConfidence = 0.2f;
    public float rangeNoiseStdDev = 0.03f;
    public float dopplerNoiseStdDev = 0.15f;

    [Header("RCS Approximation (근사 반사강도)")]
    [Tooltip("레이어 기반 반사강도 조정 - 고반사 레이어(예: 금속)")]
    public LayerMask highReflectivityLayers;
    [Tooltip("레이어 기반 반사강도 조정 - 저반사 레이어(예: 나무, 흙)")]
    public LayerMask lowReflectivityLayers;
    public float minRcs = 0.1f;
    public float maxRcs = 40f;

    [Header("Relative Velocity Estimation (상대속도 추정)")]
    [Range(0.05f, 1f)]
    public float relativeVelocityFilterAlpha = 0.35f;
    public float maxRelativeVelocity = 8f;
    [Tooltip("Ego 속도 기준 (없으면 부모에서 자동 탐색)")]
    public ArticulationBody egoArticulationBody;
    public Rigidbody egoRigidbody;

    [Header("Ghost / Clutter Modeling")]
    [Tooltip("고스트 타깃 활성화 여부 (실제 반사 대신 존재하지 않는 고스트 타깃이 감지되는 경우)")]
    public bool enableGhostTargets = false;
    [Range(0f, 0.5f)]
    public float ghostProbability = 0.08f;
    public float ghostDistanceOffsetMin = 0.6f;
    public float ghostDistanceOffsetMax = 2.5f;

    [Tooltip("클러터 활성화 여부 (실제 물체와 무관하게 레이더에 잡히는 잡음 타깃)")]
    public bool enableClutter = false;
    [Range(0f, 1f)]
    public float clutterScanProbability = 0.2f;
    public int clutterCountMin = 0;
    public int clutterCountMax = 2;
    public float clutterMinDistance = 0.5f;
    public float clutterMaxDistance = 12f;
    public float clutterVelocityStdDev = 0.2f;

    [Header("Collision Detection Layer (감지할 레이어)")]
    public LayerMask detectionLayer = ~0;

    [Header("Debug (디버그 설정)")]
    public bool showDebugRays = true;
    public Color hitColor = Color.red;
    public Color missColor = new Color(1f, 0.5f, 0.5f, 0.3f);

    // Legacy output (기존 호환)
    public float Distance { get; private set; } = float.PositiveInfinity;
    public float DetectedAngle { get; private set; } = 0f;
    public Vector3 DetectedPoint { get; private set; } = Vector3.zero;
    public float RelativeVelocity { get; private set; } = 0f;
    public bool HasDetection => !float.IsInfinity(Distance);

    public string SensorName => sensorPosition == SensorPosition.Front ? "Front" : "Rear";
    public bool IsFrontSensor => sensorPosition == SensorPosition.Front;
    public bool IsRearSensor => sensorPosition == SensorPosition.Rear;

    public int DetectionCount => detections.Count;

    private float scanInterval = 0.05f; // 20 Hz
    private float lastScanTime;
    private readonly List<RadarDetection> detections = new List<RadarDetection>();
    private readonly Dictionary<int, int> colliderIndexMap = new Dictionary<int, int>();

    void Start()
    {
        rangeMax = Mathf.Max(rangeMax, rangeMin + 0.1f);
        maxHitsPerRay = Mathf.Max(1, maxHitsPerRay);
        maxDetections = Mathf.Max(1, maxDetections);
        horizontalRayNum = Mathf.Max(1, horizontalRayNum);
        verticalRayNum = Mathf.Max(1, verticalRayNum);
        maxRelativeVelocity = Mathf.Max(0.1f, maxRelativeVelocity);

        AutoFindEgoVelocitySources();
        lastScanTime = Time.time;
        Debug.Log($"[Radar-{SensorName}] Initialized - Range: {rangeMin}-{rangeMax}m, H-FOV: {horizontalFOV}°, V-FOV: {verticalFOV}°");
    }

    void Update()
    {
        if (Time.time - lastScanTime >= scanInterval)
        {
            PerformScan();
            lastScanTime = Time.time;
        }
    }

    public void PerformScan()
    {
        detections.Clear();
        colliderIndexMap.Clear();

        Vector3 origin = transform.position;
        float halfHFOV = horizontalFOV * 0.5f;
        float halfVFOV = verticalFOV * 0.5f;

        float hAngleStep = horizontalRayNum > 1 ? horizontalFOV / (horizontalRayNum - 1) : 0f;
        float vAngleStep = verticalRayNum > 1 ? verticalFOV / (verticalRayNum - 1) : 0f;

        for (int h = 0; h < horizontalRayNum; h++)
        {
            float hAngle = -halfHFOV + (h * hAngleStep);
            bool rayHadDetection = false;

            for (int v = 0; v < verticalRayNum; v++)
            {
                float vAngle = -halfVFOV + (v * vAngleStep);
                Vector3 direction = GetDirectionFromAngles(hAngle, vAngle);

                RaycastHit[] hits = Physics.RaycastAll(origin, direction, rangeMax, detectionLayer, QueryTriggerInteraction.Ignore);
                if (hits.Length == 0)
                {
                    continue;
                }

                Array.Sort(hits, (a, b) => a.distance.CompareTo(b.distance));

                int acceptedHits = 0;
                foreach (var hit in hits)
                {
                    if (acceptedHits >= maxHitsPerRay) break;
                    if (hit.distance < rangeMin || hit.distance > rangeMax) continue;

                    RadarDetection candidate = BuildDetection(origin, direction, hAngle, hit);
                    if (candidate.confidence < minDetectionConfidence) continue;

                    AddOrUpdateDetection(hit.collider, candidate);
                    acceptedHits++;
                    rayHadDetection = true;

                    if (showDebugRays)
                    {
                        Debug.DrawLine(origin, hit.point, hitColor, scanInterval);
                    }
                }
            }

            if (showDebugRays && !rayHadDetection)
            {
                Vector3 direction = GetDirectionFromAngles(hAngle, 0f);
                Debug.DrawRay(origin, direction * rangeMax, missColor, scanInterval);
            }
        }

        InjectGhostDetections(origin);
        InjectClutterDetections(origin);
        SortAndTrimDetections();
        UpdateLegacyPrimaryDetection();
    }

    public RadarDetection[] GetDetectionsSnapshot()
    {
        return detections.ToArray();
    }

    private void AutoFindEgoVelocitySources()
    {
        if (egoArticulationBody == null)
        {
            egoArticulationBody = GetComponentInParent<ArticulationBody>();
        }
        if (egoRigidbody == null)
        {
            egoRigidbody = GetComponentInParent<Rigidbody>();
        }
    }

    private RadarDetection BuildDetection(Vector3 origin, Vector3 rayDirection, float horizontalAngle, RaycastHit hit)
    {
        float measuredDistance = ApplyGaussianNoise(hit.distance, rangeNoiseStdDev);
        measuredDistance = Mathf.Clamp(measuredDistance, rangeMin, rangeMax);

        float radialVelocity = CalculateDopplerRadialVelocity(origin, hit, rayDirection);
        radialVelocity = ApplyGaussianNoise(radialVelocity, dopplerNoiseStdDev);
        radialVelocity = Mathf.Clamp(radialVelocity, -maxRelativeVelocity, maxRelativeVelocity);

        float rcs = CalculateApproxRcs(hit, rayDirection);
        float confidence = CalculateDetectionConfidence(measuredDistance, rcs, rayDirection, hit.normal);

        return new RadarDetection
        {
            distance = measuredDistance,
            angle = horizontalAngle,
            point = hit.point,
            radialVelocity = radialVelocity,
            rcs = rcs,
            confidence = confidence,
            isGhost = false,
            isClutter = false,
            colliderName = hit.collider != null ? hit.collider.name : "unknown"
        };
    }

    private void AddOrUpdateDetection(Collider collider, RadarDetection candidate)
    {
        if (collider == null)
        {
            if (detections.Count < maxDetections * 2)
            {
                detections.Add(candidate);
            }
            return;
        }

        int id = collider.GetInstanceID();
        if (colliderIndexMap.TryGetValue(id, out int existingIndex))
        {
            RadarDetection existing = detections[existingIndex];
            bool replace = candidate.confidence > existing.confidence ||
                           (candidate.distance < existing.distance && candidate.confidence >= existing.confidence - 0.1f);
            if (replace)
            {
                detections[existingIndex] = candidate;
            }
            return;
        }

        if (detections.Count < maxDetections * 2)
        {
            colliderIndexMap[id] = detections.Count;
            detections.Add(candidate);
        }
    }

    private void InjectGhostDetections(Vector3 origin)
    {
        if (!enableGhostTargets || detections.Count == 0) return;

        int sourceCount = detections.Count;
        for (int i = 0; i < sourceCount; i++)
        {
            if (detections.Count >= maxDetections * 2) break;

            RadarDetection source = detections[i];
            if (source.isGhost || source.isClutter) continue;
            if (source.confidence < 0.35f) continue;
            if (UnityEngine.Random.value > ghostProbability) continue;

            RadarDetection ghost = source;
            ghost.distance = Mathf.Clamp(
                source.distance + UnityEngine.Random.Range(ghostDistanceOffsetMin, ghostDistanceOffsetMax),
                rangeMin,
                rangeMax);
            ghost.angle = Mathf.Clamp(
                source.angle + UnityEngine.Random.Range(-4f, 4f),
                -horizontalFOV * 0.5f,
                horizontalFOV * 0.5f);
            ghost.radialVelocity = Mathf.Clamp(
                source.radialVelocity * UnityEngine.Random.Range(0.7f, 1.1f),
                -maxRelativeVelocity,
                maxRelativeVelocity);
            ghost.rcs = Mathf.Clamp(source.rcs * UnityEngine.Random.Range(0.35f, 0.8f), minRcs, maxRcs);
            ghost.confidence = Mathf.Clamp01(source.confidence * UnityEngine.Random.Range(0.35f, 0.65f));
            ghost.isGhost = true;
            ghost.isClutter = false;
            ghost.colliderName = source.colliderName + "_ghost";
            ghost.point = origin + GetDirectionFromAngles(ghost.angle, 0f) * ghost.distance;

            detections.Add(ghost);
        }
    }

    private void InjectClutterDetections(Vector3 origin)
    {
        if (!enableClutter || UnityEngine.Random.value > clutterScanProbability) return;

        float minDist = Mathf.Clamp(clutterMinDistance, rangeMin, rangeMax);
        float maxDist = Mathf.Clamp(clutterMaxDistance, minDist, rangeMax);
        int clutterCount = UnityEngine.Random.Range(Mathf.Max(0, clutterCountMin), Mathf.Max(clutterCountMin, clutterCountMax) + 1);

        for (int i = 0; i < clutterCount; i++)
        {
            if (detections.Count >= maxDetections * 2) break;

            float angle = UnityEngine.Random.Range(-horizontalFOV * 0.5f, horizontalFOV * 0.5f);
            float distance = UnityEngine.Random.Range(minDist, maxDist);
            float radialVelocity = ApplyGaussianNoise(0f, clutterVelocityStdDev);

            RadarDetection clutter = new RadarDetection
            {
                distance = distance,
                angle = angle,
                point = origin + GetDirectionFromAngles(angle, 0f) * distance,
                radialVelocity = Mathf.Clamp(radialVelocity, -maxRelativeVelocity, maxRelativeVelocity),
                rcs = UnityEngine.Random.Range(minRcs * 0.6f, minRcs * 2f),
                confidence = UnityEngine.Random.Range(0.12f, 0.35f),
                isGhost = false,
                isClutter = true,
                colliderName = "clutter"
            };

            detections.Add(clutter);
        }
    }

    private void SortAndTrimDetections()
    {
        detections.Sort((a, b) =>
        {
            int distanceCompare = a.distance.CompareTo(b.distance);
            if (distanceCompare != 0) return distanceCompare;
            return b.confidence.CompareTo(a.confidence);
        });

        if (detections.Count > maxDetections)
        {
            detections.RemoveRange(maxDetections, detections.Count - maxDetections);
        }
    }

    private void UpdateLegacyPrimaryDetection()
    {
        RadarDetection? primary = null;

        // 우선순위: 실제 반사 > (없으면) 고스트/클러터
        for (int i = 0; i < detections.Count; i++)
        {
            if (!detections[i].isGhost && !detections[i].isClutter)
            {
                primary = detections[i];
                break;
            }
        }
        if (!primary.HasValue && detections.Count > 0)
        {
            primary = detections[0];
        }

        if (!primary.HasValue)
        {
            Distance = float.PositiveInfinity;
            DetectedAngle = 0f;
            DetectedPoint = Vector3.zero;
            RelativeVelocity = 0f;
            return;
        }

        RadarDetection p = primary.Value;
        Distance = p.distance;
        DetectedAngle = p.angle;
        DetectedPoint = p.point;

        if (Mathf.Abs(RelativeVelocity) < 0.001f)
        {
            RelativeVelocity = p.radialVelocity;
        }
        else
        {
            RelativeVelocity = Mathf.Lerp(RelativeVelocity, p.radialVelocity, relativeVelocityFilterAlpha);
        }
    }

    private float CalculateDopplerRadialVelocity(Vector3 origin, RaycastHit hit, Vector3 rayDirection)
    {
        Vector3 losDirection = (hit.point - origin).sqrMagnitude > 1e-6f
            ? (hit.point - origin).normalized
            : rayDirection.normalized;

        Vector3 targetVelocity = GetTargetVelocity(hit);
        Vector3 egoVelocity = GetEgoVelocity();
        Vector3 relativeVelocity = targetVelocity - egoVelocity;

        // +: approaching, -: receding
        return -Vector3.Dot(relativeVelocity, losDirection);
    }

    private Vector3 GetTargetVelocity(RaycastHit hit)
    {
        if (hit.rigidbody != null)
        {
            return hit.rigidbody.velocity;
        }

        if (hit.collider != null)
        {
            ArticulationBody articulation = hit.collider.GetComponentInParent<ArticulationBody>();
            if (articulation != null)
            {
                return articulation.velocity;
            }
        }

        return Vector3.zero;
    }

    private Vector3 GetEgoVelocity()
    {
        if (egoRigidbody != null)
        {
            return egoRigidbody.velocity;
        }
        if (egoArticulationBody != null)
        {
            return egoArticulationBody.velocity;
        }
        return Vector3.zero;
    }

    private float CalculateApproxRcs(RaycastHit hit, Vector3 rayDirection)
    {
        Bounds b = hit.collider.bounds;
        float projectedArea = Mathf.Max(b.size.x * b.size.y, b.size.x * b.size.z, b.size.y * b.size.z);
        float incidence = Mathf.Clamp01(Vector3.Dot(-rayDirection.normalized, hit.normal.normalized));

        float reflectivityFactor = 1f;
        int bit = 1 << hit.collider.gameObject.layer;
        if ((highReflectivityLayers.value & bit) != 0) reflectivityFactor = 1.8f;
        else if ((lowReflectivityLayers.value & bit) != 0) reflectivityFactor = 0.6f;

        float approxRcs = projectedArea * Mathf.Lerp(0.35f, 1f, incidence) * reflectivityFactor * 10f;
        return Mathf.Clamp(approxRcs, minRcs, maxRcs);
    }

    private float CalculateDetectionConfidence(float distance, float rcs, Vector3 rayDirection, Vector3 hitNormal)
    {
        float rangeScore = 1f - Mathf.InverseLerp(rangeMin, rangeMax, distance);
        float rcsScore = Mathf.InverseLerp(minRcs, maxRcs, rcs);
        float incidenceScore = Mathf.Clamp01(Vector3.Dot(-rayDirection.normalized, hitNormal.normalized));

        float confidence = (0.45f * rangeScore) + (0.35f * rcsScore) + (0.20f * incidenceScore);

        if (distance > rangeMax * 0.85f)
        {
            confidence *= 0.8f;
        }

        return Mathf.Clamp01(confidence);
    }

    private Vector3 GetDirectionFromAngles(float horizontalAngleDeg, float verticalAngleDeg)
    {
        float hRad = horizontalAngleDeg * Mathf.Deg2Rad;
        float vRad = verticalAngleDeg * Mathf.Deg2Rad;
        Vector3 localDirection = new Vector3(
            Mathf.Sin(hRad) * Mathf.Cos(vRad),
            Mathf.Sin(vRad),
            Mathf.Cos(hRad) * Mathf.Cos(vRad)
        );
        return transform.TransformDirection(localDirection).normalized;
    }

    private float ApplyGaussianNoise(float mean, float stdDev)
    {
        if (stdDev <= 0f) return mean;

        float u1 = Mathf.Max(1e-6f, 1f - UnityEngine.Random.value);
        float u2 = Mathf.Max(1e-6f, 1f - UnityEngine.Random.value);
        float standardNormal = Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Sin(2f * Mathf.PI * u2);
        return mean + (stdDev * standardNormal);
    }

    public void SetScanInterval(float interval)
    {
        scanInterval = Mathf.Max(0.005f, interval);
    }
}
