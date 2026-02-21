using System;
using UnityEngine;

/// <summary>
/// 개별 초음파 센서를 시뮬레이션하는 클래스입니다.
/// 레이캐스트(Raycast)를 사용하여 설정된 범위와 시야각 내의 장애물을 감지합니다.
/// 감지된 거리, 각도 및 위치 정보를 제공합니다.
/// </summary>
public class SingleUltrasonicSensor : MonoBehaviour
{
    // 센서가 장착된 위치를 정의하는 열거형
    public enum SensorPosition
    {
        FrontLeft = 0,  // 전방 좌측
        FrontRight = 1, // 전방 우측
        RearLeft = 2,   // 후방 좌측
        RearRight = 3,  // 후방 우측
        FrontCenter = 4, // 전방 중앙
        RearCenter = 5  // 후방 중앙
    }

    [Header("Sensor Identity")]
    [Tooltip("센서 위치 (FL, FR, FC, RL, RR, RC) - 이 설정에 따라 센서 이름과 역할이 결정됨")]
    public SensorPosition sensorPosition = SensorPosition.FrontLeft;

    [Header("Ultrasonic Specifications (초음파 센서 사양)")]
    [Tooltip("최소 감지 거리 (m) - 이 거리 미만의 물체는 무시됨")]
    public float rangeMin = 0.02f;
    [Tooltip("최대 감지 거리 (m) - 센서의 최대 탐지 거리")]
    public float rangeMax = 4.0f;
    [Tooltip("감지 시야각 (도) - 센서가 커버하는 부채꼴 각도")]
    public float fieldOfView = 30f;
    [Tooltip("레이(Ray) 개수 - 감지 해상도를 결정 (많을수록 정밀하지만 연산 부하 증가)")]
    public int rayCount = 5;

    [Header("Measurement Reliability (측정 신뢰도)")]
    [Tooltip("신뢰 가능한 최소 거리 (m) - 이보다 가까우면 신뢰도 감소")]
    public float reliableRangeMin = 0.05f;
    [Tooltip("신뢰 가능한 최대 거리 (m) - 이보다 멀면 신뢰도 감소")]
    public float reliableRangeMax = 2.5f;
    [Tooltip("근거리에서 드롭아웃(미검출) 확률")]
    [Range(0f, 0.5f)]
    public float dropoutProbabilityNear = 0.02f;
    [Tooltip("원거리에서 드롭아웃(미검출) 확률")]
    [Range(0f, 0.5f)]
    public float dropoutProbabilityFar = 0.12f;
    [Tooltip("신뢰 감지로 인정하는 최소 confidence")]
    [Range(0f, 1f)]
    public float reliableConfidenceThreshold = 0.45f;

    [Header("Collision Detection Layer (감지할 레이어)")]
    public LayerMask detectionLayer = ~0; // 기본값: 모든 레이어 감지

    [Header("Self Filtering (자차 감지 제외)")]
    [Tooltip("true면 센서가 자기 차량 콜라이더를 감지하지 않음")]
    public bool ignoreSelfColliders = true;
    [Tooltip("자차 루트(미할당 시 transform.root 사용)")]
    public Transform selfRoot;

    [Header("Debug (디버그 설정)")]
    public bool showDebugRays = true; // Scene 뷰에 레이를 그릴지 여부
    public Color hitColor = Color.cyan; // 장애물 감지 시 레이 색상
    public Color missColor = Color.blue; // 미감지 시 레이 색상

    // 감지 결과 데이터
    
    // 가장 가까운 장애물과의 거리 (감지 없을 시 무한대)
    public float Distance { get; private set; } = float.PositiveInfinity;
    
    // 감지된 물체의 각도 (센서 정면 기준)
    public float DetectedAngle { get; private set; } = 0f;
    
    // 감지된 물체의 월드 좌표
    public Vector3 DetectedPoint { get; private set; } = Vector3.zero;

    // 감지 confidence (0~1)
    public float Confidence { get; private set; } = 0f;
    
    // 유효한 감지가 있는지 여부 확인
    public bool HasDetection => !float.IsInfinity(Distance);
    public bool IsReliableDetection => HasDetection && Confidence >= reliableConfidenceThreshold;

    // 센서 이름을 약어로 반환하는 헬퍼 속성 (로그 출력 등에 사용)
    public string SensorName => sensorPosition switch
    {
        SensorPosition.FrontLeft => "FL",
        SensorPosition.FrontRight => "FR",
        SensorPosition.FrontCenter => "FC",
        SensorPosition.RearLeft => "RL",
        SensorPosition.RearRight => "RR",
        SensorPosition.RearCenter => "RC",
        _ => "Unknown"
    };

    // 센서의 위치/방향 확인을 위한 헬퍼 속성들
    public bool IsFrontSensor => sensorPosition == SensorPosition.FrontLeft ||
                                  sensorPosition == SensorPosition.FrontRight ||
                                  sensorPosition == SensorPosition.FrontCenter;

    public bool IsRearSensor => sensorPosition == SensorPosition.RearLeft ||
                                 sensorPosition == SensorPosition.RearRight ||
                                 sensorPosition == SensorPosition.RearCenter;

    public bool IsLeftSensor => sensorPosition == SensorPosition.FrontLeft ||
                                 sensorPosition == SensorPosition.RearLeft;

    public bool IsRightSensor => sensorPosition == SensorPosition.FrontRight ||
                                  sensorPosition == SensorPosition.RearRight;

    // 내부 동작 제어 변수
    private float scanInterval = 0.05f; // 스캔 주기(초)
    private float lastScanTime;

    void Start()
    {
        reliableRangeMin = Mathf.Clamp(reliableRangeMin, rangeMin, rangeMax);
        reliableRangeMax = Mathf.Clamp(reliableRangeMax, reliableRangeMin, rangeMax);
        if (selfRoot == null)
            selfRoot = transform.root;

        lastScanTime = Time.time;
        Debug.Log($"[Ultrasonic-{SensorName}] Initialized - Range: {rangeMin}-{rangeMax}m, FOV: {fieldOfView}°");
    }

    void Update()
    {
        // 정해진 주기마다 스캔 수행
        if (Time.time - lastScanTime >= scanInterval)
        {
            PerformScan();
            lastScanTime = Time.time;
        }
    }

    /// <summary>
    /// 실제 레이캐스트를 수행하여 장애물을 감지하는 메서드입니다.
    /// 시야각(FOV) 내에서 부채꼴 모양으로 여러 개의 레이를 쏘아 가장 가까운 물체를 찾습니다.
    /// </summary>
    public void PerformScan()
    {
        Vector3 origin = transform.position; // 센서 위치
        float halfFOV = fieldOfView * 0.5f;
        // 레이 간의 각도 간격 계산
        float angleStep = rayCount > 1 ? fieldOfView / (rayCount - 1) : 0f;

        // 감지 데이터 초기화
        Distance = float.PositiveInfinity;
        DetectedAngle = 0f;
        DetectedPoint = Vector3.zero;
        Confidence = 0f;

        // 설정된 레이 개수만큼 반복
        for (int i = 0; i < rayCount; i++)
        {
            // 현재 레이의 각도 계산 (왼쪽 -> 오른쪽 순서)
            float angle = -halfFOV + (i * angleStep);
            float angleRad = angle * Mathf.Deg2Rad;

            // 로컬 방향 벡터 계산 (Y축 회전)
            // X축: Sin(angle), Z축: Cos(angle) -> 평면 상의 부채꼴 확산
            Vector3 direction = transform.TransformDirection(
                new Vector3(Mathf.Sin(angleRad), 0, Mathf.Cos(angleRad))
            );

            bool hitAccepted = false;
            RaycastHit acceptedHit = default;
            RaycastHit[] hits = Physics.RaycastAll(
                origin,
                direction,
                rangeMax,
                detectionLayer,
                QueryTriggerInteraction.Ignore
            );
            if (hits != null && hits.Length > 0)
            {
                Array.Sort(hits, (a, b) => a.distance.CompareTo(b.distance));
                for (int h = 0; h < hits.Length; h++)
                {
                    RaycastHit hit = hits[h];
                    if (hit.collider == null)
                        continue;
                    if (IsSelfCollider(hit.collider))
                        continue;
                    if (hit.distance < rangeMin || hit.distance >= rangeMax)
                        continue;

                    float confidence = CalculateDetectionConfidence(hit.distance, direction, hit.normal);
                    if (confidence <= 0f)
                        continue;

                    bool isCloser = hit.distance < Distance;
                    bool isSameDistanceButMoreReliable =
                        Mathf.Abs(hit.distance - Distance) < 0.02f && confidence > Confidence;

                    if (isCloser || isSameDistanceButMoreReliable)
                    {
                        Distance = hit.distance;
                        DetectedAngle = angle;
                        DetectedPoint = hit.point;
                        Confidence = confidence;
                    }

                    acceptedHit = hit;
                    hitAccepted = true;
                    break;
                }
            }

            if (showDebugRays)
            {
                if (hitAccepted)
                    Debug.DrawLine(origin, acceptedHit.point, hitColor, scanInterval);
                else
                    Debug.DrawRay(origin, direction * rangeMax, missColor, scanInterval);
            }
        }
    }

    bool IsSelfCollider(Collider collider)
    {
        if (!ignoreSelfColliders || collider == null || selfRoot == null)
            return false;

        Transform ct = collider.transform;
        return ct == selfRoot || ct.IsChildOf(selfRoot);
    }

    float CalculateDetectionConfidence(float distance, Vector3 rayDirection, Vector3 hitNormal)
    {
        float rangeConfidence = CalculateRangeConfidence(distance);

        // 초음파는 정면 반사(법선 정렬)가 유리, 사각 입사에서 신뢰도 감소
        float incidenceAngle = Vector3.Angle(-rayDirection, hitNormal); // 0deg=정면, 90deg=사각
        float incidenceConfidence = Mathf.Clamp01((75f - incidenceAngle) / 75f);

        float distanceRatio = Mathf.InverseLerp(reliableRangeMin, rangeMax, distance);
        float dropoutProbability = Mathf.Lerp(dropoutProbabilityNear, dropoutProbabilityFar, distanceRatio);
        dropoutProbability += (1f - incidenceConfidence) * 0.2f;
        dropoutProbability = Mathf.Clamp(dropoutProbability, 0f, 0.95f);

        if (UnityEngine.Random.value < dropoutProbability)
        {
            return 0f;
        }

        return rangeConfidence * incidenceConfidence;
    }

    float CalculateRangeConfidence(float distance)
    {
        if (distance < reliableRangeMin)
        {
            return Mathf.Lerp(0.25f, 1f, Mathf.InverseLerp(rangeMin, reliableRangeMin, distance));
        }

        if (distance <= reliableRangeMax)
        {
            return 1f;
        }

        return Mathf.Lerp(1f, 0.35f, Mathf.InverseLerp(reliableRangeMax, rangeMax, distance));
    }

    // 외부에서 스캔 주기를 변경할 때 사용
    public void SetScanInterval(float interval)
    {
        scanInterval = interval;
    }

    public float GetDistance() => Distance;

    public (float distance, float angle, Vector3 point) GetDetectionData()
    {
        return (Distance, DetectedAngle, DetectedPoint);
    }
}
