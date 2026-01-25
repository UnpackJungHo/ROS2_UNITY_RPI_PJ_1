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
        FrontLeft,  // 전방 좌측
        FrontRight, // 전방 우측
        RearLeft,   // 후방 좌측
        RearRight   // 후방 우측
    }

    [Header("Sensor Identity")]
    [Tooltip("센서 위치 (FL, FR, RL, RR) - 이 설정에 따라 센서 이름과 역할이 결정됨")]
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

    [Header("Collision Detection Layer (감지할 레이어)")]
    public LayerMask detectionLayer = ~0; // 기본값: 모든 레이어 감지

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
    
    // 유효한 감지가 있는지 여부 확인
    public bool HasDetection => !float.IsInfinity(Distance);

    // 센서 이름을 약어로 반환하는 헬퍼 속성 (로그 출력 등에 사용)
    public string SensorName => sensorPosition switch
    {
        SensorPosition.FrontLeft => "FL",
        SensorPosition.FrontRight => "FR",
        SensorPosition.RearLeft => "RL",
        SensorPosition.RearRight => "RR",
        _ => "Unknown"
    };

    // 센서의 위치/방향 확인을 위한 헬퍼 속성들
    public bool IsFrontSensor => sensorPosition == SensorPosition.FrontLeft ||
                                  sensorPosition == SensorPosition.FrontRight;

    public bool IsRearSensor => sensorPosition == SensorPosition.RearLeft ||
                                 sensorPosition == SensorPosition.RearRight;

    public bool IsLeftSensor => sensorPosition == SensorPosition.FrontLeft ||
                                 sensorPosition == SensorPosition.RearLeft;

    public bool IsRightSensor => sensorPosition == SensorPosition.FrontRight ||
                                  sensorPosition == SensorPosition.RearRight;

    // 내부 동작 제어 변수
    private float scanInterval = 0.05f; // 스캔 주기(초)
    private float lastScanTime;

    void Start()
    {
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

            RaycastHit hit;
            // 레이캐스트 수행
            if (Physics.Raycast(origin, direction, out hit, rangeMax, detectionLayer))
            {
                // 최소 거리 이상이고, 기존 최단 거리보다 가까우면 갱신
                if (hit.distance >= rangeMin && hit.distance < Distance)
                {
                    Distance = hit.distance;
                    DetectedAngle = angle;
                    DetectedPoint = hit.point;
                }

                // 디버그 레이 (감지됨: Cyan)
                if (showDebugRays)
                {
                    Debug.DrawLine(origin, hit.point, hitColor, scanInterval);
                }
            }
            else
            {
                // 디버그 레이 (감지안됨: Blue)
                if (showDebugRays)
                {
                    Debug.DrawRay(origin, direction * rangeMax, missColor, scanInterval);
                }
            }
        }
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

