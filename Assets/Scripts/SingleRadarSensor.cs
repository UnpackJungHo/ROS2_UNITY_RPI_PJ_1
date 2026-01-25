using UnityEngine;

/// <summary>
/// 단일 레이더 센서를 시뮬레이션하는 클래스입니다.
/// Physics.Raycast를 사용하여 일정 범위와 시야각 내의 물체를 감지합니다.
/// 감지된 물체와의 거리, 각도, 위치 및 상대 속도를 계산하여 제공합니다.
/// </summary>
public class SingleRadarSensor : MonoBehaviour
{
    // 센서의 위치를 정의하는 열거형 (전방, 후방)
    public enum SensorPosition
    {
        Front, // 전방 센서
        Rear   // 후방 센서
    }

    [Header("Sensor Identity")]
    [Tooltip("센서 위치 (Front, Rear)")]
    public SensorPosition sensorPosition = SensorPosition.Front;

    [Header("Radar Specifications (레이더 사양)")]
    [Tooltip("최소 감지 거리 (m) - 이 거리보다 가까운 물체는 무시됨")]
    public float rangeMin = 0.2f;

    [Tooltip("최대 감지 거리 (m) - 이 거리보다 먼 물체는 감지하지 않음")]
    public float rangeMax = 50f;

    [Tooltip("수평 시야각 (도) - 좌우 감지 범위")]
    public float horizontalFOV = 60f;

    [Tooltip("수직 시야각 (도) - 상하 감지 범위")]
    public float verticalFOV = 10f;

    [Tooltip("수평 레이(Ray) 개수 - 수평 해상도 결정")]
    public int horizontalRayNum = 15;

    [Tooltip("수직 레이(Ray) 개수 - 수직 해상도 결정")]
    public int verticalRayNum = 3;

    [Header("Collision Detection Layer (감지할 레이어)")]
    public LayerMask detectionLayer = ~0; // 기본값은 모든 레이어 감지

    [Header("Debug (디버그 설정)")]
    public bool showDebugRays = true; // Scene 뷰에 레이를 그릴지 여부
    public Color hitColor = Color.red; // 물체 감지 시 레이 색상
    public Color missColor = new Color(1f, 0.5f, 0.5f, 0.3f); // 물체 미감지 시 레이 색상

    // 감지 결과 속성들
    
    // 가장 가까운 물체와의 거리 (감지된 게 없으면 Infinity)
    public float Distance { get; private set; } = float.PositiveInfinity;
    
    // 감지된 물체의 각도 (수평 기준)
    public float DetectedAngle { get; private set; } = 0f;
    
    // 감지된 물체의 월드 좌표 상 위치
    public Vector3 DetectedPoint { get; private set; } = Vector3.zero;
    
    // 감지된 지점의 이동 속도 (이전 프레임과의 위치 차이로 계산)
    public float RelativeVelocity { get; private set; } = 0f;
    
    // 유효한 감지가 있는지 여부 확인
    public bool HasDetection => !float.IsInfinity(Distance);

    // 센서 이름 및 위치 확인 헬퍼 속성
    public string SensorName => sensorPosition == SensorPosition.Front ? "Front" : "Rear";
    public bool IsFrontSensor => sensorPosition == SensorPosition.Front;
    public bool IsRearSensor => sensorPosition == SensorPosition.Rear;

    // 내부 동작 변수
    private float scanInterval = 0.05f; // 스캔 주기 (초)
    private float lastScanTime;
    private Vector3 lastDetectedPoint;
    private float lastDetectionTime;

    void Start()
    {
        lastScanTime = Time.time;
        Debug.Log($"[Radar-{SensorName}] Initialized - Range: {rangeMin}-{rangeMax}m, H-FOV: {horizontalFOV}°, V-FOV: {verticalFOV}°");
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
    /// 실제 레이지(Ray)를 발사하여 스캔을 수행하는 메서드입니다.
    /// 설정된 수평/수직 레이 개수와 시야각에 따라 여러 개의 레이를 쏘아 가장 가까운 물체를 찾습니다.
    /// </summary>
    public void PerformScan()
    {
        Vector3 origin = transform.position; // 센서 위치 -> 센서 본인 오브젝트의 위치
        float halfHFOV = horizontalFOV * 0.5f;
        float halfVFOV = verticalFOV * 0.5f;
        
        // 레이 간의 각도 간격 계산
        float hAngleStep = horizontalRayNum > 1 ? horizontalFOV / (horizontalRayNum - 1) : 0f;
        float vAngleStep = verticalRayNum > 1 ? verticalFOV / (verticalRayNum - 1) : 0f;

        float prevDistance = Distance;
        Vector3 prevPoint = DetectedPoint;

        // 감지 데이터 초기화
        Distance = float.PositiveInfinity;
        DetectedAngle = 0f;
        DetectedPoint = Vector3.zero;

        // 수평 및 수직으로 반복하며 레이 발사
        for (int h = 0; h < horizontalRayNum; h++)
        {
            // 현재 수평 각도 계산
            float hAngle = -halfHFOV + (h * hAngleStep);

            for (int v = 0; v < verticalRayNum; v++)
            {
                // 현재 수직 각도 계산
                float vAngle = -halfVFOV + (v * vAngleStep);

                // 각도를 라디안으로 변환
                float hRad = hAngle * Mathf.Deg2Rad;
                float vRad = vAngle * Mathf.Deg2Rad;

                // 로컬 좌표계에서의 방향 벡터 계산 (구면 좌표계 변환)
                Vector3 localDirection = new Vector3(
                    Mathf.Sin(hRad) * Mathf.Cos(vRad),
                    Mathf.Sin(vRad),
                    Mathf.Cos(hRad) * Mathf.Cos(vRad)
                );

                // 로컬 방향을 월드 방향으로 변환
                Vector3 direction = transform.TransformDirection(localDirection);

                RaycastHit hit;
                // 레이캐스트 수행 (최대 거리까지, 설정된 레이어만)
                if (Physics.Raycast(origin, direction, out hit, rangeMax, detectionLayer))
                {
                    // 최소 거리 이상이고, 기존에 찾은 거리보다 더 가까운 경우 데이터 갱신
                    if (hit.distance >= rangeMin && hit.distance < Distance)
                    {
                        Distance = hit.distance;
                        DetectedAngle = hAngle;
                        DetectedPoint = hit.point;
                    }

                    // 디버그 레이 그리기 (충돌 시 hitColor)
                    if (showDebugRays)
                    {
                        Debug.DrawLine(origin, hit.point, hitColor, scanInterval);
                    }
                }
                else
                {
                    // 디버그 레이 그리기 (미충돌 시 missColor)
                    if (showDebugRays)
                    {
                        Debug.DrawRay(origin, direction * rangeMax, missColor, scanInterval);
                    }
                }
            }
        }

        // 상대 속도 계산 (이전 감지 지점과 비교)
        CalculateRelativeVelocity(prevPoint);
    }

    /// <summary>
    /// 감지된 지점의 이동 속도를 계산합니다.
    /// </summary>
    /// <param name="prevPoint">이전 프레임에서 감지된 지점</param>
    void CalculateRelativeVelocity(Vector3 prevPoint)
    {
        // 현재 감지된 물체가 있고, 이전에도 감지된 지점이 있다면 (0이 아니면)
        if (!float.IsInfinity(Distance) && prevPoint != Vector3.zero)
        {
            float deltaTime = Time.time - lastDetectionTime;
            if (deltaTime > 0.001f)
            {
                // 거리 변화량을 시간으로 나누어 속력 계산
                // 주의: 이는 물체의 실제 상대 속도라기보단 감지된 표면 지점의 이동 속도에 가깝습니다.
                float deltaDistance = Vector3.Distance(DetectedPoint, prevPoint);
                RelativeVelocity = deltaDistance / deltaTime;
            }
        }
        else
        {
            RelativeVelocity = 0f;
        }

        // 감지가 유효할 때만 상태 업데이트
        if (!float.IsInfinity(Distance))
        {
            lastDetectedPoint = DetectedPoint;
            lastDetectionTime = Time.time;
        }
    }

    // 외부 참조를 위한 Getter/Setter 메서드들
    public void SetScanInterval(float interval)
    {
        scanInterval = interval;
    }
}

