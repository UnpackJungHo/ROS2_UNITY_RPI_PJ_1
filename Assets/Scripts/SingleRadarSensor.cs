using UnityEngine;

public class SingleRadarSensor : MonoBehaviour
{
    public enum SensorPosition
    {
        Front,
        Rear
    }

    [Header("Sensor Identity")]
    [Tooltip("센서 위치 (Front, Rear)")]
    public SensorPosition sensorPosition = SensorPosition.Front;

    [Header("Radar Specifications")]
    [Tooltip("최소 감지 거리 (m)")]
    public float rangeMin = 0.5f;
    [Tooltip("최대 감지 거리 (m)")]
    public float rangeMax = 50f;
    [Tooltip("수평 시야각 (도)")]
    public float horizontalFOV = 60f;
    [Tooltip("수직 시야각 (도)")]
    public float verticalFOV = 10f;
    [Tooltip("수평 레이 개수")]
    public int horizontalRays = 15;
    [Tooltip("수직 레이 개수")]
    public int verticalRays = 3;

    [Header("Collision Detection Layer")]
    public LayerMask detectionLayer = ~0;

    [Header("Debug")]
    public bool showDebugRays = true;
    public Color hitColor = Color.red;
    public Color missColor = new Color(1f, 0.5f, 0.5f, 0.3f);

    public float Distance { get; private set; } = float.PositiveInfinity;
    public float DetectedAngle { get; private set; } = 0f;
    public Vector3 DetectedPoint { get; private set; } = Vector3.zero;
    public float RelativeVelocity { get; private set; } = 0f;
    public bool HasDetection => !float.IsInfinity(Distance);

    public string SensorName => sensorPosition == SensorPosition.Front ? "Front" : "Rear";
    public bool IsFrontSensor => sensorPosition == SensorPosition.Front;
    public bool IsRearSensor => sensorPosition == SensorPosition.Rear;

    private float scanInterval = 0.05f;
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
        if (Time.time - lastScanTime >= scanInterval)
        {
            PerformScan();
            lastScanTime = Time.time;
        }
    }

    public void PerformScan()
    {
        Vector3 origin = transform.position;
        float halfHFOV = horizontalFOV * 0.5f;
        float halfVFOV = verticalFOV * 0.5f;
        float hAngleStep = horizontalRays > 1 ? horizontalFOV / (horizontalRays - 1) : 0f;
        float vAngleStep = verticalRays > 1 ? verticalFOV / (verticalRays - 1) : 0f;

        float prevDistance = Distance;
        Vector3 prevPoint = DetectedPoint;

        Distance = float.PositiveInfinity;
        DetectedAngle = 0f;
        DetectedPoint = Vector3.zero;

        for (int h = 0; h < horizontalRays; h++)
        {
            float hAngle = -halfHFOV + (h * hAngleStep);

            for (int v = 0; v < verticalRays; v++)
            {
                float vAngle = -halfVFOV + (v * vAngleStep);

                float hRad = hAngle * Mathf.Deg2Rad;
                float vRad = vAngle * Mathf.Deg2Rad;

                Vector3 localDirection = new Vector3(
                    Mathf.Sin(hRad) * Mathf.Cos(vRad),
                    Mathf.Sin(vRad),
                    Mathf.Cos(hRad) * Mathf.Cos(vRad)
                );

                Vector3 direction = transform.TransformDirection(localDirection);

                RaycastHit hit;
                if (Physics.Raycast(origin, direction, out hit, rangeMax, detectionLayer))
                {
                    if (hit.distance >= rangeMin && hit.distance < Distance)
                    {
                        Distance = hit.distance;
                        DetectedAngle = hAngle;
                        DetectedPoint = hit.point;
                    }

                    if (showDebugRays)
                    {
                        Debug.DrawLine(origin, hit.point, hitColor, scanInterval);
                    }
                }
                else
                {
                    if (showDebugRays)
                    {
                        Debug.DrawRay(origin, direction * rangeMax, missColor, scanInterval);
                    }
                }
            }
        }

        CalculateRelativeVelocity(prevPoint);
    }

    void CalculateRelativeVelocity(Vector3 prevPoint)
    {
        if (!float.IsInfinity(Distance) && prevPoint != Vector3.zero)
        {
            float deltaTime = Time.time - lastDetectionTime;
            if (deltaTime > 0.001f)
            {
                float deltaDistance = Vector3.Distance(DetectedPoint, prevPoint);
                RelativeVelocity = deltaDistance / deltaTime;
            }
        }
        else
        {
            RelativeVelocity = 0f;
        }

        if (!float.IsInfinity(Distance))
        {
            lastDetectedPoint = DetectedPoint;
            lastDetectionTime = Time.time;
        }
    }

    public void SetScanInterval(float interval)
    {
        scanInterval = interval;
    }

    public float GetDistance() => Distance;

    public (float distance, float angle, Vector3 point, float velocity) GetDetectionData()
    {
        return (Distance, DetectedAngle, DetectedPoint, RelativeVelocity);
    }
}
