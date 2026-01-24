using UnityEngine;

public class SingleUltrasonicSensor : MonoBehaviour
{
    public enum SensorPosition
    {
        FrontLeft,
        FrontRight,
        RearLeft,
        RearRight
    }

    [Header("Sensor Identity")]
    [Tooltip("센서 위치 (FL, FR, RL, RR)")]
    public SensorPosition sensorPosition = SensorPosition.FrontLeft;

    [Header("Ultrasonic Specifications")]
    [Tooltip("최소 감지 거리 (m)")]
    public float rangeMin = 0.02f;
    [Tooltip("최대 감지 거리 (m)")]
    public float rangeMax = 4.0f;
    [Tooltip("감지 시야각 (도)")]
    public float fieldOfView = 30f;
    [Tooltip("레이 개수")]
    public int rayCount = 5;

    [Header("Collision Detection Layer")]
    public LayerMask detectionLayer = ~0;

    [Header("Debug")]
    public bool showDebugRays = true;
    public Color hitColor = Color.cyan;
    public Color missColor = Color.blue;

    public float Distance { get; private set; } = float.PositiveInfinity;
    public float DetectedAngle { get; private set; } = 0f;
    public Vector3 DetectedPoint { get; private set; } = Vector3.zero;
    public bool HasDetection => !float.IsInfinity(Distance);

    public string SensorName => sensorPosition switch
    {
        SensorPosition.FrontLeft => "FL",
        SensorPosition.FrontRight => "FR",
        SensorPosition.RearLeft => "RL",
        SensorPosition.RearRight => "RR",
        _ => "Unknown"
    };

    public bool IsFrontSensor => sensorPosition == SensorPosition.FrontLeft ||
                                  sensorPosition == SensorPosition.FrontRight;

    public bool IsRearSensor => sensorPosition == SensorPosition.RearLeft ||
                                 sensorPosition == SensorPosition.RearRight;

    public bool IsLeftSensor => sensorPosition == SensorPosition.FrontLeft ||
                                 sensorPosition == SensorPosition.RearLeft;

    public bool IsRightSensor => sensorPosition == SensorPosition.FrontRight ||
                                  sensorPosition == SensorPosition.RearRight;

    private float scanInterval = 0.05f;
    private float lastScanTime;

    void Start()
    {
        lastScanTime = Time.time;
        Debug.Log($"[Ultrasonic-{SensorName}] Initialized - Range: {rangeMin}-{rangeMax}m, FOV: {fieldOfView}°");
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
        float halfFOV = fieldOfView * 0.5f;
        float angleStep = rayCount > 1 ? fieldOfView / (rayCount - 1) : 0f;

        Distance = float.PositiveInfinity;
        DetectedAngle = 0f;
        DetectedPoint = Vector3.zero;

        for (int i = 0; i < rayCount; i++)
        {
            float angle = -halfFOV + (i * angleStep);
            float angleRad = angle * Mathf.Deg2Rad;

            Vector3 direction = transform.TransformDirection(
                new Vector3(Mathf.Sin(angleRad), 0, Mathf.Cos(angleRad))
            );

            RaycastHit hit;
            if (Physics.Raycast(origin, direction, out hit, rangeMax, detectionLayer))
            {
                if (hit.distance >= rangeMin && hit.distance < Distance)
                {
                    Distance = hit.distance;
                    DetectedAngle = angle;
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
