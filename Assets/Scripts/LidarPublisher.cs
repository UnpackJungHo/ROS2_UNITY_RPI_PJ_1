using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

public class LidarPublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    public string topicName = "/scan";
    public string frameId = "lidar_link";
    public float publishRate = 10f;

    [Header("Lidar Specifications")]
    [Tooltip("최소 감지 거리 (m)")]
    public float rangeMin = 0.1f;
    [Tooltip("최대 감지 거리 (m)")]
    public float rangeMax = 12f;
    [Tooltip("스캔 레이 개수 (360도 기준)")]
    public int numRays = 360;
    [Tooltip("스캔 시작 각도 (라디안, 0 = 전방)")]
    public float angleMin = -Mathf.PI;
    [Tooltip("스캔 종료 각도 (라디안)")]
    public float angleMax = Mathf.PI;

    [Header("Collision Detection Layer")]
    [Tooltip("라이다가 감지할 레이어")]
    public LayerMask detectionLayer = ~0;

    [Header("Lidar Reference")]
    [Tooltip("라이다 센서 위치 Transform을 직접 할당하세요 (lidar_link)")]
    public Transform lidarTransform;

    [Header("Debug")]
    public bool showDebugRays = true;
    public Color hitColor = Color.red;
    public Color missColor = Color.green;

    private ROSConnection ros;
    private float publishInterval;
    private float lastPublishTime;
    private float[] ranges;
    private float[] intensities;
    private float angleIncrement;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(topicName);

        if (lidarTransform == null)
        {
            Debug.LogError("[LidarPublisher] lidarTransform이 할당되지 않았습니다! Inspector에서 lidar_link를 할당하세요.");
            lidarTransform = transform;
        }

        InitializeLidar();
        Debug.Log($"[LidarPublisher] Initialized - {numRays} rays, range: {rangeMin}-{rangeMax}m, topic: {topicName}");
    }

    void InitializeLidar()
    {
        publishInterval = 1f / publishRate;
        lastPublishTime = Time.time;

        ranges = new float[numRays];
        intensities = new float[numRays];
        angleIncrement = (angleMax - angleMin) / numRays;
    }

    void Update()
    {
        if (Time.time - lastPublishTime >= publishInterval)
        {
            PerformScan();
            PublishScan();
            lastPublishTime = Time.time;
        }
    }

    void PerformScan()
    {
        if (lidarTransform == null) return;

        Vector3 origin = lidarTransform.position;

        for (int i = 0; i < numRays; i++)
        {
            float angle = angleMin + (i * angleIncrement);

            Vector3 direction = lidarTransform.TransformDirection(
                new Vector3(Mathf.Sin(angle), 0, Mathf.Cos(angle))
            );

            RaycastHit hit;
            if (Physics.Raycast(origin, direction, out hit, rangeMax, detectionLayer))
            {
                float distance = hit.distance;
                if (distance >= rangeMin && distance <= rangeMax)
                {
                    ranges[i] = distance;
                    intensities[i] = 1.0f;
                }
                else
                {
                    ranges[i] = float.PositiveInfinity;
                    intensities[i] = 0f;
                }

                if (showDebugRays)
                {
                    Debug.DrawLine(origin, hit.point, hitColor, publishInterval);
                }
            }
            else
            {
                ranges[i] = float.PositiveInfinity;
                intensities[i] = 0f;

                if (showDebugRays)
                {
                    Debug.DrawRay(origin, direction * rangeMax, missColor, publishInterval);
                }
            }
        }
    }

    void PublishScan()
    {
        float scanTime = publishInterval;
        float timeIncrement = scanTime / numRays;

        LaserScanMsg scanMsg = new LaserScanMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1) * 1e9)
                },
                frame_id = frameId
            },
            angle_min = angleMin,
            angle_max = angleMax,
            angle_increment = angleIncrement,
            time_increment = timeIncrement,
            scan_time = scanTime,
            range_min = rangeMin,
            range_max = rangeMax,
            ranges = ranges,
            intensities = intensities
        };

        ros.Publish(topicName, scanMsg);
    }

    public float GetRangeAtAngle(float angleDegrees)
    {
        float angleRad = angleDegrees * Mathf.Deg2Rad;
        int index = Mathf.RoundToInt((angleRad - angleMin) / angleIncrement);
        index = Mathf.Clamp(index, 0, numRays - 1);
        return ranges[index];
    }

    public float GetMinRangeInFront(float fovDegrees = 60f)
    {
        float halfFov = fovDegrees * 0.5f * Mathf.Deg2Rad;
        float minRange = float.PositiveInfinity;

        for (int i = 0; i < numRays; i++)
        {
            float angle = angleMin + (i * angleIncrement);
            if (Mathf.Abs(angle) <= halfFov)
            {
                if (ranges[i] < minRange)
                {
                    minRange = ranges[i];
                }
            }
        }

        return minRange;
    }

    public (float distance, float angle) GetClosestObstacle(float startAngleDeg, float endAngleDeg)
    {
        float startRad = startAngleDeg * Mathf.Deg2Rad;
        float endRad = endAngleDeg * Mathf.Deg2Rad;
        float minRange = float.PositiveInfinity;
        float minAngle = 0f;

        for (int i = 0; i < numRays; i++)
        {
            float angle = angleMin + (i * angleIncrement);
            if (angle >= startRad && angle <= endRad)
            {
                if (ranges[i] < minRange)
                {
                    minRange = ranges[i];
                    minAngle = angle * Mathf.Rad2Deg;
                }
            }
        }

        return (minRange, minAngle);
    }
}
