using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class RadarSensorPublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    public string topicName = "/radar";
    public float publishRate = 20f;

    [Header("Sensor References (각 센서 오브젝트에서 할당)")]
    [Tooltip("전방 레이더 센서")]
    public SingleRadarSensor sensorFront;
    [Tooltip("후방 레이더 센서")]
    public SingleRadarSensor sensorRear;

    [Header("Debug")]
    public bool showDebugInfo = false;

    public float FrontDistance => sensorFront != null ? sensorFront.Distance : float.PositiveInfinity;
    public float RearDistance => sensorRear != null ? sensorRear.Distance : float.PositiveInfinity;
    public float FrontAngle => sensorFront != null ? sensorFront.DetectedAngle : 0f;
    public float RearAngle => sensorRear != null ? sensorRear.DetectedAngle : 0f;
    public float FrontRelativeVelocity => sensorFront != null ? sensorFront.RelativeVelocity : 0f;
    public float RearRelativeVelocity => sensorRear != null ? sensorRear.RelativeVelocity : 0f;

    public float MinDistance => Mathf.Min(FrontDistance, RearDistance);

    public SingleRadarSensor.SensorPosition ClosestSensorPosition { get; private set; }
    public float ClosestDistance { get; private set; } = float.PositiveInfinity;

    private ROSConnection ros;
    private float publishInterval;
    private float lastPublishTime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float32MultiArrayMsg>(topicName);

        ValidateSensors();
        SyncScanIntervals();

        publishInterval = 1f / publishRate;
        lastPublishTime = Time.time;

        Debug.Log($"[RadarManager] Initialized - {CountActiveSensors()}/2 sensors active");
    }

    void ValidateSensors()
    {
        if (sensorFront == null) Debug.LogWarning("[RadarManager] sensorFront가 할당되지 않았습니다.");
        if (sensorRear == null) Debug.LogWarning("[RadarManager] sensorRear가 할당되지 않았습니다.");
    }

    void SyncScanIntervals()
    {
        float interval = 1f / publishRate;
        if (sensorFront != null) sensorFront.SetScanInterval(interval);
        if (sensorRear != null) sensorRear.SetScanInterval(interval);
    }

    int CountActiveSensors()
    {
        int count = 0;
        if (sensorFront != null) count++;
        if (sensorRear != null) count++;
        return count;
    }

    void Update()
    {
        if (Time.time - lastPublishTime >= publishInterval)
        {
            UpdateClosestSensor();
            PublishData();
            lastPublishTime = Time.time;

            if (showDebugInfo)
            {
                PrintDebugInfo();
            }
        }
    }

    void UpdateClosestSensor()
    {
        ClosestDistance = float.PositiveInfinity;
        ClosestSensorPosition = SingleRadarSensor.SensorPosition.Front;

        if (sensorFront != null && sensorFront.Distance < ClosestDistance)
        {
            ClosestDistance = sensorFront.Distance;
            ClosestSensorPosition = SingleRadarSensor.SensorPosition.Front;
        }
        if (sensorRear != null && sensorRear.Distance < ClosestDistance)
        {
            ClosestDistance = sensorRear.Distance;
            ClosestSensorPosition = SingleRadarSensor.SensorPosition.Rear;
        }
    }

    void PublishData()
    {
        Float32MultiArrayMsg msg = new Float32MultiArrayMsg
        {
            layout = new MultiArrayLayoutMsg
            {
                dim = new MultiArrayDimensionMsg[]
                {
                    new MultiArrayDimensionMsg
                    {
                        label = "radar_data",
                        size = 8,
                        stride = 8
                    }
                },
                data_offset = 0
            },
            data = new float[]
            {
                float.IsInfinity(FrontDistance) ? -1f : FrontDistance,
                FrontAngle,
                FrontRelativeVelocity,
                float.IsInfinity(RearDistance) ? -1f : RearDistance,
                RearAngle,
                RearRelativeVelocity,
                float.IsInfinity(ClosestDistance) ? -1f : ClosestDistance,
                (float)ClosestSensorPosition
            }
        };

        ros.Publish(topicName, msg);
    }

    void PrintDebugInfo()
    {
        string front = float.IsInfinity(FrontDistance) ? "∞" : $"{FrontDistance:F2}m @{FrontAngle:F1}°";
        string rear = float.IsInfinity(RearDistance) ? "∞" : $"{RearDistance:F2}m @{RearAngle:F1}°";

        Debug.Log($"[Radar] Front:{front} Rear:{rear} | Closest: {ClosestSensorPosition}");
    }

    public float GetDistance(SingleRadarSensor.SensorPosition position)
    {
        return position switch
        {
            SingleRadarSensor.SensorPosition.Front => FrontDistance,
            SingleRadarSensor.SensorPosition.Rear => RearDistance,
            _ => float.PositiveInfinity
        };
    }

    public SingleRadarSensor GetSensor(SingleRadarSensor.SensorPosition position)
    {
        return position switch
        {
            SingleRadarSensor.SensorPosition.Front => sensorFront,
            SingleRadarSensor.SensorPosition.Rear => sensorRear,
            _ => null
        };
    }

    public bool IsFrontClear(float threshold = 2.0f)
    {
        return FrontDistance > threshold || float.IsInfinity(FrontDistance);
    }

    public bool IsRearClear(float threshold = 2.0f)
    {
        return RearDistance > threshold || float.IsInfinity(RearDistance);
    }

    public (float distance, float angle, float velocity) GetFrontData()
    {
        return (FrontDistance, FrontAngle, FrontRelativeVelocity);
    }

    public (float distance, float angle, float velocity) GetRearData()
    {
        return (RearDistance, RearAngle, RearRelativeVelocity);
    }
}
