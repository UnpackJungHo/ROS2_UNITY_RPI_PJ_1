using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class UltrasonicSensorPublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    public string topicName = "/ultrasonic";
    public float publishRate = 20f;

    [Header("Sensor References (각 센서 오브젝트에서 자동 검색 또는 수동 할당)")]
    [Tooltip("전방 좌측 초음파 센서")]
    public SingleUltrasonicSensor sensorFL;
    [Tooltip("전방 우측 초음파 센서")]
    public SingleUltrasonicSensor sensorFR;
    [Tooltip("후방 좌측 초음파 센서")]
    public SingleUltrasonicSensor sensorRL;
    [Tooltip("후방 우측 초음파 센서")]
    public SingleUltrasonicSensor sensorRR;

    [Header("Debug")]
    public bool showDebugInfo = false;

    public float FrontLeftDistance => sensorFL != null ? sensorFL.Distance : float.PositiveInfinity;
    public float FrontRightDistance => sensorFR != null ? sensorFR.Distance : float.PositiveInfinity;
    public float RearLeftDistance => sensorRL != null ? sensorRL.Distance : float.PositiveInfinity;
    public float RearRightDistance => sensorRR != null ? sensorRR.Distance : float.PositiveInfinity;

    public float MinFrontDistance => Mathf.Min(FrontLeftDistance, FrontRightDistance);
    public float MinRearDistance => Mathf.Min(RearLeftDistance, RearRightDistance);
    public float MinDistance => Mathf.Min(MinFrontDistance, MinRearDistance);

    public SingleUltrasonicSensor.SensorPosition ClosestSensorPosition { get; private set; }
    public float ClosestDistance { get; private set; } = float.PositiveInfinity;

    private ROSConnection ros;
    private float publishInterval;
    private float lastPublishTime;
    private SingleUltrasonicSensor[] allSensors;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float32MultiArrayMsg>(topicName);

        ValidateSensors();
        SyncScanIntervals();

        publishInterval = 1f / publishRate;
        lastPublishTime = Time.time;

        Debug.Log($"[UltrasonicManager] Initialized - {CountActiveSensors()}/4 sensors active");
    }

    void ValidateSensors()
    {
        allSensors = new SingleUltrasonicSensor[] { sensorFL, sensorFR, sensorRL, sensorRR };

        if (sensorFL == null) Debug.LogWarning("[UltrasonicManager] sensorFL이 할당되지 않았습니다.");
        if (sensorFR == null) Debug.LogWarning("[UltrasonicManager] sensorFR이 할당되지 않았습니다.");
        if (sensorRL == null) Debug.LogWarning("[UltrasonicManager] sensorRL이 할당되지 않았습니다.");
        if (sensorRR == null) Debug.LogWarning("[UltrasonicManager] sensorRR이 할당되지 않았습니다.");
    }

    void SyncScanIntervals()
    {
        float interval = 1f / publishRate;
        foreach (var sensor in allSensors)
        {
            if (sensor != null)
            {
                sensor.SetScanInterval(interval);
            }
        }
    }

    int CountActiveSensors()
    {
        int count = 0;
        foreach (var sensor in allSensors)
        {
            if (sensor != null) count++;
        }
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
        ClosestSensorPosition = SingleUltrasonicSensor.SensorPosition.FrontLeft;

        if (sensorFL != null && sensorFL.Distance < ClosestDistance)
        {
            ClosestDistance = sensorFL.Distance;
            ClosestSensorPosition = SingleUltrasonicSensor.SensorPosition.FrontLeft;
        }
        if (sensorFR != null && sensorFR.Distance < ClosestDistance)
        {
            ClosestDistance = sensorFR.Distance;
            ClosestSensorPosition = SingleUltrasonicSensor.SensorPosition.FrontRight;
        }
        if (sensorRL != null && sensorRL.Distance < ClosestDistance)
        {
            ClosestDistance = sensorRL.Distance;
            ClosestSensorPosition = SingleUltrasonicSensor.SensorPosition.RearLeft;
        }
        if (sensorRR != null && sensorRR.Distance < ClosestDistance)
        {
            ClosestDistance = sensorRR.Distance;
            ClosestSensorPosition = SingleUltrasonicSensor.SensorPosition.RearRight;
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
                        label = "ultrasonic_data",
                        size = 8,
                        stride = 8
                    }
                },
                data_offset = 0
            },
            data = new float[]
            {
                float.IsInfinity(FrontLeftDistance) ? -1f : FrontLeftDistance,
                float.IsInfinity(FrontRightDistance) ? -1f : FrontRightDistance,
                float.IsInfinity(RearLeftDistance) ? -1f : RearLeftDistance,
                float.IsInfinity(RearRightDistance) ? -1f : RearRightDistance,
                float.IsInfinity(MinFrontDistance) ? -1f : MinFrontDistance,
                float.IsInfinity(MinRearDistance) ? -1f : MinRearDistance,
                float.IsInfinity(ClosestDistance) ? -1f : ClosestDistance,
                (float)ClosestSensorPosition
            }
        };

        ros.Publish(topicName, msg);
    }

    void PrintDebugInfo()
    {
        string fl = float.IsInfinity(FrontLeftDistance) ? "∞" : $"{FrontLeftDistance:F2}";
        string fr = float.IsInfinity(FrontRightDistance) ? "∞" : $"{FrontRightDistance:F2}";
        string rl = float.IsInfinity(RearLeftDistance) ? "∞" : $"{RearLeftDistance:F2}";
        string rr = float.IsInfinity(RearRightDistance) ? "∞" : $"{RearRightDistance:F2}";

        Debug.Log($"[Ultrasonic] FL:{fl} FR:{fr} RL:{rl} RR:{rr} | Closest: {ClosestSensorPosition}");
    }

    public float GetDistance(SingleUltrasonicSensor.SensorPosition position)
    {
        return position switch
        {
            SingleUltrasonicSensor.SensorPosition.FrontLeft => FrontLeftDistance,
            SingleUltrasonicSensor.SensorPosition.FrontRight => FrontRightDistance,
            SingleUltrasonicSensor.SensorPosition.RearLeft => RearLeftDistance,
            SingleUltrasonicSensor.SensorPosition.RearRight => RearRightDistance,
            _ => float.PositiveInfinity
        };
    }

    public SingleUltrasonicSensor GetSensor(SingleUltrasonicSensor.SensorPosition position)
    {
        return position switch
        {
            SingleUltrasonicSensor.SensorPosition.FrontLeft => sensorFL,
            SingleUltrasonicSensor.SensorPosition.FrontRight => sensorFR,
            SingleUltrasonicSensor.SensorPosition.RearLeft => sensorRL,
            SingleUltrasonicSensor.SensorPosition.RearRight => sensorRR,
            _ => null
        };
    }

    public bool IsFrontClear(float threshold = 0.5f)
    {
        return MinFrontDistance > threshold || float.IsInfinity(MinFrontDistance);
    }

    public bool IsRearClear(float threshold = 0.5f)
    {
        return MinRearDistance > threshold || float.IsInfinity(MinRearDistance);
    }

    public bool IsLeftClear(float threshold = 0.5f)
    {
        float leftMin = Mathf.Min(FrontLeftDistance, RearLeftDistance);
        return leftMin > threshold || float.IsInfinity(leftMin);
    }

    public bool IsRightClear(float threshold = 0.5f)
    {
        float rightMin = Mathf.Min(FrontRightDistance, RearRightDistance);
        return rightMin > threshold || float.IsInfinity(rightMin);
    }
}
