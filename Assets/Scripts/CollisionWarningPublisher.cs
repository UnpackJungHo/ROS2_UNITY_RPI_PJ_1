using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class CollisionWarningPublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    public string warningTopicName = "/collision_warning";
    public string distanceTopicName = "/obstacle_distance";
    public float publishRate = 20f;

    [Header("Sensor Manager References")]
    [Tooltip("초음파 센서 매니저 (근거리 감지)")]
    public UltrasonicSensorPublisher ultrasonicManager;
    [Tooltip("레이더 센서 매니저 (중장거리 감지)")]
    public RadarSensorPublisher radarManager;

    [Header("Warning Thresholds - Ultrasonic (근거리)")]
    [Tooltip("초음파 위험 거리 - 즉시 정지 (m)")]
    public float ultrasonicDangerDistance = 0.3f;
    [Tooltip("초음파 경고 거리 - 감속 (m)")]
    public float ultrasonicWarningDistance = 0.8f;
    [Tooltip("초음파 주의 거리 (m)")]
    public float ultrasonicCautionDistance = 1.5f;

    [Header("Warning Thresholds - Radar (중장거리)")]
    [Tooltip("레이더 위험 거리 - 즉시 정지 (m)")]
    public float radarDangerDistance = 2.0f;
    [Tooltip("레이더 경고 거리 - 감속 (m)")]
    public float radarWarningDistance = 5.0f;
    [Tooltip("레이더 주의 거리 (m)")]
    public float radarCautionDistance = 10.0f;

    [Header("Speed Reference")]
    [Tooltip("속도 계산용 ArticulationBody (base_link)")]
    public ArticulationBody velocitySource;
    public bool useVelocityForTTC = true;

    [Header("Debug")]
    public bool showDebugInfo = true;

    [HideInInspector] public float currentMinDistance = float.PositiveInfinity;
    [HideInInspector] public float currentTTC = float.PositiveInfinity;
    [HideInInspector] public WarningLevel currentWarningLevel = WarningLevel.Safe;
    [HideInInspector] public string detectionSource = "None";
    [HideInInspector] public string detectionSensor = "None";

    public enum WarningLevel
    {
        Safe = 0,
        Caution = 1,
        Warning = 2,
        Danger = 3
    }

    public struct SensorData
    {
        public float ultrasonicFL;
        public float ultrasonicFR;
        public float ultrasonicRL;
        public float ultrasonicRR;
        public float ultrasonicMinFront;
        public float ultrasonicMinRear;
        public float radarFront;
        public float radarRear;
        public SingleUltrasonicSensor.SensorPosition ultrasonicClosest;
        public SingleRadarSensor.SensorPosition radarClosest;
    }

    public SensorData CurrentSensorData { get; private set; }

    private ROSConnection ros;
    private float publishInterval;
    private float lastPublishTime;
    private float currentSpeed = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float32MultiArrayMsg>(warningTopicName);
        ros.RegisterPublisher<Float32Msg>(distanceTopicName);

        ValidateSensorReferences();

        publishInterval = 1f / publishRate;
        lastPublishTime = Time.time;

        Debug.Log($"[CollisionWarning] Initialized with distributed sensor fusion");
        Debug.Log($"[CollisionWarning] Ultrasonic thresholds - Danger: {ultrasonicDangerDistance}m, Warning: {ultrasonicWarningDistance}m");
        Debug.Log($"[CollisionWarning] Radar thresholds - Danger: {radarDangerDistance}m, Warning: {radarWarningDistance}m");
    }

    void ValidateSensorReferences()
    {
        if (ultrasonicManager == null)
            Debug.LogWarning("[CollisionWarning] UltrasonicSensorPublisher가 할당되지 않았습니다.");
        if (radarManager == null)
            Debug.LogWarning("[CollisionWarning] RadarSensorPublisher가 할당되지 않았습니다.");
        if (useVelocityForTTC && velocitySource == null)
            Debug.LogWarning("[CollisionWarning] velocitySource가 할당되지 않았습니다. TTC 계산이 비활성화됩니다.");
    }

    void Update()
    {
        UpdateCurrentSpeed();

        if (Time.time - lastPublishTime >= publishInterval)
        {
            CollectSensorData();
            CalculateWarningLevel();
            CalculateTTC();
            PublishWarning();
            lastPublishTime = Time.time;

            if (showDebugInfo)
            {
                PrintDebugInfo();
            }
        }
    }

    void UpdateCurrentSpeed()
    {
        if (!useVelocityForTTC || velocitySource == null)
        {
            currentSpeed = 0f;
            return;
        }

        currentSpeed = velocitySource.velocity.magnitude;
    }

    void CollectSensorData()
    {
        SensorData data = new SensorData();

        if (ultrasonicManager != null)
        {
            data.ultrasonicFL = ultrasonicManager.FrontLeftDistance;
            data.ultrasonicFR = ultrasonicManager.FrontRightDistance;
            data.ultrasonicRL = ultrasonicManager.RearLeftDistance;
            data.ultrasonicRR = ultrasonicManager.RearRightDistance;
            data.ultrasonicMinFront = ultrasonicManager.MinFrontDistance;
            data.ultrasonicMinRear = ultrasonicManager.MinRearDistance;
            data.ultrasonicClosest = ultrasonicManager.ClosestSensorPosition;
        }
        else
        {
            data.ultrasonicFL = float.PositiveInfinity;
            data.ultrasonicFR = float.PositiveInfinity;
            data.ultrasonicRL = float.PositiveInfinity;
            data.ultrasonicRR = float.PositiveInfinity;
            data.ultrasonicMinFront = float.PositiveInfinity;
            data.ultrasonicMinRear = float.PositiveInfinity;
        }

        if (radarManager != null)
        {
            data.radarFront = radarManager.FrontDistance;
            data.radarRear = radarManager.RearDistance;
            data.radarClosest = radarManager.ClosestSensorPosition;
        }
        else
        {
            data.radarFront = float.PositiveInfinity;
            data.radarRear = float.PositiveInfinity;
        }

        CurrentSensorData = data;
    }

    void CalculateWarningLevel()
    {
        currentWarningLevel = WarningLevel.Safe;
        currentMinDistance = float.PositiveInfinity;
        detectionSource = "None";
        detectionSensor = "None";

        WarningLevel ultrasonicLevel = EvaluateUltrasonicWarning();
        WarningLevel radarLevel = EvaluateRadarWarning();

        if (ultrasonicLevel >= radarLevel && ultrasonicLevel != WarningLevel.Safe)
        {
            currentWarningLevel = ultrasonicLevel;
            currentMinDistance = ultrasonicManager != null ? ultrasonicManager.ClosestDistance : float.PositiveInfinity;
            detectionSource = "Ultrasonic";
            detectionSensor = CurrentSensorData.ultrasonicClosest.ToString();
        }
        else if (radarLevel != WarningLevel.Safe)
        {
            currentWarningLevel = radarLevel;
            currentMinDistance = radarManager != null ? radarManager.ClosestDistance : float.PositiveInfinity;
            detectionSource = "Radar";
            detectionSensor = CurrentSensorData.radarClosest.ToString();
        }

        if (float.IsInfinity(currentMinDistance))
        {
            float ultrasonicMin = Mathf.Min(CurrentSensorData.ultrasonicMinFront, CurrentSensorData.ultrasonicMinRear);
            float radarMin = Mathf.Min(CurrentSensorData.radarFront, CurrentSensorData.radarRear);
            currentMinDistance = Mathf.Min(ultrasonicMin, radarMin);
        }
    }

    WarningLevel EvaluateUltrasonicWarning()
    {
        if (ultrasonicManager == null) return WarningLevel.Safe;

        float minUltrasonic = ultrasonicManager.ClosestDistance;

        if (float.IsInfinity(minUltrasonic))
            return WarningLevel.Safe;

        if (minUltrasonic <= ultrasonicDangerDistance)
            return WarningLevel.Danger;
        if (minUltrasonic <= ultrasonicWarningDistance)
            return WarningLevel.Warning;
        if (minUltrasonic <= ultrasonicCautionDistance)
            return WarningLevel.Caution;

        return WarningLevel.Safe;
    }

    WarningLevel EvaluateRadarWarning()
    {
        if (radarManager == null) return WarningLevel.Safe;

        float minRadar = radarManager.ClosestDistance;

        if (float.IsInfinity(minRadar))
            return WarningLevel.Safe;

        if (minRadar <= radarDangerDistance)
            return WarningLevel.Danger;
        if (minRadar <= radarWarningDistance)
            return WarningLevel.Warning;
        if (minRadar <= radarCautionDistance)
            return WarningLevel.Caution;

        return WarningLevel.Safe;
    }

    void CalculateTTC()
    {
        if (currentSpeed <= 0.01f || float.IsInfinity(currentMinDistance))
        {
            currentTTC = float.PositiveInfinity;
            return;
        }

        currentTTC = currentMinDistance / currentSpeed;
    }

    void PublishWarning()
    {
        Float32MultiArrayMsg warningMsg = new Float32MultiArrayMsg
        {
            layout = new MultiArrayLayoutMsg
            {
                dim = new MultiArrayDimensionMsg[]
                {
                    new MultiArrayDimensionMsg
                    {
                        label = "collision_data",
                        size = 14,
                        stride = 14
                    }
                },
                data_offset = 0
            },
            data = new float[]
            {
                float.IsInfinity(currentMinDistance) ? -1f : currentMinDistance,
                float.IsInfinity(currentTTC) ? -1f : currentTTC,
                (float)currentWarningLevel,
                currentSpeed,
                float.IsInfinity(CurrentSensorData.ultrasonicFL) ? -1f : CurrentSensorData.ultrasonicFL,
                float.IsInfinity(CurrentSensorData.ultrasonicFR) ? -1f : CurrentSensorData.ultrasonicFR,
                float.IsInfinity(CurrentSensorData.ultrasonicRL) ? -1f : CurrentSensorData.ultrasonicRL,
                float.IsInfinity(CurrentSensorData.ultrasonicRR) ? -1f : CurrentSensorData.ultrasonicRR,
                float.IsInfinity(CurrentSensorData.radarFront) ? -1f : CurrentSensorData.radarFront,
                float.IsInfinity(CurrentSensorData.radarRear) ? -1f : CurrentSensorData.radarRear,
                detectionSource == "Ultrasonic" ? 1f : (detectionSource == "Radar" ? 2f : 0f),
                (float)CurrentSensorData.ultrasonicClosest,
                (float)CurrentSensorData.radarClosest,
                0f
            }
        };

        ros.Publish(warningTopicName, warningMsg);

        Float32Msg distanceMsg = new Float32Msg
        {
            data = float.IsInfinity(currentMinDistance) ? -1f : currentMinDistance
        };
        ros.Publish(distanceTopicName, distanceMsg);
    }

    void PrintDebugInfo()
    {
        string levelStr = currentWarningLevel switch
        {
            WarningLevel.Safe => "SAFE",
            WarningLevel.Caution => "CAUTION",
            WarningLevel.Warning => "WARNING",
            WarningLevel.Danger => "!! DANGER !!",
            _ => "UNKNOWN"
        };

        string distStr = float.IsInfinity(currentMinDistance) ? "∞" : $"{currentMinDistance:F2}m";
        string ttcStr = float.IsInfinity(currentTTC) ? "∞" : $"{currentTTC:F2}s";

        if (currentWarningLevel != WarningLevel.Safe)
        {
            Debug.Log($"[Collision] {levelStr} | Dist: {distStr} | TTC: {ttcStr} | Source: {detectionSource}-{detectionSensor} | Speed: {currentSpeed:F2}m/s");
        }
    }

    public bool IsDanger() => currentWarningLevel == WarningLevel.Danger;
    public bool IsWarning() => currentWarningLevel >= WarningLevel.Warning;
    public bool IsCaution() => currentWarningLevel >= WarningLevel.Caution;

    public float GetDistanceToObstacle() => currentMinDistance;
    public float GetTimeToCollision() => currentTTC;
    public WarningLevel GetWarningLevel() => currentWarningLevel;

    public float GetSafetyMargin(float reactionTime = 0.5f, float deceleration = 2f)
    {
        if (currentSpeed <= 0.01f)
            return currentMinDistance;

        float reactionDistance = currentSpeed * reactionTime;
        float brakingDistance = (currentSpeed * currentSpeed) / (2f * deceleration);
        float requiredDistance = reactionDistance + brakingDistance;

        return currentMinDistance - requiredDistance;
    }

    public bool IsFrontClear(float ultrasonicThreshold = 0.5f, float radarThreshold = 2.0f)
    {
        bool ultrasonicClear = ultrasonicManager == null || ultrasonicManager.IsFrontClear(ultrasonicThreshold);
        bool radarClear = radarManager == null || radarManager.IsFrontClear(radarThreshold);
        return ultrasonicClear && radarClear;
    }

    public bool IsRearClear(float ultrasonicThreshold = 0.5f, float radarThreshold = 2.0f)
    {
        bool ultrasonicClear = ultrasonicManager == null || ultrasonicManager.IsRearClear(ultrasonicThreshold);
        bool radarClear = radarManager == null || radarManager.IsRearClear(radarThreshold);
        return ultrasonicClear && radarClear;
    }

    public (string source, string sensor, float distance) GetClosestObstacleInfo()
    {
        return (detectionSource, detectionSensor, currentMinDistance);
    }
}
