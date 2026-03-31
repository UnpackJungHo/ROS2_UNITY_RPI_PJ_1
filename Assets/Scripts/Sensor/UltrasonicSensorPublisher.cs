using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// 8개 초음파 센서를 집계하여 단일 ROS 토픽으로 발행하고,
/// 필요 시 동일한 집계 포맷의 외부 토픽을 구독해 각 센서에 주입한다.
/// 집계 데이터 순서는 FL, FR, FC, RL, RR, RC, SL, SR이며
/// 각 센서마다 [distance_m, confidence, angle_deg] 3개 값을 사용한다.
/// </summary>
public class UltrasonicSensorPublisher : MonoBehaviour
{
    private const int SensorCount = 8;
    private const int FieldsPerSensor = 3;
    private const int DistanceIndex = 0;
    private const int ConfidenceIndex = 1;
    private const int AngleIndex = 2;

    [Header("ROS Output Settings")]
    [Tooltip("집계 초음파 발행 토픽")]
    public string topicName = "/ultrasonic";
    public float publishRate = 20f;
    [Tooltip("true면 집계 초음파 토픽을 발행")]
    public bool publishAggregatedTopic = true;

    [Header("External Topic Input")]
    [Tooltip("true면 개별 레이캐스트 대신 집계 초음파 입력 토픽을 우선 사용")]
    public bool useExternalTopicInput = false;
    [Tooltip("비어 있으면 topicName과 동일한 입력 토픽을 사용")]
    public string inputTopicName = "";
    [Tooltip("이 시간(초) 이상 새 메시지가 없으면 stale로 판단")]
    public float externalDataTimeoutSec = 0.5f;
    [Tooltip("외부 입력이 stale일 때 시뮬레이션 레이캐스트로 fallback")]
    public bool fallbackToRaycastWhenExternalStale = true;
    [Tooltip("confidence가 메시지에 없을 때 사용할 기본값")]
    [Range(0f, 1f)]
    public float defaultExternalConfidence = 1f;

    [Header("Sensor References")]
    public SingleUltrasonicSensor sensorFL;
    public SingleUltrasonicSensor sensorFR;
    public SingleUltrasonicSensor sensorFC;
    public SingleUltrasonicSensor sensorRL;
    public SingleUltrasonicSensor sensorRR;
    public SingleUltrasonicSensor sensorRC;
    public SingleUltrasonicSensor sensorSL;
    public SingleUltrasonicSensor sensorSR;

    [Header("Debug")]
    public bool showDebugInfo = false;

    [Header("Topic Debug (Read Only)")]
    [SerializeField] private string resolvedOutputTopicName = "";
    [SerializeField] private string resolvedInputTopicName = "";
    [SerializeField] private bool hasExternalMessage = false;
    [SerializeField] private float lastExternalMessageTime = -999f;

    public float FrontLeftDistance => sensorFL != null ? sensorFL.Distance : float.PositiveInfinity;
    public float FrontRightDistance => sensorFR != null ? sensorFR.Distance : float.PositiveInfinity;
    public float FrontCenterDistance => sensorFC != null ? sensorFC.Distance : float.PositiveInfinity;
    public float RearLeftDistance => sensorRL != null ? sensorRL.Distance : float.PositiveInfinity;
    public float RearRightDistance => sensorRR != null ? sensorRR.Distance : float.PositiveInfinity;
    public float RearCenterDistance => sensorRC != null ? sensorRC.Distance : float.PositiveInfinity;
    public float SideLeftDistance => sensorSL != null ? sensorSL.Distance : float.PositiveInfinity;
    public float SideRightDistance => sensorSR != null ? sensorSR.Distance : float.PositiveInfinity;

    public float MinFrontDistance => Mathf.Min(FrontLeftDistance, FrontRightDistance, FrontCenterDistance);
    public float MinRearDistance => Mathf.Min(RearLeftDistance, RearRightDistance, RearCenterDistance);
    public float MinSideDistance => Mathf.Min(SideLeftDistance, SideRightDistance);
    public float MinDistance => Mathf.Min(MinFrontDistance, Mathf.Min(MinRearDistance, MinSideDistance));

    public SingleUltrasonicSensor.SensorPosition ClosestSensorPosition { get; private set; }
    public float ClosestDistance { get; private set; } = float.PositiveInfinity;
    public float ClosestConfidence { get; private set; } = 0f;

    private ROSConnection ros;
    private float publishInterval;
    private float lastPublishTime;
    private SingleUltrasonicSensor[] allSensors;
    private bool aggregatedPublisherReady = false;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ValidateSensors();
        SyncScanIntervals();

        if (publishAggregatedTopic)
            SetupAggregatedTopicOutput();
        if (useExternalTopicInput)
            SetupExternalTopicInput();

        publishInterval = 1f / Mathf.Max(1f, publishRate);
        lastPublishTime = Time.time;

        Debug.Log($"[UltrasonicManager] Initialized - {CountActiveSensors()}/8 sensors active");
    }

    void ValidateSensors()
    {
        if (sensorFL == null)
        {
            var obj = GameObject.Find("ultrasonic_fl_link");
            if (obj != null) sensorFL = obj.GetComponent<SingleUltrasonicSensor>();
        }
        if (sensorFR == null)
        {
            var obj = GameObject.Find("ultrasonic_fr_link");
            if (obj != null) sensorFR = obj.GetComponent<SingleUltrasonicSensor>();
        }
        if (sensorFC == null)
        {
            var obj = GameObject.Find("ultrasonic_fc_link");
            if (obj != null) sensorFC = obj.GetComponent<SingleUltrasonicSensor>();
        }
        if (sensorRL == null)
        {
            var obj = GameObject.Find("ultrasonic_rl_link");
            if (obj != null) sensorRL = obj.GetComponent<SingleUltrasonicSensor>();
        }
        if (sensorRR == null)
        {
            var obj = GameObject.Find("ultrasonic_rr_link");
            if (obj != null) sensorRR = obj.GetComponent<SingleUltrasonicSensor>();
        }
        if (sensorRC == null)
        {
            var obj = GameObject.Find("ultrasonic_rc_link");
            if (obj != null) sensorRC = obj.GetComponent<SingleUltrasonicSensor>();
        }
        if (sensorSL == null)
        {
            var obj = GameObject.Find("ultrasonic_sl_link");
            if (obj != null) sensorSL = obj.GetComponent<SingleUltrasonicSensor>();
        }
        if (sensorSR == null)
        {
            var obj = GameObject.Find("ultrasonic_sr_link");
            if (obj != null) sensorSR = obj.GetComponent<SingleUltrasonicSensor>();
        }

        allSensors = new[]
        {
            sensorFL, sensorFR, sensorFC, sensorRL,
            sensorRR, sensorRC, sensorSL, sensorSR
        };

        if (sensorFL == null) Debug.LogWarning("[UltrasonicManager] sensorFL이 할당되지 않았습니다. ('ultrasonic_fl_link' 오브젝트를 찾을 수 없음)");
        if (sensorFR == null) Debug.LogWarning("[UltrasonicManager] sensorFR이 할당되지 않았습니다. ('ultrasonic_fr_link' 오브젝트를 찾을 수 없음)");
        if (sensorFC == null) Debug.LogWarning("[UltrasonicManager] sensorFC가 할당되지 않았습니다. ('ultrasonic_fc_link' 오브젝트를 찾을 수 없음)");
        if (sensorRL == null) Debug.LogWarning("[UltrasonicManager] sensorRL이 할당되지 않았습니다. ('ultrasonic_rl_link' 오브젝트를 찾을 수 없음)");
        if (sensorRR == null) Debug.LogWarning("[UltrasonicManager] sensorRR이 할당되지 않았습니다. ('ultrasonic_rr_link' 오브젝트를 찾을 수 없음)");
        if (sensorRC == null) Debug.LogWarning("[UltrasonicManager] sensorRC가 할당되지 않았습니다. ('ultrasonic_rc_link' 오브젝트를 찾을 수 없음)");
        if (sensorSL == null) Debug.LogWarning("[UltrasonicManager] sensorSL이 할당되지 않았습니다. ('ultrasonic_sl_link' 오브젝트를 찾을 수 없음)");
        if (sensorSR == null) Debug.LogWarning("[UltrasonicManager] sensorSR이 할당되지 않았습니다. ('ultrasonic_sr_link' 오브젝트를 찾을 수 없음)");
    }

    void SetupAggregatedTopicOutput()
    {
        resolvedOutputTopicName = RosTopicNamespace.Resolve(gameObject, topicName);
        ros.RegisterPublisher<Float32MultiArrayMsg>(resolvedOutputTopicName);
        aggregatedPublisherReady = true;
    }

    void SetupExternalTopicInput()
    {
        string requestedInputTopic = string.IsNullOrWhiteSpace(inputTopicName) ? topicName : inputTopicName;
        if (string.IsNullOrWhiteSpace(requestedInputTopic))
        {
            Debug.LogWarning("[UltrasonicManager] inputTopicName이 비어 있어 외부 입력 모드를 비활성화합니다.");
            useExternalTopicInput = false;
            return;
        }

        resolvedInputTopicName = RosTopicNamespace.Resolve(gameObject, requestedInputTopic);
        if (aggregatedPublisherReady && resolvedInputTopicName == resolvedOutputTopicName)
        {
            Debug.LogWarning($"[UltrasonicManager] input/output topic이 동일({resolvedInputTopicName})하여 self-loop 방지를 위해 집계 발행을 비활성화합니다.");
            aggregatedPublisherReady = false;
        }

        ros.Subscribe<Float32MultiArrayMsg>(resolvedInputTopicName, OnExternalData);
        Debug.Log($"[UltrasonicManager] External topic subscribed: {resolvedInputTopicName}");
    }

    void SyncScanIntervals()
    {
        float interval = 1f / Mathf.Max(1f, publishRate);
        foreach (var sensor in allSensors)
        {
            if (sensor != null)
                sensor.SetScanInterval(interval);
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
        ApplyExternalInputState();

        if (Time.time - lastPublishTime >= publishInterval)
        {
            UpdateClosestSensor();
            if (publishAggregatedTopic && aggregatedPublisherReady)
                PublishData();
            lastPublishTime = Time.time;

            if (showDebugInfo)
                PrintDebugInfo();
        }
    }

    void ApplyExternalInputState()
    {
        if (useExternalTopicInput)
        {
            bool externalFresh = hasExternalMessage &&
                                 (Time.time - lastExternalMessageTime) <= Mathf.Max(0.02f, externalDataTimeoutSec);
            foreach (var sensor in allSensors)
            {
                if (sensor != null)
                    sensor.SetExternalInputState(externalFresh, fallbackToRaycastWhenExternalStale);
            }
        }
        else
        {
            foreach (var sensor in allSensors)
            {
                if (sensor != null)
                    sensor.SetExternalInputState(false, true);
            }
        }
    }

    void UpdateClosestSensor()
    {
        ClosestDistance = float.PositiveInfinity;
        ClosestConfidence = 0f;
        ClosestSensorPosition = SingleUltrasonicSensor.SensorPosition.FrontCenter;

        foreach (var sensor in allSensors)
        {
            if (sensor == null)
                continue;

            bool isCloser = sensor.Distance < ClosestDistance;
            bool isSameDistanceButMoreReliable =
                Mathf.Abs(sensor.Distance - ClosestDistance) < 0.02f && sensor.Confidence > ClosestConfidence;

            if (isCloser || isSameDistanceButMoreReliable)
            {
                ClosestDistance = sensor.Distance;
                ClosestSensorPosition = sensor.sensorPosition;
                ClosestConfidence = sensor.Confidence;
            }
        }
    }

    void PublishData()
    {
        if (ros == null || string.IsNullOrWhiteSpace(resolvedOutputTopicName))
            return;

        float[] payload = new float[SensorCount * FieldsPerSensor];
        for (int i = 0; i < allSensors.Length; i++)
        {
            int baseIndex = i * FieldsPerSensor;
            SingleUltrasonicSensor sensor = allSensors[i];

            if (sensor == null || !sensor.HasDetection)
            {
                payload[baseIndex + DistanceIndex] = -1f;
                payload[baseIndex + ConfidenceIndex] = 0f;
                payload[baseIndex + AngleIndex] = 0f;
                continue;
            }

            payload[baseIndex + DistanceIndex] = sensor.Distance;
            payload[baseIndex + ConfidenceIndex] = sensor.Confidence;
            payload[baseIndex + AngleIndex] = sensor.DetectedAngle;
        }

        Float32MultiArrayMsg msg = new Float32MultiArrayMsg
        {
            layout = new MultiArrayLayoutMsg
            {
                dim = new MultiArrayDimensionMsg[]
                {
                    new MultiArrayDimensionMsg
                    {
                        label = "sensor",
                        size = SensorCount,
                        stride = SensorCount * FieldsPerSensor
                    },
                    new MultiArrayDimensionMsg
                    {
                        label = "distance_confidence_angle",
                        size = FieldsPerSensor,
                        stride = FieldsPerSensor
                    }
                },
                data_offset = 0
            },
            data = payload
        };

        ros.Publish(resolvedOutputTopicName, msg);
    }

    void OnExternalData(Float32MultiArrayMsg msg)
    {
        hasExternalMessage = true;
        lastExternalMessageTime = Time.time;

        float[] data = msg != null ? msg.data : null;
        if (data == null || data.Length == 0)
        {
            ResetAllSensors();
            return;
        }

        bool hasTriplets = data.Length >= SensorCount * FieldsPerSensor;
        for (int i = 0; i < allSensors.Length; i++)
        {
            SingleUltrasonicSensor sensor = allSensors[i];
            if (sensor == null)
                continue;

            if (hasTriplets)
            {
                int baseIndex = i * FieldsPerSensor;
                sensor.ApplyExternalData(new[]
                {
                    data[baseIndex + DistanceIndex],
                    data[baseIndex + ConfidenceIndex],
                    data[baseIndex + AngleIndex]
                });
                continue;
            }

            if (i < data.Length)
            {
                sensor.ApplyExternalData(new[]
                {
                    data[i],
                    sensor.defaultExternalConfidence > 0f ? sensor.defaultExternalConfidence : defaultExternalConfidence,
                    0f
                });
            }
            else
            {
                sensor.ApplyExternalData(new[] { -1f });
            }
        }
    }

    void ResetAllSensors()
    {
        foreach (var sensor in allSensors)
        {
            if (sensor != null)
                sensor.ApplyExternalData(new[] { -1f });
        }
    }

    void PrintDebugInfo()
    {
        string fl = float.IsInfinity(FrontLeftDistance) ? "∞" : $"{FrontLeftDistance:F2}";
        string fr = float.IsInfinity(FrontRightDistance) ? "∞" : $"{FrontRightDistance:F2}";
        string fc = float.IsInfinity(FrontCenterDistance) ? "∞" : $"{FrontCenterDistance:F2}";
        string rl = float.IsInfinity(RearLeftDistance) ? "∞" : $"{RearLeftDistance:F2}";
        string rr = float.IsInfinity(RearRightDistance) ? "∞" : $"{RearRightDistance:F2}";
        string rc = float.IsInfinity(RearCenterDistance) ? "∞" : $"{RearCenterDistance:F2}";
        string sl = float.IsInfinity(SideLeftDistance) ? "∞" : $"{SideLeftDistance:F2}";
        string sr = float.IsInfinity(SideRightDistance) ? "∞" : $"{SideRightDistance:F2}";

        Debug.Log($"[Ultrasonic] FL:{fl} FR:{fr} FC:{fc} RL:{rl} RR:{rr} RC:{rc} SL:{sl} SR:{sr} | Closest: {ClosestSensorPosition} (conf:{ClosestConfidence:F2})");
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
        float leftMin = Mathf.Min(FrontLeftDistance, Mathf.Min(RearLeftDistance, SideLeftDistance));
        return leftMin > threshold || float.IsInfinity(leftMin);
    }

    public bool IsRightClear(float threshold = 0.5f)
    {
        float rightMin = Mathf.Min(FrontRightDistance, Mathf.Min(RearRightDistance, SideRightDistance));
        return rightMin > threshold || float.IsInfinity(rightMin);
    }

    public bool IsSideClear(float threshold = 0.5f)
    {
        return MinSideDistance > threshold || float.IsInfinity(MinSideDistance);
    }
}
