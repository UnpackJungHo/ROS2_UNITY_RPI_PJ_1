using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// 6개의 초음파 센서(전/후 코너 + 전/후 중앙) 데이터를 수집하여 ROS 메시지로 발행하는 클래스입니다.
/// 각 개별 센서(SingleUltrasonicSensor)의 거리 데이터를 종합 관리하고,
/// ROS 2 시스템으로 Float32MultiArray 메시지를 전송합니다.
/// </summary>
public class UltrasonicSensorPublisher : MonoBehaviour
{
    [Header("ROS Settings (ROS 설정)")]
    public string topicName = "/ultrasonic"; // 발행할 토픽 이름
    public float publishRate = 20f; // 초당 발행 횟수 (Hz)

    [Header("Sensor References (각 센서 오브젝트에서 수동 할당)")]
    // 각 위치별 개별 초음파 센서 참조
    [Tooltip("전방 좌측 초음파 센서")]
    public SingleUltrasonicSensor sensorFL;
    [Tooltip("전방 우측 초음파 센서")]
    public SingleUltrasonicSensor sensorFR;
    [Tooltip("전방 중앙 초음파 센서")]
    public SingleUltrasonicSensor sensorFC;
    [Tooltip("후방 좌측 초음파 센서")]
    public SingleUltrasonicSensor sensorRL;
    [Tooltip("후방 우측 초음파 센서")]
    public SingleUltrasonicSensor sensorRR;
    [Tooltip("후방 중앙 초음파 센서")]
    public SingleUltrasonicSensor sensorRC;

    [Header("Debug (디버그)")]
    public bool showDebugInfo = false; // 거리 정보 로그 출력 여부

    // 각 센서의 거리 값 접근 프로퍼티 (센서가 없으면 무한대 반환)
    public float FrontLeftDistance => sensorFL != null ? sensorFL.Distance : float.PositiveInfinity;
    public float FrontRightDistance => sensorFR != null ? sensorFR.Distance : float.PositiveInfinity;
    public float FrontCenterDistance => sensorFC != null ? sensorFC.Distance : float.PositiveInfinity;
    public float RearLeftDistance => sensorRL != null ? sensorRL.Distance : float.PositiveInfinity;
    public float RearRightDistance => sensorRR != null ? sensorRR.Distance : float.PositiveInfinity;
    public float RearCenterDistance => sensorRC != null ? sensorRC.Distance : float.PositiveInfinity;

    // 방향별 최소 거리 계산
    public float MinFrontDistance => Mathf.Min(FrontLeftDistance, FrontRightDistance, FrontCenterDistance);
    public float MinRearDistance => Mathf.Min(RearLeftDistance, RearRightDistance, RearCenterDistance);
    // 전체 센서 중 가장 가까운 거리
    public float MinDistance => Mathf.Min(MinFrontDistance, MinRearDistance);

    // 가장 가까운 장애물이 감지된 센서의 위치와 거리
    public SingleUltrasonicSensor.SensorPosition ClosestSensorPosition { get; private set; }
    public float ClosestDistance { get; private set; } = float.PositiveInfinity;
    public float ClosestConfidence { get; private set; } = 0f;

    // 내부 변수
    private ROSConnection ros;
    private float publishInterval;
    private float lastPublishTime;
    private SingleUltrasonicSensor[] allSensors; // 전체 센서 배열

    void Start()
    {
        // ROS 연결 인스턴스 가져오기 및 퍼블리셔 등록
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float32MultiArrayMsg>(topicName);

        // 센서 배열 초기화 및 검증
        ValidateSensors();
        // 모든 센서의 스캔 주기를 발행 주기에 맞춤
        SyncScanIntervals();

        publishInterval = 1f / publishRate;
        lastPublishTime = Time.time;

        Debug.Log($"[UltrasonicManager] Initialized - {CountActiveSensors()}/6 sensors active");
    }

    /// <summary>
    /// 할당된 센서들을 배열로 묶고 누락된 참조를 경고합니다.
    /// </summary>
    void ValidateSensors()
    {
        // 자동 할당 로직 추가
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

        allSensors = new SingleUltrasonicSensor[] { sensorFL, sensorFR, sensorFC, sensorRL, sensorRR, sensorRC };

        if (sensorFL == null) Debug.LogWarning("[UltrasonicManager] sensorFL이 할당되지 않았습니다. ('ultrasonic_fl_link' 오브젝트를 찾을 수 없음)");
        if (sensorFR == null) Debug.LogWarning("[UltrasonicManager] sensorFR이 할당되지 않았습니다. ('ultrasonic_fr_link' 오브젝트를 찾을 수 없음)");
        if (sensorFC == null) Debug.LogWarning("[UltrasonicManager] sensorFC가 할당되지 않았습니다. ('ultrasonic_fc_link' 오브젝트를 찾을 수 없음)");
        if (sensorRL == null) Debug.LogWarning("[UltrasonicManager] sensorRL이 할당되지 않았습니다. ('ultrasonic_rl_link' 오브젝트를 찾을 수 없음)");
        if (sensorRR == null) Debug.LogWarning("[UltrasonicManager] sensorRR이 할당되지 않았습니다. ('ultrasonic_rr_link' 오브젝트를 찾을 수 없음)");
        if (sensorRC == null) Debug.LogWarning("[UltrasonicManager] sensorRC가 할당되지 않았습니다. ('ultrasonic_rc_link' 오브젝트를 찾을 수 없음)");
    }

    /// <summary>
    /// 모든 개별 센서의 스캔 주기를 퍼블리셔의 발행 주기에 맞춰 동기화합니다.
    /// </summary>
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
        // 일정 주기마다 데이터 갱신 및 발행
        if (Time.time - lastPublishTime >= publishInterval)
        {
            UpdateClosestSensor(); // 가장 가까운 센서 정보 갱신
            PublishData();         // ROS 메시지 발행
            lastPublishTime = Time.time;

            if (showDebugInfo)
            {
                PrintDebugInfo();
            }
        }
    }

    /// <summary>
    /// 모든 센서를 순회하며 가장 가까운 장애물을 감지한 센서를 찾습니다.
    /// </summary>
    void UpdateClosestSensor()
    {
        ClosestDistance = float.PositiveInfinity;
        ClosestConfidence = 0f;
        ClosestSensorPosition = SingleUltrasonicSensor.SensorPosition.FrontCenter;

        if (sensorFL != null && (sensorFL.Distance < ClosestDistance ||
            (Mathf.Abs(sensorFL.Distance - ClosestDistance) < 0.02f && sensorFL.Confidence > ClosestConfidence)))
        {
            ClosestDistance = sensorFL.Distance;
            ClosestSensorPosition = SingleUltrasonicSensor.SensorPosition.FrontLeft;
            ClosestConfidence = sensorFL.Confidence;
        }
        if (sensorFR != null && (sensorFR.Distance < ClosestDistance ||
            (Mathf.Abs(sensorFR.Distance - ClosestDistance) < 0.02f && sensorFR.Confidence > ClosestConfidence)))
        {
            ClosestDistance = sensorFR.Distance;
            ClosestSensorPosition = SingleUltrasonicSensor.SensorPosition.FrontRight;
            ClosestConfidence = sensorFR.Confidence;
        }
        if (sensorFC != null && (sensorFC.Distance < ClosestDistance ||
            (Mathf.Abs(sensorFC.Distance - ClosestDistance) < 0.02f && sensorFC.Confidence > ClosestConfidence)))
        {
            ClosestDistance = sensorFC.Distance;
            ClosestSensorPosition = SingleUltrasonicSensor.SensorPosition.FrontCenter;
            ClosestConfidence = sensorFC.Confidence;
        }
        if (sensorRL != null && (sensorRL.Distance < ClosestDistance ||
            (Mathf.Abs(sensorRL.Distance - ClosestDistance) < 0.02f && sensorRL.Confidence > ClosestConfidence)))
        {
            ClosestDistance = sensorRL.Distance;
            ClosestSensorPosition = SingleUltrasonicSensor.SensorPosition.RearLeft;
            ClosestConfidence = sensorRL.Confidence;
        }
        if (sensorRR != null && (sensorRR.Distance < ClosestDistance ||
            (Mathf.Abs(sensorRR.Distance - ClosestDistance) < 0.02f && sensorRR.Confidence > ClosestConfidence)))
        {
            ClosestDistance = sensorRR.Distance;
            ClosestSensorPosition = SingleUltrasonicSensor.SensorPosition.RearRight;
            ClosestConfidence = sensorRR.Confidence;
        }
        if (sensorRC != null && (sensorRC.Distance < ClosestDistance ||
            (Mathf.Abs(sensorRC.Distance - ClosestDistance) < 0.02f && sensorRC.Confidence > ClosestConfidence)))
        {
            ClosestDistance = sensorRC.Distance;
            ClosestSensorPosition = SingleUltrasonicSensor.SensorPosition.RearCenter;
            ClosestConfidence = sensorRC.Confidence;
        }
    }

    /// <summary>
    /// 수집된 초음파 데이터를 ROS 메시지(Float32MultiArray)로 변환하여 발행합니다.
    /// 데이터 순서: FL, FR, FC, RL, RR, RC, MinFront, MinRear, ClosestDist, ClosestPos, ClosestConf, MinDist
    /// </summary>
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
                        size = 12,
                        stride = 12
                    }
                },
                data_offset = 0
            },
            data = new float[]
            {
                // 무한대(감지 안됨)일 경우 -1로 변환하여 전송 (ROS 표준에 맞춤)
                float.IsInfinity(FrontLeftDistance) ? -1f : FrontLeftDistance,
                float.IsInfinity(FrontRightDistance) ? -1f : FrontRightDistance,
                float.IsInfinity(FrontCenterDistance) ? -1f : FrontCenterDistance,
                float.IsInfinity(RearLeftDistance) ? -1f : RearLeftDistance,
                float.IsInfinity(RearRightDistance) ? -1f : RearRightDistance,
                float.IsInfinity(RearCenterDistance) ? -1f : RearCenterDistance,
                float.IsInfinity(MinFrontDistance) ? -1f : MinFrontDistance,
                float.IsInfinity(MinRearDistance) ? -1f : MinRearDistance,
                float.IsInfinity(ClosestDistance) ? -1f : ClosestDistance,
                (float)ClosestSensorPosition,
                ClosestConfidence,
                float.IsInfinity(MinDistance) ? -1f : MinDistance
            }
        };

        ros.Publish(topicName, msg);
    }

    void PrintDebugInfo()
    {
        string fl = float.IsInfinity(FrontLeftDistance) ? "∞" : $"{FrontLeftDistance:F2}";
        string fr = float.IsInfinity(FrontRightDistance) ? "∞" : $"{FrontRightDistance:F2}";
        string fc = float.IsInfinity(FrontCenterDistance) ? "∞" : $"{FrontCenterDistance:F2}";
        string rl = float.IsInfinity(RearLeftDistance) ? "∞" : $"{RearLeftDistance:F2}";
        string rr = float.IsInfinity(RearRightDistance) ? "∞" : $"{RearRightDistance:F2}";
        string rc = float.IsInfinity(RearCenterDistance) ? "∞" : $"{RearCenterDistance:F2}";

        Debug.Log($"[Ultrasonic] FL:{fl} FR:{fr} FC:{fc} RL:{rl} RR:{rr} RC:{rc} | Closest: {ClosestSensorPosition} (conf:{ClosestConfidence:F2})");
    }

    // 전방 안전 여부 확인 (임계값 이상이면 안전)
    public bool IsFrontClear(float threshold = 0.5f)
    {
        return MinFrontDistance > threshold || float.IsInfinity(MinFrontDistance);
    }

    // 후방 안전 여부 확인
    public bool IsRearClear(float threshold = 0.5f)
    {
        return MinRearDistance > threshold || float.IsInfinity(MinRearDistance);
    }

    // 좌측(전/후) 안전 여부 확인
    public bool IsLeftClear(float threshold = 0.5f)
    {
        float leftMin = Mathf.Min(FrontLeftDistance, RearLeftDistance);
        return leftMin > threshold || float.IsInfinity(leftMin);
    }

    // 우측(전/후) 안전 여부 확인
    public bool IsRightClear(float threshold = 0.5f)
    {
        float rightMin = Mathf.Min(FrontRightDistance, RearRightDistance);
        return rightMin > threshold || float.IsInfinity(rightMin);
    }
}
