using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// 4개의 초음파 센서(전방/후방, 좌/우) 데이터를 수집하여 ROS 메시지로 발행하는 클래스입니다.
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
    [Tooltip("후방 좌측 초음파 센서")]
    public SingleUltrasonicSensor sensorRL;
    [Tooltip("후방 우측 초음파 센서")]
    public SingleUltrasonicSensor sensorRR;

    [Header("Debug (디버그)")]
    public bool showDebugInfo = false; // 거리 정보 로그 출력 여부

    // 각 센서의 거리 값 접근 프로퍼티 (센서가 없으면 무한대 반환)
    public float FrontLeftDistance => sensorFL != null ? sensorFL.Distance : float.PositiveInfinity;
    public float FrontRightDistance => sensorFR != null ? sensorFR.Distance : float.PositiveInfinity;
    public float RearLeftDistance => sensorRL != null ? sensorRL.Distance : float.PositiveInfinity;
    public float RearRightDistance => sensorRR != null ? sensorRR.Distance : float.PositiveInfinity;

    // 방향별 최소 거리 계산
    public float MinFrontDistance => Mathf.Min(FrontLeftDistance, FrontRightDistance);
    public float MinRearDistance => Mathf.Min(RearLeftDistance, RearRightDistance);
    // 전체 센서 중 가장 가까운 거리
    public float MinDistance => Mathf.Min(MinFrontDistance, MinRearDistance);

    // 가장 가까운 장애물이 감지된 센서의 위치와 거리
    public SingleUltrasonicSensor.SensorPosition ClosestSensorPosition { get; private set; }
    public float ClosestDistance { get; private set; } = float.PositiveInfinity;

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

        Debug.Log($"[UltrasonicManager] Initialized - {CountActiveSensors()}/4 sensors active");
    }

    /// <summary>
    /// 할당된 센서들을 배열로 묶고 누락된 참조를 경고합니다.
    /// </summary>
    void ValidateSensors()
    {
        allSensors = new SingleUltrasonicSensor[] { sensorFL, sensorFR, sensorRL, sensorRR };

        if (sensorFL == null) Debug.LogWarning("[UltrasonicManager] sensorFL이 할당되지 않았습니다.");
        if (sensorFR == null) Debug.LogWarning("[UltrasonicManager] sensorFR이 할당되지 않았습니다.");
        if (sensorRL == null) Debug.LogWarning("[UltrasonicManager] sensorRL이 할당되지 않았습니다.");
        if (sensorRR == null) Debug.LogWarning("[UltrasonicManager] sensorRR이 할당되지 않았습니다.");
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

    /// <summary>
    /// 수집된 초음파 데이터를 ROS 메시지(Float32MultiArray)로 변환하여 발행합니다.
    /// 데이터 순서: FL, FR, RL, RR, MinFront, MinRear, ClosestDist, ClosestPos
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
                        size = 8,
                        stride = 8
                    }
                },
                data_offset = 0
            },
            data = new float[]
            {
                // 무한대(감지 안됨)일 경우 -1로 변환하여 전송 (ROS 표준에 맞춤)
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
