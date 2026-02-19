using UnityEngine;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

// 여러 레이더 센서를 관리하고 통합 데이터를 ROS 토픽으로 발행하는 클래스
public class RadarSensorPublisher : MonoBehaviour
{
    // ========== ROS 설정 ==========
    [Header("ROS Settings")]
    // ROS 토픽 이름 - 레이더 데이터가 발행될 토픽 경로
    public string topicName = "/radar";
    // 멀티 타깃 상세 데이터 토픽
    public string targetsTopicName = "/radar/targets";
    // 초당 발행 횟수 (Hz) - 20이면 초당 20번 데이터 발행
    public float publishRate = 20f;
    // 발행할 최대 타깃 수 (전방+후방 합산)
    public int maxPublishedTargets = 32;

    // ========== 센서 참조 ==========
    [Header("Sensor References (각 센서 오브젝트에서 할당)")]
    [Tooltip("전방 레이더 센서")]
    // 전방에 장착된 레이더 센서 컴포넌트 참조
    public SingleRadarSensor sensorFront;
    [Tooltip("후방 레이더 센서")]
    // 후방에 장착된 레이더 센서 컴포넌트 참조
    public SingleRadarSensor sensorRear;

    // ========== 디버그 설정 ==========
    [Header("Debug")]
    // 콘솔에 디버그 정보를 출력할지 여부
    public bool showDebugInfo = false;

    // ========== 프로퍼티 (Property): 외부에서 센서 데이터를 쉽게 읽을 수 있게 제공 ==========
    // 전방 레이더의 감지 거리 (미터) - 센서가 없으면 무한대 반환
    public float FrontDistance => sensorFront != null ? sensorFront.Distance : float.PositiveInfinity;
    // 후방 레이더의 감지 거리 (미터) - 센서가 없으면 무한대 반환
    public float RearDistance => sensorRear != null ? sensorRear.Distance : float.PositiveInfinity;
    // 전방 레이더가 감지한 물체의 각도 (도 단위) - 센서가 없으면 0 반환
    public float FrontAngle => sensorFront != null ? sensorFront.DetectedAngle : 0f;
    // 후방 레이더가 감지한 물체의 각도 (도 단위) - 센서가 없으면 0 반환
    public float RearAngle => sensorRear != null ? sensorRear.DetectedAngle : 0f;
    // 전방 물체의 상대 속도 (m/s) - 양수는 접근, 음수는 멀어짐
    public float FrontRelativeVelocity => sensorFront != null ? sensorFront.RelativeVelocity : 0f;
    // 후방 물체의 상대 속도 (m/s) - 양수는 접근, 음수는 멀어짐
    public float RearRelativeVelocity => sensorRear != null ? sensorRear.RelativeVelocity : 0f;

    // 전방과 후방 중 더 가까운 거리 (미터)
    public float MinDistance => Mathf.Min(FrontDistance, RearDistance);

    // 가장 가까운 물체를 감지한 센서의 위치 (Front 또는 Rear)
    public SingleRadarSensor.SensorPosition ClosestSensorPosition { get; private set; }
    // 가장 가까운 물체까지의 거리 (미터)
    public float ClosestDistance { get; private set; } = float.PositiveInfinity;

    // ========== Private 변수들 ==========
    // ROS와의 연결을 관리하는 객체
    private ROSConnection ros;
    // 발행 주기 (초 단위, publishRate의 역수)
    private float publishInterval;
    // 마지막으로 데이터를 발행한 시간
    private float lastPublishTime;
    private const int TARGET_FIELD_COUNT = 8;

    void Start()
    {
        // ROS 연결 인스턴스를 가져오거나 없으면 새로 생성 (싱글톤 패턴)
        ros = ROSConnection.GetOrCreateInstance();
        topicName = RosTopicNamespace.Resolve(gameObject, topicName);
        targetsTopicName = RosTopicNamespace.Resolve(gameObject, targetsTopicName);
        // 이 스크립트를 Float32MultiArrayMsg 타입의 퍼블리셔로 등록
        ros.RegisterPublisher<Float32MultiArrayMsg>(topicName);
        ros.RegisterPublisher<Float32MultiArrayMsg>(targetsTopicName);

        // 센서가 제대로 할당되었는지 검증
        ValidateSensors();
        // 모든 센서의 스캔 간격을 발행 주기와 동기화
        SyncScanIntervals();

        // 발행 간격 계산 (초 단위, 예: publishRate=20이면 0.05초마다 발행)
        publishInterval = 1f / publishRate;
        // 마지막 발행 시간을 현재 시간으로 설정
        lastPublishTime = Time.time;

        // 초기화 완료 로그 출력 (활성 센서 개수 정보 포함)
        // Debug.Log($"[RadarManager] Initialized - {CountActiveSensors()}/2 sensors active");
    }

    // 센서 참조가 올바르게 할당되었는지 검증하는 함수
    void ValidateSensors()
    {
        // 자동 할당 로직 추가
        if (sensorFront == null)
        {
            var obj = GameObject.Find("radar_front_link");
            if (obj != null) sensorFront = obj.GetComponent<SingleRadarSensor>();
        }
        if (sensorRear == null)
        {
            var obj = GameObject.Find("radar_rear_link");
            if (obj != null) sensorRear = obj.GetComponent<SingleRadarSensor>();
        }

        if (sensorFront == null) Debug.LogWarning("[RadarManager] sensorFront가 할당되지 않았습니다. ('radar_front_link' 오브젝트를 찾을 수 없음)");
        if (sensorRear == null) Debug.LogWarning("[RadarManager] sensorRear가 할당되지 않았습니다. ('radar_rear_link' 오브젝트를 찾을 수 없음)");
    }

    // 모든 센서의 스캔 간격을 발행 주기와 동기화하는 함수
    void SyncScanIntervals()
    {
        // 발행 주기를 초 단위 간격으로 변환
        float interval = 1f / publishRate;
        // 전방 센서가 있으면 스캔 간격 설정
        if (sensorFront != null) sensorFront.SetScanInterval(interval);
        // 후방 센서가 있으면 스캔 간격 설정
        if (sensorRear != null) sensorRear.SetScanInterval(interval);
    }

    // 현재 활성화된 센서의 개수를 세는 함수
    int CountActiveSensors()
    {
        // 카운터 초기화
        int count = 0;
        // 전방 센서가 있으면 카운트 증가
        if (sensorFront != null) count++;
        // 후방 센서가 있으면 카운트 증가
        if (sensorRear != null) count++;
        // 총 활성 센서 개수 반환
        return count;
    }

    void Update()
    {
        // 마지막 발행 이후 충분한 시간이 지났는지 확인 (publishInterval 이상)
        if (Time.time - lastPublishTime >= publishInterval)
        {
            // 가장 가까운 센서 정보 업데이트
            UpdateClosestSensor();
            // 레이더 데이터를 ROS 토픽으로 발행
            PublishData();
            PublishTargetsData();
            // 마지막 발행 시간을 현재 시간으로 업데이트
            lastPublishTime = Time.time;

            // 디버그 정보 표시가 활성화되어 있으면
            if (showDebugInfo)
            {
                // 콘솔에 디버그 정보 출력
                PrintDebugInfo();
            }
        }
    }

    // 전방과 후방 센서 중 가장 가까운 센서를 찾아 업데이트하는 함수
    void UpdateClosestSensor()
    {
        // 최소 거리를 무한대로 초기화
        ClosestDistance = float.PositiveInfinity;
        // 기본값으로 전방 센서 설정
        ClosestSensorPosition = SingleRadarSensor.SensorPosition.Front;

        // 전방 센서가 있고, 그 거리가 현재 최소 거리보다 가까우면
        if (sensorFront != null && sensorFront.Distance < ClosestDistance)
        {
            // 최소 거리를 전방 센서의 거리로 업데이트
            ClosestDistance = sensorFront.Distance;
            // 가장 가까운 센서 위치를 전방으로 설정
            ClosestSensorPosition = SingleRadarSensor.SensorPosition.Front;
        }
        // 후방 센서가 있고, 그 거리가 현재 최소 거리보다 가까우면
        if (sensorRear != null && sensorRear.Distance < ClosestDistance)
        {
            // 최소 거리를 후방 센서의 거리로 업데이트
            ClosestDistance = sensorRear.Distance;
            // 가장 가까운 센서 위치를 후방으로 설정
            ClosestSensorPosition = SingleRadarSensor.SensorPosition.Rear;
        }
    }

    // 모든 레이더 센서 데이터를 ROS 토픽으로 발행하는 함수
    void PublishData()
    {
        // ROS Float32MultiArray 메시지 객체 생성 및 초기화
        Float32MultiArrayMsg msg = new Float32MultiArrayMsg
        {
            // 메시지 레이아웃 정보 (배열 구조 설명)
            layout = new MultiArrayLayoutMsg
            {
                // 배열 차원 정보
                dim = new MultiArrayDimensionMsg[]
                {
                    new MultiArrayDimensionMsg
                    {
                        // 데이터 레이블 (이 배열이 레이더 데이터임을 표시)
                        label = "radar_data",
                        // 배열 크기 (8개의 float 값)
                        size = 8,
                        // 메모리 stride (연속된 8개의 요소)
                        stride = 8
                    }
                },
                // 데이터 시작 오프셋 (0부터 시작)
                data_offset = 0
            },
            // 실제 데이터 배열 (8개의 float 값)
            data = new float[]
            {
                // [0] 전방 거리 (무한대면 -1로 변환, 아니면 실제 거리)
                float.IsInfinity(FrontDistance) ? -1f : FrontDistance,
                // [1] 전방 물체의 각도 (도 단위)
                FrontAngle,
                // [2] 전방 물체의 상대 속도 (m/s)
                FrontRelativeVelocity,
                // [3] 후방 거리 (무한대면 -1로 변환, 아니면 실제 거리)
                float.IsInfinity(RearDistance) ? -1f : RearDistance,
                // [4] 후방 물체의 각도 (도 단위)
                RearAngle,
                // [5] 후방 물체의 상대 속도 (m/s)
                RearRelativeVelocity,
                // [6] 가장 가까운 물체까지의 거리 (무한대면 -1로 변환)
                float.IsInfinity(ClosestDistance) ? -1f : ClosestDistance,
                // [7] 가장 가까운 센서의 위치 (Front=0, Rear=1 등)
                (float)ClosestSensorPosition
            }
        };

        // ROS 토픽으로 메시지 발행
        ros.Publish(topicName, msg);
    }

    // 멀티 타깃 상세 데이터 발행
    // 필드: sensor_id, distance, angle, radial_velocity, rcs, confidence, is_ghost, is_clutter
    void PublishTargetsData()
    {
        List<float> packed = new List<float>(maxPublishedTargets * TARGET_FIELD_COUNT);

        AppendSensorTargets(sensorFront, 0f, packed);
        AppendSensorTargets(sensorRear, 1f, packed);

        Float32MultiArrayMsg msg = new Float32MultiArrayMsg
        {
            layout = new MultiArrayLayoutMsg
            {
                dim = new MultiArrayDimensionMsg[0],
                data_offset = 0
            },
            data = packed.ToArray()
        };

        ros.Publish(targetsTopicName, msg);
    }

    void AppendSensorTargets(SingleRadarSensor sensor, float sensorId, List<float> packed)
    {
        if (sensor == null || packed.Count / TARGET_FIELD_COUNT >= maxPublishedTargets) return;

        SingleRadarSensor.RadarDetection[] detections = sensor.GetDetectionsSnapshot();
        for (int i = 0; i < detections.Length; i++)
        {
            if (packed.Count / TARGET_FIELD_COUNT >= maxPublishedTargets) break;

            var d = detections[i];
            packed.Add(sensorId);                      // 0: sensor_id (front=0, rear=1)
            packed.Add(d.distance);                    // 1: distance (m)
            packed.Add(d.angle);                       // 2: angle (deg)
            packed.Add(d.radialVelocity);              // 3: radial velocity (m/s)
            packed.Add(d.rcs);                         // 4: approx rcs
            packed.Add(d.confidence);                  // 5: confidence
            packed.Add(d.isGhost ? 1f : 0f);          // 6: ghost flag
            packed.Add(d.isClutter ? 1f : 0f);        // 7: clutter flag
        }
    }

    // 레이더 센서 데이터를 콘솔에 출력하는 디버그 함수
    void PrintDebugInfo()
    {
        // 전방 센서 정보 문자열 생성 (무한대면 "∞", 아니면 "거리 @각도" 형식)
        string front = float.IsInfinity(FrontDistance) ? "∞" : $"{FrontDistance:F2}m @{FrontAngle:F1}°";
        // 후방 센서 정보 문자열 생성 (무한대면 "∞", 아니면 "거리 @각도" 형식)
        string rear = float.IsInfinity(RearDistance) ? "∞" : $"{RearDistance:F2}m @{RearAngle:F1}°";

        // 전방, 후방, 가장 가까운 센서 정보를 한 줄로 출력
        Debug.Log($"[Radar] Front:{front} Rear:{rear} | Closest: {ClosestSensorPosition}");
    }

    // 전방이 안전한지 확인하는 유틸리티 함수
    // threshold: 안전 거리 기준값 (미터, 기본값 2.0m)
    // 반환값: 전방 거리가 threshold보다 크거나 물체가 없으면 true
    public bool IsFrontClear(float threshold = 2.0f)
    {
        // 전방 거리가 임계값보다 크거나 무한대(물체 없음)인 경우 안전
        return FrontDistance > threshold || float.IsInfinity(FrontDistance);
    }

    // 후방이 안전한지 확인하는 유틸리티 함수
    // threshold: 안전 거리 기준값 (미터, 기본값 2.0m)
    // 반환값: 후방 거리가 threshold보다 크거나 물체가 없으면 true
    public bool IsRearClear(float threshold = 2.0f)
    {
        // 후방 거리가 임계값보다 크거나 무한대(물체 없음)인 경우 안전
        return RearDistance > threshold || float.IsInfinity(RearDistance);
    }
}
