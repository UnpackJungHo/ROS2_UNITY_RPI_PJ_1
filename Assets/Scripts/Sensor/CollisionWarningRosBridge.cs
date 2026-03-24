using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// CollisionWarningEngine의 ROS 토픽 발행 브리지.
/// /collision_warning (상세) + /obstacle_distance (단순) 토픽을 발행한다.
/// </summary>
public class CollisionWarningRosBridge : MonoBehaviour
{
    [Header("Engine Reference")]
    [Tooltip("같은 GameObject 또는 부모에서 자동 탐색")]
    public CollisionWarningEngine engine;

    [Header("ROS Settings (ROS 설정)")]
    public string warningTopicName = "/collision_warning";
    public string distanceTopicName = "/obstacle_distance";
    public float publishRate = 20f;
    [Tooltip("false면 ROS 발행만 비활성 (Engine의 계산은 계속 동작)")]
    public bool rosPublishingEnabled = true;

    private ROSConnection ros;
    private float publishInterval;
    private float lastPublishTime;

    void Start()
    {
        if (engine == null)
            engine = GetComponent<CollisionWarningEngine>() ?? GetComponentInParent<CollisionWarningEngine>();

        if (engine == null)
        {
            Debug.LogError("[CollisionWarningRosBridge] CollisionWarningEngine을 찾지 못했습니다.");
            enabled = false;
            return;
        }

        ros = ROSConnection.GetOrCreateInstance();
        warningTopicName = RosTopicNamespace.Resolve(gameObject, warningTopicName);
        distanceTopicName = RosTopicNamespace.Resolve(gameObject, distanceTopicName);
        ros.RegisterPublisher<Float32MultiArrayMsg>(warningTopicName);
        ros.RegisterPublisher<Float32Msg>(distanceTopicName);

        publishInterval = 1f / publishRate;
        lastPublishTime = Time.time;
    }

    void Update()
    {
        if (!rosPublishingEnabled || engine == null)
            return;

        if (Time.time - lastPublishTime >= publishInterval)
        {
            PublishWarning();
            lastPublishTime = Time.time;
        }
    }

    void PublishWarning()
    {
        var sensorData = engine.CurrentSensorData;

        Float32MultiArrayMsg warningMsg = new Float32MultiArrayMsg
        {
            layout = new MultiArrayLayoutMsg
            {
                dim = new MultiArrayDimensionMsg[]
                {
                    new MultiArrayDimensionMsg
                    {
                        label = "collision_data",
                        size = 21,
                        stride = 21
                    }
                },
                data_offset = 0
            },
            data = new float[]
            {
                // [0-2] 글로벌
                float.IsInfinity(engine.currentMinDistance) ? -1f : engine.currentMinDistance,
                (float)engine.currentWarningLevel,
                engine.GetEgoSpeed(),
                // [3-8] 초음파 개별
                float.IsInfinity(sensorData.ultrasonicFL) ? -1f : sensorData.ultrasonicFL,
                float.IsInfinity(sensorData.ultrasonicFR) ? -1f : sensorData.ultrasonicFR,
                float.IsInfinity(sensorData.ultrasonicFC) ? -1f : sensorData.ultrasonicFC,
                float.IsInfinity(sensorData.ultrasonicRL) ? -1f : sensorData.ultrasonicRL,
                float.IsInfinity(sensorData.ultrasonicRR) ? -1f : sensorData.ultrasonicRR,
                float.IsInfinity(sensorData.ultrasonicRC) ? -1f : sensorData.ultrasonicRC,
                // [9-10] 레이더 (미사용 — 슬롯 유지, 값 -1 고정)
                -1f, // radarFront 비활성
                -1f, // radarRear 비활성
                // [11-14] 기존 감지 소스 정보
                engine.detectionSource == "Ultrasonic" ? 1f : 0f,
                (float)sensorData.ultrasonicClosest,
                0f, // radarClosest 비활성
                sensorData.ultrasonicClosestConfidence,
                // [15-18] 방향별 위험도 레벨
                (float)engine.frontWarning.level,
                (float)engine.rearWarning.level,
                (float)engine.leftWarning.level,
                (float)engine.rightWarning.level,
                // [19-20] 방향별 주요 거리 (front, rear)
                float.IsInfinity(engine.frontWarning.dominantDistance) ? -1f : engine.frontWarning.dominantDistance,
                float.IsInfinity(engine.rearWarning.dominantDistance) ? -1f : engine.rearWarning.dominantDistance
            }
        };

        ros.Publish(warningTopicName, warningMsg);

        Float32Msg distanceMsg = new Float32Msg
        {
            data = float.IsInfinity(engine.currentMinDistance) ? -1f : engine.currentMinDistance
        };
        ros.Publish(distanceTopicName, distanceMsg);
    }
}
