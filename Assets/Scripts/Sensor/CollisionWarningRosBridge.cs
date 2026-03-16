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
                        size = 17,
                        stride = 17
                    }
                },
                data_offset = 0
            },
            data = new float[]
            {
                float.IsInfinity(engine.currentMinDistance) ? -1f : engine.currentMinDistance,
                float.IsInfinity(engine.currentTTC) ? -1f : engine.currentTTC,
                (float)engine.currentWarningLevel,
                engine.GetEgoSpeed(),
                engine.GetClosingSpeed(),
                float.IsInfinity(sensorData.ultrasonicFL) ? -1f : sensorData.ultrasonicFL,
                float.IsInfinity(sensorData.ultrasonicFR) ? -1f : sensorData.ultrasonicFR,
                float.IsInfinity(sensorData.ultrasonicFC) ? -1f : sensorData.ultrasonicFC,
                float.IsInfinity(sensorData.ultrasonicRL) ? -1f : sensorData.ultrasonicRL,
                float.IsInfinity(sensorData.ultrasonicRR) ? -1f : sensorData.ultrasonicRR,
                float.IsInfinity(sensorData.ultrasonicRC) ? -1f : sensorData.ultrasonicRC,
                float.IsInfinity(sensorData.radarFront) ? -1f : sensorData.radarFront,
                float.IsInfinity(sensorData.radarRear) ? -1f : sensorData.radarRear,
                engine.detectionSource == "Ultrasonic" ? 1f : (engine.detectionSource == "Radar" ? 2f : 0f),
                (float)sensorData.ultrasonicClosest,
                (float)sensorData.radarClosest,
                sensorData.ultrasonicClosestConfidence
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
