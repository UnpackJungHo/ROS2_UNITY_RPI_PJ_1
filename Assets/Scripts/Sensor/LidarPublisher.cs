using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

/// <summary>
/// LidarSensor의 ROS 토픽 발행 브리지.
/// PointCloud2Msg를 발행한다.
/// </summary>
public class LidarPublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    public string topicName = "/velodyne_points";
    public string frameId = "lidar_link";
    public float publishRate = 10f;

    [Header("Sensor Reference")]
    [Tooltip("같은 GameObject 또는 부모에서 자동 탐색")]
    public LidarSensor sensor;

    [Header("Performance (Multi-Vehicle)")]
    [Tooltip("다중 차량 시 발행 시점을 분산")]
    public bool enablePublishStagger = true;

    private ROSConnection ros;
    private float publishInterval;
    private float lastPublishTime;

    // 재사용 버퍼 (GC 방지)
    private PointFieldMsg[] cachedFields;

    void Start()
    {
        if (sensor == null)
            sensor = GetComponent<LidarSensor>() ?? GetComponentInParent<LidarSensor>();

        if (sensor == null)
        {
            Debug.LogError("[LidarPublisher] LidarSensor를 찾지 못했습니다.");
            enabled = false;
            return;
        }

        ros = ROSConnection.GetOrCreateInstance();
        topicName = RosTopicNamespace.Resolve(gameObject, topicName);
        ros.RegisterPublisher<PointCloud2Msg>(topicName, queue_size: 1);

        publishInterval = 1f / publishRate;
        lastPublishTime = Time.time;

        if (enablePublishStagger)
        {
            float stagger = Mathf.Abs(gameObject.GetInstanceID() % 97) / 97f * publishInterval;
            lastPublishTime = Time.time + stagger - publishInterval;
        }

        cachedFields = new PointFieldMsg[6];
        cachedFields[0] = new PointFieldMsg("x", 0, PointFieldMsg.FLOAT32, 1);
        cachedFields[1] = new PointFieldMsg("y", 4, PointFieldMsg.FLOAT32, 1);
        cachedFields[2] = new PointFieldMsg("z", 8, PointFieldMsg.FLOAT32, 1);
        cachedFields[3] = new PointFieldMsg("intensity", 12, PointFieldMsg.FLOAT32, 1);
        cachedFields[4] = new PointFieldMsg("ring", 16, PointFieldMsg.UINT16, 1);
        cachedFields[5] = new PointFieldMsg("time", 18, PointFieldMsg.FLOAT32, 1);
    }

    void Update()
    {
        if (Time.time - lastPublishTime >= publishInterval)
        {
            PublishScan();
            lastPublishTime = Time.time;
        }
    }

    void PublishScan()
    {
        if (sensor == null) return;

        int numPoints = sensor.NumPoints;
        byte[] data = sensor.CopyPointData();
        int dataSize = data.Length;

        PointCloud2Msg msg = new PointCloud2Msg
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
            height = 1,
            width = (uint)numPoints,
            fields = cachedFields,
            is_bigendian = false,
            point_step = (uint)LidarSensor.POINT_STEP,
            row_step = (uint)dataSize,
            data = data,
            is_dense = true
        };

        ros.Publish(topicName, msg);
    }
}
