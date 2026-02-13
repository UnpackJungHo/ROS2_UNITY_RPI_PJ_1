using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Tf2;

/// <summary>
/// 정적 TF(Static Transform)를 주기적으로 또는 한 번 발행하는 클래스
/// </summary>
public class StaticTfPublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    public string tfStaticTopic = "/tf"; // ROS 2 QoS issues with /tf_static, sending to /tf (Volatile) is safer for Unity

    [System.Serializable]
    public struct StaticLink
    {
        public string parentFrameId;
        public string childFrameId;
        public Vector3 position; // Unity Coordinate (z-forward, y-up)
        public Vector3 rotation; // Euler Angles (degrees)
    }

    [Header("Transforms")]
    public List<StaticLink> staticLinks = new List<StaticLink>();

    private ROSConnection ros;
    private float lastPublishTime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TFMessageMsg>(tfStaticTopic);

        // 기본값 설정 (Inspector에서 설정 가능하도록)
        if (staticLinks.Count == 0)
        {
            // 예시: URDF 기반 기본값
            staticLinks.Add(new StaticLink 
            { 
                parentFrameId = "base_link", 
                childFrameId = "lidar_link", 
                position = new Vector3(0, 0.745f, 0), 
                rotation = Vector3.zero 
            });

            staticLinks.Add(new StaticLink 
            { 
                parentFrameId = "base_link", 
                childFrameId = "imu_link", 
                position = new Vector3(0, 0.41f, 0), 
                rotation = Vector3.zero 
            });
        }
    }

    void Update()
    {
        // ROS 2에서 static TF는 latch되지만, 늦게 접속한 노드(RViz2 등)가 못 받는 경우가 있음.
        // 따라서 주기적으로(예: 1초마다) 다시 보내주는 것이 안전함.
        if (Time.time - lastPublishTime > 1.0f)
        {
            PublishStaticTF();
            lastPublishTime = Time.time;
        }
    }

    public void PublishStaticTF()
    {
        var tfTime = ConvertToRosTime(Time.time);
        List<TransformStampedMsg> tfList = new List<TransformStampedMsg>();

        foreach (var link in staticLinks)
        {
            // Unity 좌표 -> ROS 좌표 변환
            Vector3 unityPos = link.position;
            Quaternion unityRot = Quaternion.Euler(link.rotation);

            PointMsg posMsg = unityPos.To<FLU>();
            QuaternionMsg rotMsg = unityRot.To<FLU>();

            tfList.Add(new TransformStampedMsg
            {
                header = new HeaderMsg
                {
                    stamp = tfTime,
                    frame_id = link.parentFrameId
                },
                child_frame_id = link.childFrameId,
                transform = new TransformMsg
                {
                    translation = new Vector3Msg(posMsg.x, posMsg.y, posMsg.z),
                    rotation = rotMsg
                }
            });
        }

        TFMessageMsg tfMsg = new TFMessageMsg
        {
            transforms = tfList.ToArray()
        };

        ros.Publish(tfStaticTopic, tfMsg);
    }

    private TimeMsg ConvertToRosTime(float time)
    {
        int sec = (int)time;
        uint nanosec = (uint)((time - sec) * 1e9);
        return new TimeMsg { sec = sec, nanosec = nanosec };
    }
}
