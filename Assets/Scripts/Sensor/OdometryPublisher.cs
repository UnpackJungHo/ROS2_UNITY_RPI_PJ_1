using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Std;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Tf2;

/// <summary>
/// 차량의 Odometry 정보를 ROS로 발행하고, odom -> base_link TF를 방송하는 클래스
/// </summary>
public class OdometryPublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    public string odomTopic = "/odom";
    public string childFrameId = "base_link";
    public string odomFrameId = "odom";
    public float publishRate = 20f; // Hz

    [Header("Vehicle Settings")]
    public ArticulationBody vehicleBody; // 차량의 물리 바디

    private ROSConnection ros;
    private float timeElapsed;
    private float publishInterval;

    private Vector3 initialPosition;
    private Quaternion initialRotation;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OdometryMsg>(odomTopic);
        ros.RegisterPublisher<TFMessageMsg>("/tf");

        publishInterval = 1.0f / publishRate;

        if (vehicleBody == null)
        {
            vehicleBody = GetComponentInParent<ArticulationBody>();
            if (vehicleBody == null)
            {
                Debug.LogError("OdometryPublisher: ArticulationBody not found in parent or self!");
            }
        }

        // 초기 위치 저장
        initialPosition = transform.position;
        initialRotation = transform.rotation;
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed >= publishInterval)
        {
            // Debug.Log($"Odom Pub: Pos={transform.position}, Vel={vehicleBody?.velocity}");
            PublishOdometryAndTF();
            timeElapsed = 0;
        }
    }

    void PublishOdometryAndTF()
    {
        if (vehicleBody == null) return;

        // 1. 현재 시간
        var tfTime = ConvertToRosTime(Time.time);

        // 2. 차량의 위치와 회전 (ROS 좌표계로 변환)
        Vector3 currentPos = transform.position;
        Quaternion currentRot = transform.rotation;

        // ROS 좌표 변환 (<Vector3> -> <PointMsg>)
        PointMsg position = currentPos.To<FLU>();
        QuaternionMsg orientation = currentRot.To<FLU>();

        // 3. 차량의 속도 (Linear & Angular)
        Vector3 linearVel = vehicleBody.velocity; 
        Vector3 angularVel = vehicleBody.angularVelocity;

        // 월드 속도 -> 로컬 속도 변환
        Vector3 localLinearVel = transform.InverseTransformDirection(linearVel);
        Vector3 localAngularVel = transform.InverseTransformDirection(angularVel);

        Vector3Msg linearMsg = localLinearVel.To<FLU>();
        Vector3Msg angularMsg = localAngularVel.To<FLU>();

        // Odometry Message 생성
        OdometryMsg odomMsg = new OdometryMsg
        {
            header = new HeaderMsg
            {
                stamp = tfTime,
                frame_id = odomFrameId
            },
            child_frame_id = childFrameId,
            pose = new PoseWithCovarianceMsg
            {
                pose = new PoseMsg
                {
                    position = position,
                    orientation = orientation
                }
            },
            twist = new TwistWithCovarianceMsg
            {
                twist = new TwistMsg
                {
                    linear = linearMsg,
                    angular = angularMsg
                }
            }
        };

        ros.Publish(odomTopic, odomMsg);

        // TF Message 생성 (odom -> base_link)
        TransformStampedMsg tfStamped = new TransformStampedMsg
        {
            header = new HeaderMsg
            {
                stamp = tfTime,
                frame_id = odomFrameId
            },
            child_frame_id = childFrameId,
            transform = new TransformMsg
            {
                translation = new Vector3Msg(position.x, position.y, position.z),
                rotation = orientation
            }
        };

        TFMessageMsg tfMsg = new TFMessageMsg
        {
            transforms = new TransformStampedMsg[] { tfStamped }
        };

        ros.Publish("/tf", tfMsg);
    }
    
    private TimeMsg ConvertToRosTime(float time)
    {
        int sec = (int)time;
        uint nanosec = (uint)((time - sec) * 1e9);
        return new TimeMsg { sec = sec, nanosec = nanosec };
    }
}
