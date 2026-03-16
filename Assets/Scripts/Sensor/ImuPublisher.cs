using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;

/// <summary>
/// ImuSensor의 ROS 토픽 발행 브리지.
/// ImuMsg를 발행한다.
/// </summary>
public class ImuPublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    public string topicName = "/imu";
    public string frameId = "imu_link";

    [Header("Sensor Reference")]
    [Tooltip("같은 GameObject 또는 부모에서 자동 탐색")]
    public ImuSensor sensor;

    [Header("Publish Settings")]
    [Tooltip("FixedUpdate N번째마다 발행 (1=50Hz, 2=25Hz)")]
    public int publishEveryNthTick = 1;

    private ROSConnection ros;
    private int fixedUpdateCount;

    // IMU Covariance
    private double[] orientationCovariance;
    private double[] angularVelocityCovariance;
    private double[] linearAccelerationCovariance;

    void Start()
    {
        if (sensor == null)
            sensor = GetComponent<ImuSensor>() ?? GetComponentInParent<ImuSensor>();

        if (sensor == null)
        {
            Debug.LogError("[ImuPublisher] ImuSensor를 찾지 못했습니다.");
            enabled = false;
            return;
        }

        ros = ROSConnection.GetOrCreateInstance();
        topicName = RosTopicNamespace.Resolve(gameObject, topicName);
        ros.RegisterPublisher<ImuMsg>(topicName);

        fixedUpdateCount = 0;

        orientationCovariance = new double[9];
        orientationCovariance[0] = 0.0001;
        orientationCovariance[4] = 0.0001;
        orientationCovariance[8] = 0.0001;

        angularVelocityCovariance = new double[9];
        angularVelocityCovariance[0] = 0.001;
        angularVelocityCovariance[4] = 0.001;
        angularVelocityCovariance[8] = 0.001;

        linearAccelerationCovariance = new double[9];
        linearAccelerationCovariance[0] = 0.005;
        linearAccelerationCovariance[4] = 0.005;
        linearAccelerationCovariance[8] = 0.005;
    }

    void FixedUpdate()
    {
        if (sensor == null) return;

        fixedUpdateCount++;
        if (fixedUpdateCount >= publishEveryNthTick)
        {
            fixedUpdateCount = 0;
            PublishImu();
        }
    }

    void PublishImu()
    {
        Transform baseLinkTransform = sensor.BaseLinkTransform;
        if (baseLinkTransform == null) return;

        var tfTime = ConvertToRosTime(Time.time);

        Quaternion currentRot = baseLinkTransform.rotation;
        QuaternionMsg orientation = currentRot.To<FLU>();

        Vector3 localAngularVel = baseLinkTransform.InverseTransformDirection(sensor.FilteredAngularVelocity);
        Vector3Msg angularVelMsg = localAngularVel.To<FLU>();

        Vector3 localAccel = baseLinkTransform.InverseTransformDirection(sensor.FilteredAcceleration);
        Vector3Msg linearAccelMsg = localAccel.To<FLU>();

        ImuMsg imuMsg = new ImuMsg
        {
            header = new HeaderMsg
            {
                stamp = tfTime,
                frame_id = frameId
            },
            orientation = orientation,
            orientation_covariance = orientationCovariance,
            angular_velocity = angularVelMsg,
            angular_velocity_covariance = angularVelocityCovariance,
            linear_acceleration = linearAccelMsg,
            linear_acceleration_covariance = linearAccelerationCovariance
        };

        ros.Publish(topicName, imuMsg);
    }

    private TimeMsg ConvertToRosTime(float time)
    {
        int sec = (int)time;
        uint nanosec = (uint)((time - sec) * 1e9);
        return new TimeMsg { sec = sec, nanosec = nanosec };
    }
}
