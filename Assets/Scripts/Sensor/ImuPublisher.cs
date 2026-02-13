using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;

public class ImuPublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    public string topicName = "/imu";
    public string frameId = "imu_link";
    public float publishRate = 200f; // IMU should be fast (200Hz+)

    [Header("Vehicle Reference")]
    public ArticulationBody vehicleBody;

    private ROSConnection ros;
    private float publishInterval;
    private float timeElapsed;

    private Vector3 previousVelocity;
    private Vector3 currentAcceleration;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImuMsg>(topicName);

        publishInterval = 1.0f / publishRate;

        if (vehicleBody == null)
        {
            vehicleBody = GetComponentInParent<ArticulationBody>();
             if (vehicleBody == null)
            {
                Debug.LogError("ImuPublisher: ArticulationBody not found in parent or self!");
            }
        }

        if (vehicleBody != null)
        {
            Debug.Log($"[ImuDebug] Initialized with ArticulationBody: {vehicleBody.name}");
            previousVelocity = vehicleBody.velocity;
        }
    }

    void FixedUpdate()
    {
        // 가속도 계산 (물리 업데이트 주기마다)
        if (vehicleBody != null)
        {
            Vector3 currentVelocity = vehicleBody.velocity;
            // 가속도 = (v2 - v1) / dt
            currentAcceleration = (currentVelocity - previousVelocity) / Time.fixedDeltaTime;
            previousVelocity = currentVelocity;
            
            // IMU는 '중력'을 포함한 가속도를 측정함 (정지 상태에서 +g가 나와야 함 -> Unity Gravity가 -Y이므로 뺌)
            // 혹은 LIO-SAM 설정에 따라 gravity를 뺄지 말지 결정. 보통 Raw IMU는 gravity를 포함함.
            currentAcceleration -= Physics.gravity; 
        }
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed >= publishInterval)
        {
            PublishImu();
            timeElapsed = 0;
        }
    }

    void PublishImu()
    {
        if (vehicleBody == null) return;

        var tfTime = ConvertToRosTime(Time.time);

        // 회전 (Orientation)
        Quaternion currentRot = transform.rotation;
        QuaternionMsg orientation = currentRot.To<FLU>();

        // 각속도 (Angular Velocity)
        Vector3 angularVel = vehicleBody.angularVelocity;
        // Local Frame 변환
        Vector3 localAngularVel = transform.InverseTransformDirection(angularVel);
        Vector3Msg angularVelMsg = localAngularVel.To<FLU>();

        // 선형 가속도 (Linear Acceleration)
        // Local Frame 변환
        Vector3 localAccel = transform.InverseTransformDirection(currentAcceleration);
        Vector3Msg linearAccelMsg = localAccel.To<FLU>();

        ImuMsg imuMsg = new ImuMsg
        {
            header = new HeaderMsg
            {
                stamp = tfTime,
                frame_id = frameId
            },
            orientation = orientation,
            angular_velocity = angularVelMsg,
            linear_acceleration = linearAccelMsg
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
