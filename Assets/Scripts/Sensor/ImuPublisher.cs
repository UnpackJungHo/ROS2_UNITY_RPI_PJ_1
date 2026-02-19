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

    [Header("Vehicle Reference")]
    public ArticulationBody vehicleBody;

    [Header("Base Link Reference")]
    [Tooltip("base_link 게임오브젝트를 지정. 비워두면 이 오브젝트 자신을 사용.")]
    public GameObject baseLinkObject;

    private Transform baseLinkTransform;

    [Header("Filter Settings")]
    [Tooltip("가속도 저역통과 필터 계수 (0.0~1.0). 낮을수록 더 부드러움")]
    [Range(0.05f, 1.0f)]
    public float accelFilterAlpha = 0.3f;

    [Tooltip("각속도 저역통과 필터 계수 (0.0~1.0). 낮을수록 더 부드러움")]
    [Range(0.05f, 1.0f)]
    public float gyroFilterAlpha = 0.3f;

    [Header("Acceleration Estimation")]
    [Tooltip("가속도 계산에 사용할 프레임 간격 (높을수록 부드럽지만 지연 증가)")]
    [Range(1, 8)]
    public int velocityBufferSize = 4;

    [Header("Publish Settings")]
    [Tooltip("FixedUpdate N번째마다 발행 (1=50Hz, 2=25Hz)")]
    public int publishEveryNthTick = 1;

    private ROSConnection ros;
    private int fixedUpdateCount;

    // 다중 프레임 속도 버퍼 (원형 버퍼)
    // 단일 프레임 미분 (v2-v1)/dt 대신, N프레임 간격으로 미분하여 노이즈 감소
    // dt=0.02s(50Hz)에서 4프레임 → dt=0.08s, 노이즈 ~4배 감소
    private Vector3[] velocityBuffer;
    private int bufferIndex;
    private int bufferCount;

    // 저역통과 필터 적용된 값
    private Vector3 filteredAcceleration;
    private Vector3 filteredAngularVelocity;
    private bool filterInitialized;

    // IMU Covariance
    private double[] orientationCovariance;
    private double[] angularVelocityCovariance;
    private double[] linearAccelerationCovariance;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        topicName = RosTopicNamespace.Resolve(gameObject, topicName);
        ros.RegisterPublisher<ImuMsg>(topicName);

        // baseLinkTransform 설정
        baseLinkTransform = baseLinkObject != null ? baseLinkObject.transform : transform;

        if (vehicleBody == null)
        {
            vehicleBody = baseLinkTransform.GetComponentInChildren<ArticulationBody>();
            if (vehicleBody == null)
                vehicleBody = baseLinkTransform.GetComponentInParent<ArticulationBody>();
            if (vehicleBody == null)
            {
                Debug.LogError("ImuPublisher: ArticulationBody not found!");
            }
        }

        filterInitialized = false;
        fixedUpdateCount = 0;

        // 속도 원형 버퍼 초기화
        velocityBuffer = new Vector3[velocityBufferSize];
        bufferIndex = 0;
        bufferCount = 0;

        // Covariance 행렬 초기화 (대각 성분만 설정, 9개 원소의 3x3 행렬)
        orientationCovariance = new double[9];
        orientationCovariance[0] = 0.0001;  // roll — Ground Truth
        orientationCovariance[4] = 0.0001;  // pitch
        orientationCovariance[8] = 0.0001;  // yaw

        angularVelocityCovariance = new double[9];
        angularVelocityCovariance[0] = 0.001;  // 물리 엔진 직접값
        angularVelocityCovariance[4] = 0.001;
        angularVelocityCovariance[8] = 0.001;

        // 다중 프레임 미분 + EMA 필터 → 이전보다 낮은 분산
        linearAccelerationCovariance = new double[9];
        linearAccelerationCovariance[0] = 0.005;
        linearAccelerationCovariance[4] = 0.005;
        linearAccelerationCovariance[8] = 0.005;
    }

    void FixedUpdate()
    {
        if (vehicleBody == null) return;

        Vector3 currentVelocity = vehicleBody.velocity;

        // ── 가속도 계산: 다중 프레임 미분 ──
        // 원형 버퍼에서 N프레임 전 속도와 현재 속도의 차이로 가속도 추정
        // (v_current - v_N_frames_ago) / (N * fixedDeltaTime)
        // 긴 시간 기저로 나누므로 수치 미분 노이즈가 크게 감소
        Vector3 rawAcceleration;

        if (bufferCount >= velocityBufferSize)
        {
            // 버퍼가 가득 찬 후: N프레임 전 속도와 비교
            Vector3 oldVelocity = velocityBuffer[bufferIndex];
            float deltaTime = velocityBufferSize * Time.fixedDeltaTime;
            rawAcceleration = (currentVelocity - oldVelocity) / deltaTime;
        }
        else
        {
            // 초기화 기간: 직전 프레임과 비교
            if (bufferCount > 0)
            {
                int prevIndex = (bufferIndex - 1 + velocityBufferSize) % velocityBufferSize;
                rawAcceleration = (currentVelocity - velocityBuffer[prevIndex]) / Time.fixedDeltaTime;
            }
            else
            {
                rawAcceleration = Vector3.zero;
            }
        }

        // 원형 버퍼에 현재 속도 저장
        velocityBuffer[bufferIndex] = currentVelocity;
        bufferIndex = (bufferIndex + 1) % velocityBufferSize;
        bufferCount++;

        // 중력 포함 (실제 IMU 비력 측정: 정지 시 +9.81 m/s² 상방향)
        // specific_force = F_non_gravitational / m = a_total - g
        rawAcceleration -= Physics.gravity;

        // ── 각속도: 물리 엔진 직접 제공 (미분 불필요) ──
        Vector3 rawAngularVelocity = vehicleBody.angularVelocity;

        // ── 저역통과 필터 (EMA) ──
        if (!filterInitialized)
        {
            filteredAcceleration = rawAcceleration;
            filteredAngularVelocity = rawAngularVelocity;
            filterInitialized = true;
        }
        else
        {
            filteredAcceleration = Vector3.Lerp(filteredAcceleration, rawAcceleration, accelFilterAlpha);
            filteredAngularVelocity = Vector3.Lerp(filteredAngularVelocity, rawAngularVelocity, gyroFilterAlpha);
        }

        // ── 발행 ──
        fixedUpdateCount++;
        if (fixedUpdateCount >= publishEveryNthTick)
        {
            fixedUpdateCount = 0;
            PublishImu();
        }
    }

    void PublishImu()
    {
        var tfTime = ConvertToRosTime(Time.time);

        // 회전 (Orientation) — Unity Ground Truth
        Quaternion currentRot = baseLinkTransform.rotation;
        QuaternionMsg orientation = currentRot.To<FLU>();

        // 각속도 → 로컬 프레임 변환 후 ROS 좌표계 변환
        Vector3 localAngularVel = baseLinkTransform.InverseTransformDirection(filteredAngularVelocity);
        Vector3Msg angularVelMsg = localAngularVel.To<FLU>();

        // 선형 가속도 → 로컬 프레임 변환 후 ROS 좌표계 변환
        Vector3 localAccel = baseLinkTransform.InverseTransformDirection(filteredAcceleration);
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
