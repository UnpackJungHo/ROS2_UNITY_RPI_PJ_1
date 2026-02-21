using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

/// <summary>
/// ROS geometry_msgs/Twist 명령을 WheelTest 입력으로 변환한다.
/// T01 1차 구현: cmd_vel 기반 외부 제어 파이프라인.
/// </summary>
public class VehicleCmdSubscriber : MonoBehaviour
{
    [Header("ROS Settings")]
    [Tooltip("Twist 제어 토픽. RosTopicNamespace가 있으면 prefix가 자동 적용됨")]
    public string cmdTopicName = "/vehicle/cmd";

    [Header("Control Target")]
    public WheelTest wheelController;
    public bool autoFindWheelController = true;
    [Tooltip("true면 RegressionDrivingController.isAutonomousMode(P키)일 때만 외부 cmd 적용")]
    public bool useRegressionAutonomyGate = true;
    public RegressionDrivingController regressionDrivingController;
    public bool autoFindRegressionController = true;
    [Tooltip("Autonomous gate가 꺼지면 externalControlEnabled를 false로 전환")]
    public bool releaseExternalControlWhenGateClosed = true;
    [Tooltip("Autonomous gate가 꺼지면 즉시 정지 브레이크 적용")]
    public bool holdBrakeWhenGateClosed = false;

    [Header("Twist to Wheel Mapping")]
    [Tooltip("선속도 +max일 때 throttle=1")]
    public float maxForwardSpeed = 2.0f;
    [Tooltip("선속도 -max일 때 throttle=-1 또는 brake=1")]
    public float maxReverseSpeed = 1.0f;
    [Tooltip("angular.z가 이 값(rad/s)일 때 steering=1")]
    public float maxYawRateForFullSteer = 1.2f;
    [Tooltip("true면 음수 속도 명령을 후진(throttle<0)으로 전달")]
    public bool allowReverse = false;
    [Tooltip("allowReverse=true일 때 방향 전환 전 브레이크를 우선 적용할 속도 임계값(m/s)")]
    public float directionChangeBrakeSpeedThreshold = 0.35f;

    [Header("Timeout / Fail-safe")]
    [Tooltip("이 시간(초) 동안 명령 미수신 시 timeout 처리")]
    public float commandTimeoutSec = 0.35f;
    [Tooltip("timeout 시 차량을 정지(brake=1)")]
    public bool holdBrakeOnTimeout = true;
    [Tooltip("timeout 시 externalControlEnabled를 해제")]
    public bool releaseExternalControlOnTimeout = false;

    [Header("Debug (Read Only)")]
    [SerializeField] private string resolvedTopicName = "";
    [SerializeField] private bool hasReceivedCommand = false;
    [SerializeField] private float lastCommandTime = -999f;
    [SerializeField] private float lastLinearX = 0f;
    [SerializeField] private float lastAngularZ = 0f;
    [SerializeField] private float lastAppliedSteering = 0f;
    [SerializeField] private float lastAppliedThrottle = 0f;
    [SerializeField] private float lastAppliedBrake = 0f;

    private ROSConnection ros;

    void Start()
    {
        if (autoFindWheelController && wheelController == null)
            wheelController = GetComponent<WheelTest>() ?? GetComponentInParent<WheelTest>() ?? FindObjectOfType<WheelTest>();

        if (autoFindRegressionController && regressionDrivingController == null)
            regressionDrivingController = GetComponent<RegressionDrivingController>()
                ?? GetComponentInParent<RegressionDrivingController>()
                ?? FindObjectOfType<RegressionDrivingController>();

        if (wheelController == null)
        {
            Debug.LogError("[VehicleCmdSubscriber] WheelTest를 찾지 못했습니다.");
            enabled = false;
            return;
        }

        ros = ROSConnection.GetOrCreateInstance();
        resolvedTopicName = RosTopicNamespace.Resolve(gameObject, cmdTopicName);
        ros.Subscribe<TwistMsg>(resolvedTopicName, OnCmdReceived);
        Debug.Log($"[VehicleCmdSubscriber] Subscribed: {resolvedTopicName}");
    }

    void FixedUpdate()
    {
        if (wheelController == null)
            return;

        bool commandFresh = hasReceivedCommand && (Time.time - lastCommandTime) <= Mathf.Max(0.01f, commandTimeoutSec);
        if (IsAutonomyGateClosed())
        {
            HandleAutonomyGateClosed();
            return;
        }

        if (commandFresh)
        {
            ApplyTwist(lastLinearX, lastAngularZ);
            return;
        }

        HandleTimeout();
    }

    void OnCmdReceived(TwistMsg msg)
    {
        if (msg == null)
            return;

        lastLinearX = (float)msg.linear.x;
        lastAngularZ = (float)msg.angular.z;
        lastCommandTime = Time.time;
        hasReceivedCommand = true;
    }

    void ApplyTwist(float linearX, float angularZ)
    {
        float safeMaxForward = Mathf.Max(0.01f, maxForwardSpeed);
        float safeMaxReverse = Mathf.Max(0.01f, maxReverseSpeed);
        float safeYaw = Mathf.Max(0.01f, maxYawRateForFullSteer);

        float targetSteer = Mathf.Clamp(angularZ / safeYaw, -1f, 1f);
        float targetSpeed = Mathf.Clamp(linearX, -safeMaxReverse, safeMaxForward);
        float currentSpeed = wheelController.GetSpeedMS();
        float directionThreshold = Mathf.Max(0f, directionChangeBrakeSpeedThreshold);

        float throttle = 0f;
        float brake = 0f;

        if (targetSpeed >= 0f)
        {
            float forwardCmd = Mathf.Clamp01(targetSpeed / safeMaxForward);
            if (allowReverse && currentSpeed < -directionThreshold)
            {
                throttle = 0f;
                brake = forwardCmd;
            }
            else
            {
                throttle = forwardCmd;
            }
        }
        else
        {
            float reverseCmd = Mathf.Clamp01((-targetSpeed) / safeMaxReverse);
            if (!allowReverse)
            {
                throttle = 0f;
                brake = reverseCmd;
            }
            else
            {
                if (currentSpeed > directionThreshold)
                {
                    throttle = 0f;
                    brake = reverseCmd;
                }
                else
                {
                    throttle = -reverseCmd;
                    brake = 0f;
                }
            }
        }

        wheelController.externalControlEnabled = true;
        wheelController.SetSteering(targetSteer);
        wheelController.SetThrottle(throttle);
        wheelController.SetBrake(brake);

        lastAppliedSteering = targetSteer;
        lastAppliedThrottle = throttle;
        lastAppliedBrake = brake;
    }

    bool IsAutonomyGateClosed()
    {
        return useRegressionAutonomyGate &&
               regressionDrivingController != null &&
               !regressionDrivingController.isAutonomousMode;
    }

    void HandleAutonomyGateClosed()
    {
        if (wheelController == null)
            return;

        if (holdBrakeWhenGateClosed)
        {
            wheelController.externalControlEnabled = true;
            wheelController.SetThrottle(0f);
            wheelController.SetSteering(0f);
            wheelController.SetBrake(1f);

            lastAppliedSteering = 0f;
            lastAppliedThrottle = 0f;
            lastAppliedBrake = 1f;
            return;
        }

        if (releaseExternalControlWhenGateClosed)
        {
            wheelController.externalControlEnabled = false;
            return;
        }

        wheelController.externalControlEnabled = true;
        wheelController.SetThrottle(0f);
        wheelController.SetSteering(0f);
        wheelController.SetBrake(0f);

        lastAppliedSteering = 0f;
        lastAppliedThrottle = 0f;
        lastAppliedBrake = 0f;
    }

    void HandleTimeout()
    {
        if (releaseExternalControlOnTimeout)
            wheelController.externalControlEnabled = false;

        if (!holdBrakeOnTimeout)
            return;

        wheelController.externalControlEnabled = true;
        wheelController.SetThrottle(0f);
        wheelController.SetSteering(0f);
        wheelController.SetBrake(1f);

        lastAppliedSteering = 0f;
        lastAppliedThrottle = 0f;
        lastAppliedBrake = 1f;
    }
}
