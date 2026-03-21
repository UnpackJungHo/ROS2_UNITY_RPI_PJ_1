using UnityEngine;

/// <summary>
/// Twist 명령을 VehicleMotionController의 steering/throttle/brake 입력으로 매핑하는 컨트롤러.
/// ROS 구독은 VehicleCmdSubscriber가 담당한다.
/// </summary>
public class VehicleCmdController : MonoBehaviour
{
    [Header("Control Target")]
    public VehicleMotionController wheelController;
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
    [Tooltip("true면 음수 linear.x 명령을 후진 대신 제동으로 해석(안전 우선)")]
    public bool preferBrakeForNegativeSpeed = true;
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
    [SerializeField] private float lastAppliedSteering = 0f;
    [SerializeField] private float lastAppliedThrottle = 0f;
    [SerializeField] private float lastAppliedBrake = 0f;

    // Subscriber가 설정하는 입력 상태
    private bool hasReceivedCommand = false;
    private float lastCommandTime = -999f;
    private float lastLinearX = 0f;
    private float lastAngularZ = 0f;

    void Start()
    {
        if (autoFindWheelController && wheelController == null)
            wheelController = GetComponent<VehicleMotionController>() ?? GetComponentInParent<VehicleMotionController>() ?? FindObjectOfType<VehicleMotionController>();

        if (autoFindRegressionController && regressionDrivingController == null)
            regressionDrivingController = GetComponent<RegressionDrivingController>()
                ?? GetComponentInParent<RegressionDrivingController>()
                ?? FindObjectOfType<RegressionDrivingController>();

        if (wheelController == null)
        {
            Debug.LogError("[VehicleCmdController] VehicleMotionController를 찾지 못했습니다.");
            enabled = false;
        }
    }

    /// <summary>
    /// Subscriber가 ROS 메시지 수신 시 호출하여 최신 명령을 전달한다.
    /// </summary>
    public void SetCommand(float linearX, float angularZ)
    {
        lastLinearX = linearX;
        lastAngularZ = angularZ;
        lastCommandTime = Time.time;
        hasReceivedCommand = true;
    }

    void FixedUpdate()
    {
        if (wheelController == null) return;

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
            if (preferBrakeForNegativeSpeed || !allowReverse)
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
        if (wheelController == null) return;

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

        if (!holdBrakeOnTimeout) return;

        wheelController.externalControlEnabled = true;
        wheelController.SetThrottle(0f);
        wheelController.SetSteering(0f);
        wheelController.SetBrake(1f);
        lastAppliedSteering = 0f;
        lastAppliedThrottle = 0f;
        lastAppliedBrake = 1f;
    }
}
