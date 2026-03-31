using UnityEngine;
using Unity.Robotics.UrdfImporter;
using Unity.Robotics.UrdfImporter.Control;

// Run before most startup scripts so the visual URDF rig is disabled
// before articulation physics can kick the robot upward on play enter.
[DefaultExecutionOrder(-5000)]
public class VehicleMotionController : MonoBehaviour
{
    [Header("Steering Links (Auto-assigned)")]
    public ArticulationBody frontLeftSteering;
    public ArticulationBody frontRightSteering;

    [Header("Wheels (Auto-assigned)")]
    public ArticulationBody frontLeftWheel;
    public ArticulationBody frontRightWheel;
    public ArticulationBody rearLeftWheel;
    public ArticulationBody rearRightWheel;

    [Header("Ackermann Geometry")]
    [Tooltip("축거: 앞바퀴 축과 뒷바퀴 축 사이 거리 (m). 길수록 회전반경 증가, 직진 안정성 향상")]
    public float wheelBase = 0.6f;
    [Tooltip("윤거: 좌우 바퀴 중심 사이 거리 (m). 넓을수록 횡방향 안정성 향상")]
    public float trackWidth = 0.65f;
    [Tooltip("최대 조향 각도 (도). 클수록 급회전 가능, 일반 차량 30~40도")]
    public float maxSteeringAngle = 30f;
    [Tooltip("조향 속도 (도/초). 핸들을 얼마나 빨리 꺾을 수 있는지")]
    public float steeringSpeed = 100f;

    [Header("Vehicle Specs")]
    [Tooltip("차량 질량 (kg)")]
    public float vehicleMass = 65f;
    [Tooltip("바퀴 반지름 (m)")]
    public float wheelRadius = 0.1f;
    [Tooltip("무게중심 높이 (m)")]
    public float centerOfMassHeight = 0.25f;

    [Header("Electric Motor (배달 AMR)")]
    [Tooltip("최대 모터 토크 (Nm)")]
    public float maxMotorTorque = 5f;
    [Tooltip("최대 모터 RPM")]
    public float maxMotorRPM = 400f;
    [Tooltip("모터 토크 커브 (X: RPM 비율 0~1, Y: 토크 비율 0~1)")]
    public AnimationCurve torqueCurve;
    [Tooltip("감속비 (전기모터 → 바퀴)")]
    public float reductionRatio = 3.0f;

    [Header("Speed & Stability Limits (배달 AMR: 0.5~2.0 m/s)")]
    [Tooltip("최고 속도 제한 (m/s)")]
    public float maxSpeed = 2.0f;
    [Tooltip("최대 가속도 제한 (m/s²) - 부드러운 주행: 0.3~0.5")]
    public float maxAcceleration = 0.8f;
    [Tooltip("최대 감속도 제한 (m/s²)")]
    public float maxDeceleration = 1.5f;
    [Tooltip("고속 조향 제한 활성화")]
    public bool enableSpeedBasedSteeringLimit = true;
    [Tooltip("조향 제한 없는 최대 속도 (m/s)")]
    public float fullSteeringSpeed = 0.5f;

    [Header("Resistance Coefficients (실외)")]
    [Tooltip("구름저항 계수 - 실외 아스팔트: 0.015, 거친 노면: 0.03")]
    public float rollingResistance = 0.02f;
    [Tooltip("공기저항 계수 (Cd) - 저속에서는 영향 미미")]
    public float dragCoefficient = 0.1f;
    [Tooltip("전면 투영 면적 (m²)")]
    public float frontalArea = 0.4f;
    [Tooltip("공기 밀도 (kg/m³)")]
    public float airDensity = 1.225f;

    [Header("Brakes")]
    [Tooltip("최대 브레이크 힘 (N)")]
    public float maxBrakeForce = 600f;
    [Tooltip("엔진 브레이크 힘 (N)")]
    public float engineBrakeForce = 20f;
    [Tooltip("브레이크 hold가 차를 완전히 붙잡기 시작하는 속도 임계값 (m/s)")]
    public float standstillHoldSpeed = 0.15f;
    [Tooltip("스로틀/브레이크 입력이 없을 때 정지로 스냅하는 속도 임계값 (m/s)")]
    public float coastStopSpeed = 0.05f;

    [Header("Hybrid Physics Backend")]
    [Tooltip("true면 Rigidbody + WheelCollider 기반 root를 런타임에 생성한다.")]
    public bool useHybridWheelColliderBackend = true;
    [Tooltip("차체 단순 충돌체를 활성화한다. 초기 전환 단계에서는 false로 두고 WheelCollider 접지부터 안정화한다.")]
    public bool enableHybridChassisCollider = false;
    [Tooltip("비주얼 URDF rig의 일반 충돌체를 비활성화한다.")]
    public bool disableVisualRigColliders = true;
    [Tooltip("비주얼 URDF rig의 Articulation 물리를 비활성화한다.")]
    public bool disableVisualRigArticulations = true;

    [Header("WheelCollider Suspension")]
    public float suspensionDistance = 0.08f;
    public float suspensionSpring = 5000f;
    public float suspensionDamper = 1200f;
    [Range(0f, 1f)] public float suspensionTargetPosition = 0.5f;
    public float wheelDampingRate = 1.2f;
    public float hybridAngularDrag = 1.5f;

    [Header("WheelCollider Friction")]
    public float forwardExtremumSlip = 0.35f;
    public float forwardExtremumValue = 1.2f;
    public float forwardAsymptoteSlip = 0.8f;
    public float forwardAsymptoteValue = 0.9f;
    public float forwardStiffness = 1.4f;
    public float sidewaysExtremumSlip = 0.22f;
    public float sidewaysExtremumValue = 1.15f;
    public float sidewaysAsymptoteSlip = 0.7f;
    public float sidewaysAsymptoteValue = 0.85f;
    public float frontSidewaysStiffness = 1.05f;
    public float rearSidewaysStiffness = 0.95f;

    [Header("Input Filtering")]
    [Range(0f, 0.3f)] public float steeringDeadzone = 0.05f;
    [Range(0f, 0.3f)] public float throttleDeadzone = 0.05f;
    [Tooltip("스로틀 입력 상승 속도 (입력/초)")]
    public float throttleRiseRate = 2.5f;
    [Tooltip("스로틀 입력 하강 속도 (입력/초)")]
    public float throttleFallRate = 4.0f;
    [Tooltip("브레이크 입력 상승 속도 (입력/초)")]
    public float brakeRiseRate = 5.0f;
    [Tooltip("브레이크 입력 하강 속도 (입력/초)")]
    public float brakeFallRate = 7.0f;

    [Header("Drive Axle")]
    public bool driveFrontAxle = true;
    public bool driveRearAxle = true;

    [Header("Auto-Find Settings")]
    public bool autoFindReferences = true;

    [Header("External Control (AI)")]
    [Tooltip("외부 제어 활성화 시 키보드 입력 무시")]
    public bool externalControlEnabled = false;

    [Header("Debug Info (Read Only)")]
    [SerializeField] private float currentSpeed_ms;
    [SerializeField] private float currentSpeed_kmh;
    [SerializeField] private float currentMotorRPM;
    [SerializeField] private float currentAcceleration;
    [SerializeField] private float currentSteeringAngle;
    [SerializeField] private float appliedThrottleInput;
    [SerializeField] private float appliedBrakeInput;
    [SerializeField] private float currentAppliedDriveTorque;
    [SerializeField] private float currentAppliedBrakeTorque;

    private float rawThrottleInput;
    private float rawBrakeInput;
    private float steeringInput;
    private float leftVisualSteerAngle;
    private float rightVisualSteerAngle;

    private WheelColliderVehicleDynamics hybridDynamics;
    private Transform visualRigRoot;
    private Transform steeringLeftTransform;
    private Transform steeringRightTransform;
    private Quaternion steeringLeftBaseRotation;
    private Quaternion steeringRightBaseRotation;
    private Vector3 visualRigOffsetPosition;
    private Quaternion visualRigOffsetRotation;
    private bool runtimePrepared;
    private bool backendInitialized;

    void Awake()
    {
        PrepareRuntime();
    }

    void Start()
    {
        PrepareRuntime();
        InitializeBackendIfNeeded();
        SyncVisualSteering();
        SyncDebugState();
    }

    void PrepareRuntime()
    {
        if (runtimePrepared)
            return;

        runtimePrepared = true;

        if (autoFindReferences)
            FindReferences();

        if (torqueCurve == null || torqueCurve.keys.Length == 0)
        {
            torqueCurve = new AnimationCurve();
            torqueCurve.AddKey(0f, 1f);
            torqueCurve.AddKey(0.3f, 1f);
            torqueCurve.AddKey(0.7f, 0.9f);
            torqueCurve.AddKey(1f, 0.7f);
        }

        steeringLeftTransform = frontLeftSteering != null ? frontLeftSteering.transform : null;
        steeringRightTransform = frontRightSteering != null ? frontRightSteering.transform : null;
        steeringLeftBaseRotation = steeringLeftTransform != null ? steeringLeftTransform.localRotation : Quaternion.identity;
        steeringRightBaseRotation = steeringRightTransform != null ? steeringRightTransform.localRotation : Quaternion.identity;

        if (!useHybridWheelColliderBackend)
            return;

        visualRigRoot = ResolveVisualRigRoot();
        if (visualRigRoot == null)
        {
            Debug.LogError("[VehicleMotionController] visualRigRoot를 찾지 못했습니다.");
            return;
        }

        if (disableVisualRigColliders)
            DisableVisualRigColliders();

        if (disableVisualRigArticulations)
            DisableVisualRigArticulations();
    }

    void InitializeBackendIfNeeded()
    {
        if (backendInitialized || !useHybridWheelColliderBackend)
            return;

        backendInitialized = true;
        InitializeHybridBackend();
    }

    void Update()
    {
        if (!externalControlEnabled)
        {
            steeringInput = ApplyDeadzone(Input.GetAxis("Horizontal"), steeringDeadzone);
            float vertical = ApplyDeadzone(Input.GetAxis("Vertical"), throttleDeadzone);

            if (vertical > 0f)
            {
                rawThrottleInput = vertical;
                rawBrakeInput = 0f;
            }
            else if (vertical < 0f)
            {
                if (currentSpeed_ms > 0.5f)
                {
                    rawThrottleInput = 0f;
                    rawBrakeInput = -vertical;
                }
                else
                {
                    rawThrottleInput = vertical;
                    rawBrakeInput = 0f;
                }
            }
            else
            {
                rawThrottleInput = 0f;
                rawBrakeInput = 0f;
            }

            if (Input.GetKey(KeyCode.Space))
            {
                rawBrakeInput = 1f;
                rawThrottleInput = 0f;
            }
        }
    }

    float ApplyDeadzone(float value, float deadzone)
    {
        return Mathf.Abs(value) < deadzone ? 0f : value;
    }

    void FixedUpdate()
    {
        UpdateLongitudinalInputs();
        UpdateSteering(steeringInput);

        if (hybridDynamics != null)
        {
            hybridDynamics.Step(leftVisualSteerAngle, rightVisualSteerAngle, appliedThrottleInput, appliedBrakeInput);
            SyncVisualRigPose();
            SyncVisualSteering();
            SyncDebugState();
        }
    }

    void UpdateLongitudinalInputs()
    {
        appliedThrottleInput = MoveInputTowards(appliedThrottleInput, rawThrottleInput, throttleRiseRate, throttleFallRate);
        appliedBrakeInput = MoveInputTowards(appliedBrakeInput, rawBrakeInput, brakeRiseRate, brakeFallRate);
    }

    float MoveInputTowards(float current, float target, float riseRate, float fallRate)
    {
        float step;
        if (Mathf.Abs(target) > Mathf.Abs(current))
        {
            bool sameDirection = Mathf.Sign(target) == Mathf.Sign(current == 0f ? target : current);
            step = sameDirection ? riseRate : fallRate;
        }
        else
        {
            step = fallRate;
        }

        return Mathf.MoveTowards(current, target, Mathf.Max(step, 0.01f) * Time.fixedDeltaTime);
    }

    void FindReferences()
    {
        frontLeftSteering = FindArticulationBody("front_left_steering");
        frontRightSteering = FindArticulationBody("front_right_steering");
        frontLeftWheel = FindArticulationBody("front_left_wheel");
        frontRightWheel = FindArticulationBody("front_right_wheel");
        rearLeftWheel = FindArticulationBody("rear_left_wheel");
        rearRightWheel = FindArticulationBody("rear_right_wheel");
    }

    ArticulationBody FindArticulationBody(string name)
    {
        Transform found = FindChildRecursive(transform, name);
        if (found != null)
            return found.GetComponent<ArticulationBody>();

        Debug.LogWarning($"[VehicleMotionController] Not found: {name}");
        return null;
    }

    Transform FindChildRecursive(Transform parent, string name)
    {
        foreach (Transform child in parent)
        {
            if (child.name == name)
                return child;

            Transform found = FindChildRecursive(child, name);
            if (found != null)
                return found;
        }
        return null;
    }

    void InitializeHybridBackend()
    {
        if (visualRigRoot == null)
        {
            Debug.LogError("[VehicleMotionController] visualRigRoot를 찾지 못했습니다.");
            return;
        }

        Transform backendParent = visualRigRoot.parent != null ? visualRigRoot.parent : visualRigRoot;
        string backendName = $"{visualRigRoot.name}_PhysicsRoot";
        Transform existing = backendParent.Find(backendName);
        Transform backendTransform = existing;
        if (backendTransform == null)
        {
            backendTransform = new GameObject(backendName).transform;
            backendTransform.SetParent(backendParent, true);
        }

        backendTransform.SetPositionAndRotation(transform.position, transform.rotation);

        hybridDynamics = backendTransform.GetComponent<WheelColliderVehicleDynamics>();
        if (hybridDynamics == null)
            hybridDynamics = backendTransform.gameObject.AddComponent<WheelColliderVehicleDynamics>();

        Bounds chassisBounds = BuildChassisBounds();
        Vector3 centerOfMassOffset = new Vector3(0f, -centerOfMassHeight * 0.5f, 0f);

        Vector3 frontLeftPosition = frontLeftWheel != null ? frontLeftWheel.transform.position : transform.position + transform.TransformVector(new Vector3(-trackWidth * 0.5f, 0f, wheelBase * 0.5f));
        Vector3 frontRightPosition = frontRightWheel != null ? frontRightWheel.transform.position : transform.position + transform.TransformVector(new Vector3(trackWidth * 0.5f, 0f, wheelBase * 0.5f));
        Vector3 rearLeftPosition = rearLeftWheel != null ? rearLeftWheel.transform.position : transform.position + transform.TransformVector(new Vector3(-trackWidth * 0.5f, 0f, -wheelBase * 0.5f));
        Vector3 rearRightPosition = rearRightWheel != null ? rearRightWheel.transform.position : transform.position + transform.TransformVector(new Vector3(trackWidth * 0.5f, 0f, -wheelBase * 0.5f));
        Vector3 spawnPosition = ResolveHybridSpawnPosition(
            transform.position,
            transform.rotation,
            frontLeftPosition,
            frontRightPosition,
            rearLeftPosition,
            rearRightPosition);

        Vector3 frontLeftLocalMount = Quaternion.Inverse(transform.rotation) * (frontLeftPosition - transform.position);
        Vector3 frontRightLocalMount = Quaternion.Inverse(transform.rotation) * (frontRightPosition - transform.position);
        Vector3 rearLeftLocalMount = Quaternion.Inverse(transform.rotation) * (rearLeftPosition - transform.position);
        Vector3 rearRightLocalMount = Quaternion.Inverse(transform.rotation) * (rearRightPosition - transform.position);

        hybridDynamics.Initialize(
            this,
            spawnPosition,
            transform.rotation,
            chassisBounds,
            centerOfMassOffset,
            frontLeftLocalMount,
            frontRightLocalMount,
            rearLeftLocalMount,
            rearRightLocalMount);

        CacheVisualRigOffset();

        SyncVisualRigPose();
    }

    Transform ResolveVisualRigRoot()
    {
        UrdfRobot robot = GetComponentInParent<UrdfRobot>();
        if (robot != null)
            return robot.transform;

        if (transform.parent != null && transform.parent.parent != null)
            return transform.parent.parent;

        return transform.root;
    }

    void CacheVisualRigOffset()
    {
        visualRigOffsetPosition = Quaternion.Inverse(transform.rotation) * (visualRigRoot.position - transform.position);
        visualRigOffsetRotation = Quaternion.Inverse(transform.rotation) * visualRigRoot.rotation;
    }

    Vector3 ResolveHybridSpawnPosition(
        Vector3 referencePosition,
        Quaternion referenceRotation,
        Vector3 frontLeftPosition,
        Vector3 frontRightPosition,
        Vector3 rearLeftPosition,
        Vector3 rearRightPosition)
    {
        Vector3[] wheelPositions = { frontLeftPosition, frontRightPosition, rearLeftPosition, rearRightPosition };
        float suspensionRestOffset = suspensionDistance * Mathf.Clamp01(1f - suspensionTargetPosition);
        float resolvedY = 0f;
        int hitCount = 0;

        Physics.SyncTransforms();

        for (int i = 0; i < wheelPositions.Length; i++)
        {
            Vector3 wheelPosition = wheelPositions[i];
            Vector3 rayOrigin = new Vector3(wheelPosition.x, Mathf.Max(referencePosition.y, wheelPosition.y) + 5f, wheelPosition.z);
            if (!Physics.Raycast(rayOrigin, Vector3.down, out RaycastHit hit, 20f, ~0, QueryTriggerInteraction.Ignore))
                continue;

            Vector3 localWheelMount = Quaternion.Inverse(referenceRotation) * (wheelPosition - referencePosition);
            float candidateRootY = hit.point.y + wheelRadius + suspensionRestOffset - localWheelMount.y;
            resolvedY += candidateRootY;
            hitCount++;
        }

        if (hitCount == 0)
        {
            Debug.LogWarning($"[VehicleMotionController] Hybrid spawn fallback used. refY={referencePosition.y:F3}");
            return referencePosition;
        }

        referencePosition.y = resolvedY / hitCount;
        return referencePosition;
    }

    Bounds BuildChassisBounds()
    {
        Bounds bounds = new Bounds();
        bounds.center = transform.position + transform.up * (wheelRadius + 0.16f);
        bounds.size = new Vector3(
            Mathf.Max(0.75f, trackWidth + 0.18f),
            0.22f,
            Mathf.Max(0.9f, wheelBase + 0.32f));
        return bounds;
    }

    void DisableVisualRigColliders()
    {
        Collider[] colliders = visualRigRoot.GetComponentsInChildren<Collider>(true);
        for (int i = 0; i < colliders.Length; i++)
        {
            Collider collider = colliders[i];
            if (collider == null)
                continue;

            collider.enabled = false;
        }
    }

    void DisableVisualRigArticulations()
    {
        ArticulationBody[] bodies = visualRigRoot.GetComponentsInChildren<ArticulationBody>(true);
        for (int i = 0; i < bodies.Length; i++)
        {
            ArticulationBody body = bodies[i];
            if (body == null)
                continue;

            body.velocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
            body.enabled = false;
        }

        FKRobot[] fkRobots = visualRigRoot.GetComponentsInChildren<FKRobot>(true);
        for (int i = 0; i < fkRobots.Length; i++)
        {
            if (fkRobots[i] != null)
                fkRobots[i].enabled = false;
        }
    }

    void SyncVisualRigPose()
    {
        if (hybridDynamics == null || visualRigRoot == null)
            return;

        Transform physicsTransform = hybridDynamics.transform;
        visualRigRoot.rotation = physicsTransform.rotation * visualRigOffsetRotation;
        visualRigRoot.position = physicsTransform.position + physicsTransform.rotation * visualRigOffsetPosition;
    }

    void SyncVisualSteering()
    {
        if (steeringLeftTransform != null)
            steeringLeftTransform.localRotation = steeringLeftBaseRotation * Quaternion.Euler(0f, leftVisualSteerAngle, 0f);

        if (steeringRightTransform != null)
            steeringRightTransform.localRotation = steeringRightBaseRotation * Quaternion.Euler(0f, rightVisualSteerAngle, 0f);
    }

    void UpdateSteering(float input)
    {
        float effectiveMaxSteeringAngle = maxSteeringAngle;
        if (enableSpeedBasedSteeringLimit)
        {
            float speed = Mathf.Abs(currentSpeed_ms);
            if (speed > fullSteeringSpeed)
            {
                float speedRatio = (speed - fullSteeringSpeed) / Mathf.Max(0.01f, maxSpeed - fullSteeringSpeed);
                float steeringMultiplier = Mathf.Lerp(1f, 0.35f, Mathf.Clamp01(speedRatio));
                effectiveMaxSteeringAngle = maxSteeringAngle * steeringMultiplier;
            }
        }

        float targetAngle = input * effectiveMaxSteeringAngle;
        currentSteeringAngle = Mathf.MoveTowards(currentSteeringAngle, targetAngle, steeringSpeed * Time.fixedDeltaTime);
        CalculateAckermannAngles(currentSteeringAngle, out leftVisualSteerAngle, out rightVisualSteerAngle);
    }

    void CalculateAckermannAngles(float steerAngle, out float leftAngle, out float rightAngle)
    {
        if (Mathf.Abs(steerAngle) < 0.01f)
        {
            leftAngle = 0f;
            rightAngle = 0f;
            return;
        }

        float steerRad = steerAngle * Mathf.Deg2Rad;
        float turnRadius = wheelBase / Mathf.Tan(Mathf.Abs(steerRad));

        float innerRadius = turnRadius - (trackWidth * 0.5f);
        float outerRadius = turnRadius + (trackWidth * 0.5f);
        float innerAngle = Mathf.Atan(wheelBase / innerRadius) * Mathf.Rad2Deg;
        float outerAngle = Mathf.Atan(wheelBase / outerRadius) * Mathf.Rad2Deg;

        if (steerAngle > 0f)
        {
            leftAngle = innerAngle;
            rightAngle = outerAngle;
        }
        else
        {
            leftAngle = -outerAngle;
            rightAngle = -innerAngle;
        }
    }

    void SyncDebugState()
    {
        if (hybridDynamics == null)
            return;

        currentSpeed_ms = hybridDynamics.GetSignedPlanarSpeed();
        currentSpeed_kmh = currentSpeed_ms * 3.6f;
        currentMotorRPM = hybridDynamics.GetMotorRPM();
        currentAcceleration = hybridDynamics.GetAcceleration();
        currentAppliedDriveTorque = hybridDynamics.GetAppliedDriveTorque();
        currentAppliedBrakeTorque = hybridDynamics.GetAppliedBrakeTorque();
    }

    public bool UsingHybridBackend() => hybridDynamics != null;
    public GameObject GetCollisionSource() => hybridDynamics != null ? hybridDynamics.GetCollisionSource() : gameObject;
    public Vector3 GetWorldVelocity() => hybridDynamics != null ? hybridDynamics.GetWorldVelocity() : Vector3.zero;
    public Vector3 GetWorldAngularVelocity() => hybridDynamics != null ? hybridDynamics.GetWorldAngularVelocity() : Vector3.zero;

    public void ResetVehiclePose(Vector3 position, Quaternion rotation)
    {
        if (hybridDynamics == null)
            return;

        hybridDynamics.Teleport(position, rotation);
        SyncVisualRigPose();
        SyncVisualSteering();
        Physics.SyncTransforms();
        SyncDebugState();
    }

    public float GetSpeedMS() => currentSpeed_ms;
    public float GetSpeedKMH() => currentSpeed_kmh;
    public float GetMotorRPM() => currentMotorRPM;
    public float GetAcceleration() => currentAcceleration;
    public float GetSteeringInput() => steeringInput;
    public float GetThrottleInput() => appliedThrottleInput;
    public float GetBrakeInput() => appliedBrakeInput;
    public float GetSteeringAngle() => currentSteeringAngle;
    public float GetAppliedDriveTorque() => currentAppliedDriveTorque;
    public float GetAppliedBrakeTorque() => currentAppliedBrakeTorque;

    public void SetThrottle(float value)
    {
        rawThrottleInput = Mathf.Clamp(value, -1f, 1f);
    }

    public void SetBrake(float value)
    {
        rawBrakeInput = Mathf.Clamp01(value);
    }

    public void SetSteering(float value)
    {
        steeringInput = Mathf.Clamp(value, -1f, 1f);
    }
}
