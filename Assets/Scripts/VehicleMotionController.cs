using UnityEngine;
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

    [Header("Articulation Backend")]
    [Tooltip("차량 내부 collider끼리의 충돌을 런타임에 무시한다.")]
    public bool ignoreVehicleSelfCollisions = true;
    [Tooltip("조향 articulation의 stiffness")]
    public float steeringDriveStiffness = 12000f;
    [Tooltip("조향 articulation의 damping")]
    public float steeringDriveDamping = 1200f;
    [Tooltip("조향 articulation의 최대 힘")]
    public float steeringDriveForceLimit = 400f;
    [Tooltip("주행 중 휠 articulation damping")]
    public float wheelDriveDamping = 35f;
    [Tooltip("브레이크 중 휠 articulation damping")]
    public float wheelBrakeDamping = 120f;
    [Tooltip("coast 상태 휠 articulation damping")]
    public float wheelCoastDamping = 2f;
    [Tooltip("휠 joint friction")]
    public float wheelJointFriction = 0.02f;
    [Tooltip("base_link articulation linear damping")]
    public float articulationLinearDamping = 0.05f;
    [Tooltip("base_link articulation angular damping")]
    public float articulationAngularDamping = 0.05f;
    [Tooltip("스로틀 해제 시 목표 구동 속도를 줄이는 감속도 (m/s²)")]
    public float articulationCoastDeceleration = 0.45f;
    [Tooltip("조향 articulation target 부호 반전")]
    public bool invertSteeringDirection = true;

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

    private ArticulationBody baseLinkBody;
    private ArticulationBody[] articulationBodies;
    private Collider[] vehicleColliders;
    private bool runtimePrepared;
    private bool articulationBackendReady;
    private bool articulationRuntimeControlsSuppressed;
    private int articulationSuppressAttempts;
    private float lastSignedSpeed;
    private float commandedDriveSpeed_ms;

    void Awake()
    {
        PrepareRuntime();
    }

    void Start()
    {
        PrepareRuntime();
        InitializeBackendIfNeeded();
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
    }

    void InitializeBackendIfNeeded()
    {
        if (articulationBackendReady)
            return;

        InitializeArticulationBackend();
    }

    void Update()
    {
        if (!externalControlEnabled)
        {
            steeringInput = ApplyDeadzone(Input.GetAxis("Horizontal"), steeringDeadzone);
            UpdateKeyboardLongitudinalInputs();
        }
    }

    float ApplyDeadzone(float value, float deadzone)
    {
        return Mathf.Abs(value) < deadzone ? 0f : value;
    }

    void UpdateKeyboardLongitudinalInputs()
    {
        float vertical = ApplyDeadzone(Input.GetAxis("Vertical"), throttleDeadzone);
        bool hardBrakeRequested = Input.GetKey(KeyCode.Space);

        rawThrottleInput = 0f;
        rawBrakeInput = 0f;

        if (hardBrakeRequested)
        {
            rawBrakeInput = 1f;
            return;
        }

        if (Mathf.Abs(vertical) < 0.001f)
            return;

        float longitudinalSpeed = GetLongitudinalSpeedForDirectionChange();
        if (ShouldBrakeForDirectionChange(vertical, longitudinalSpeed))
        {
            rawBrakeInput = Mathf.Abs(vertical);
            return;
        }

        rawThrottleInput = vertical;
    }

    float GetLongitudinalSpeedForDirectionChange()
    {
        if (baseLinkBody == null)
            return currentSpeed_ms;

        Vector3 planarVelocity = baseLinkBody.velocity;
        planarVelocity.y = 0f;
        return Vector3.Dot(planarVelocity, transform.forward);
    }

    bool ShouldBrakeForDirectionChange(float desiredLongitudinalInput, float longitudinalSpeed)
    {
        if (Mathf.Abs(desiredLongitudinalInput) < 0.001f)
            return false;

        if (Mathf.Abs(longitudinalSpeed) <= standstillHoldSpeed)
            return false;

        return Mathf.Sign(desiredLongitudinalInput) != Mathf.Sign(longitudinalSpeed);
    }

    void FixedUpdate()
    {
        UpdateLongitudinalInputs();
        UpdateSteering(steeringInput);
        StepArticulationBackend();
        SyncDebugState();
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

    void InitializeArticulationBackend()
    {
        baseLinkBody = GetComponent<ArticulationBody>();
        articulationBodies = transform.root.GetComponentsInChildren<ArticulationBody>(true);
        vehicleColliders = transform.root.GetComponentsInChildren<Collider>(true);
        articulationBackendReady = HasCompleteArticulationReferences();

        if (!articulationBackendReady)
        {
            Debug.LogError("[VehicleMotionController] Articulation backend references are incomplete.");
            return;
        }

        ConfigureBaseLinkBody();
        ConfigureAllArticulationJoints();

        if (ignoreVehicleSelfCollisions)
            IgnoreVehicleSelfCollisions();

        lastSignedSpeed = GetArticulationSignedPlanarSpeed();
        ResetArticulationTelemetry();
    }

    bool HasCompleteArticulationReferences()
    {
        return
            baseLinkBody != null &&
            frontLeftSteering != null &&
            frontRightSteering != null &&
            frontLeftWheel != null &&
            frontRightWheel != null &&
            rearLeftWheel != null &&
            rearRightWheel != null;
    }

    void ConfigureAllArticulationJoints()
    {
        ConfigureSteeringJoint(frontLeftSteering);
        ConfigureSteeringJoint(frontRightSteering);
        ConfigureWheelJoint(frontLeftWheel);
        ConfigureWheelJoint(frontRightWheel);
        ConfigureWheelJoint(rearLeftWheel);
        ConfigureWheelJoint(rearRightWheel);
    }

    void ConfigureBaseLinkBody()
    {
        if (baseLinkBody == null)
            return;

        baseLinkBody.mass = vehicleMass;
        Vector3 centerOfMass = baseLinkBody.centerOfMass;
        centerOfMass.y = centerOfMassHeight;
        baseLinkBody.centerOfMass = centerOfMass;
        baseLinkBody.linearDamping = articulationLinearDamping;
        baseLinkBody.angularDamping = articulationAngularDamping;
        baseLinkBody.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
    }

    void ConfigureSteeringJoint(ArticulationBody steeringBody)
    {
        if (steeringBody == null)
            return;

        steeringBody.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        steeringBody.angularDamping = Mathf.Min(steeringBody.angularDamping, 0.2f);
        steeringBody.linearDamping = Mathf.Min(steeringBody.linearDamping, 0.2f);
        steeringBody.jointFriction = Mathf.Min(steeringBody.jointFriction, wheelJointFriction);

        ArticulationDrive drive = steeringBody.xDrive;
        drive.stiffness = steeringDriveStiffness;
        drive.damping = steeringDriveDamping;
        drive.forceLimit = Mathf.Max(drive.forceLimit, steeringDriveForceLimit);
        drive.target = 0f;
        drive.targetVelocity = 0f;
        steeringBody.xDrive = drive;
    }

    void ConfigureWheelJoint(ArticulationBody wheelBody)
    {
        if (wheelBody == null)
            return;

        wheelBody.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        wheelBody.angularDamping = Mathf.Min(wheelBody.angularDamping, 0.1f);
        wheelBody.linearDamping = Mathf.Min(wheelBody.linearDamping, 0.1f);
        wheelBody.jointFriction = Mathf.Min(wheelBody.jointFriction, wheelJointFriction);

        ArticulationDrive drive = wheelBody.xDrive;
        drive.stiffness = 0f;
        drive.damping = wheelCoastDamping;
        drive.forceLimit = 0f;
        drive.target = 0f;
        drive.targetVelocity = 0f;
        wheelBody.xDrive = drive;
    }

    void IgnoreVehicleSelfCollisions()
    {
        if (vehicleColliders == null || vehicleColliders.Length == 0)
            return;

        for (int i = 0; i < vehicleColliders.Length; i++)
        {
            Collider left = vehicleColliders[i];
            if (left == null)
                continue;

            for (int j = i + 1; j < vehicleColliders.Length; j++)
            {
                Collider right = vehicleColliders[j];
                if (right == null)
                    continue;

                Physics.IgnoreCollision(left, right, true);
            }
        }
    }

    void EnsureArticulationControlOwnership()
    {
        if (articulationRuntimeControlsSuppressed)
            return;

        articulationSuppressAttempts++;

        bool foundRuntimeController = false;
        MonoBehaviour[] behaviours = transform.root.GetComponentsInChildren<MonoBehaviour>(true);
        for (int i = 0; i < behaviours.Length; i++)
        {
            MonoBehaviour behaviour = behaviours[i];
            if (behaviour == null)
                continue;

            string typeName = behaviour.GetType().Name;
            if (typeName != "JointControl" && typeName != "Controller")
                continue;

            foundRuntimeController = true;
            behaviour.enabled = false;
        }

        if (!foundRuntimeController && articulationSuppressAttempts < 8)
            return;

        articulationRuntimeControlsSuppressed = true;

        ConfigureBaseLinkBody();
        ConfigureAllArticulationJoints();
    }

    void StepArticulationBackend()
    {
        if (!articulationBackendReady || baseLinkBody == null)
            return;

        EnsureArticulationControlOwnership();

        ApplySteeringTarget(frontLeftSteering, leftVisualSteerAngle);
        ApplySteeringTarget(frontRightSteering, rightVisualSteerAngle);

        float signedSpeed = GetArticulationSignedPlanarSpeed();
        UpdateCommandedDriveSpeed(appliedThrottleInput, appliedBrakeInput);
        float perWheelDriveTorque = CalculateArticulationPerWheelDriveTorque(appliedThrottleInput, appliedBrakeInput, signedSpeed);
        float perWheelBrakeTorque = CalculatePerWheelBrakeTorque(appliedBrakeInput);
        float targetWheelVelocityDeg = CalculateWheelTargetVelocityDeg(commandedDriveSpeed_ms);

        ApplyWheelDriveToAxle(frontLeftWheel, frontRightWheel, driveFrontAxle, targetWheelVelocityDeg, perWheelDriveTorque, perWheelBrakeTorque);
        ApplyWheelDriveToAxle(rearLeftWheel, rearRightWheel, driveRearAxle, targetWheelVelocityDeg, perWheelDriveTorque, perWheelBrakeTorque);

        ApplyArticulationPassiveResistance(appliedThrottleInput, appliedBrakeInput);

        if (ShouldHoldVehicleAtRest(appliedBrakeInput, signedSpeed))
        {
            HoldArticulationAtRest();
            signedSpeed = 0f;
        }

        currentMotorRPM = EstimateArticulationMotorRPM();
        currentAcceleration = (signedSpeed - lastSignedSpeed) / Mathf.Max(Time.fixedDeltaTime, 1e-4f);
        lastSignedSpeed = signedSpeed;
    }

    void ApplySteeringTarget(ArticulationBody steeringBody, float targetAngle)
    {
        if (steeringBody == null)
            return;

        ArticulationDrive drive = steeringBody.xDrive;
        drive.target = invertSteeringDirection ? -targetAngle : targetAngle;
        drive.targetVelocity = 0f;
        steeringBody.xDrive = drive;
    }

    void ApplyWheelDrive(ArticulationBody wheelBody, float targetVelocityDeg, float driveForceLimit, float brakeTorque)
    {
        if (wheelBody == null)
            return;

        ArticulationDrive drive = wheelBody.xDrive;
        drive.stiffness = 0f;
        drive.target = 0f;

        if (brakeTorque > 0.01f)
        {
            float brakeRatio = GetBrakeTorqueRatio(brakeTorque);
            drive.targetVelocity = 0f;
            drive.forceLimit = brakeTorque;
            drive.damping = Mathf.Lerp(wheelCoastDamping, wheelBrakeDamping, brakeRatio);
        }
        else if (driveForceLimit > 0.01f && Mathf.Abs(targetVelocityDeg) > 0.01f)
        {
            drive.targetVelocity = targetVelocityDeg;
            drive.forceLimit = driveForceLimit;
            drive.damping = wheelDriveDamping;
        }
        else
        {
            drive.targetVelocity = 0f;
            drive.forceLimit = 0f;
            drive.damping = wheelCoastDamping;
        }

        wheelBody.xDrive = drive;
    }

    void ApplyWheelDriveToAxle(
        ArticulationBody leftWheel,
        ArticulationBody rightWheel,
        bool axleEnabled,
        float targetVelocityDeg,
        float driveForceLimit,
        float brakeTorque)
    {
        float axleTargetVelocity = axleEnabled ? targetVelocityDeg : 0f;
        float axleDriveForceLimit = axleEnabled ? driveForceLimit : 0f;
        ApplyWheelDrive(leftWheel, axleTargetVelocity, axleDriveForceLimit, brakeTorque);
        ApplyWheelDrive(rightWheel, axleTargetVelocity, axleDriveForceLimit, brakeTorque);
    }

    float CalculateArticulationPerWheelDriveTorque(float throttleInput, float brakeInput, float signedSpeed)
    {
        int drivenWheelCount = GetDrivenWheelCount();
        if (drivenWheelCount <= 0)
        {
            currentAppliedDriveTorque = 0f;
            return 0f;
        }

        float clampedThrottle = Mathf.Clamp(throttleInput, -1f, 1f);
        float clampedBrake = Mathf.Clamp01(brakeInput);
        if (Mathf.Abs(clampedThrottle) < 0.001f || clampedBrake > 0.01f)
        {
            currentAppliedDriveTorque = MoveDriveTorqueTowards(0f);
            return Mathf.Abs(currentAppliedDriveTorque) / drivenWheelCount;
        }

        float rpmRatio = Mathf.Clamp01(currentMotorRPM / Mathf.Max(maxMotorRPM, 1f));
        float torqueMultiplier = torqueCurve != null && torqueCurve.keys.Length > 0
            ? torqueCurve.Evaluate(rpmRatio)
            : 1f;

        float maxAvailableTotalTorque = maxMotorTorque * reductionRatio * torqueMultiplier;
        float signedTargetTotalDriveTorque = Mathf.Sign(clampedThrottle) * Mathf.Abs(clampedThrottle) * maxAvailableTotalTorque;

        bool pushingSameDirection = Mathf.Abs(signedSpeed) > 0.05f && Mathf.Sign(clampedThrottle) == Mathf.Sign(signedSpeed);
        if (pushingSameDirection && Mathf.Abs(signedSpeed) >= maxSpeed)
            signedTargetTotalDriveTorque = 0f;

        currentAppliedDriveTorque = MoveDriveTorqueTowards(signedTargetTotalDriveTorque);
        return Mathf.Abs(currentAppliedDriveTorque) / drivenWheelCount;
    }

    float MoveDriveTorqueTowards(float signedTargetTotalDriveTorque)
    {
        float sameDirectionIncreaseStep = vehicleMass * maxAcceleration * wheelRadius * Time.fixedDeltaTime;
        float releaseStep = vehicleMass * maxDeceleration * wheelRadius * Time.fixedDeltaTime;

        bool sameDirection = Mathf.Sign(signedTargetTotalDriveTorque) ==
                             Mathf.Sign(currentAppliedDriveTorque == 0f ? signedTargetTotalDriveTorque : currentAppliedDriveTorque);
        bool acceleratingMagnitude = Mathf.Abs(signedTargetTotalDriveTorque) > Mathf.Abs(currentAppliedDriveTorque);
        float torqueStep = sameDirection && acceleratingMagnitude ? sameDirectionIncreaseStep : releaseStep;

        return Mathf.MoveTowards(
            currentAppliedDriveTorque,
            signedTargetTotalDriveTorque,
            Mathf.Max(torqueStep, 1e-4f));
    }

    float CalculatePerWheelBrakeTorque(float brakeInput)
    {
        float maxTotalBrakeTorque = GetMaxTotalBrakeTorque();
        float requestedBrakeTorque = maxTotalBrakeTorque * Mathf.Clamp01(brakeInput);
        float torqueStep = maxTotalBrakeTorque * Mathf.Max(
            requestedBrakeTorque > currentAppliedBrakeTorque ? brakeRiseRate : brakeFallRate,
            0.01f) * Time.fixedDeltaTime;

        currentAppliedBrakeTorque = Mathf.MoveTowards(
            currentAppliedBrakeTorque,
            requestedBrakeTorque,
            Mathf.Max(torqueStep, 1e-4f));

        return currentAppliedBrakeTorque * 0.25f;
    }

    float GetMaxTotalBrakeTorque()
    {
        return maxBrakeForce * wheelRadius;
    }

    float GetBrakeTorqueRatio(float perWheelBrakeTorque)
    {
        float maxPerWheelBrakeTorque = GetMaxTotalBrakeTorque() * 0.25f;
        return Mathf.Clamp01(perWheelBrakeTorque / Mathf.Max(maxPerWheelBrakeTorque, 1e-4f));
    }

    void UpdateCommandedDriveSpeed(float throttleInput, float brakeInput)
    {
        float requestedSpeed = Mathf.Clamp(throttleInput, -1f, 1f) * maxSpeed;

        float step;
        if (Mathf.Abs(throttleInput) > 0.01f)
        {
            bool sameDirection = Mathf.Sign(requestedSpeed) == Mathf.Sign(commandedDriveSpeed_ms == 0f ? requestedSpeed : commandedDriveSpeed_ms);
            step = sameDirection ? maxAcceleration : maxDeceleration;
        }
        else if (brakeInput > 0.01f)
        {
            step = Mathf.Lerp(articulationCoastDeceleration, maxDeceleration, Mathf.Clamp01(brakeInput));
        }
        else
        {
            step = articulationCoastDeceleration;
        }

        commandedDriveSpeed_ms = Mathf.MoveTowards(
            commandedDriveSpeed_ms,
            requestedSpeed,
            Mathf.Max(step, 0.01f) * Time.fixedDeltaTime);
    }

    float CalculateWheelTargetVelocityDeg(float targetLinearSpeed)
    {
        float wheelAngularSpeed = targetLinearSpeed / Mathf.Max(wheelRadius, 1e-3f);
        float maxWheelAngularSpeed = maxMotorRPM * (2f * Mathf.PI / 60f) / Mathf.Max(reductionRatio, 1e-3f);
        wheelAngularSpeed = Mathf.Clamp(wheelAngularSpeed, -maxWheelAngularSpeed, maxWheelAngularSpeed);
        return wheelAngularSpeed * Mathf.Rad2Deg;
    }

    void ApplyArticulationPassiveResistance(float throttleInput, float brakeInput)
    {
        if (baseLinkBody == null)
            return;

        Vector3 planarVelocity = baseLinkBody.velocity;
        planarVelocity.y = 0f;

        float speed = planarVelocity.magnitude;
        if (speed < 1e-4f)
            return;

        Vector3 direction = planarVelocity / speed;
        float totalForce = EstimateArticulationPassiveResistanceForce(speed, Mathf.Abs(throttleInput), brakeInput);
        baseLinkBody.AddForce(-direction * totalForce, ForceMode.Force);
    }

    float EstimateArticulationPassiveResistanceForce(float speed, float throttleMagnitude, float brakeInput)
    {
        float rollingForce = rollingResistance * vehicleMass * 9.81f;
        float dragForce = 0.5f * airDensity * dragCoefficient * frontalArea * speed * speed;
        float coastFactor = brakeInput > 0.01f ? 0f : 1f - Mathf.Clamp01(throttleMagnitude);
        float regenForce = engineBrakeForce * coastFactor;
        return rollingForce + dragForce + regenForce;
    }

    bool ShouldHoldVehicleAtRest(float brakeInput, float signedSpeed)
    {
        return brakeInput >= 0.1f && Mathf.Abs(signedSpeed) <= standstillHoldSpeed;
    }

    void HoldArticulationAtRest()
    {
        ResetArticulationVelocities();
    }

    void ResetArticulationVelocities()
    {
        if (articulationBodies == null || articulationBodies.Length == 0)
            articulationBodies = transform.root.GetComponentsInChildren<ArticulationBody>(true);

        for (int i = 0; i < articulationBodies.Length; i++)
        {
            ArticulationBody body = articulationBodies[i];
            if (body == null)
                continue;

            body.velocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
        }

        ResetArticulationTelemetry();
        ClearArticulationDriveTargets();
    }

    void ResetArticulationTelemetry()
    {
        currentAppliedDriveTorque = 0f;
        currentAppliedBrakeTorque = 0f;
        currentAcceleration = 0f;
        currentMotorRPM = 0f;
        lastSignedSpeed = 0f;
        commandedDriveSpeed_ms = 0f;
    }

    void ClearArticulationDriveTargets()
    {
        SetWheelDriveIdle(frontLeftWheel);
        SetWheelDriveIdle(frontRightWheel);
        SetWheelDriveIdle(rearLeftWheel);
        SetWheelDriveIdle(rearRightWheel);
        SetSteeringTargets(0f, 0f);
    }

    void SetWheelDriveIdle(ArticulationBody wheelBody)
    {
        if (wheelBody == null)
            return;

        ArticulationDrive drive = wheelBody.xDrive;
        drive.stiffness = 0f;
        drive.damping = wheelCoastDamping;
        drive.forceLimit = 0f;
        drive.target = 0f;
        drive.targetVelocity = 0f;
        wheelBody.xDrive = drive;
    }

    void SetSteeringTarget(ArticulationBody steeringBody, float targetAngle)
    {
        if (steeringBody == null)
            return;

        ArticulationDrive drive = steeringBody.xDrive;
        drive.target = targetAngle;
        drive.targetVelocity = 0f;
        steeringBody.xDrive = drive;
    }

    void SetSteeringTargets(float leftTargetAngle, float rightTargetAngle)
    {
        SetSteeringTarget(frontLeftSteering, leftTargetAngle);
        SetSteeringTarget(frontRightSteering, rightTargetAngle);
    }

    float EstimateArticulationMotorRPM()
    {
        float wheelRPM = Mathf.Abs(GetArticulationSignedPlanarSpeed()) / Mathf.Max(wheelRadius, 1e-3f) * Mathf.Rad2Deg / 6f;
        return wheelRPM * reductionRatio;
    }

    float GetArticulationSignedPlanarSpeed()
    {
        Vector3 worldVelocity = GetWorldVelocity();
        Vector3 planarVelocity = worldVelocity;
        planarVelocity.y = 0f;

        float magnitude = planarVelocity.magnitude;
        if (magnitude < 1e-4f)
            return 0f;

        float forwardComponent = Vector3.Dot(planarVelocity, transform.forward);
        if (Mathf.Abs(forwardComponent) < 1e-4f)
        {
            float sign = Mathf.Sign(appliedThrottleInput);
            if (Mathf.Abs(sign) < 0.5f)
                sign = 1f;
            return magnitude * sign;
        }

        return magnitude * Mathf.Sign(forwardComponent);
    }

    int GetDrivenWheelCount()
    {
        int drivenWheelCount = 0;
        if (driveFrontAxle)
            drivenWheelCount += 2;
        if (driveRearAxle)
            drivenWheelCount += 2;
        return drivenWheelCount;
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
        if (articulationBackendReady && baseLinkBody != null)
        {
            currentSpeed_ms = GetArticulationSignedPlanarSpeed();
            currentSpeed_kmh = currentSpeed_ms * 3.6f;
            currentMotorRPM = EstimateArticulationMotorRPM();
            return;
        }

        currentSpeed_ms = 0f;
        currentSpeed_kmh = 0f;
        currentMotorRPM = 0f;
        currentAcceleration = 0f;
        currentAppliedDriveTorque = 0f;
        currentAppliedBrakeTorque = 0f;
    }

    public Vector3 GetWorldVelocity() => baseLinkBody != null ? baseLinkBody.velocity : Vector3.zero;
    public Vector3 GetWorldAngularVelocity() => baseLinkBody != null ? baseLinkBody.angularVelocity : Vector3.zero;

    public void ResetVehiclePose(Vector3 position, Quaternion rotation)
    {
        if (baseLinkBody != null)
        {
            baseLinkBody.TeleportRoot(position, rotation);
            ResetArticulationVelocities();
            Physics.SyncTransforms();
            SyncDebugState();
            return;
        }

        transform.SetPositionAndRotation(position, rotation);
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
