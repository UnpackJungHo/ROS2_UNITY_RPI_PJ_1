using UnityEngine;

/// <summary>
/// Rigidbody + WheelCollider 기반 차량 동역학.
/// VehicleMotionController가 public facade 역할을 맡고,
/// 이 컴포넌트는 실제 차체 이동/조향/제동/토크 적용만 담당한다.
/// </summary>
[DisallowMultipleComponent]
public class WheelColliderVehicleDynamics : MonoBehaviour
{
    private const float DefaultWheelMass = 5f;

    private VehicleMotionController owner;
    private Rigidbody chassisBody;
    private WheelCollider frontLeftCollider;
    private WheelCollider frontRightCollider;
    private WheelCollider rearLeftCollider;
    private WheelCollider rearRightCollider;

    private float lastSignedSpeed;
    private float currentAcceleration;
    private float currentMotorRPM;
    private float currentAppliedTotalDriveTorque;
    private float currentAppliedTotalBrakeTorque;

    public void Initialize(
        VehicleMotionController controller,
        Vector3 spawnPosition,
        Quaternion spawnRotation,
        Bounds chassisBounds,
        Vector3 centerOfMassOffset,
        Vector3 frontLeftLocalMount,
        Vector3 frontRightLocalMount,
        Vector3 rearLeftLocalMount,
        Vector3 rearRightLocalMount)
    {
        owner = controller;

        transform.SetPositionAndRotation(spawnPosition, spawnRotation);

        chassisBody = GetComponent<Rigidbody>();
        if (chassisBody == null)
            chassisBody = gameObject.AddComponent<Rigidbody>();

        chassisBody.mass = controller.vehicleMass;
        chassisBody.centerOfMass = centerOfMassOffset;
        chassisBody.interpolation = RigidbodyInterpolation.Interpolate;
        chassisBody.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        chassisBody.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        chassisBody.drag = 0f;
        chassisBody.angularDrag = controller.hybridAngularDrag;

        RemoveRuntimeChassisColliders();

        frontLeftCollider = EnsureWheelCollider("FrontLeftWheelCollider", frontLeftLocalMount);
        frontRightCollider = EnsureWheelCollider("FrontRightWheelCollider", frontRightLocalMount);
        rearLeftCollider = EnsureWheelCollider("RearLeftWheelCollider", rearLeftLocalMount);
        rearRightCollider = EnsureWheelCollider("RearRightWheelCollider", rearRightLocalMount);

        ConfigureWheel(frontLeftCollider, controller, true);
        ConfigureWheel(frontRightCollider, controller, true);
        ConfigureWheel(rearLeftCollider, controller, false);
        ConfigureWheel(rearRightCollider, controller, false);

        lastSignedSpeed = 0f;
        currentAcceleration = 0f;
        currentMotorRPM = 0f;
        currentAppliedTotalDriveTorque = 0f;
        currentAppliedTotalBrakeTorque = 0f;
    }

    public void Step(float leftSteerAngle, float rightSteerAngle, float throttleInput, float brakeInput)
    {
        if (chassisBody == null)
            return;

        frontLeftCollider.steerAngle = leftSteerAngle;
        frontRightCollider.steerAngle = rightSteerAngle;

        float signedSpeed = GetSignedPlanarSpeed();
        float perWheelMotorTorque = CalculatePerWheelMotorTorque(throttleInput, brakeInput, signedSpeed);
        float frontMotorTorque = owner != null && owner.driveFrontAxle ? perWheelMotorTorque : 0f;
        float rearMotorTorque = owner != null && owner.driveRearAxle ? perWheelMotorTorque : 0f;
        float brakeTorque = CalculatePerWheelBrakeTorque(brakeInput);
        bool holdAtRest = ShouldHoldVehicleAtRest(brakeInput, signedSpeed);

        if (holdAtRest)
        {
            currentAppliedTotalDriveTorque = 0f;
            currentAppliedTotalBrakeTorque = Mathf.Max(currentAppliedTotalBrakeTorque, owner.maxBrakeForce * owner.wheelRadius);
            frontMotorTorque = 0f;
            rearMotorTorque = 0f;
            brakeTorque = Mathf.Max(brakeTorque, currentAppliedTotalBrakeTorque * 0.25f);
            HoldVehicleAtRest();
            signedSpeed = 0f;
        }

        ApplyDrive(frontLeftCollider, frontMotorTorque, brakeTorque);
        ApplyDrive(frontRightCollider, frontMotorTorque, brakeTorque);
        ApplyDrive(rearLeftCollider, rearMotorTorque, brakeTorque);
        ApplyDrive(rearRightCollider, rearMotorTorque, brakeTorque);

        currentMotorRPM = GetAverageWheelRPM() * owner.reductionRatio;
        ApplyPassiveResistance(throttleInput, brakeInput);
        SnapVehicleToRestIfNeeded(throttleInput, brakeInput);
        currentAcceleration = (signedSpeed - lastSignedSpeed) / Mathf.Max(Time.fixedDeltaTime, 1e-4f);
        lastSignedSpeed = signedSpeed;
    }

    public void Teleport(Vector3 position, Quaternion rotation)
    {
        if (chassisBody == null)
            return;

        chassisBody.position = position;
        chassisBody.rotation = rotation;
        chassisBody.velocity = Vector3.zero;
        chassisBody.angularVelocity = Vector3.zero;
        lastSignedSpeed = 0f;
        currentAcceleration = 0f;
        currentMotorRPM = 0f;
        currentAppliedTotalDriveTorque = 0f;
        currentAppliedTotalBrakeTorque = 0f;
    }

    public void ResetVelocities()
    {
        if (chassisBody == null)
            return;

        chassisBody.velocity = Vector3.zero;
        chassisBody.angularVelocity = Vector3.zero;
        lastSignedSpeed = 0f;
        currentAcceleration = 0f;
        currentAppliedTotalDriveTorque = 0f;
        currentAppliedTotalBrakeTorque = 0f;
    }

    public Rigidbody GetRigidbody() => chassisBody;
    public GameObject GetCollisionSource() => gameObject;
    public Vector3 GetWorldVelocity() => chassisBody != null ? chassisBody.velocity : Vector3.zero;
    public Vector3 GetWorldAngularVelocity() => chassisBody != null ? chassisBody.angularVelocity : Vector3.zero;
    public float GetSignedPlanarSpeed()
    {
        if (chassisBody == null)
            return 0f;

        Vector3 planarVelocity = chassisBody.velocity;
        planarVelocity.y = 0f;

        float magnitude = planarVelocity.magnitude;
        if (magnitude < 1e-4f)
            return 0f;

        float forwardComponent = Vector3.Dot(planarVelocity, transform.forward);
        if (Mathf.Abs(forwardComponent) < 1e-4f)
        {
            float sign = Mathf.Sign(owner != null ? owner.GetThrottleInput() : 0f);
            if (Mathf.Abs(sign) < 0.5f)
                sign = 1f;
            return magnitude * sign;
        }

        return magnitude * Mathf.Sign(forwardComponent);
    }

    public float GetPlanarSpeedMagnitude()
    {
        Vector3 planarVelocity = GetWorldVelocity();
        planarVelocity.y = 0f;
        return planarVelocity.magnitude;
    }

    public float GetAcceleration() => currentAcceleration;
    public float GetMotorRPM() => Mathf.Clamp(currentMotorRPM, 0f, owner != null ? owner.maxMotorRPM : currentMotorRPM);
    public float GetAppliedDriveTorque() => currentAppliedTotalDriveTorque;
    public float GetAppliedBrakeTorque() => currentAppliedTotalBrakeTorque;

    WheelCollider EnsureWheelCollider(string name, Vector3 localMount)
    {
        Transform child = transform.Find(name);
        if (child == null)
        {
            child = new GameObject(name).transform;
            child.SetParent(transform, false);
        }

        child.localPosition = localMount;
        child.localRotation = Quaternion.identity;

        WheelCollider collider = child.GetComponent<WheelCollider>();
        if (collider == null)
            collider = child.gameObject.AddComponent<WheelCollider>();

        return collider;
    }

    void RemoveRuntimeChassisColliders()
    {
        BoxCollider[] bodyColliders = GetComponents<BoxCollider>();
        for (int i = 0; i < bodyColliders.Length; i++)
        {
            if (bodyColliders[i] == null)
                continue;

            bodyColliders[i].enabled = false;

            if (Application.isPlaying)
                Destroy(bodyColliders[i]);
            else
                DestroyImmediate(bodyColliders[i]);
        }
    }

    void ConfigureWheel(WheelCollider collider, VehicleMotionController controller, bool isFront)
    {
        collider.mass = DefaultWheelMass;
        collider.radius = controller.wheelRadius;
        collider.suspensionDistance = controller.suspensionDistance;
        collider.wheelDampingRate = controller.wheelDampingRate;
        collider.forceAppPointDistance = -controller.wheelRadius * 0.5f;
        collider.ConfigureVehicleSubsteps(5f, 12, 15);

        JointSpring spring = collider.suspensionSpring;
        spring.spring = controller.suspensionSpring;
        spring.damper = controller.suspensionDamper;
        spring.targetPosition = controller.suspensionTargetPosition;
        collider.suspensionSpring = spring;

        WheelFrictionCurve forward = collider.forwardFriction;
        forward.extremumSlip = controller.forwardExtremumSlip;
        forward.extremumValue = controller.forwardExtremumValue;
        forward.asymptoteSlip = controller.forwardAsymptoteSlip;
        forward.asymptoteValue = controller.forwardAsymptoteValue;
        forward.stiffness = controller.forwardStiffness;
        collider.forwardFriction = forward;

        WheelFrictionCurve sideways = collider.sidewaysFriction;
        sideways.extremumSlip = controller.sidewaysExtremumSlip;
        sideways.extremumValue = controller.sidewaysExtremumValue;
        sideways.asymptoteSlip = controller.sidewaysAsymptoteSlip;
        sideways.asymptoteValue = controller.sidewaysAsymptoteValue;
        sideways.stiffness = isFront ? controller.frontSidewaysStiffness : controller.rearSidewaysStiffness;
        collider.sidewaysFriction = sideways;
    }

    void ApplyDrive(WheelCollider collider, float motorTorque, float brakeTorque)
    {
        collider.motorTorque = motorTorque;
        collider.brakeTorque = brakeTorque;
    }

    float CalculatePerWheelMotorTorque(float throttleInput, float brakeInput, float signedSpeed)
    {
        if (owner == null)
            return 0f;

        int drivenWheelCount = GetDrivenWheelCount();
        if (drivenWheelCount <= 0)
        {
            currentAppliedTotalDriveTorque = 0f;
            return 0f;
        }

        float clampedThrottle = Mathf.Clamp(throttleInput, -1f, 1f);
        float clampedBrake = Mathf.Clamp01(brakeInput);
        if (Mathf.Abs(clampedThrottle) < 0.001f || clampedBrake > 0.01f)
        {
            currentAppliedTotalDriveTorque = MoveDriveTorqueTowards(0f);
            return currentAppliedTotalDriveTorque / drivenWheelCount;
        }

        float rpmRatio = Mathf.Clamp01(GetAverageWheelRPM() * owner.reductionRatio / Mathf.Max(owner.maxMotorRPM, 1f));
        float torqueMultiplier = owner.torqueCurve != null && owner.torqueCurve.keys.Length > 0
            ? owner.torqueCurve.Evaluate(rpmRatio)
            : 1f;

        float maxAvailableTotalTorque = owner.maxMotorTorque * owner.reductionRatio * torqueMultiplier;
        float signedTargetTotalDriveTorque = Mathf.Sign(clampedThrottle) * Mathf.Abs(clampedThrottle) * maxAvailableTotalTorque;

        bool pushingSameDirection = Mathf.Abs(signedSpeed) > 0.05f && Mathf.Sign(clampedThrottle) == Mathf.Sign(signedSpeed);
        if (pushingSameDirection && Mathf.Abs(signedSpeed) >= owner.maxSpeed)
            signedTargetTotalDriveTorque = 0f;

        currentAppliedTotalDriveTorque = MoveDriveTorqueTowards(signedTargetTotalDriveTorque);
        return currentAppliedTotalDriveTorque / drivenWheelCount;
    }

    float CalculatePerWheelBrakeTorque(float brakeInput)
    {
        if (owner == null)
            return 0f;

        float requestedBrakeTorque = owner.maxBrakeForce * owner.wheelRadius * Mathf.Clamp01(brakeInput);
        currentAppliedTotalBrakeTorque = requestedBrakeTorque;
        return requestedBrakeTorque * 0.25f;
    }

    float GetAverageWheelRPM()
    {
        return (
            Mathf.Abs(frontLeftCollider.rpm) +
            Mathf.Abs(frontRightCollider.rpm) +
            Mathf.Abs(rearLeftCollider.rpm) +
            Mathf.Abs(rearRightCollider.rpm)) * 0.25f;
    }

    int GetDrivenWheelCount()
    {
        int drivenWheelCount = 0;
        if (owner != null && owner.driveFrontAxle)
            drivenWheelCount += 2;
        if (owner != null && owner.driveRearAxle)
            drivenWheelCount += 2;
        return drivenWheelCount;
    }

    float MoveDriveTorqueTowards(float signedTargetTotalDriveTorque)
    {
        if (owner == null)
            return 0f;

        float sameDirectionIncreaseStep = owner.vehicleMass * owner.maxAcceleration * owner.wheelRadius * Time.fixedDeltaTime;
        float releaseStep = owner.vehicleMass * owner.maxDeceleration * owner.wheelRadius * Time.fixedDeltaTime;

        bool sameDirection = Mathf.Sign(signedTargetTotalDriveTorque) ==
                             Mathf.Sign(currentAppliedTotalDriveTorque == 0f ? signedTargetTotalDriveTorque : currentAppliedTotalDriveTorque);
        bool acceleratingMagnitude = Mathf.Abs(signedTargetTotalDriveTorque) > Mathf.Abs(currentAppliedTotalDriveTorque);
        float torqueStep = sameDirection && acceleratingMagnitude ? sameDirectionIncreaseStep : releaseStep;

        return Mathf.MoveTowards(
            currentAppliedTotalDriveTorque,
            signedTargetTotalDriveTorque,
            Mathf.Max(torqueStep, 1e-4f));
    }

    void ApplyPassiveResistance(float throttleInput, float brakeInput)
    {
        if (owner == null || chassisBody == null)
            return;

        Vector3 planarVelocity = chassisBody.velocity;
        planarVelocity.y = 0f;

        float speed = planarVelocity.magnitude;
        if (speed < 1e-4f)
            return;

        Vector3 direction = planarVelocity / speed;
        float totalForce = EstimatePassiveResistanceForce(speed, Mathf.Abs(throttleInput), brakeInput);
        chassisBody.AddForce(-direction * totalForce, ForceMode.Force);
    }

    float EstimatePassiveResistanceForce(float speed, float throttleMagnitude, float brakeInput)
    {
        if (owner == null)
            return 0f;

        float rollingForce = owner.rollingResistance * owner.vehicleMass * 9.81f;
        float dragForce = 0.5f * owner.airDensity * owner.dragCoefficient * owner.frontalArea * speed * speed;
        float coastFactor = brakeInput > 0.01f ? 0f : 1f - Mathf.Clamp01(throttleMagnitude);
        float regenForce = owner.engineBrakeForce * coastFactor;
        float coastBrakeForce = owner.vehicleMass * owner.maxAcceleration * coastFactor;
        return rollingForce + dragForce + regenForce + coastBrakeForce;
    }

    bool ShouldHoldVehicleAtRest(float brakeInput, float signedSpeed)
    {
        if (owner == null)
            return false;

        return brakeInput >= 0.1f && Mathf.Abs(signedSpeed) <= owner.standstillHoldSpeed;
    }

    void HoldVehicleAtRest()
    {
        if (chassisBody == null)
            return;

        chassisBody.velocity = Vector3.zero;
        chassisBody.angularVelocity = Vector3.zero;
    }

    void SnapVehicleToRestIfNeeded(float throttleInput, float brakeInput)
    {
        if (owner == null || chassisBody == null)
            return;

        if (Mathf.Abs(throttleInput) > 0.01f || brakeInput > 0.01f)
            return;

        float signedSpeed = GetSignedPlanarSpeed();
        if (Mathf.Abs(signedSpeed) > owner.coastStopSpeed)
            return;

        if (Mathf.Abs(currentAppliedTotalDriveTorque) > 0.05f)
            return;

        chassisBody.velocity = Vector3.zero;
        chassisBody.angularVelocity = Vector3.zero;
    }
}
