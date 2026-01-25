using UnityEngine;

public class WheelTest : MonoBehaviour
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

    // ============================================
    // 차량 물리 파라미터 (Vehicle Physics Parameters)
    // ============================================

    [Header("Vehicle Specs")]
    [Tooltip("차량 질량 (kg)")]
    public float vehicleMass = 65f;
    [Tooltip("바퀴 반지름 (m)")]
    public float wheelRadius = 0.1f;
    [Tooltip("무게중심 높이 (m)")]
    public float centerOfMassHeight = 0.22f;

    [Header("Motor/Engine")]
    [Tooltip("최대 모터 토크 (Nm) - 안정적 주행을 위해 낮게 설정")]
    public float maxMotorTorque = 3f;
    [Tooltip("최대 모터 RPM")]
    public float maxMotorRPM = 1000f;
    [Tooltip("모터 토크 커브 (X: RPM 비율 0~1, Y: 토크 비율 0~1)")]
    public AnimationCurve torqueCurve;

    [Header("Transmission")]
    [Tooltip("각 기어의 기어비 - 단일 기어로 단순화")]
    public float[] gearRatios = { 1.5f };

    [Tooltip("최종 감속비 (차동기어)")]
    public float finalDriveRatio = 2.0f;

    [Tooltip("자동 변속 여부")]
    public bool autoTransmission = true;

    [Tooltip("기어 변속 RPM 상한")]
    public float shiftUpRPM = 800f;

    [Tooltip("기어 변속 RPM 하한")]
    public float shiftDownRPM = 300f;

    [Header("Speed & Stability Limits")]
    [Tooltip("최고 속도 제한 (m/s) - 롤오버 방지")]
    public float maxSpeed = 4.0f;
    [Tooltip("최대 가속도 제한 (m/s²) - 뒤집힘 방지, 권장: 5 이하")]
    public float maxAcceleration = 4.0f;
    [Tooltip("고속 조향 제한 활성화 - 속도가 빠를수록 조향각 감소")]
    public bool enableSpeedBasedSteeringLimit = true;
    [Tooltip("조향 제한 없는 최대 속도 (m/s) - 이 속도까지는 풀 조향 가능")]
    public float fullSteeringSpeed = 1.0f;

    [Header("Resistance Coefficients")]
    [Tooltip("공기저항 계수 (Cd)")]
    public float dragCoefficient = 0.5f;

    [Tooltip("전면 투영 면적 (m²)")]
    public float frontalArea = 0.4f;

    [Tooltip("구름저항 계수 (Cr)")]
    public float rollingResistance = 0.02f;

    [Tooltip("공기 밀도 (kg/m³)")]
    public float airDensity = 1.225f;

    [Header("Tire Physics")]
    [Tooltip("최대 마찰 계수")]
    public float peakFriction = 0.8f;
    [Tooltip("최적 슬립률 (0.1 = 10%)")]
    public float optimalSlipRatio = 0.15f;
    [Tooltip("타이어 그립 강성")]
    public float tireStiffness = 10f;

    [Header("Brakes")]
    [Tooltip("최대 브레이크 힘 (N)")]
    public float maxBrakeForce = 300f;
    [Tooltip("엔진 브레이크 힘 (N)")]
    public float engineBrakeForce = 30f;

    [Header("Auto-Find Settings")]
    public bool autoFindReferences = true;

    [Header("External Control (AI)")]
    [Tooltip("외부 제어 활성화 시 키보드 입력 무시")]
    public bool externalControlEnabled = false;

    [Header("Debug Info (Read Only)")]
    [SerializeField] private float currentSpeed_ms;      // 현재 속도 (m/s)
    [SerializeField] private float currentSpeed_kmh;     // 현재 속도 (km/h)
    [SerializeField] private float currentMotorRPM;      // 현재 모터 RPM
    [SerializeField] private int currentGear;            // 현재 기어 (0-indexed)
    [SerializeField] private float currentAcceleration;  // 현재 가속도 (m/s²)
    [SerializeField] private float currentDriveForce;    // 현재 구동력 (N)
    [SerializeField] private float currentDragForce;     // 현재 공기저항 (N)
    [SerializeField] private float currentRollResist;    // 현재 구름저항 (N)
    [SerializeField] private float frontAxleLoad;        // 앞축 하중 (N)
    [SerializeField] private float rearAxleLoad;         // 뒷축 하중 (N)
    [SerializeField] private float averageSlipRatio;     // 평균 슬립률

    // 내부 상태 변수
    private float currentSteeringAngle = 0f;
    private float throttleInput = 0f;
    private float brakeInput = 0f;
    private float steeringInput = 0f;
    private const float GRAVITY = 9.81f;

    void Start()
    {
        if (autoFindReferences)
        {
            FindReferences();
        }

        // 토크 커브 기본값 설정 (전기모터 특성: 저속에서 최대 토크)
        if (torqueCurve == null || torqueCurve.keys.Length == 0)
        {
            torqueCurve = new AnimationCurve();
            torqueCurve.AddKey(0f, 1f);      // 0 RPM: 100% 토크
            torqueCurve.AddKey(0.3f, 1f);    // 30% RPM: 100% 토크
            torqueCurve.AddKey(0.7f, 0.9f);  // 70% RPM: 90% 토크
            torqueCurve.AddKey(1f, 0.7f);    // 100% RPM: 70% 토크
        }

        // 초기 하중 분배 (정지 시 균등 분배)
        float totalWeight = vehicleMass * GRAVITY;
        frontAxleLoad = totalWeight * 0.5f;
        rearAxleLoad = totalWeight * 0.5f;

        currentGear = 0;
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
        {
            ArticulationBody ab = found.GetComponent<ArticulationBody>();
            if (ab != null)
            {
                return ab;
            }
        }
        Debug.LogWarning($"[WheelTest] Not found: {name}");
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

    void Update()
    {
        // 외부 제어 모드일 때는 키보드 입력 무시
        if (!externalControlEnabled)
        {
            // 입력 처리 (Update에서 수행)
            steeringInput = Input.GetAxis("Horizontal");
            float vertical = Input.GetAxis("Vertical");

            // 전진/후진과 브레이크 입력 분리
            if (vertical > 0)
            {
                throttleInput = vertical;
                brakeInput = 0f;
            }
            else if (vertical < 0)
            {
                // 전진 중 후진 입력 = 브레이크
                if (currentSpeed_ms > 0.5f)
                {
                    throttleInput = 0f;
                    brakeInput = -vertical;
                }
                // 정지 또는 저속 = 후진
                else
                {
                    throttleInput = vertical;  // 음수값으로 후진
                    brakeInput = 0f;
                }
            }
            else
            {
                throttleInput = 0f;
                brakeInput = 0f;
            }

            // 스페이스바 = 브레이크
            if (Input.GetKey(KeyCode.Space))
            {
                brakeInput = 1f;
                throttleInput = 0f;  // 브레이크 시 가속 해제
            }

            // 수동 변속 (Q/E 키)
            if (!autoTransmission)
            {
                if (Input.GetKeyDown(KeyCode.E) && currentGear < gearRatios.Length - 1)
                    currentGear++;
                if (Input.GetKeyDown(KeyCode.Q) && currentGear > 0)
                    currentGear--;
            }
        }

        // 조향 업데이트 (Update에서 수행 - 시각적 반응성)
        UpdateSteering(steeringInput);
    }

    void FixedUpdate()
    {
        // 물리 연산은 FixedUpdate에서 수행
        UpdateVehiclePhysics();
    }

    // ============================================
    // 핵심 물리 연산
    // ============================================

    void UpdateVehiclePhysics()
    {
        float dt = Time.fixedDeltaTime;

        // 1. 무게 이동 계산
        CalculateWeightTransfer();

        // 2. 모터 RPM 계산 (현재 속도 기반)
        CalculateMotorRPM();

        // 3. 자동 변속
        if (autoTransmission)
        {
            AutoShift();
        }

        // 4. 구동력 계산
        float driveForce = CalculateDriveForce();

        // 5. 저항력 계산
        float dragForce = CalculateDragForce();
        float rollingForce = CalculateRollingResistance();

        // 6. 브레이크력 계산
        float brakeForce = CalculateBrakeForce();

        // 7. 엔진 브레이크 (스로틀 해제 시)
        float engineBrake = 0f;
        if (Mathf.Abs(throttleInput) < 0.1f && Mathf.Abs(currentSpeed_ms) > 0.1f)
        {
            engineBrake = engineBrakeForce * Mathf.Sign(currentSpeed_ms);
        }

        // 8. 타이어 그립 한계 계산
        float maxTractionForce = CalculateMaxTractionForce();

        // 구동력을 그립 한계로 제한
        float effectiveDriveForce = Mathf.Sign(driveForce) *
            Mathf.Min(Mathf.Abs(driveForce), maxTractionForce);

        // 9. 순 힘 계산
        // 저항력들은 sign(속도) 방향으로 계산되어 있으므로, 항상 빼면 속도를 0으로 되돌림
        float netForce;
        if (Mathf.Abs(currentSpeed_ms) < 0.05f && Mathf.Abs(throttleInput) < 0.1f && brakeInput < 0.1f)
        {
            // 정지 상태 유지
            netForce = 0f;
            currentSpeed_ms = 0f;
        }
        else
        {
            // 구동력에서 모든 저항력을 뺌 (전진/후진 동일)
            netForce = effectiveDriveForce - dragForce - rollingForce - brakeForce - engineBrake;

            // 브레이크 중 속도가 거의 0이면 완전 정지
            if (brakeInput > 0.5f && Mathf.Abs(currentSpeed_ms) < 0.3f)
            {
                netForce = -currentSpeed_ms * 50f;  // 강제 정지력
            }
        }

        // 10. 가속도 계산 (F = ma → a = F/m)
        currentAcceleration = netForce / vehicleMass;

        // 11. 가속도 제한 (뒤집힘 방지)
        currentAcceleration = Mathf.Clamp(currentAcceleration, -maxAcceleration * 1.5f, maxAcceleration);

        // 12. 속도 적분 (v = v0 + a*t)
        currentSpeed_ms += currentAcceleration * dt;

        // 13. 최고속도 제한 (롤오버 방지)
        currentSpeed_ms = Mathf.Clamp(currentSpeed_ms, -maxSpeed * 0.5f, maxSpeed);

        // 디버그 정보 업데이트
        currentSpeed_kmh = currentSpeed_ms * 3.6f;
        currentDriveForce = effectiveDriveForce;
        currentDragForce = dragForce;
        currentRollResist = rollingForce;

        // 12. 슬립률 계산 및 바퀴 속도 적용
        ApplyWheelVelocities();
    }

    // ============================================
    // 무게 이동 계산
    // ============================================

    void CalculateWeightTransfer()
    {
        float totalWeight = vehicleMass * GRAVITY;
        float staticFrontLoad = totalWeight * 0.5f;
        float staticRearLoad = totalWeight * 0.5f;

        // 가속/감속에 의한 하중 이동
        // ΔW = (m × a × h) / wheelBase
        float weightTransfer = (vehicleMass * currentAcceleration * centerOfMassHeight) / wheelBase;

        // 가속 시: 뒷바퀴 하중 증가, 앞바퀴 하중 감소
        frontAxleLoad = staticFrontLoad - weightTransfer;
        rearAxleLoad = staticRearLoad + weightTransfer;

        // 하중이 음수가 되지 않도록 제한
        frontAxleLoad = Mathf.Max(frontAxleLoad, totalWeight * 0.1f);
        rearAxleLoad = Mathf.Max(rearAxleLoad, totalWeight * 0.1f);
    }

    // ============================================
    // 모터 RPM 계산
    // ============================================

    void CalculateMotorRPM()
    {
        if (gearRatios.Length == 0) return;

        // 바퀴 회전속도 (rad/s) = 차량속도 / 바퀴반지름
        float wheelAngularVelocity = Mathf.Abs(currentSpeed_ms) / wheelRadius;

        // 바퀴 RPM = (rad/s) × (60 / 2π)
        float wheelRPM = wheelAngularVelocity * 60f / (2f * Mathf.PI);

        // 모터 RPM = 바퀴 RPM × 기어비 × 최종감속비
        float gearRatio = gearRatios[currentGear];
        currentMotorRPM = wheelRPM * gearRatio * finalDriveRatio;

        // RPM 제한
        currentMotorRPM = Mathf.Clamp(currentMotorRPM, 0f, maxMotorRPM);
    }

    // ============================================
    // 자동 변속
    // ============================================

    void AutoShift()
    {
        if (gearRatios.Length <= 1) return;

        // 업시프트
        if (currentMotorRPM > shiftUpRPM && currentGear < gearRatios.Length - 1)
        {
            currentGear++;
        }
        // 다운시프트
        else if (currentMotorRPM < shiftDownRPM && currentGear > 0)
        {
            currentGear--;
        }
    }

    // ============================================
    // 구동력 계산
    // ============================================

    float CalculateDriveForce()
    {
        if (Mathf.Abs(throttleInput) < 0.01f) return 0f;
        if (gearRatios.Length == 0) return 0f;

        // 토크 커브에서 현재 RPM의 토크 비율 가져오기
        float rpmRatio = currentMotorRPM / maxMotorRPM;
        float torqueMultiplier = torqueCurve.Evaluate(rpmRatio);

        // 모터 토크 = 최대 토크 × 스로틀 × 토크커브
        float motorTorque = maxMotorTorque * Mathf.Abs(throttleInput) * torqueMultiplier;

        // 바퀴 토크 = 모터 토크 × 기어비 × 최종감속비
        float gearRatio = gearRatios[currentGear];
        float wheelTorque = motorTorque * gearRatio * finalDriveRatio;

        // 구동력 = 바퀴 토크 / 바퀴 반지름
        float driveForce = wheelTorque / wheelRadius;

        // 후진 시 음수
        if (throttleInput < 0)
        {
            driveForce = -driveForce;
        }

        return driveForce;
    }

    // ============================================
    // 공기저항 계산 (Drag Force)
    // ============================================

    float CalculateDragForce()
    {
        // F_drag = 0.5 × ρ × Cd × A × v²
        float speedSquared = currentSpeed_ms * currentSpeed_ms;
        float drag = 0.5f * airDensity * dragCoefficient * frontalArea * speedSquared;
        return drag * Mathf.Sign(currentSpeed_ms);
    }

    // ============================================
    // 구름저항 계산 (Rolling Resistance)
    // ============================================

    float CalculateRollingResistance()
    {
        // F_roll = Cr × m × g
        float rollResist = rollingResistance * vehicleMass * GRAVITY;
        return rollResist * Mathf.Sign(currentSpeed_ms);
    }

    // ============================================
    // 브레이크력 계산
    // ============================================

    float CalculateBrakeForce()
    {
        if (brakeInput < 0.01f) return 0f;

        float brake = brakeInput * maxBrakeForce;

        // 속도 방향과 반대로 작용
        return brake * Mathf.Sign(currentSpeed_ms);
    }

    // ============================================
    // 최대 견인력 계산 (타이어 그립 한계)
    // ============================================

    float CalculateMaxTractionForce()
    {
        // 4WD이므로 모든 바퀴의 하중 사용
        float totalNormalForce = frontAxleLoad + rearAxleLoad;

        // 슬립률 기반 마찰 계수 계산 (간소화된 Pacejka)
        float slipRatio = CalculateSlipRatio();
        float frictionCoeff = CalculateFrictionCoefficient(slipRatio);

        averageSlipRatio = slipRatio;

        // 최대 견인력 = 수직력 × 마찰계수
        return totalNormalForce * frictionCoeff;
    }

    // ============================================
    // 슬립률 계산
    // ============================================

    float CalculateSlipRatio()
    {
        if (Mathf.Abs(currentSpeed_ms) < 0.1f && Mathf.Abs(throttleInput) > 0.1f)
        {
            // 정지 상태에서 출발 시 높은 슬립률
            return Mathf.Abs(throttleInput) * 0.3f;
        }

        // 목표 바퀴 속도 (구동력이 만들어내려는 속도)
        float targetWheelSpeed = currentSpeed_ms;
        if (Mathf.Abs(throttleInput) > 0.1f)
        {
            // 스로틀 입력에 따른 추가 속도 요구
            float additionalSpeed = throttleInput * 2f;  // m/s
            targetWheelSpeed = currentSpeed_ms + additionalSpeed;
        }

        float actualSpeed = currentSpeed_ms;
        float maxSpeed = Mathf.Max(Mathf.Abs(targetWheelSpeed), Mathf.Abs(actualSpeed), 0.1f);

        float slip = (targetWheelSpeed - actualSpeed) / maxSpeed;
        return Mathf.Clamp(slip, -1f, 1f);
    }

    // ============================================
    // 마찰 계수 계산 (간소화된 Pacejka Magic Formula)
    // ============================================

    float CalculateFrictionCoefficient(float slipRatio)
    {
        float absSlip = Mathf.Abs(slipRatio);

        // 최적 슬립률 이하: 선형 증가
        if (absSlip <= optimalSlipRatio)
        {
            return peakFriction * (absSlip / optimalSlipRatio);
        }
        // 최적 슬립률 초과: 점진적 감소 (휠스핀 영역)
        else
        {
            float overSlip = absSlip - optimalSlipRatio;
            float dropOff = 1f - (overSlip * 0.5f);  // 슬립이 증가할수록 그립 감소
            return peakFriction * Mathf.Max(dropOff, 0.3f);  // 최소 30% 그립 유지
        }
    }

    // ============================================
    // 바퀴 속도 적용
    // ============================================

    void ApplyWheelVelocities()
    {
        // 현재 속도를 바퀴 회전속도로 변환
        // ω = v / r (rad/s)
        // deg/s = ω × (180/π)
        float wheelAngularVelocity = currentSpeed_ms / wheelRadius;
        float wheelDegPerSec = wheelAngularVelocity * Mathf.Rad2Deg;

        // 4WD: 모든 바퀴에 동일한 속도 적용
        SetWheelVelocity(frontLeftWheel, wheelDegPerSec);
        SetWheelVelocity(frontRightWheel, wheelDegPerSec);
        SetWheelVelocity(rearLeftWheel, wheelDegPerSec);
        SetWheelVelocity(rearRightWheel, wheelDegPerSec);
    }

    void SetWheelVelocity(ArticulationBody wheel, float velocity)
    {
        if (wheel == null) return;

        ArticulationDrive drive = wheel.xDrive;
        drive.stiffness = 0f;
        drive.damping = 100f;
        drive.targetVelocity = velocity;
        drive.forceLimit = 10000f;
        wheel.xDrive = drive;
    }

    // ============================================
    // 조향 (기존 로직 유지)
    // ============================================

    void UpdateSteering(float input)
    {
        // 속도 기반 조향 제한 (고속에서 롤오버 방지)
        float effectiveMaxSteeringAngle = maxSteeringAngle;
        if (enableSpeedBasedSteeringLimit)
        {
            float speed = Mathf.Abs(currentSpeed_ms);
            if (speed > fullSteeringSpeed)
            {
                // 속도가 증가할수록 최대 조향각 감소
                // fullSteeringSpeed에서 100%, maxSpeed에서 30%
                float speedRatio = (speed - fullSteeringSpeed) / (maxSpeed - fullSteeringSpeed);
                speedRatio = Mathf.Clamp01(speedRatio);
                float steeringMultiplier = Mathf.Lerp(1f, 0.3f, speedRatio);
                effectiveMaxSteeringAngle = maxSteeringAngle * steeringMultiplier;
            }
        }

        float targetAngle = -input * effectiveMaxSteeringAngle;
        currentSteeringAngle = Mathf.MoveTowards(currentSteeringAngle, targetAngle, steeringSpeed * Time.deltaTime);

        float leftAngle, rightAngle;
        CalculateAckermannAngles(currentSteeringAngle, out leftAngle, out rightAngle);

        SetSteeringAngle(frontLeftSteering, leftAngle);
        SetSteeringAngle(frontRightSteering, rightAngle);
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

        float innerRadius = turnRadius - (trackWidth / 2f);
        float outerRadius = turnRadius + (trackWidth / 2f);
        float innerAngle = Mathf.Atan(wheelBase / innerRadius) * Mathf.Rad2Deg;
        float outerAngle = Mathf.Atan(wheelBase / outerRadius) * Mathf.Rad2Deg;

        if (steerAngle > 0)
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

    void SetSteeringAngle(ArticulationBody steering, float angle)
    {
        if (steering == null) return;

        ArticulationDrive drive = steering.xDrive;
        drive.stiffness = 10000f;
        drive.damping = 1000f;
        drive.forceLimit = 10000f;
        drive.target = angle;
        steering.xDrive = drive;
    }

    // ============================================
    // 공개 API (외부 접근용)
    // ============================================

    /// <summary>
    /// 현재 속도 (m/s)
    /// </summary>
    public float GetSpeedMS() => currentSpeed_ms;

    /// <summary>
    /// 현재 속도 (km/h)
    /// </summary>
    public float GetSpeedKMH() => currentSpeed_kmh;

    /// <summary>
    /// 현재 모터 RPM
    /// </summary>
    public float GetMotorRPM() => currentMotorRPM;

    /// <summary>
    /// 현재 기어 (1-indexed for display)
    /// </summary>
    public int GetCurrentGear() => currentGear + 1;

    /// <summary>
    /// 현재 가속도 (m/s²)
    /// </summary>
    public float GetAcceleration() => currentAcceleration;

    /// <summary>
    /// 외부에서 스로틀 입력 설정 (ROS 등)
    /// </summary>
    public void SetThrottle(float value)
    {
        throttleInput = Mathf.Clamp(value, -1f, 1f);
    }

    /// <summary>
    /// 외부에서 브레이크 입력 설정
    /// </summary>
    public void SetBrake(float value)
    {
        brakeInput = Mathf.Clamp01(value);
    }

    /// <summary>
    /// 외부에서 조향 입력 설정
    /// </summary>
    public void SetSteering(float value)
    {
        steeringInput = Mathf.Clamp(value, -1f, 1f);
    }
}
