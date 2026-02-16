using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// 초음파 센서와 레이더 센서의 데이터를 융합하여 충돌 위험을 감지하고 경고를 발행하는 클래스입니다.
/// 다양한 거리 단계(주의, 경고, 위험)에 따라 위험 수준을 판단하며,
/// 현재 속도를 기반으로 충돌 예상 시간(TTC)을 계산하여 ROS 2로 전송합니다.
/// </summary>
public class CollisionWarningPublisher : MonoBehaviour
{
    [Header("ROS Settings (ROS 설정)")]
    public string warningTopicName = "/collision_warning"; // 종합 충돌 경고 토픽
    public string distanceTopicName = "/obstacle_distance"; // 가장 가까운 장애물 거리 단순 토픽
    public float publishRate = 20f; // 발행 주기 (Hz)

    [Header("Sensor Manager References (센서 관리자 참조)")]
    [Tooltip("초음파 센서 매니저 (근거리 정밀 감지 담당)")]
    public UltrasonicSensorPublisher ultrasonicManager;
    [Tooltip("레이더 센서 매니저 (중장거리 감지 담당)")]
    public RadarSensorPublisher radarManager;

    [Header("TTC 기반 임계값 (Time To Collision - 충돌까지 남은 시간)")]
    [Tooltip("인지 단계 TTC (초) - 이 시간 이하면 Awareness")]
    public float ttcAwareness = 5.0f;
    [Tooltip("주의 단계 TTC (초) - 이 시간 이하면 Caution")]
    public float ttcCaution = 3.0f;
    [Tooltip("감속 단계 TTC (초) - 이 시간 이하면 SlowDown")]
    public float ttcSlowDown = 2.0f;
    [Tooltip("경고 단계 TTC (초) - 이 시간 이하면 Warning")]
    public float ttcWarning = 1.5f;
    [Tooltip("제동 단계 TTC (초) - 이 시간 이하면 Brake")]
    public float ttcBrake = 1.0f;

    [Header("초음파 긴급정지 임계값 (속도와 무관한 최종 안전장치)")]
    [Tooltip("긴급정지 거리 (m) - 초음파 감지 시 무조건 EmergencyStop")]
    public float emergencyStopDistance = 0.3f;
    [Tooltip("긴급정지로 인정할 최소 초음파 confidence")]
    [Range(0f, 1f)]
    public float minEmergencyStopConfidence = 0.55f;
    [Tooltip("레이더 최소 안전거리 (m) - 정지 상태에서도 이 거리 이하면 경고")]
    public float radarMinSafeDistance = 0.5f;

    [Header("저속 주행 시 거리 기반 판단 (속도가 낮을 때 TTC 보완)")]
    [Tooltip("저속 판단 기준 (m/s) - 이 속도 이하면 거리 기반 판단 병행")]
    public float lowSpeedThreshold = 0.5f;
    [Tooltip("저속 시 주의 거리 (m)")]
    public float lowSpeedCautionDistance = 1.5f;
    [Tooltip("저속 시 경고 거리 (m)")]
    public float lowSpeedWarningDistance = 0.8f;

    [Header("Speed Reference (속도 참조)")]
    [Tooltip("차량 속도를 가져올 물리 컴포넌트 (base_link)")]
    public ArticulationBody velocitySource;
    public bool useVelocityForTTC = true; // TTC(충돌 예측 시간) 계산에 속도 반영 여부

    [Header("Debug (디버그)")]
    public bool showDebugInfo = true;
    [Tooltip("true면 경고 레벨/감지 소스가 변할 때만 로그 출력")]
    public bool debugOnlyOnStateChange = true;
    [Tooltip("상태 변화가 없어도 이 주기(초)마다 상태 스냅샷 로그 출력. 0 이하이면 비활성")]
    public float debugSnapshotInterval = 1.0f;
    [Tooltip("true면 초음파/레이더 채널 거리 상세를 함께 출력")]
    public bool debugIncludeSensorChannels = true;

    // 현재 상태 변수들 (Inspector 확인용 HideInInspector 해제 가능)
    [HideInInspector] public float currentMinDistance = float.PositiveInfinity; // 가장 가까운 장애물 거리
    [HideInInspector] public float currentTTC = float.PositiveInfinity; // 충돌까지 남은 시간 (초)
    [HideInInspector] public WarningLevel currentWarningLevel = WarningLevel.Safe; // 현재 위험 수준
    [HideInInspector] public string detectionSource = "None"; // 감지된 센서 종류 (Ultrasonic / Radar)
    [HideInInspector] public string detectionSensor = "None"; // 감지된 센서 위치 (FL, Front 등)

    // 위험 단계 열거형 (7단계 - TTC 기반 동적 판단)
    public enum WarningLevel
    {
        Safe = 0,           // 안전 - 정상 주행 (장애물 없음)
        Awareness = 1,      // 인지 - 장애물 존재 확인 (TTC > 5초)
        Caution = 2,        // 주의 - 속도 유지하되 주시 (TTC 3~5초)
        SlowDown = 3,       // 감속 - 점진적 속도 감소 (TTC 2~3초)
        Warning = 4,        // 경고 - 적극적 감속 (TTC 1~2초)
        Brake = 5,          // 제동 - 강한 감속 (TTC < 1초)
        EmergencyStop = 6   // 긴급정지 - 즉시 정지 (초음파 < emergencyDistance)
    }

    // 센서 데이터 모음 구조체
    public struct SensorData
    {
        public float ultrasonicFL;
        public float ultrasonicFR;
        public float ultrasonicFC;
        public float ultrasonicRL;
        public float ultrasonicRR;
        public float ultrasonicRC;
        public float ultrasonicMinFront;
        public float ultrasonicMinRear;
        public float ultrasonicClosestConfidence;
        public float radarFront;
        public float radarRear;
        public SingleUltrasonicSensor.SensorPosition ultrasonicClosest;
        public SingleRadarSensor.SensorPosition radarClosest;
    }

    public SensorData CurrentSensorData { get; private set; }

    private ROSConnection ros;
    private float publishInterval;
    private float lastPublishTime;
    private float currentSpeed = 0f;

    // 상대속도(Closing Speed) 계산을 위한 변수들
    private float previousMinDistance = float.PositiveInfinity;  // 이전 프레임의 최소 거리
    private float previousMeasureTime = 0f;                      // 이전 측정 시간
    private float closingSpeed = 0f;                             // 상대속도 (양수: 가까워짐, 음수: 멀어짐)
    private float smoothedClosingSpeed = 0f;                     // 노이즈 필터링된 상대속도
    private float lastDebugLogTime = -999f;
    private WarningLevel lastLoggedWarningLevel = WarningLevel.Safe;
    private string lastLoggedDetectionSource = "None";
    private string lastLoggedDetectionSensor = "None";
    private string debugDecisionTrace = "NotCalculated";
    private WarningLevel lastTtcLevel = WarningLevel.Safe;
    private WarningLevel lastLowSpeedLevel = WarningLevel.Safe;
    [Header("Closing Speed Settings (상대속도 계산 설정)")]
    [Tooltip("상대속도 스무딩 계수 (0~1, 낮을수록 더 부드러움)")]
    [Range(0.1f, 1.0f)]
    public float closingSpeedSmoothFactor = 0.3f;
    [Tooltip("최소 유효 상대속도 (m/s) - 이 이하는 노이즈로 간주")]
    public float minValidClosingSpeed = 0.05f;

    // ros 연결 및 퍼블리셔 및 발행 주기 초기화
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        // 배열 메시지(상세 정보)와 단일 값 메시지(가장 가까운 거리) 퍼블리셔 등록
        ros.RegisterPublisher<Float32MultiArrayMsg>(warningTopicName);
        ros.RegisterPublisher<Float32Msg>(distanceTopicName);

        // 참조되어야 할 센서 매니저 유효성 검증
        ValidateSensorReferences();

        publishInterval = 1f / publishRate;
        lastPublishTime = Time.time;

        Debug.Log($"[CollisionWarning] Initialized with Closing Speed based TTC system (Real ADAS style)");
        Debug.Log($"[CollisionWarning] TTC thresholds - Brake: {ttcBrake}s, Warning: {ttcWarning}s, SlowDown: {ttcSlowDown}s, Caution: {ttcCaution}s, Awareness: {ttcAwareness}s");
        Debug.Log($"[CollisionWarning] Emergency stop distance (Ultrasonic): {emergencyStopDistance}m | Closing speed smoothing: {closingSpeedSmoothFactor}");
    }

    // 센서 매니저 참조 유효성 검증
    void ValidateSensorReferences()
    {
        if (ultrasonicManager == null)
            Debug.LogWarning("[CollisionWarning] UltrasonicSensorPublisher가 할당되지 않았습니다.");
        if (radarManager == null)
            Debug.LogWarning("[CollisionWarning] RadarSensorPublisher가 할당되지 않았습니다.");
        if (useVelocityForTTC && velocitySource == null)
            Debug.LogWarning("[CollisionWarning] velocitySource가 할당되지 않았습니다. TTC 계산이 비활성화됩니다.");
    }

    void Update()
    {
        UpdateCurrentSpeed(); // 실시간 속도 갱신

        if (Time.time - lastPublishTime >= publishInterval)
        {
            CollectSensorData();       // 1. 모든 센서 데이터 취합
            UpdateCurrentMinDistance(); // 2. 최신 최소 거리 갱신
            CalculateClosingSpeed();    // 3. 상대속도 계산 (거리 변화율)
            CalculateWarningLevel();    // 4. 위험도 판단 (상대속도 기반 TTC 사용)
            CalculateTTC();             // 5. 최종 TTC 값 갱신 (ROS 발행용)
            PublishWarning();           // 6. ROS 메시지 전송
            lastPublishTime = Time.time;

            if (showDebugInfo)
            {
                PrintDebugInfo();
            }
        }
    }

    /// <summary>
    /// 차량의 현재 이동 속도를 갱신
    /// </summary>
    void UpdateCurrentSpeed()
    {
        if (!useVelocityForTTC || velocitySource == null)
        {
            currentSpeed = 0f;
            return;
        }

        // 차량의 실제 이동 속도(스칼라 값, m/s 단위 -> 최종 속도는 WheelTest 에서 설정한 값에 따라 달라짐)
        currentSpeed = velocitySource.velocity.magnitude;
    }

    /// <summary>
    /// 각 매니저로부터 최신 센서 데이터를 수집하여 SensorData 구조체에 저장
    /// </summary>
    void CollectSensorData()
    {
        SensorData data = new SensorData();

        if (ultrasonicManager != null)
        {
            data.ultrasonicFL = ultrasonicManager.FrontLeftDistance;
            data.ultrasonicFR = ultrasonicManager.FrontRightDistance;
            data.ultrasonicFC = ultrasonicManager.FrontCenterDistance;
            data.ultrasonicRL = ultrasonicManager.RearLeftDistance;
            data.ultrasonicRR = ultrasonicManager.RearRightDistance;
            data.ultrasonicRC = ultrasonicManager.RearCenterDistance;
            data.ultrasonicMinFront = ultrasonicManager.MinFrontDistance;
            data.ultrasonicMinRear = ultrasonicManager.MinRearDistance;
            data.ultrasonicClosestConfidence = ultrasonicManager.ClosestConfidence;
            data.ultrasonicClosest = ultrasonicManager.ClosestSensorPosition;
        }
        else
        {
            // 매니저가 없을 경우 기본값 초기화
            data.ultrasonicFL = float.PositiveInfinity;
            data.ultrasonicFR = float.PositiveInfinity;
            data.ultrasonicFC = float.PositiveInfinity;
            data.ultrasonicRL = float.PositiveInfinity;
            data.ultrasonicRR = float.PositiveInfinity;
            data.ultrasonicRC = float.PositiveInfinity;
            data.ultrasonicMinFront = float.PositiveInfinity;
            data.ultrasonicMinRear = float.PositiveInfinity;
            data.ultrasonicClosestConfidence = 0f;
        }

        if (radarManager != null)
        {
            data.radarFront = radarManager.FrontDistance;
            data.radarRear = radarManager.RearDistance;
            data.radarClosest = radarManager.ClosestSensorPosition;
        }
        else
        {
            data.radarFront = float.PositiveInfinity;
            data.radarRear = float.PositiveInfinity;
        }

        CurrentSensorData = data;
    }

    void UpdateCurrentMinDistance()
    {
        float ultrasonicMin = GetMinUltrasonicDistance();
        float radarMin = GetMinRadarDistance();
        currentMinDistance = Mathf.Min(ultrasonicMin, radarMin);
    }

    /// <summary>
    /// 상대속도(Closing Speed)를 계산
    /// 연속적인 거리 측정값의 변화율로 계산하며, 노이즈 필터링을 적용
    /// 양수: 장애물이 가까워지는 중, 음수: 장애물이 멀어지는 중
    /// </summary>
    void CalculateClosingSpeed()
    {
        float currentTime = Time.time;
        float deltaTime = currentTime - previousMeasureTime;

        // 첫 프레임이거나 시간 간격이 너무 짧으면 스킵
        if (deltaTime <= 0.001f || float.IsInfinity(previousMinDistance))
        {
            previousMinDistance = currentMinDistance;
            previousMeasureTime = currentTime;
            return;
        }

        // 현재 거리가 무한대면 (장애물 없음) 상대속도 0
        if (float.IsInfinity(currentMinDistance))
        {
            closingSpeed = 0f;
            smoothedClosingSpeed = 0f;
            previousMinDistance = currentMinDistance;
            previousMeasureTime = currentTime;
            return;
        }

        // 상대속도 계산: (이전 거리 - 현재 거리) / 시간
        // 양수 = 가까워짐 (충돌 방향), 음수 = 멀어짐 (안전 방향)
        float rawClosingSpeed = (previousMinDistance - currentMinDistance) / deltaTime;

        // 이전 거리가 무한대였으면 (새로운 장애물 감지) 차량 속도를 초기값으로 사용
        if (float.IsInfinity(previousMinDistance))
        {
            rawClosingSpeed = currentSpeed;
        }

        // 노이즈 필터링: 지수 이동 평균 (Exponential Moving Average)
        smoothedClosingSpeed = Mathf.Lerp(smoothedClosingSpeed, rawClosingSpeed, closingSpeedSmoothFactor);

        // 너무 작은 값은 노이즈로 간주하여 0 처리
        if (Mathf.Abs(smoothedClosingSpeed) < minValidClosingSpeed)
        {
            smoothedClosingSpeed = 0f;
        }

        closingSpeed = rawClosingSpeed;

        // 다음 계산을 위해 현재 값 저장
        previousMinDistance = currentMinDistance;
        previousMeasureTime = currentTime;
    }

    /// <summary>
    /// 수집된 데이터를 바탕으로 종합적인 위험 수준(WarningLevel)을 결정
    /// Layer 1: 초음파 긴급정지 판단 (최우선 - 속도와 무관)
    /// Layer 2: TTC 기반 동적 판단 (레이더 + 속도)
    /// Layer 3: 저속 시 거리 기반 보완 판단
    /// </summary>
    void CalculateWarningLevel()
    {
        currentWarningLevel = WarningLevel.Safe;
        // 초음파 OR 레이더 중으로 선정
        detectionSource = "None";
        // 정확한 센서 이름(위치) 
        detectionSensor = "None";
        debugDecisionTrace = "NoObstacle";

        // 전체 센서 중 최소 거리 계산
        float ultrasonicMin = GetMinUltrasonicDistance();
        float radarMin = GetMinRadarDistance();
        currentMinDistance = Mathf.Min(ultrasonicMin, radarMin);
        float ultrasonicClosestConfidence = CurrentSensorData.ultrasonicClosestConfidence;

        // ========== Layer 1: 초음파 긴급정지 (최우선 - 속도와 무관한 최종 안전장치) ==========
        if (ultrasonicMin <= emergencyStopDistance && ultrasonicClosestConfidence >= minEmergencyStopConfidence)
        {
            currentWarningLevel = WarningLevel.EmergencyStop;
            detectionSource = "Ultrasonic";
            detectionSensor = CurrentSensorData.ultrasonicClosest.ToString();
            debugDecisionTrace = $"Layer1 EmergencyStop (ultra={ultrasonicMin:F2}m <= {emergencyStopDistance:F2}m, conf={ultrasonicClosestConfidence:F2} >= {minEmergencyStopConfidence:F2})";
            lastTtcLevel = WarningLevel.Safe;
            lastLowSpeedLevel = WarningLevel.Safe;
            return; // 즉시 반환 - 다른 판단 불필요
        }

        // ========== Layer 2: TTC 기반 동적 판단 ==========
        WarningLevel ttcLevel = EvaluateTTCWarning();
        lastTtcLevel = ttcLevel;

        // ========== Layer 3: 저속 시 거리 기반 보완 판단 ==========
        WarningLevel lowSpeedLevel = WarningLevel.Safe;
        if (currentSpeed <= lowSpeedThreshold)
        {
            lowSpeedLevel = EvaluateLowSpeedWarning(ultrasonicMin, radarMin);
        }
        lastLowSpeedLevel = lowSpeedLevel;

        // 두 판단 중 더 높은 위험도 채택
        if (ttcLevel >= lowSpeedLevel)
        {
            currentWarningLevel = ttcLevel;
            debugDecisionTrace = $"Layer2 TTC selected (ttcLevel={ttcLevel}, lowSpeedLevel={lowSpeedLevel})";
            // TTC 판단은 주로 레이더 기반
            if (!float.IsInfinity(radarMin) && radarMin <= ultrasonicMin)
            {
                detectionSource = "Radar";
                detectionSensor = CurrentSensorData.radarClosest.ToString();
            }
            else if (!float.IsInfinity(ultrasonicMin))
            {
                detectionSource = "Ultrasonic";
                detectionSensor = CurrentSensorData.ultrasonicClosest.ToString();
            }
        }
        else
        {
            currentWarningLevel = lowSpeedLevel;
            debugDecisionTrace = $"Layer3 LowSpeed selected (ttcLevel={ttcLevel}, lowSpeedLevel={lowSpeedLevel}, speed={currentSpeed:F2}m/s <= {lowSpeedThreshold:F2}m/s)";
            // 저속 판단 시 가까운 센서 기준
            if (ultrasonicMin <= radarMin)
            {
                detectionSource = "Ultrasonic";
                detectionSensor = CurrentSensorData.ultrasonicClosest.ToString();
            }
            else
            {
                detectionSource = "Radar";
                detectionSensor = CurrentSensorData.radarClosest.ToString();
            }
        }

        // 정지 상태에서 레이더 최소 안전거리 체크
        if (currentSpeed <= 0.01f && radarMin <= radarMinSafeDistance)
        {
            if (currentWarningLevel < WarningLevel.Warning)
            {
                currentWarningLevel = WarningLevel.Warning;
                detectionSource = "Radar";
                detectionSensor = CurrentSensorData.radarClosest.ToString();
                debugDecisionTrace = $"StopState RadarGuard (speed={currentSpeed:F2}m/s, radar={radarMin:F2}m <= {radarMinSafeDistance:F2}m)";
            }
        }
    }

    /// <summary>
    /// 초음파 센서의 최소 거리를 반환
    /// </summary>
    float GetMinUltrasonicDistance()
    {
        if (ultrasonicManager == null) return float.PositiveInfinity;
        // ultrasonicManager에서 최소 거리 계산 후 반환
        return ultrasonicManager.ClosestDistance;
    }

    /// <summary>
    /// 레이더 센서의 최소 거리를 반환
    /// </summary>
    float GetMinRadarDistance()
    {
        if (radarManager == null) return float.PositiveInfinity;
        // radarManager에서 최소 거리 계산 후 반환
        return radarManager.ClosestDistance;
    }

    /// <summary>
    /// TTC(Time To Collision) 기반으로 위험도를 평가
    /// 실제 ADAS와 동일하게 상대속도(Closing Speed) 기반으로 계산
    /// - 장애물이 가까워지면: 상대속도 기반 TTC
    /// - 장애물이 멀어지면: 안전 (충돌 없음)
    /// - 장애물이 정지해 있으면: 차량 속도 기반 TTC (보수적 접근)
    /// </summary>
    WarningLevel EvaluateTTCWarning()
    {
        // 거리가 무한대면 장애물 없음
        if (float.IsInfinity(currentMinDistance))
            return WarningLevel.Safe;

        float ttc;

        // 상대속도가 유효한 경우 (가까워지는 중)
        if (smoothedClosingSpeed > minValidClosingSpeed)
        {
            // 상대속도 기반 TTC (실제 ADAS 방식)
            ttc = currentMinDistance / smoothedClosingSpeed;
        }
        // 상대속도가 0 이하 (멀어지거나 정지)
        else
        {
            // 차량이 움직이고 있고 장애물이 가까우면 보수적 판단
            if (currentSpeed > 0.01f && currentMinDistance < lowSpeedCautionDistance)
            {
                // 정지된 장애물로 가정 (차량 속도 기반)
                ttc = currentMinDistance / currentSpeed;
            }
            else
            {
                // 멀어지는 장애물 또는 안전 거리 → 충돌 없음
                return WarningLevel.Safe;
            }
        }

        // TTC가 비정상이면 안전
        if (float.IsInfinity(ttc) || float.IsNaN(ttc) || ttc < 0f)
            return WarningLevel.Safe;

        // TTC 기반 단계 판단 (낮은 TTC = 높은 위험)
        if (ttc <= ttcBrake)
            return WarningLevel.Brake;
        if (ttc <= ttcWarning)
            return WarningLevel.Warning;
        if (ttc <= ttcSlowDown)
            return WarningLevel.SlowDown;
        if (ttc <= ttcCaution)
            return WarningLevel.Caution;
        if (ttc <= ttcAwareness)
            return WarningLevel.Awareness;

        return WarningLevel.Safe;
    }

    /// <summary>
    /// 저속 또는 정지 상태에서 거리 기반으로 위험도를 평가
    /// TTC 판단이 어려운 상황에서 보완 역할
    /// </summary>
    WarningLevel EvaluateLowSpeedWarning(float ultrasonicMin, float radarMin)
    {
        float minDistance = Mathf.Min(ultrasonicMin, radarMin);

        if (float.IsInfinity(minDistance))
            return WarningLevel.Safe;

        // 긴급정지는 이미 Layer 1에서 처리됨
        // 여기서는 저속 시 경고/주의 판단
        if (minDistance <= lowSpeedWarningDistance)
            return WarningLevel.Warning;
        if (minDistance <= lowSpeedCautionDistance)
            return WarningLevel.Caution;

        return WarningLevel.Safe;
    }



    /// <summary>
    /// TTC (Time To Collision, 충돌까지 남은 시간) 계산
    /// 실제 ADAS 시스템과 동일하게 상대속도(Closing Speed) 기반으로 계산
    /// 공식: TTC = 거리 / 상대속도
    /// </summary>
    void CalculateTTC()
    {
        // 장애물이 없으면 TTC 무한대
        if (float.IsInfinity(currentMinDistance))
        {
            currentTTC = float.PositiveInfinity;
            return;
        }

        // 상대속도가 0 이하면 (멀어지거나 정지) 충돌 없음
        if (smoothedClosingSpeed <= minValidClosingSpeed)
        {
            // 멀어지는 경우: 충돌 없음 → TTC 무한대
            // 하지만 차량이 움직이고 있고 장애물이 가까우면 보수적으로 판단
            if (currentSpeed > 0.01f && currentMinDistance < lowSpeedCautionDistance)
            {
                // 정지된 장애물로 가정하고 차량 속도 기반 TTC 계산 (보수적 접근)
                currentTTC = currentMinDistance / currentSpeed;
            }
            else
            {
                currentTTC = float.PositiveInfinity;
            }
            return;
        }

        // 상대속도 기반 TTC 계산 (실제 ADAS 방식)
        currentTTC = currentMinDistance / smoothedClosingSpeed;

        // TTC가 음수가 되는 비정상 상황 방지
        if (currentTTC < 0f)
        {
            currentTTC = float.PositiveInfinity;
        }
    }

    /// <summary>
    /// 계산된 모든 정보를 ROS 메시지로 포장하여 발행
    /// </summary>
    void PublishWarning()
    {
        Float32MultiArrayMsg warningMsg = new Float32MultiArrayMsg
        {
            layout = new MultiArrayLayoutMsg
            {
                dim = new MultiArrayDimensionMsg[]
                {
                    new MultiArrayDimensionMsg
                    {
                        label = "collision_data",
                        size = 17,
                        stride = 17
                    }
                },
                data_offset = 0
            },
            data = new float[]
            {
                // 인덱스별 데이터 매핑
                float.IsInfinity(currentMinDistance) ? -1f : currentMinDistance, // 0: 최소 거리
                float.IsInfinity(currentTTC) ? -1f : currentTTC,                 // 1: TTC (상대속도 기반)
                (float)currentWarningLevel,                                      // 2: 위험 등급 (0-6)
                currentSpeed,                                                    // 3: 차량 속도 (m/s)
                smoothedClosingSpeed,                                            // 4: 상대속도 (m/s, 양수=접근, 음수=이탈)
                float.IsInfinity(CurrentSensorData.ultrasonicFL) ? -1f : CurrentSensorData.ultrasonicFL, // 5-10: 초음파 개별 거리 (6채널)
                float.IsInfinity(CurrentSensorData.ultrasonicFR) ? -1f : CurrentSensorData.ultrasonicFR,
                float.IsInfinity(CurrentSensorData.ultrasonicFC) ? -1f : CurrentSensorData.ultrasonicFC,
                float.IsInfinity(CurrentSensorData.ultrasonicRL) ? -1f : CurrentSensorData.ultrasonicRL,
                float.IsInfinity(CurrentSensorData.ultrasonicRR) ? -1f : CurrentSensorData.ultrasonicRR,
                float.IsInfinity(CurrentSensorData.ultrasonicRC) ? -1f : CurrentSensorData.ultrasonicRC,
                float.IsInfinity(CurrentSensorData.radarFront) ? -1f : CurrentSensorData.radarFront,     // 11-12: 레이더 전/후 거리
                float.IsInfinity(CurrentSensorData.radarRear) ? -1f : CurrentSensorData.radarRear,
                detectionSource == "Ultrasonic" ? 1f : (detectionSource == "Radar" ? 2f : 0f),           // 13: 감지 센서 소스 (1:초음파, 2:레이더)
                (float)CurrentSensorData.ultrasonicClosest,                      // 14: 가장 가까운 초음파 센서 ID
                (float)CurrentSensorData.radarClosest,                           // 15: 가장 가까운 레이더 센서 ID
                CurrentSensorData.ultrasonicClosestConfidence                    // 16: 가장 가까운 초음파 confidence
            }
        };

        ros.Publish(warningTopicName, warningMsg);

        // 거리 정보만 별도로 간소화된 토픽으로도 발행
        Float32Msg distanceMsg = new Float32Msg
        {
            data = float.IsInfinity(currentMinDistance) ? -1f : currentMinDistance
        };
        ros.Publish(distanceTopicName, distanceMsg);
    }

    void PrintDebugInfo()
    {
        bool hasStateChanged = currentWarningLevel != lastLoggedWarningLevel ||
                               detectionSource != lastLoggedDetectionSource ||
                               detectionSensor != lastLoggedDetectionSensor;
        bool hasSnapshotIntervalElapsed = debugSnapshotInterval > 0f && (Time.time - lastDebugLogTime) >= debugSnapshotInterval;

        if (debugOnlyOnStateChange && !hasStateChanged && !hasSnapshotIntervalElapsed)
            return;

        string levelStr = currentWarningLevel switch
        {
            WarningLevel.Safe => "SAFE",
            WarningLevel.Awareness => "AWARE",
            WarningLevel.Caution => "CAUTION",
            WarningLevel.SlowDown => "SLOW DOWN",
            WarningLevel.Warning => "WARNING",
            WarningLevel.Brake => "!! BRAKE !!",
            WarningLevel.EmergencyStop => "!!! EMERGENCY STOP !!!",
            _ => "UNKNOWN"
        };

        string distStr = float.IsInfinity(currentMinDistance) ? "∞" : $"{currentMinDistance:F2}m";
        string ttcStr = float.IsInfinity(currentTTC) ? "∞" : $"{currentTTC:F2}s";

        // 상대속도 방향 표시
        string closingSpeedStr;
        if (Mathf.Abs(smoothedClosingSpeed) < minValidClosingSpeed)
            closingSpeedStr = "0.00m/s (정지)";
        else if (smoothedClosingSpeed > 0)
            closingSpeedStr = $"+{smoothedClosingSpeed:F2}m/s (접근)";
        else
            closingSpeedStr = $"{smoothedClosingSpeed:F2}m/s (이탈)";

        float ultrasonicMin = GetMinUltrasonicDistance();
        float radarMin = GetMinRadarDistance();
        string ultrasonicMinStr = float.IsInfinity(ultrasonicMin) ? "∞" : $"{ultrasonicMin:F2}m";
        string radarMinStr = float.IsInfinity(radarMin) ? "∞" : $"{radarMin:F2}m";

        Debug.Log(
            $"[Collision] Level={levelStr}({(int)currentWarningLevel}) | Dist={distStr} | TTC={ttcStr} | Ego={currentSpeed:F2}m/s | Closing={closingSpeedStr} | " +
            $"ULMin={ultrasonicMinStr} | RadarMin={radarMinStr} | Closest={detectionSource}-{detectionSensor} | UltraConf={CurrentSensorData.ultrasonicClosestConfidence:F2} | " +
            $"TTCLevel={lastTtcLevel} | LowSpeedLevel={lastLowSpeedLevel} | Decision={debugDecisionTrace}"
        );

        if (debugIncludeSensorChannels)
        {
            string usFL = float.IsInfinity(CurrentSensorData.ultrasonicFL) ? "∞" : $"{CurrentSensorData.ultrasonicFL:F2}";
            string usFR = float.IsInfinity(CurrentSensorData.ultrasonicFR) ? "∞" : $"{CurrentSensorData.ultrasonicFR:F2}";
            string usFC = float.IsInfinity(CurrentSensorData.ultrasonicFC) ? "∞" : $"{CurrentSensorData.ultrasonicFC:F2}";
            string usRL = float.IsInfinity(CurrentSensorData.ultrasonicRL) ? "∞" : $"{CurrentSensorData.ultrasonicRL:F2}";
            string usRR = float.IsInfinity(CurrentSensorData.ultrasonicRR) ? "∞" : $"{CurrentSensorData.ultrasonicRR:F2}";
            string usRC = float.IsInfinity(CurrentSensorData.ultrasonicRC) ? "∞" : $"{CurrentSensorData.ultrasonicRC:F2}";
            string rdFront = float.IsInfinity(CurrentSensorData.radarFront) ? "∞" : $"{CurrentSensorData.radarFront:F2}";
            string rdRear = float.IsInfinity(CurrentSensorData.radarRear) ? "∞" : $"{CurrentSensorData.radarRear:F2}";

            Debug.Log(
                $"[CollisionSensors] US[FL={usFL}, FR={usFR}, FC={usFC}, RL={usRL}, RR={usRR}, RC={usRC}] | " +
                $"Radar[F={rdFront}, R={rdRear}]"
            );
        }

        lastLoggedWarningLevel = currentWarningLevel;
        lastLoggedDetectionSource = detectionSource;
        lastLoggedDetectionSensor = detectionSensor;
        lastDebugLogTime = Time.time;
    }

    //// ========== 외부 참조용 안전 확인 메서드들 (7단계 기준) ==========

    /// <summary>
    /// 긴급정지가 필요한 상태인지 확인 (EmergencyStop)
    /// 초음파 근접 감지 - 즉시 모든 동작 중지 필요
    /// </summary>
    public bool IsEmergencyStop() => currentWarningLevel == WarningLevel.EmergencyStop;

    /// <summary>
    /// 제동이 필요한 상태인지 확인 (Brake 이상)
    /// TTC < 1초 또는 긴급정지 - 강한 감속 필요
    /// </summary>
    public bool ShouldBrake() => currentWarningLevel >= WarningLevel.Brake;

    /// <summary>
    /// 경고 상태인지 확인 (Warning 이상)
    /// TTC < 2초 - 적극적 감속 필요
    /// </summary>
    public bool IsWarning() => currentWarningLevel >= WarningLevel.Warning;

    /// <summary>
    /// 감속이 필요한 상태인지 확인 (SlowDown 이상)
    /// TTC < 3초 - 점진적 감속 필요
    /// </summary>
    public bool ShouldSlowDown() => currentWarningLevel >= WarningLevel.SlowDown;

    /// <summary>
    /// 주의가 필요한 상태인지 확인 (Caution 이상)
    /// TTC < 5초 - 전방 주시 필요
    /// </summary>
    public bool IsCaution() => currentWarningLevel >= WarningLevel.Caution;

    /// <summary>
    /// 장애물을 인지한 상태인지 확인 (Awareness 이상)
    /// 장애물 존재 확인됨
    /// </summary>
    public bool IsAware() => currentWarningLevel >= WarningLevel.Awareness;

    /// <summary>
    /// 완전히 안전한 상태인지 확인 (Safe)
    /// </summary>
    public bool IsSafe() => currentWarningLevel == WarningLevel.Safe;

    /// <summary>
    /// 차량 정지 판단용 통합 메서드
    /// EmergencyStop 또는 Brake 상태일 때 true
    /// 도로 주행 중 정지/분기 결정 포인트로 사용
    /// </summary>
    public bool ShouldStop() => currentWarningLevel >= WarningLevel.Brake;

    public float GetDistanceToObstacle() => currentMinDistance;
    public float GetTimeToCollision() => currentTTC;
    public WarningLevel GetWarningLevel() => currentWarningLevel;
    /// <summary>
    /// 상대속도(Closing Speed) 반환
    /// 양수: 장애물이 가까워지는 중 (위험)
    /// 음수: 장애물이 멀어지는 중 (안전)
    /// </summary>
    public float GetClosingSpeed() => smoothedClosingSpeed;
    public float GetEgoSpeed() => currentSpeed;

    // 제동 거리를 고려한 안전 여유 공간 계산
    public float GetSafetyMargin(float reactionTime = 0.5f, float deceleration = 2f)
    {
        if (currentSpeed <= 0.01f)
            return currentMinDistance;

        // 반응 시간 동안 이동 거리 + 제동 거리
        float reactionDistance = currentSpeed * reactionTime;
        float brakingDistance = (currentSpeed * currentSpeed) / (2f * deceleration);
        float requiredDistance = reactionDistance + brakingDistance;

        // 남은 여유 거리 반환 (음수면 충돌 위험)
        return currentMinDistance - requiredDistance;
    }

    public bool IsFrontClear(float ultrasonicThreshold = 0.5f, float radarThreshold = 2.0f)
    {
        bool ultrasonicClear = ultrasonicManager == null || ultrasonicManager.IsFrontClear(ultrasonicThreshold);
        bool radarClear = radarManager == null || radarManager.IsFrontClear(radarThreshold);
        return ultrasonicClear && radarClear;
    }

    public bool IsRearClear(float ultrasonicThreshold = 0.5f, float radarThreshold = 2.0f)
    {
        bool ultrasonicClear = ultrasonicManager == null || ultrasonicManager.IsRearClear(ultrasonicThreshold);
        bool radarClear = radarManager == null || radarManager.IsRearClear(radarThreshold);
        return ultrasonicClear && radarClear;
    }

    public (string source, string sensor, float distance) GetClosestObstacleInfo()
    {
        return (detectionSource, detectionSensor, currentMinDistance);
    }
}
