using System.Text;
using UnityEngine;

/// <summary>
/// 초음파 센서 데이터를 기반으로 충돌 위험을 감지하고 경고 레벨을 계산하는 엔진.
/// 계산 결과는 Unity 내부 로직에서 직접 소비한다.
///
/// [레이더 미사용 근거]
/// 본 프로젝트의 AMR은 최대 속도 2.0 m/s로 운행된다.
/// 초음파 센서의 최대 감지 거리 4m는 최고 속도 기준 약 2초의 사전 경고 시간을 확보하며,
/// Emergency(0.2m) 도달까지 약 1.9초의 반응 여유가 있다.
/// 따라서 레이더(50m 범위)의 원거리 감지는 현 운용 조건에서 불필요하여 방향별 위험도 판정에서 제외한다.
/// 향후 속도 상향 또는 실차 전환 시 레이더 융합을 재검토할 것.
///
/// [실차 전환 시 개선 필요 사항]
/// P0: Confidence-gated 판정 — 센서 Confidence가 reliableConfidenceThreshold 미만인 감지를
///     위험도 판정에서 제외하거나 가중치 감소. 시뮬레이션에서는 센서 신뢰도가 안정적이므로 미적용.
/// P1: Temporal filtering / Hysteresis — 경고 레벨 상향 시 N-of-M 확인, 하향 시 hold-off 타이머 적용.
///     시뮬레이션에서는 multipath/phantom 노이즈가 미미하므로 미적용.
/// P4: Sensor health monitoring — 센서 응답 타임아웃, 고정값 고착 감시.
///     고장 시 해당 방향을 Safe가 아닌 Unknown/Danger로 처리. 시뮬레이션에서는 센서 고장이 없으므로 미적용.
/// </summary>
[AddComponentMenu("Sensor/Collision Warning Engine")]
public class CollisionWarningEngine : MonoBehaviour
{
    [Header("Update Rate")]
    public float updateRate = 20f;

    [Header("Individual Sensor References (개별 센서 직접 참조)")]
    [Tooltip("전방 좌측 초음파")]
    public SingleUltrasonicSensor sensorFL;
    [Tooltip("전방 우측 초음파")]
    public SingleUltrasonicSensor sensorFR;
    [Tooltip("전방 중앙 초음파")]
    public SingleUltrasonicSensor sensorFC;
    [Tooltip("후방 좌측 초음파")]
    public SingleUltrasonicSensor sensorRL;
    [Tooltip("후방 우측 초음파")]
    public SingleUltrasonicSensor sensorRR;
    [Tooltip("후방 중앙 초음파")]
    public SingleUltrasonicSensor sensorRC;
    [Tooltip("측면 좌측 초음파")]
    public SingleUltrasonicSensor sensorSL;
    [Tooltip("측면 우측 초음파")]
    public SingleUltrasonicSensor sensorSR;
    // [레이더 미사용] 저속 AMR(최대 2.0 m/s)에서 초음파 4m 범위로 충분한 반응 시간 확보.
    // 향후 속도 상향 시 주석 해제 후 CalculateDirectionalWarnings()에 융합 로직 추가 필요.
    // [Tooltip("전방 레이더")]
    // public SingleRadarSensor radarFront;
    // [Tooltip("후방 레이더")]
    // public SingleRadarSensor radarRear;

    private SingleUltrasonicSensor[] allUltrasonicSensors;
    // private SingleRadarSensor[] allRadarSensors;

    [Header("거리 기반 임계값 (Distance — 정지 시 기본값, 이동 시 속도에 비례하여 확장)")]
    [Tooltip("Awareness 거리 (m) — 이 거리 이하면 인지")]
    public float distAwareness  = 1.50f;
    [Tooltip("Caution 거리 (m) — 이 거리 이하면 주의")]
    public float distCaution    = 1.00f;
    [Tooltip("SlowDown 거리 (m) — 이 거리 이하면 감속")]
    public float distSlowDown   = 0.80f;
    [Tooltip("Warning 거리 (m) — 이 거리 이하면 경고")]
    public float distWarning    = 0.55f;
    [Tooltip("Brake 거리 (m) — 이 거리 이하면 제동")]
    public float distBrake      = 0.35f;
    [Tooltip("Emergency 거리 (m) — 이 거리 이하면 비상정지")]
    public float distEmergency  = 0.20f;

    [Header("속도 기반 임계값 확장 (Speed-based Threshold Expansion)")]
    [Tooltip("속도에 비례하여 임계값을 확장하는 시간 마진(초). " +
             "effectiveThreshold = static + speed * factor. " +
             "0이면 정적 임계값만 사용. 기본값 0.5초 = 반응 시간 기준.")]
    public float speedMarginFactor = 0.5f;

    [Header("Vehicle Reference (차량 참조)")]
    [Tooltip("차량 모션 컨트롤러 — 속도/조향 입력으로 MotionDirection 자동 결정")]
    public VehicleMotionController vehicleMotion;

    [Header("Motion Direction Thresholds (운동 방향 판정 임계값)")]
    [Tooltip("이 속도(m/s) 이하면 Stopped으로 판정")]
    public float motionSpeedThreshold = 0.08f;
    [Tooltip("이 조향 입력(절댓값) 이하면 직진으로 판정")]
    public float motionSteeringThreshold = 0.05f;

    [Header("Speed Reference (속도 참조 — 모션 감지용)")]
    [Tooltip("차량 속도를 가져올 물리 컴포넌트 (base_link)")]
    public ArticulationBody velocitySource;

    [Header("Debug (디버그)")]
    public bool showDebugInfo = false;
    [Tooltip("true면 경고 레벨/감지 소스가 변할 때만 로그 출력")]
    public bool debugOnlyOnStateChange = true;
    [Tooltip("상태 변화가 없어도 이 주기(초)마다 상태 스냅샷 로그 출력. 0 이하이면 비활성")]
    public float debugSnapshotInterval = 1.0f;
    [Tooltip("true면 초음파/레이더 채널 거리 상세를 함께 출력")]
    public bool debugIncludeSensorChannels = true;

    [HideInInspector] public float currentMinDistance = float.PositiveInfinity;
    [HideInInspector] public WarningLevel currentWarningLevel = WarningLevel.Safe;
    [HideInInspector] public string detectionSource = "None";
    [HideInInspector] public string detectionSensor = "None";

    // 방향별 위험도
    [HideInInspector] public DirectionalWarning frontWarning;
    [HideInInspector] public DirectionalWarning rearWarning;
    [HideInInspector] public DirectionalWarning leftWarning;
    [HideInInspector] public DirectionalWarning rightWarning;

    public enum WarningLevel
    {
        Safe = 0,
        Awareness = 1,
        Caution = 2,
        SlowDown = 3,
        Warning = 4,
        Brake = 5,
        EmergencyStop = 6
    }

    /// <summary>
    /// 차량의 현재 운동 방향. throttle + steering 조합으로 결정된다.
    /// GetMotionRelevantWarning()에서 이 방향에 해당하는 센서만 참조하여 위험도를 반환한다.
    /// </summary>
    public enum MotionDirection
    {
        Stopped,
        MoveForward,
        MoveForwardLeft,
        MoveForwardRight,
        MoveBackward,
        MoveBackwardLeft,
        MoveBackwardRight
    }

    public struct DirectionalWarning
    {
        public WarningLevel level;
        public string dominantSensor;
        public float dominantDistance;
    }

    public struct SensorData
    {
        public float ultrasonicFL;
        public float ultrasonicFR;
        public float ultrasonicFC;
        public float ultrasonicRL;
        public float ultrasonicRR;
        public float ultrasonicRC;
        public float ultrasonicSL;
        public float ultrasonicSR;
        public float ultrasonicMinFront;
        public float ultrasonicMinRear;
        public float ultrasonicClosestConfidence;
        // public float radarFront;
        // public float radarRear;
        public SingleUltrasonicSensor.SensorPosition ultrasonicClosest;
        // public SingleRadarSensor.SensorPosition radarClosest;
    }

    public SensorData CurrentSensorData { get; private set; }

    private float updateInterval;
    private float lastUpdateTime;
    private float currentSpeed = 0f;
    private float lastDebugLogTime = -999f;
    private WarningLevel lastLoggedWarningLevel = WarningLevel.Safe;
    private string lastLoggedDetectionSource = "None";
    private string lastLoggedDetectionSensor = "None";
    private MotionDirection lastLoggedMotionDirection = MotionDirection.Stopped;

    [HideInInspector] public MotionDirection currentMotionDirection = MotionDirection.Stopped;

    void Start()
    {
        ValidateSensorReferences();
        SyncScanIntervals();

        if (vehicleMotion == null)
            vehicleMotion = GetComponentInParent<VehicleMotionController>();
        if (vehicleMotion == null)
            Debug.LogWarning("[CollisionWarning] VehicleMotionController가 할당되지 않았습니다. MotionDirection이 항상 Stopped입니다.");

        updateInterval = 1f / updateRate;
        lastUpdateTime = Time.time;
    }

    void ValidateSensorReferences()
    {
        TryFindUltrasonicSensor(ref sensorFL, "ultrasonic_fl_link", "FL");
        TryFindUltrasonicSensor(ref sensorFR, "ultrasonic_fr_link", "FR");
        TryFindUltrasonicSensor(ref sensorFC, "ultrasonic_fc_link", "FC");
        TryFindUltrasonicSensor(ref sensorRL, "ultrasonic_rl_link", "RL");
        TryFindUltrasonicSensor(ref sensorRR, "ultrasonic_rr_link", "RR");
        TryFindUltrasonicSensor(ref sensorRC, "ultrasonic_rc_link", "RC");
        TryFindUltrasonicSensor(ref sensorSL, "ultrasonic_sl_link", "SL");
        TryFindUltrasonicSensor(ref sensorSR, "ultrasonic_sr_link", "SR");

        // TryFindRadarSensor(ref radarFront, "radar_front_link", "Front");
        // TryFindRadarSensor(ref radarRear, "radar_rear_link", "Rear");

        allUltrasonicSensors = new[]
        {
            sensorFL, sensorFR, sensorFC, sensorRL, sensorRR, sensorRC, sensorSL, sensorSR
        };
        // allRadarSensors = new[] { radarFront, radarRear };

        if (velocitySource == null)
            Debug.LogWarning("[CollisionWarning] velocitySource가 할당되지 않았습니다. 모션 감지가 비활성화됩니다.");

    }

    void TryFindUltrasonicSensor(ref SingleUltrasonicSensor sensor, string gameObjectName, string label)
    {
        if (sensor != null)
            return;

        GameObject go = FindSensorGameObject(gameObjectName);
        if (go != null)
        {
            sensor = go.GetComponent<SingleUltrasonicSensor>();
            if (sensor != null)
                Debug.Log($"[CollisionWarning] 초음파 센서 {label} 자동 탐색 완료: {gameObjectName}");
            else
                Debug.LogWarning($"[CollisionWarning] {gameObjectName} 오브젝트에 SingleUltrasonicSensor 컴포넌트가 없습니다.");
        }
        else
        {
            Debug.LogWarning($"[CollisionWarning] 초음파 센서 {label} ({gameObjectName})를 찾을 수 없습니다.");
        }
    }

    // void TryFindRadarSensor(ref SingleRadarSensor sensor, string gameObjectName, string label)
    // {
    //     if (sensor != null)
    //         return;
    //
    //     GameObject go = FindSensorGameObject(gameObjectName);
    //     if (go != null)
    //     {
    //         sensor = go.GetComponent<SingleRadarSensor>();
    //         if (sensor != null)
    //             Debug.Log($"[CollisionWarning] 레이더 센서 {label} 자동 탐색 완료: {gameObjectName}");
    //         else
    //             Debug.LogWarning($"[CollisionWarning] {gameObjectName} 오브젝트에 SingleRadarSensor 컴포넌트가 없습니다.");
    //     }
    //     else
    //     {
    //         Debug.LogWarning($"[CollisionWarning] 레이더 센서 {label} ({gameObjectName})를 찾을 수 없습니다.");
    //     }
    // }

    GameObject FindSensorGameObject(string gameObjectName)
    {
        Transform scopedRoot = transform.root;
        if (scopedRoot != null)
        {
            Transform scopedMatch = FindDescendantByName(scopedRoot, gameObjectName);
            if (scopedMatch != null)
                return scopedMatch.gameObject;
        }

        return GameObject.Find(gameObjectName);
    }

    static Transform FindDescendantByName(Transform root, string targetName)
    {
        if (root == null || string.IsNullOrEmpty(targetName))
            return null;

        if (root.name == targetName)
            return root;

        for (int i = 0; i < root.childCount; i++)
        {
            Transform match = FindDescendantByName(root.GetChild(i), targetName);
            if (match != null)
                return match;
        }

        return null;
    }

    void SyncScanIntervals()
    {
        float interval = 1f / updateRate;
        foreach (var sensor in allUltrasonicSensors)
            if (sensor != null)
                sensor.SetScanInterval(interval);

        // foreach (var sensor in allRadarSensors)
        //     if (sensor != null)
        //         sensor.SetScanInterval(interval);
    }

    void Update()
    {
        UpdateCurrentSpeed();
        UpdateMotionDirection();

        if (Time.time - lastUpdateTime < updateInterval)
            return;

        CollectSensorData();
        CalculateDirectionalWarnings();
        lastUpdateTime = Time.time;
        if (showDebugInfo)
            PrintDebugInfo();
    }

    void UpdateCurrentSpeed()
    {
        currentSpeed = velocitySource != null ? vehicleMotion.GetSpeedMS() : 0f;
    }

    void UpdateMotionDirection()
    {
        if (vehicleMotion == null)
        {
            currentMotionDirection = MotionDirection.Stopped;
            return;
        }

        float speed = vehicleMotion.GetSpeedMS();
        float steering = vehicleMotion.GetSteeringInput();

        // 속도가 임계값 이하면 정지
        if (Mathf.Abs(speed) <= motionSpeedThreshold)
        {
            currentMotionDirection = MotionDirection.Stopped;
            return;
        }

        bool forward = speed > 0f;
        bool turningRight = steering > motionSteeringThreshold;
        bool turningLeft = steering < -motionSteeringThreshold;

        if (forward)
        {
            if (turningLeft)       currentMotionDirection = MotionDirection.MoveForwardLeft;
            else if (turningRight) currentMotionDirection = MotionDirection.MoveForwardRight;
            else                   currentMotionDirection = MotionDirection.MoveForward;
        }
        else
        {
            if (turningLeft)       currentMotionDirection = MotionDirection.MoveBackwardLeft;
            else if (turningRight) currentMotionDirection = MotionDirection.MoveBackwardRight;
            else                   currentMotionDirection = MotionDirection.MoveBackward;
        }
    }

    void CollectSensorData()
    {
        SensorData data = new SensorData
        {
            ultrasonicFL = sensorFL != null ? sensorFL.Distance : float.PositiveInfinity,
            ultrasonicFR = sensorFR != null ? sensorFR.Distance : float.PositiveInfinity,
            ultrasonicFC = sensorFC != null ? sensorFC.Distance : float.PositiveInfinity,
            ultrasonicRL = sensorRL != null ? sensorRL.Distance : float.PositiveInfinity,
            ultrasonicRR = sensorRR != null ? sensorRR.Distance : float.PositiveInfinity,
            ultrasonicRC = sensorRC != null ? sensorRC.Distance : float.PositiveInfinity,
            ultrasonicSL = sensorSL != null ? sensorSL.Distance : float.PositiveInfinity,
            ultrasonicSR = sensorSR != null ? sensorSR.Distance : float.PositiveInfinity
        };

        data.ultrasonicMinFront = Mathf.Min(data.ultrasonicFL, Mathf.Min(data.ultrasonicFR, data.ultrasonicFC));
        data.ultrasonicMinRear = Mathf.Min(data.ultrasonicRL, Mathf.Min(data.ultrasonicRR, data.ultrasonicRC));

        UpdateClosestUltrasonic(ref data);

        // data.radarFront = radarFront != null ? radarFront.Distance : float.PositiveInfinity;
        // data.radarRear = radarRear != null ? radarRear.Distance : float.PositiveInfinity;
        // data.radarClosest = data.radarFront <= data.radarRear
        //     ? SingleRadarSensor.SensorPosition.Front
        //     : SingleRadarSensor.SensorPosition.Rear;

        CurrentSensorData = data;
    }

    void UpdateClosestUltrasonic(ref SensorData data)
    {
        float closestDistance = float.PositiveInfinity;
        float closestConfidence = 0f;
        SingleUltrasonicSensor.SensorPosition closestPosition = default;

        for (int i = 0; i < allUltrasonicSensors.Length; i++)
        {
            SingleUltrasonicSensor sensor = allUltrasonicSensors[i];
            if (sensor == null)
                continue;

            float distance = sensor.Distance;
            if (float.IsInfinity(distance))
                continue;

            float confidence = sensor.Confidence;
            bool isCloser = distance < closestDistance;
            bool isTieButMoreConfident = Mathf.Abs(distance - closestDistance) < 0.02f && confidence > closestConfidence;
            if (!isCloser && !isTieButMoreConfident)
                continue;

            closestDistance = distance;
            closestConfidence = confidence;
            closestPosition = sensor.sensorPosition;
        }

        data.ultrasonicClosest = closestPosition;
        data.ultrasonicClosestConfidence = closestConfidence;
    }

    void CalculateDirectionalWarnings()
    {
        SensorData d = CurrentSensorData;

        // Front: FC만
        frontWarning = EvaluateDirectionSingle(d.ultrasonicFC, "FC");

        // Rear: RC만
        rearWarning = EvaluateDirectionSingle(d.ultrasonicRC, "RC");

        // Left: FL, SL, RL (초음파만)
        leftWarning = EvaluateDirection(
            (d.ultrasonicFL, "FL"),
            (d.ultrasonicSL, "SL"),
            (d.ultrasonicRL, "RL"));

        // Right: FR, SR, RR (초음파만)
        rightWarning = EvaluateDirection(
            (d.ultrasonicFR, "FR"),
            (d.ultrasonicSR, "SR"),
            (d.ultrasonicRR, "RR"));

        // 글로벌 값 (하위 호환)
        currentMinDistance = Mathf.Min(
            frontWarning.dominantDistance,
            Mathf.Min(rearWarning.dominantDistance,
            Mathf.Min(leftWarning.dominantDistance, rightWarning.dominantDistance)));

        currentWarningLevel = Max4(
            frontWarning.level, rearWarning.level,
            leftWarning.level, rightWarning.level);

        if (currentWarningLevel > WarningLevel.Safe)
            ResolveWarningSource(out detectionSource, out detectionSensor);
        else
        {
            detectionSource = "None";
            detectionSensor = "None";
        }
    }

    DirectionalWarning EvaluateDirectionSingle(float dist, string name)
    {
        return new DirectionalWarning
        {
            level = EvaluateDistanceWarning(dist, currentSpeed),
            dominantSensor = name,
            dominantDistance = dist
        };
    }

    DirectionalWarning EvaluateDirection(
        (float dist, string name) s0,
        (float dist, string name) s1,
        (float dist, string name) s2)
    {
        float minDist = s0.dist;
        string minName = s0.name;

        if (s1.dist < minDist) { minDist = s1.dist; minName = s1.name; }
        if (s2.dist < minDist) { minDist = s2.dist; minName = s2.name; }

        return new DirectionalWarning
        {
            level = EvaluateDistanceWarning(minDist, currentSpeed),
            dominantSensor = minName,
            dominantDistance = minDist
        };
    }

    static WarningLevel Max4(WarningLevel a, WarningLevel b, WarningLevel c, WarningLevel d)
    {
        WarningLevel ab = a > b ? a : b;
        WarningLevel cd = c > d ? c : d;
        return ab > cd ? ab : cd;
    }

    // 거리 기반 경고: 정지 시 정적 임계값, 이동 시 속도 비례 확장
    WarningLevel EvaluateDistanceWarning(float distance, float speed)
    {
        if (float.IsInfinity(distance)) return WarningLevel.Safe;

        float expansion = speed * speedMarginFactor;

        if (distance <= distEmergency + expansion) return WarningLevel.EmergencyStop;
        if (distance <= distBrake     + expansion) return WarningLevel.Brake;
        if (distance <= distWarning   + expansion) return WarningLevel.Warning;
        if (distance <= distSlowDown  + expansion) return WarningLevel.SlowDown;
        if (distance <= distCaution   + expansion) return WarningLevel.Caution;
        if (distance <= distAwareness + expansion) return WarningLevel.Awareness;

        return WarningLevel.Safe;
    }



    void PrintDebugInfo()
    {
        bool hasStateChanged = currentWarningLevel != lastLoggedWarningLevel ||
                               detectionSource != lastLoggedDetectionSource ||
                               detectionSensor != lastLoggedDetectionSensor ||
                               currentMotionDirection != lastLoggedMotionDirection;
        bool hasSnapshotIntervalElapsed = debugSnapshotInterval > 0f && (Time.time - lastDebugLogTime) >= debugSnapshotInterval;
        bool shouldLogContinuously = !debugOnlyOnStateChange;

        if (!shouldLogContinuously && !hasStateChanged && !hasSnapshotIntervalElapsed)
            return;

        WarningLevel motionWarning = GetMotionRelevantWarning(currentMotionDirection);

        StringBuilder logBuilder = new StringBuilder();
        logBuilder.Append($"[Collision] Speed={currentSpeed:F2} m/s | Motion={currentMotionDirection} → {motionWarning}");
        logBuilder.Append('\n');
        logBuilder.Append($"  Front: {FormatDirectional(frontWarning)} | Rear: {FormatDirectional(rearWarning)} | Left: {FormatDirectional(leftWarning)} | Right: {FormatDirectional(rightWarning)}");

        if (debugIncludeSensorChannels)
        {
            logBuilder.Append('\n');
            logBuilder.Append($"  Ultra   : {BuildUltrasonicSummary()}");
        }

        Debug.Log(logBuilder.ToString());

        lastLoggedWarningLevel = currentWarningLevel;
        lastLoggedDetectionSource = detectionSource;
        lastLoggedDetectionSensor = detectionSensor;
        lastLoggedMotionDirection = currentMotionDirection;
        lastDebugLogTime = Time.time;
    }

    static string FormatDirectional(DirectionalWarning w)
    {
        if (w.level == WarningLevel.Safe)
            return "Safe";
        return $"{w.level} ({w.dominantSensor}: {FormatDist(w.dominantDistance)}m)";
    }

    static string FormatDist(float value) => float.IsInfinity(value) ? "∞" : $"{value:F2}";

    string BuildUltrasonicSummary()
    {
        return
            $"Front[FL={FormatDist(CurrentSensorData.ultrasonicFL)} FR={FormatDist(CurrentSensorData.ultrasonicFR)} FC={FormatDist(CurrentSensorData.ultrasonicFC)}] " +
            $"Rear[RL={FormatDist(CurrentSensorData.ultrasonicRL)} RR={FormatDist(CurrentSensorData.ultrasonicRR)} RC={FormatDist(CurrentSensorData.ultrasonicRC)}] " +
            $"Side[L={FormatDist(CurrentSensorData.ultrasonicSL)} R={FormatDist(CurrentSensorData.ultrasonicSR)}]";
    }

    // string BuildRadarSummary()
    // {
    //     return $"Front={FormatDist(CurrentSensorData.radarFront)} | Rear={FormatDist(CurrentSensorData.radarRear)}";
    // }

    void ResolveWarningSource(out string source, out string sensor)
    {
        if (TryResolveGlobalSource(out source, out sensor))
            return;

        source = "None";
        sensor = "None";
    }

    bool TryResolveGlobalSource(out string source, out string sensor)
    {
        float ultrasonicMin = GetUltrasonicDistance(CurrentSensorData.ultrasonicClosest);

        if (!float.IsInfinity(ultrasonicMin))
        {
            source = "Ultrasonic";
            sensor = CurrentSensorData.ultrasonicClosest.ToString();
            return true;
        }

        source = "None";
        sensor = "None";
        return false;
    }

    float GetUltrasonicDistance(SingleUltrasonicSensor.SensorPosition position)
    {
        return position switch
        {
            SingleUltrasonicSensor.SensorPosition.FrontLeft => CurrentSensorData.ultrasonicFL,
            SingleUltrasonicSensor.SensorPosition.FrontRight => CurrentSensorData.ultrasonicFR,
            SingleUltrasonicSensor.SensorPosition.FrontCenter => CurrentSensorData.ultrasonicFC,
            SingleUltrasonicSensor.SensorPosition.RearLeft => CurrentSensorData.ultrasonicRL,
            SingleUltrasonicSensor.SensorPosition.RearRight => CurrentSensorData.ultrasonicRR,
            SingleUltrasonicSensor.SensorPosition.RearCenter => CurrentSensorData.ultrasonicRC,
            SingleUltrasonicSensor.SensorPosition.SideLeft => CurrentSensorData.ultrasonicSL,
            SingleUltrasonicSensor.SensorPosition.SideRight => CurrentSensorData.ultrasonicSR,
            _ => float.PositiveInfinity
        };
    }

    public bool IsEmergencyStop() => currentWarningLevel == WarningLevel.EmergencyStop;
    public bool ShouldBrake() => currentWarningLevel >= WarningLevel.Brake;
    public bool IsWarning() => currentWarningLevel >= WarningLevel.Warning;
    public bool ShouldSlowDown() => currentWarningLevel >= WarningLevel.SlowDown;
    public bool IsCaution() => currentWarningLevel >= WarningLevel.Caution;
    public bool IsAware() => currentWarningLevel >= WarningLevel.Awareness;
    public bool IsSafe() => currentWarningLevel == WarningLevel.Safe;
    public bool ShouldStop() => currentWarningLevel >= WarningLevel.Brake;

    public MotionDirection GetMotionDirection() => currentMotionDirection;

    // 방향별 위험도 API
    public DirectionalWarning GetFrontWarning() => frontWarning;
    public DirectionalWarning GetRearWarning()  => rearWarning;
    public DirectionalWarning GetLeftWarning()  => leftWarning;
    public DirectionalWarning GetRightWarning() => rightWarning;

    // 클리어런스 API — "이 방향으로 조향해도 되는가"
    public WarningLevel GetClearanceLeft()  => leftWarning.level;
    public WarningLevel GetClearanceRight() => rightWarning.level;

    // 조향 가능 여부 (Warning 이상이면 해당 방향 조향 제한)
    public bool CanTurnLeft(WarningLevel limit = WarningLevel.Warning)
        => leftWarning.level < limit;
    public bool CanTurnRight(WarningLevel limit = WarningLevel.Warning)
        => rightWarning.level < limit;

    public float GetDistanceToObstacle() => currentMinDistance;
    public WarningLevel GetWarningLevel() => currentWarningLevel;
    public float GetEgoSpeed() => currentSpeed;

    public float GetSafetyMargin(float reactionTime = 0.5f, float deceleration = 2f)
    {
        if (currentSpeed <= 0.01f)
            return currentMinDistance;

        float reactionDistance = currentSpeed * reactionTime;
        float brakingDistance = (currentSpeed * currentSpeed) / (2f * deceleration);
        return currentMinDistance - (reactionDistance + brakingDistance);
    }

    public bool IsFrontClear(float ultrasonicThreshold = 0.5f)
    {
        return CurrentSensorData.ultrasonicMinFront > ultrasonicThreshold ||
               float.IsInfinity(CurrentSensorData.ultrasonicMinFront);
    }

    public bool IsRearClear(float ultrasonicThreshold = 0.5f)
    {
        return CurrentSensorData.ultrasonicMinRear > ultrasonicThreshold ||
               float.IsInfinity(CurrentSensorData.ultrasonicMinRear);
    }

    public (string source, string sensor, float distance) GetClosestObstacleInfo()
    {
        return (detectionSource, detectionSensor, currentMinDistance);
    }

    // ============================================
    // 방향별 주행 제어 API
    // ============================================

    /// <summary>
    /// throttle과 steering 입력으로부터 현재 운동 방향을 결정한다.
    /// </summary>
    /// <param name="throttle">스로틀 입력 (-1~1, 양수=전진, 음수=후진)</param>
    /// <param name="steering">조향 입력 (-1~1, 양수=좌회전, 음수=우회전)</param>
    /// <param name="steeringDeadzone">조향 입력이 이 값 이하면 직진으로 판정</param>
    /// <param name="throttleDeadzone">스로틀 입력이 이 값 이하면 정지로 판정</param>
    public static MotionDirection ResolveMotionDirection(
        float throttle, float steering,
        float throttleDeadzone = 0.05f, float steeringDeadzone = 0.05f)
    {
        bool isStopped = Mathf.Abs(throttle) <= throttleDeadzone;
        if (isStopped)
            return MotionDirection.Stopped;

        bool forward = throttle > 0f;
        bool turningRight = steering > steeringDeadzone;
        bool turningLeft = steering < -steeringDeadzone;

        if (forward)
        {
            if (turningLeft)  return MotionDirection.MoveForwardLeft;
            if (turningRight) return MotionDirection.MoveForwardRight;
            return MotionDirection.MoveForward;
        }
        else
        {
            if (turningLeft)  return MotionDirection.MoveBackwardLeft;
            if (turningRight) return MotionDirection.MoveBackwardRight;
            return MotionDirection.MoveBackward;
        }
    }

    /// <summary>
    /// 운동 방향에 해당하는 센서들의 위험도 중 최댓값을 반환한다.
    /// 방향별 directional warning 조합이 아닌, 개별 센서 거리를 직접 평가하여
    /// 측면 센서(SL/SR)의 오판을 방지한다.
    ///
    /// 센서 매핑:
    ///   Stopped            → 전방향 (기존 4개 directional max)
    ///   MoveForward        → FC
    ///   MoveForwardLeft    → FC, FL, FR (전방 3개 — 회전 시 차체 전면 전체 스윕)
    ///   MoveForwardRight   → FC, FL, FR (전방 3개)
    ///   MoveBackward       → RC
    ///   MoveBackwardLeft   → RC, RL, RR (후방 3개 — 회전 시 차체 후면 전체 스윕)
    ///   MoveBackwardRight  → RC, RL, RR (후방 3개)
    /// </summary>
    public WarningLevel GetMotionRelevantWarning(MotionDirection direction)
    {
        SensorData d = CurrentSensorData;

        switch (direction)
        {
            case MotionDirection.Stopped:
                return Max4(frontWarning.level, rearWarning.level,
                            leftWarning.level, rightWarning.level);

            case MotionDirection.MoveForward:
                return EvaluateDistanceWarning(d.ultrasonicFC, currentSpeed);

            case MotionDirection.MoveForwardLeft:
            case MotionDirection.MoveForwardRight:
                return Max3(
                    EvaluateDistanceWarning(d.ultrasonicFC, currentSpeed),
                    EvaluateDistanceWarning(d.ultrasonicFL, currentSpeed),
                    EvaluateDistanceWarning(d.ultrasonicFR, currentSpeed));

            case MotionDirection.MoveBackward:
                return EvaluateDistanceWarning(d.ultrasonicRC, currentSpeed);

            case MotionDirection.MoveBackwardLeft:
            case MotionDirection.MoveBackwardRight:
                return Max3(
                    EvaluateDistanceWarning(d.ultrasonicRC, currentSpeed),
                    EvaluateDistanceWarning(d.ultrasonicRL, currentSpeed),
                    EvaluateDistanceWarning(d.ultrasonicRR, currentSpeed));

            default:
                return Max4(frontWarning.level, rearWarning.level,
                            leftWarning.level, rightWarning.level);
        }
    }

    /// <summary>
    /// 운동 방향에 해당하는 센서들의 방향별 위험도 상세를 반환한다.
    /// primary=진행 방향 중앙, sideL=좌측 코너, sideR=우측 코너.
    /// 회전 시에는 해당 면(전방/후방) 3개 센서 전부 반환.
    /// </summary>
    public (DirectionalWarning primary, DirectionalWarning sideL, DirectionalWarning sideR)
        GetMotionRelevantWarnings(MotionDirection direction)
    {
        DirectionalWarning none = new DirectionalWarning
        {
            level = WarningLevel.Safe,
            dominantSensor = "None",
            dominantDistance = float.PositiveInfinity
        };

        SensorData d = CurrentSensorData;

        switch (direction)
        {
            case MotionDirection.Stopped:
                return (frontWarning, leftWarning, rightWarning);

            case MotionDirection.MoveForward:
                return (EvaluateDirectionSingle(d.ultrasonicFC, "FC"), none, none);

            case MotionDirection.MoveForwardLeft:
            case MotionDirection.MoveForwardRight:
                return (EvaluateDirectionSingle(d.ultrasonicFC, "FC"),
                        EvaluateDirectionSingle(d.ultrasonicFL, "FL"),
                        EvaluateDirectionSingle(d.ultrasonicFR, "FR"));

            case MotionDirection.MoveBackward:
                return (EvaluateDirectionSingle(d.ultrasonicRC, "RC"), none, none);

            case MotionDirection.MoveBackwardLeft:
            case MotionDirection.MoveBackwardRight:
                return (EvaluateDirectionSingle(d.ultrasonicRC, "RC"),
                        EvaluateDirectionSingle(d.ultrasonicRL, "RL"),
                        EvaluateDirectionSingle(d.ultrasonicRR, "RR"));

            default:
                return (frontWarning, leftWarning, rightWarning);
        }
    }

    static WarningLevel Max2(WarningLevel a, WarningLevel b) => a > b ? a : b;
    static WarningLevel Max3(WarningLevel a, WarningLevel b, WarningLevel c)
    {
        WarningLevel ab = a > b ? a : b;
        return ab > c ? ab : c;
    }
}
