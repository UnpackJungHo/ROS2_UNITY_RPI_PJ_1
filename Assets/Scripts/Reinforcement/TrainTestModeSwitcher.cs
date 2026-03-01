using TMPro;
using Unity.MLAgents.Policies;
using UnityEngine;

/// <summary>
/// Train/Test 모드 전환을 한 곳에서 관리한다.
/// - Train: Unity 내부 로직(내부 제어 + 내부 센서)
/// - Test : ROS 외부 토픽(외부 제어 + 외부 센서)
/// </summary>
[ExecuteAlways]
[DisallowMultipleComponent]
public class TrainTestModeSwitcher : MonoBehaviour
{
    public enum RuntimeMode
    {
        Train = 0,
        Test = 1
    }

    [Header("Mode")]
    public RuntimeMode selectedMode = RuntimeMode.Train;
    [Tooltip("플레이 시작 시 selectedMode를 즉시 적용")]
    public bool autoApplyOnStart = true;
    [Tooltip("에디터에서 값 변경 시 selectedMode를 즉시 적용")]
    public bool autoApplyInEditor = true;

    [Header("Auto Find")]
    public bool autoFindReferences = true;
    [Tooltip("비활성 오브젝트까지 탐색")]
    public bool includeInactiveObjects = true;

    [Header("Core References")]
    public AutoDriverRLAgent autoDriverRLAgent;
    public RegressionDrivingController regressionDrivingController;
    public VehicleCmdSubscriber vehicleCmdSubscriber;
    public DrivingStatusUIController drivingStatusUIController;
    public TrafficLightDecisionEngine trafficLightDecisionEngine;
    public WheelTest wheelController;
    public BehaviorParameters behaviorParameters;

    [Header("Sensor References")]
    public SingleUltrasonicSensor[] ultrasonicSensors;
    public SingleRadarSensor[] radarSensors;

    [Header("Optional UI")]
    public TMP_Text modeText;

    [Header("Debug (Read Only)")]
    [SerializeField] private RuntimeMode lastAppliedMode = (RuntimeMode)(-1);
    [SerializeField] private string lastAppliedSummary = "Not Applied";

    private bool isApplying = false;

    void Reset()
    {
        AutoFindReferences();
        ApplySelectedMode();
    }

    void Awake()
    {
        if (autoFindReferences)
            AutoFindReferences();
    }

    void Start()
    {
        if (autoApplyOnStart)
            ApplySelectedMode();
    }

    void OnValidate()
    {
        if (isApplying || Application.isPlaying || !autoApplyInEditor)
            return;

        ApplySelectedMode();
    }

    [ContextMenu("Apply Selected Mode")]
    public void ApplySelectedMode()
    {
        if (isApplying)
            return;

        isApplying = true;
        try
        {
            if (autoFindReferences)
                AutoFindReferences();

            if (selectedMode == RuntimeMode.Train)
                ApplyTrainMode();
            else
                ApplyTestMode();

            UpdateModeText();
            lastAppliedMode = selectedMode;
            lastAppliedSummary =
                $"Mode={selectedMode}, Ultrasonic={CountValid(ultrasonicSensors)}, Radar={CountValid(radarSensors)}";

            Debug.Log($"[TrainTestModeSwitcher] {lastAppliedSummary}");
        }
        finally
        {
            isApplying = false;
        }
    }

    [ContextMenu("Auto Find References")]
    public void AutoFindReferences()
    {
        if (autoDriverRLAgent == null)
            autoDriverRLAgent = FindOne<AutoDriverRLAgent>(includeInactiveObjects);

        if (regressionDrivingController == null)
            regressionDrivingController = FindOne<RegressionDrivingController>(includeInactiveObjects);

        if (vehicleCmdSubscriber == null)
            vehicleCmdSubscriber = FindOne<VehicleCmdSubscriber>(includeInactiveObjects);

        if (drivingStatusUIController == null)
            drivingStatusUIController = FindOne<DrivingStatusUIController>(includeInactiveObjects);

        if (trafficLightDecisionEngine == null)
            trafficLightDecisionEngine = FindOne<TrafficLightDecisionEngine>(includeInactiveObjects);

        if (wheelController == null)
            wheelController = FindOne<WheelTest>(includeInactiveObjects);

        if (behaviorParameters == null)
        {
            if (autoDriverRLAgent != null)
                behaviorParameters = autoDriverRLAgent.GetComponent<BehaviorParameters>();

            if (behaviorParameters == null)
                behaviorParameters = FindOne<BehaviorParameters>(includeInactiveObjects);
        }

        ultrasonicSensors = FindAll<SingleUltrasonicSensor>(includeInactiveObjects);
        radarSensors = FindAll<SingleRadarSensor>(includeInactiveObjects);

        if (modeText == null)
        {
            TMP_Text[] texts = FindAll<TMP_Text>(includeInactiveObjects);
            for (int i = 0; i < texts.Length; i++)
            {
                if (texts[i] != null && texts[i].name == "ModeText")
                {
                    modeText = texts[i];
                    break;
                }
            }
        }
    }

    [ContextMenu("Switch To Train")]
    public void SwitchToTrain()
    {
        selectedMode = RuntimeMode.Train;
        ApplySelectedMode();
    }

    [ContextMenu("Switch To Test")]
    public void SwitchToTest()
    {
        selectedMode = RuntimeMode.Test;
        ApplySelectedMode();
    }

    void ApplyTrainMode()
    {
        // 학습: Unity 내부 로직만 사용
        if (autoDriverRLAgent != null)
        {
            autoDriverRLAgent.externalRosCmdInputOnly = false;
            autoDriverRLAgent.forceInternalControlWhenTraining = true;
        }

        if (behaviorParameters != null)
            behaviorParameters.BehaviorType = BehaviorType.Default;

        if (regressionDrivingController != null)
        {
            regressionDrivingController.isAutonomousMode = true;
            regressionDrivingController.predictionOnlyMode = true;
        }

        if (vehicleCmdSubscriber != null)
        {
            vehicleCmdSubscriber.useRegressionAutonomyGate = false;
            vehicleCmdSubscriber.enabled = false;
        }

        if (drivingStatusUIController != null)
        {
            drivingStatusUIController.useExternalCollisionTopicForUI = false;
            drivingStatusUIController.fallbackToLocalCollisionPublisher = true;
            drivingStatusUIController.forceLocalCollisionUiWhenTraining = true;
        }

        if (trafficLightDecisionEngine != null)
        {
            trafficLightDecisionEngine.useSceneTrafficLightState = true;
            trafficLightDecisionEngine.fallbackToRosSubscriberState = false;
            trafficLightDecisionEngine.useStopLinePerception = false;
        }

        if (wheelController != null)
            wheelController.externalControlEnabled = true;

        ApplySensorMode(useExternalTopicInput: false, fallbackToRaycastWhenExternalStale: true);
    }

    void ApplyTestMode()
    {
        // 추론: 외부 ROS 토픽 로직만 사용
        if (autoDriverRLAgent != null)
        {
            autoDriverRLAgent.externalRosCmdInputOnly = true;
            autoDriverRLAgent.forceInternalControlWhenTraining = true;
        }

        if (behaviorParameters != null)
            behaviorParameters.BehaviorType = BehaviorType.HeuristicOnly;

        if (regressionDrivingController != null)
        {
            regressionDrivingController.isAutonomousMode = false;
            regressionDrivingController.predictionOnlyMode = true;
        }

        if (vehicleCmdSubscriber != null)
        {
            vehicleCmdSubscriber.useRegressionAutonomyGate = false;
            vehicleCmdSubscriber.enabled = true;
        }

        if (drivingStatusUIController != null)
        {
            drivingStatusUIController.useExternalCollisionTopicForUI = true;
            drivingStatusUIController.fallbackToLocalCollisionPublisher = false;
            drivingStatusUIController.forceLocalCollisionUiWhenTraining = false;
        }

        if (trafficLightDecisionEngine != null)
        {
            trafficLightDecisionEngine.useSceneTrafficLightState = false;
            trafficLightDecisionEngine.fallbackToRosSubscriberState = true;
            trafficLightDecisionEngine.useStopLinePerception = true;
        }

        ApplySensorMode(useExternalTopicInput: true, fallbackToRaycastWhenExternalStale: false);
    }

    void ApplySensorMode(bool useExternalTopicInput, bool fallbackToRaycastWhenExternalStale)
    {
        if (ultrasonicSensors != null)
        {
            for (int i = 0; i < ultrasonicSensors.Length; i++)
            {
                SingleUltrasonicSensor sensor = ultrasonicSensors[i];
                if (sensor == null)
                    continue;

                sensor.useExternalTopicInput = useExternalTopicInput;
                sensor.fallbackToRaycastWhenExternalStale = fallbackToRaycastWhenExternalStale;
            }
        }

        if (radarSensors != null)
        {
            for (int i = 0; i < radarSensors.Length; i++)
            {
                SingleRadarSensor sensor = radarSensors[i];
                if (sensor == null)
                    continue;

                sensor.useExternalTopicInput = useExternalTopicInput;
                sensor.fallbackToRaycastWhenExternalStale = fallbackToRaycastWhenExternalStale;
            }
        }
    }

    void UpdateModeText()
    {
        if (modeText == null)
            return;

        modeText.text = selectedMode == RuntimeMode.Train
            ? "Mode: TRAIN (Internal)"
            : "Mode: TEST (External)";
    }

    static int CountValid<T>(T[] items) where T : Object
    {
        if (items == null)
            return 0;

        int count = 0;
        for (int i = 0; i < items.Length; i++)
        {
            if (items[i] != null)
                count++;
        }
        return count;
    }

    static T FindOne<T>(bool includeInactive) where T : Object
    {
        return includeInactive ? FindObjectOfType<T>(true) : FindObjectOfType<T>();
    }

    static T[] FindAll<T>(bool includeInactive) where T : Object
    {
        return includeInactive ? FindObjectsOfType<T>(true) : FindObjectsOfType<T>();
    }
}
