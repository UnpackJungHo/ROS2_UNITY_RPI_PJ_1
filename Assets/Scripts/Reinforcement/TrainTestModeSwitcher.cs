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
    /// <summary>
    /// 개별 ROS 토픽 컴포넌트의 모드별 활성화 설정.
    /// </summary>
    [System.Serializable]
    public class RosTopicEntry
    {
        [Tooltip("ROS 퍼블리셔/서브스크라이버/브리지 컴포넌트")]
        public MonoBehaviour component;
        [Tooltip("Train 모드에서 활성화")]
        public bool enableInTrain;
        [Tooltip("Test 모드에서 활성화")]
        public bool enableInTest = true;
    }

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
    public TrafficLightDecisionEngine trafficLightDecisionEngine;
    public VehicleMotionController wheelController;
    public BehaviorParameters behaviorParameters;

    [Header("ROS 토픽 개별 제어")]
    [Tooltip("각 ROS 퍼블리셔/서브스크라이버를 할당하고 모드별 활성화를 개별 설정.\n" +
             "CameraPublisher, CollisionWarningRosBridge는 rosPublishingEnabled 제어.\n" +
             "나머지는 enabled 제어.\n" +
             "[Populate ROS Topics] 컨텍스트 메뉴로 자동 탐색 가능.")]
    public RosTopicEntry[] rosTopics = new RosTopicEntry[0];

    [Header("Performance")]
    [Tooltip("학습 시 CameraRenderer 렌더 주기 (Hz). 0이면 변경 안 함.\n" +
             "DecisionPeriod=5 기준 5Hz면 충분 (기본 10Hz 대비 Camera.Render() 50% 감소)")]
    public float trainRenderRate = 5f;
    [Tooltip("테스트 시 CameraRenderer 렌더 주기 (Hz). 0이면 변경 안 함.")]
    public float testRenderRate = 10f;

    [Tooltip("학습 시 Physics.fixedDeltaTime 값. 0이면 변경 안 함.\n" +
             "0.04 = 25Hz (기본 0.02=50Hz 대비 FixedUpdate 50% 감소, 물리 안정성 테스트 필요)")]
    public float trainFixedDeltaTime = 0f;
    [Tooltip("테스트 시 Physics.fixedDeltaTime 복원 값. 0이면 변경 안 함.")]
    public float testFixedDeltaTime = 0f;

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
            int rosTopicCount = rosTopics != null ? rosTopics.Length : 0;
            lastAppliedSummary = $"Mode={selectedMode}, RosTopics={rosTopicCount}";

            //Debug.Log($"[TrainTestModeSwitcher] {lastAppliedSummary}");
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

        if (trafficLightDecisionEngine == null)
            trafficLightDecisionEngine = FindOne<TrafficLightDecisionEngine>(includeInactiveObjects);

        if (wheelController == null)
            wheelController = FindOne<VehicleMotionController>(includeInactiveObjects);

        if (behaviorParameters == null)
        {
            if (autoDriverRLAgent != null)
                behaviorParameters = autoDriverRLAgent.GetComponent<BehaviorParameters>();

            if (behaviorParameters == null)
                behaviorParameters = FindOne<BehaviorParameters>(includeInactiveObjects);
        }

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
        ApplyCameraRendererResolution(200, 66);
        if (trainRenderRate > 0f)
            ApplyCameraRendererRate(trainRenderRate);
        if (trainFixedDeltaTime > 0f && Application.isPlaying)
            Time.fixedDeltaTime = trainFixedDeltaTime;

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
            // #1: 카메라(trainRenderRate=5Hz)와 추론 주기 동기화 → 동일 이미지 중복 추론 제거
            if (trainRenderRate > 0f)
                regressionDrivingController.inferenceInterval = 1f / trainRenderRate;
            // #2: Academy 스텝 기반 분산 활성화 → 프레임 내 동시 추론 스파이크 감소
            regressionDrivingController.useAcademyStepStagger = true;
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
        ApplyRosTopicToggles(RuntimeMode.Train);
    }

    void ApplyTestMode()
    {
        // 추론: 외부 ROS 토픽 로직만 사용
        ApplyCameraRendererResolution(640, 480);
        if (testRenderRate > 0f)
            ApplyCameraRendererRate(testRenderRate);
        if (testFixedDeltaTime > 0f && Application.isPlaying)
            Time.fixedDeltaTime = testFixedDeltaTime;

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

        if (trafficLightDecisionEngine != null)
        {
            trafficLightDecisionEngine.useSceneTrafficLightState = false;
            trafficLightDecisionEngine.fallbackToRosSubscriberState = true;
            trafficLightDecisionEngine.useStopLinePerception = true;
        }

        ApplySensorMode(useExternalTopicInput: true, fallbackToRaycastWhenExternalStale: false);
        ApplyRosTopicToggles(RuntimeMode.Test);
    }

    /// <summary>
    /// rosTopics 배열의 각 항목에 대해 모드별 활성화/비활성화를 적용한다.
    /// CameraPublisher, CollisionWarningRosBridge는 rosPublishingEnabled 제어.
    /// 나머지는 enabled 제어.
    /// </summary>
    void ApplyRosTopicToggles(RuntimeMode mode)
    {
        if (rosTopics == null) return;

        foreach (var entry in rosTopics)
        {
            if (entry == null || entry.component == null) continue;

            bool shouldEnable = mode == RuntimeMode.Train ? entry.enableInTrain : entry.enableInTest;

            switch (entry.component)
            {
                case CameraPublisher cam:
                    cam.rosPublishingEnabled = shouldEnable;
                    break;
                case CollisionWarningRosBridge cwb:
                    cwb.rosPublishingEnabled = shouldEnable;
                    break;
                default:
                    entry.component.enabled = shouldEnable;
                    break;
            }
        }
    }

    /// <summary>
    /// 하위 오브젝트에서 모든 ROS 퍼블리셔/서브스크라이버를 탐색하여 rosTopics에 추가한다.
    /// 이미 등록된 컴포넌트는 건너뛴다.
    /// </summary>
    [ContextMenu("Populate ROS Topics")]
    public void PopulateRosTopics()
    {
        var list = new System.Collections.Generic.List<RosTopicEntry>();
        var existing = new System.Collections.Generic.HashSet<MonoBehaviour>();

        if (rosTopics != null)
        {
            foreach (var entry in rosTopics)
            {
                if (entry?.component != null)
                    existing.Add(entry.component);
            }
            list.AddRange(rosTopics);
        }

        System.Type[] rosTypes = new System.Type[]
        {
            typeof(CameraPublisher),
            typeof(PolicyCameraPublisher),
            typeof(LidarPublisher),
            typeof(ImuPublisher),
            typeof(OdometryPublisher),
            typeof(UltrasonicSensorPublisher),
            typeof(RadarSensorPublisher),
            typeof(SingleRadarSensorRosBridge),
            typeof(SingleUltrasonicSensorRosBridge),
            typeof(CollisionWarningRosBridge),
            typeof(ReinforcementObservationPublisher),
            typeof(ClockPublisher),
            typeof(StaticTfPublisher),
            typeof(TrafficLightStateSubscriber),
            typeof(StopLineStateSubscriber),
            typeof(VehicleCmdSubscriber),
        };

        foreach (var t in rosTypes)
        {
            foreach (var c in GetComponentsInChildren(t, includeInactiveObjects))
            {
                var mb = (MonoBehaviour)c;
                if (!existing.Contains(mb))
                {
                    list.Add(new RosTopicEntry
                    {
                        component = mb,
                        enableInTrain = false,
                        enableInTest = true
                    });
                    existing.Add(mb);
                }
            }
        }

        rosTopics = list.ToArray();
    }

    void ApplySensorMode(bool useExternalTopicInput, bool fallbackToRaycastWhenExternalStale)
    {
        // 초음파 Bridge에서 외부 입력 모드 제어
        foreach (var bridge in GetComponentsInChildren<SingleUltrasonicSensorRosBridge>(includeInactiveObjects))
        {
            if (bridge == null) continue;
            bridge.useExternalTopicInput = useExternalTopicInput;
            bridge.fallbackToRaycastWhenExternalStale = fallbackToRaycastWhenExternalStale;
        }

        // 레이더 Bridge에서 외부 입력 모드 제어
        foreach (var bridge in GetComponentsInChildren<SingleRadarSensorRosBridge>(includeInactiveObjects))
        {
            if (bridge == null) continue;
            bridge.useExternalTopicInput = useExternalTopicInput;
            bridge.fallbackToRaycastWhenExternalStale = fallbackToRaycastWhenExternalStale;
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

    /// <summary>
    /// 씬의 모든 CameraRenderer renderRate를 변경한다.
    /// </summary>
    void ApplyCameraRendererRate(float rate)
    {
        foreach (var cr in FindAll<CameraRenderer>(includeInactiveObjects))
            cr.renderRate = rate;
    }

    /// <summary>
    /// 씬의 모든 CameraRenderer 해상도를 변경한다.
    /// 런타임이면 즉시 RenderTexture를 재생성하고, 에디터 모드면 Inspector 값만 갱신한다.
    /// </summary>
    void ApplyCameraRendererResolution(int width, int height)
    {
        foreach (var cr in FindAll<CameraRenderer>(includeInactiveObjects))
        {
            if (Application.isPlaying)
                cr.SetResolution(width, height);
            else
            {
                cr.imageWidth = width;
                cr.imageHeight = height;
            }
        }
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
