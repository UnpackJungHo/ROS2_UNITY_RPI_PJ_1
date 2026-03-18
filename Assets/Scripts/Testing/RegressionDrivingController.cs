using UnityEngine;
using UnityEngine.Rendering;
using Unity.Sentis;
using Unity.Collections;

/// <summary>
/// Speed-Aware 회귀(Regression) 자율주행 컨트롤러 (Single View)
///
/// 분류 모델(AutonomousDrivingController)과 달리 7개 클래스가 아닌
/// steering [-1,1], throttle [0,1] 연속값을 직접 예측합니다.
///
/// 기능:
/// - P키: 자율주행 모드 토글
/// - WASD: 자율주행 중 개입 (자동으로 수동 모드 전환)
/// - 3초 후 자동으로 AI 모드 복귀
/// - DrivingDataCollectorV2 연동하여 개입 데이터 수집
///
/// 모델 입력: front_image (1,3,66,200) + speed (1,1)
/// 모델 출력: output (1,2) → [steering, throttle]
/// </summary>
public class RegressionDrivingController : MonoBehaviour
{
    [Header("AI Model")]
    [Tooltip("ONNX 회귀 모델 파일 (driving_regression.onnx)")]
    public ModelAsset modelAsset;

    [Header("References")]
    [Tooltip("차량 컨트롤러")]
    public WheelTest wheelController;

    [Tooltip("CameraPublisher (Front View 카메라)")]
    public CameraPublisher cameraPublisher;

    [Tooltip("데이터 수집기 (DAgger 연동)")]
    public DrivingDataCollectorV2 dataCollector;

    [Tooltip("충돌 경고 융합기 (선택 - 안전 오버라이드용)")]
    public CollisionWarningEngine collisionWarningEngine;

    [Tooltip("신호등 판단 엔진 (선택 - 안전 오버라이드용)")]
    public TrafficLightDecisionEngine trafficLightDecisionEngine;

    [Header("Image Settings")]
    public int frontImageWidth = 200;
    public int frontImageHeight = 66;

    [Header("Inference Settings")]
    [Tooltip("추론 주기 (초)")]
    public float inferenceInterval = 0.1f;

    [Tooltip("속도 정규화 기준값 (학습 시 speed_norm과 동일하게)")]
    public float speedNormalize = 3.0f;

    [Header("DAgger Settings")]
    public KeyCode toggleKey = KeyCode.P;

    [Tooltip("개입 후 AI 모드 자동 복귀 대기 시간 (초)")]
    public float autoResumeDelay = 3.0f;

    [Tooltip("현재 모드")]
    public bool isAutonomousMode = false;

    [Header("Safety")]
    [Tooltip("최대 속도 제한 (m/s)")]
    public float maxSpeedLimit = 3f;
    [Tooltip("센서/신호등 기반 오버라이드 활성화")]
    public bool enableSafetyOverride = true;
    [Range(0f, 1f)]
    [Tooltip("SlowDown 단계에서 throttle 스케일")]
    public float slowDownThrottleScale = 0.55f;
    [Range(0f, 1f)]
    [Tooltip("Warning 단계 최소 브레이크")]
    public float warningBrake = 0.45f;
    [Tooltip("Warning 단계에서 브레이크를 적용할 최소 속도 (m/s). 저속에서는 크리핑을 허용")]
    public float warningBrakeMinSpeed = 0.6f;
    [Range(0f, 1f)]
    [Tooltip("Brake 단계 최소 브레이크")]
    public float brakeLevelBrake = 0.8f;

    [Header("Performance (Multi-Vehicle)")]
    [Tooltip("다중 차량 시 추론 시점을 분산")]
    public bool enableInferenceStagger = true;

    [Tooltip("predictionOnlyMode=true 시 Academy.StepCount 기반으로 추론 시점 분산\n" +
             "(TrainTestModeSwitcher가 학습 모드 진입 시 자동 활성화)")]
    public bool useAcademyStepStagger = false;
    [Tooltip("Academy 스텝 기반 분산 주기 (DecisionPeriod와 동일 값으로 설정)")]
    public int staggerPeriod = 5;

    [Header("RL Residual Mode")]
    [Tooltip("true면 추론만 하고 차량에 직접 적용하지 않음 (RL Agent가 읽어서 보정)")]
    public bool predictionOnlyMode = false;

    [Header("Debug (Read Only)")]
    [SerializeField] private float predictedSteering = 0f;
    [SerializeField] private float predictedThrottle = 0f;
    [SerializeField] private float appliedSteering = 0f;
    [SerializeField] private float appliedThrottle = 0f;
    [SerializeField] private bool isInterventionActive = false;
    [SerializeField] private float interventionTimer = 0f;
    [SerializeField] private int interventionCount = 0;
    [SerializeField] private int inferenceSuccessCount = 0;
    [SerializeField] private int inferenceFailureCount = 0;
    [SerializeField] private string lastInferenceError = "None";

    // Sentis
    private Model runtimeModel;
    private IWorker worker;

    // 카메라 및 렌더링
    private Camera frontCamera;
    private RenderTexture frontRenderTexture;
    private Texture2D frontTexture;

    // 텐서 (Start()에서 미리 생성하여 매 추론마다 재사용 - GC 방지)
    private TensorFloat frontInputTensor;
    private TensorFloat speedInputTensor;
    private TensorShape frontInputShape;
    private TensorShape speedInputShape;

    // ImageNet 정규화 상수
    private readonly float[] mean = { 0.485f, 0.456f, 0.406f };
    private readonly float[] std = { 0.229f, 0.224f, 0.225f };

    // 프리할당 버퍼 (GC 방지)
    private float[] tensorDataBuffer;
    private float[] speedBuffer = new float[1];

    private float lastInferenceTime;
    private int academyStaggerOffset = 0;
    private int lastAcademyStep = -1;
    private int staggerDivisor = 1; // Mathf.Max(1, staggerPeriod) 캐시 — Start()에서 확정

    // 정적 카운터: Play 진입 시 리셋 → 8대가 0~(staggerPeriod-1)에 균등 배분
    private static int s_academyOffsetCounter;
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    static void ResetAcademyOffsetCounter() => s_academyOffsetCounter = 0;

    private bool isModelLoaded = false;
    private bool warnedMissingCamera = false;
    private bool warnedMissingWorker = false;

    // AsyncGPUReadback 상태 (CaptureCamera GPU stall 제거)
    private byte[] latestCaptureData;
    private bool hasCaptureData = false;
    private bool captureReadbackInProgress = false;

    void Start()
    {
        AutoFindReferences();
        InitializeRenderTextures();
        LoadModel();

        if (enableInferenceStagger)
        {
            float stagger = Mathf.Abs(gameObject.GetInstanceID() % 97) / 97f * inferenceInterval;
            lastInferenceTime = Time.time + stagger - inferenceInterval;
        }

        // Academy 스텝 기반 분산: 등록 순서 기반 균등 배분 (0,1,2,3,4,0,1,2 → 최대 2대/슬롯)
        staggerDivisor = Mathf.Max(1, staggerPeriod);
        academyStaggerOffset = System.Threading.Interlocked.Increment(ref s_academyOffsetCounter) % staggerDivisor;
    }

    void AutoFindReferences()
    {
        if (cameraPublisher == null)
            cameraPublisher = FindObjectOfType<CameraPublisher>();

        if (wheelController == null)
        {
            wheelController = GetComponent<WheelTest>();
            if (wheelController == null)
                wheelController = GetComponentInParent<WheelTest>();
            if (wheelController == null)
                wheelController = FindObjectOfType<WheelTest>();
        }

        if (dataCollector == null)
            dataCollector = FindObjectOfType<DrivingDataCollectorV2>();
        if (collisionWarningEngine == null)
            collisionWarningEngine = FindObjectOfType<CollisionWarningEngine>();
        if (trafficLightDecisionEngine == null)
            trafficLightDecisionEngine = FindObjectOfType<TrafficLightDecisionEngine>();

        if (cameraPublisher == null)
            Debug.LogError("[RegressionDriving] CameraPublisher를 찾을 수 없습니다!");
        if (wheelController == null)
            Debug.LogError("[RegressionDriving] WheelTest를 찾을 수 없습니다!");
    }

    void InitializeRenderTextures()
    {
        frontRenderTexture = new RenderTexture(frontImageWidth, frontImageHeight, 24);
        frontTexture = new Texture2D(frontImageWidth, frontImageHeight, TextureFormat.RGB24, false);
        tensorDataBuffer = new float[3 * frontImageHeight * frontImageWidth];

        // TensorFloat를 미리 생성하여 추론 루프에서 재사용 (매 추론마다 new 방지 → GC 감소)
        frontInputShape = new TensorShape(1, 3, frontImageHeight, frontImageWidth);
        speedInputShape = new TensorShape(1, 1);
        frontInputTensor = TensorFloat.Zeros(frontInputShape);
        speedInputTensor = TensorFloat.Zeros(speedInputShape);
    }

    void LoadModel()
    {
        if (modelAsset == null)
        {
            Debug.LogError("[RegressionDriving] Model Asset이 할당되지 않았습니다!");
            return;
        }

        try
        {
            worker?.Dispose();
            worker = null;
            runtimeModel = ModelLoader.Load(modelAsset);
            // CPU 백엔드 유지: GPUCompute + FlushSchedule(true)는 단기에는 stale이 없으나
            // 장기 운용(~35초+) 시 동일 값 고착 재발 확인됨 (Sentis 1.2.0-exp.2).
            // Sentis 업그레이드 후 재시도 필요.
            worker = WorkerFactory.CreateWorker(BackendType.CPU, runtimeModel);

            isModelLoaded = runtimeModel != null && worker != null;
            warnedMissingWorker = false;
            lastInferenceError = "None";
            Debug.Log("[RegressionDriving] Regression 모델 로드 완료");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[RegressionDriving] 모델 로드 실패: {e.Message}");
            isModelLoaded = false;
            worker = null;
            lastInferenceError = e.Message;
        }
    }

    /// <summary>
    /// predictionOnlyMode=true 일 때 물리 프레임에 동기화된 추론 수행.
    /// AutoDriverRLAgent(DecisionRequester) 와 같은 FixedUpdate 체인에서 실행되어
    /// base prediction 스테일 문제를 해소한다.
    /// Script Execution Order: RegressionDrivingController → AutoDriverRLAgent 순서로 설정 필요.
    /// </summary>
    void FixedUpdate()
    {
        if (!predictionOnlyMode) return;
        if (!isAutonomousMode) return;

        if (useAcademyStepStagger)
        {
            int step = Unity.MLAgents.Academy.Instance.StepCount;
            if (step == lastAcademyStep) return;
            if ((step + academyStaggerOffset) % staggerDivisor != 0) return;
            lastAcademyStep = step;
        }

        if (EnsureInferenceReady())
            RunInference();
    }

    void Update()
    {
        // 카메라 초기화는 항상 수행
        if (frontCamera == null && cameraPublisher != null)
        {
            frontCamera = cameraPublisher.GetCamera();
            warnedMissingCamera = frontCamera == null;
        }

        // 토글 키는 항상 처리
        if (Input.GetKeyDown(toggleKey))
        {
            ToggleAutonomousMode();
        }

        // predictionOnlyMode 시 추론은 FixedUpdate에서 처리 — Update에서는 스킵
        if (predictionOnlyMode) return;

        if (isAutonomousMode)
        {
            if (IsManualInputDetected())
            {
                StartIntervention();
            }

            if (isInterventionActive)
            {
                interventionTimer += Time.deltaTime;

                if (IsManualInputDetected())
                {
                    interventionTimer = 0f;
                }

                if (interventionTimer >= autoResumeDelay)
                {
                    EndIntervention();
                }
            }
            else
            {
                bool shouldInfer = EnsureInferenceReady() && Time.time - lastInferenceTime >= inferenceInterval;

                if (shouldInfer)
                {
                    if (RunInference())
                        lastInferenceTime = Time.time;
                }

                ApplyAIControl();
            }
        }
    }

    bool IsManualInputDetected()
    {
        return Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow) ||
               Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow) ||
               Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow) ||
               Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow);
    }

    void ToggleAutonomousMode()
    {
        isAutonomousMode = !isAutonomousMode;
        isInterventionActive = false;
        interventionTimer = 0f;

        if (wheelController != null)
            wheelController.externalControlEnabled = isAutonomousMode && !isInterventionActive;

        Debug.Log($"[RegressionDriving] 자율주행 모드: {(isAutonomousMode ? "ON" : "OFF")}");
    }

    public bool IsInterventionActive() => isInterventionActive;
    public float GetPredictedSteering() => predictedSteering;
    public float GetPredictedThrottle() => predictedThrottle;
    public float GetAppliedSteering() => appliedSteering;
    public float GetAppliedThrottle() => appliedThrottle;
    public int GetInterventionCount() => interventionCount;
    public float GetInterventionRemainingTime() => Mathf.Max(0f, autoResumeDelay - interventionTimer);
    public bool IsModelLoaded() => isModelLoaded;
    public float GetCurrentSpeedMS() => wheelController != null ? wheelController.GetSpeedMS() : 0f;

    /// <summary>
    /// 에피소드 재시작 시 회귀 컨트롤러 내부 상태를 초기화하고,
    /// stale 예측값 재사용을 막기 위해 필요 시 즉시 1회 추론을 수행한다.
    /// </summary>
    public void ResetForEpisodeRestart(bool forceInference)
    {
        isInterventionActive = false;
        interventionTimer = 0f;

        predictedSteering = 0f;
        predictedThrottle = 0f;
        appliedSteering = 0f;
        appliedThrottle = 0f;

        lastInferenceTime = Time.time - Mathf.Max(0.001f, inferenceInterval);
        lastAcademyStep = -1;

        if (frontCamera == null && cameraPublisher != null)
            frontCamera = cameraPublisher.GetCamera();

        if (!forceInference) return;
        if (!isAutonomousMode) return;
        if (!EnsureInferenceReady()) return;

        RunInference();
    }

    bool EnsureInferenceReady()
    {
        if (cameraPublisher == null)
            cameraPublisher = GetComponentInChildren<CameraPublisher>();

        if (frontCamera == null && cameraPublisher != null)
            frontCamera = cameraPublisher.GetCamera();

        if (frontCamera == null)
        {
            if (!warnedMissingCamera)
            {
                Debug.LogWarning("[RegressionDriving] frontCamera가 null이라 추론을 건너뜁니다.");
                warnedMissingCamera = true;
            }
            return false;
        }
        warnedMissingCamera = false;

        if (runtimeModel == null || !isModelLoaded)
            LoadModel();

        if (worker == null && runtimeModel != null)
        {
            try
            {
                worker = WorkerFactory.CreateWorker(BackendType.CPU, runtimeModel);
            }
            catch (System.Exception e)
            {
                lastInferenceError = e.Message;
                worker = null;
            }
        }

        if (worker == null)
        {
            if (!warnedMissingWorker)
            {
                Debug.LogError("[RegressionDriving] worker가 null이라 추론을 수행할 수 없습니다.");
                warnedMissingWorker = true;
            }
            return false;
        }

        warnedMissingWorker = false;
        return true;
    }

    void StartIntervention()
    {
        if (isInterventionActive) return;

        isInterventionActive = true;
        interventionTimer = 0f;
        interventionCount++;

        if (wheelController != null)
            wheelController.externalControlEnabled = false;

        Debug.Log($"[RegressionDriving] 개입 #{interventionCount} 시작 - 수동 모드");
    }

    void EndIntervention()
    {
        isInterventionActive = false;
        interventionTimer = 0f;

        if (wheelController != null)
            wheelController.externalControlEnabled = true;

        Debug.Log($"[RegressionDriving] 개입 종료 - AI 모드 복귀");
    }

    bool RunInference()
    {
        if (!EnsureInferenceReady())
            return false;

        try
        {
            CaptureCamera(frontCamera, frontRenderTexture, frontTexture);

            if (!hasCaptureData)
                return false;

            FillTensorDataBuffer(latestCaptureData, frontImageHeight, frontImageWidth);
            var frontData = frontInputTensor?.tensorOnDevice as ArrayTensorData;
            if (frontData != null)
            {
                NativeTensorArray.Copy(tensorDataBuffer, frontData.array, tensorDataBuffer.Length);
            }
            else
            {
                frontInputTensor?.Dispose();
                frontInputTensor = new TensorFloat(frontInputShape, tensorDataBuffer);
            }

            float currentSpeed = wheelController != null ? wheelController.GetSpeedMS() : 0f;
            speedBuffer[0] = currentSpeed / speedNormalize;
            var speedData = speedInputTensor?.tensorOnDevice as ArrayTensorData;
            if (speedData != null)
            {
                speedData.array.Set<float>(0, speedBuffer[0]);
            }
            else
            {
                speedInputTensor?.Dispose();
                speedInputTensor = new TensorFloat(speedInputShape, speedBuffer);
            }

            worker.SetInput("front_image", frontInputTensor);
            worker.SetInput("speed", speedInputTensor);
            worker.Execute();

            TensorFloat outputTensor = worker.PeekOutput("output") as TensorFloat;
            if (outputTensor != null)
            {
                outputTensor.MakeReadable();
                predictedSteering = outputTensor[0];
                predictedThrottle = outputTensor[1];
                appliedSteering = predictedSteering;
                appliedThrottle = predictedThrottle;
            }

            inferenceSuccessCount++;
            lastInferenceError = "None";
            return true;
        }
        catch (System.Exception e)
        {
            predictedSteering = 0f;
            predictedThrottle = 0f;
            appliedSteering = 0f;
            appliedThrottle = 0f;

            inferenceFailureCount++;
            lastInferenceError = e.Message;
            Debug.LogError($"[RegressionDriving] 추론 실패: {e.Message}");

            worker?.Dispose();
            worker = null;
            isModelLoaded = runtimeModel != null;
            return false;
        }
    }

    void CaptureCamera(Camera cam, RenderTexture rt, Texture2D tex)
    {
        RenderTexture sourceRT = cameraPublisher != null ? cameraPublisher.GetRenderTexture() : null;
        if (sourceRT != null)
        {
            Graphics.Blit(sourceRT, rt);
        }
        else
        {
            RenderTexture originalTarget = cam.targetTexture;
            cam.targetTexture = rt;
            cam.Render();
            cam.targetTexture = originalTarget;
        }

        if (!captureReadbackInProgress)
        {
            captureReadbackInProgress = true;
            AsyncGPUReadback.Request(rt, 0, TextureFormat.RGB24, OnCaptureReadbackComplete);
        }
    }

    void OnCaptureReadbackComplete(AsyncGPUReadbackRequest request)
    {
        captureReadbackInProgress = false;

        if (request.hasError)
            return;

        var data = request.GetData<byte>();
        if (latestCaptureData == null || latestCaptureData.Length != data.Length)
            latestCaptureData = new byte[data.Length];

        data.CopyTo(latestCaptureData);
        hasCaptureData = true;
    }

    void FillTensorDataBuffer(byte[] rawBytes, int height, int width)
    {
        int hw = height * width;
        int chR = 0;
        int chG = hw;
        int chB = hw * 2;

        float invStdR = 1f / std[0], invStdG = 1f / std[1], invStdB = 1f / std[2];

        for (int y = 0; y < height; y++)
        {
            int srcY = height - 1 - y;
            int srcRowOffset = srcY * width * 3;
            int dstRowOffset = y * width;

            for (int x = 0; x < width; x++)
            {
                int byteIdx = srcRowOffset + x * 3;
                float r = rawBytes[byteIdx] / 255f;
                float g = rawBytes[byteIdx + 1] / 255f;
                float b = rawBytes[byteIdx + 2] / 255f;

                tensorDataBuffer[chR + dstRowOffset + x] = (r - mean[0]) * invStdR;
                tensorDataBuffer[chG + dstRowOffset + x] = (g - mean[1]) * invStdG;
                tensorDataBuffer[chB + dstRowOffset + x] = (b - mean[2]) * invStdB;
            }
        }
    }

    void ApplyAIControl()
    {
        if (wheelController == null) return;

        float currentSpeed = wheelController.GetSpeedMS();
        float adjustedThrottle = appliedThrottle;
        float brakeCommand = 0f;

        if (currentSpeed >= maxSpeedLimit)
        {
            adjustedThrottle = Mathf.Min(adjustedThrottle, 0f);
        }

        if (enableSafetyOverride)
        {
            if (collisionWarningEngine != null)
            {
                CollisionWarningEngine.WarningLevel level = collisionWarningEngine.GetWarningLevel();
                if (level >= CollisionWarningEngine.WarningLevel.EmergencyStop)
                {
                    adjustedThrottle = 0f;
                    brakeCommand = 1f;
                }
                else if (level >= CollisionWarningEngine.WarningLevel.Brake)
                {
                    adjustedThrottle = 0f;
                    brakeCommand = Mathf.Max(brakeCommand, brakeLevelBrake);
                }
                else if (level >= CollisionWarningEngine.WarningLevel.Warning)
                {
                    adjustedThrottle = Mathf.Min(adjustedThrottle, 0.15f);
                    if (Mathf.Abs(currentSpeed) >= Mathf.Max(0f, warningBrakeMinSpeed))
                        brakeCommand = Mathf.Max(brakeCommand, warningBrake);
                }
                else if (level >= CollisionWarningEngine.WarningLevel.SlowDown)
                {
                    adjustedThrottle = Mathf.Min(adjustedThrottle, appliedThrottle * slowDownThrottleScale);
                }
            }

            if (trafficLightDecisionEngine != null)
            {
                TrafficLightDecisionEngine.TrafficDecision decision = trafficLightDecisionEngine.GetDecision();
                if (decision == TrafficLightDecisionEngine.TrafficDecision.Stop)
                {
                    adjustedThrottle = 0f;
                    brakeCommand = Mathf.Max(brakeCommand, trafficLightDecisionEngine.GetRecommendedBrake());
                }
                else if (decision == TrafficLightDecisionEngine.TrafficDecision.Caution)
                {
                    adjustedThrottle = Mathf.Min(
                        adjustedThrottle,
                        appliedThrottle * trafficLightDecisionEngine.GetRecommendedThrottleScale()
                    );
                    brakeCommand = Mathf.Max(brakeCommand, trafficLightDecisionEngine.GetRecommendedBrake());
                }
            }
        }

        wheelController.SetSteering(appliedSteering);
        wheelController.SetThrottle(adjustedThrottle);
        wheelController.SetBrake(brakeCommand);
    }

    void OnDestroy()
    {
        frontInputTensor?.Dispose();
        speedInputTensor?.Dispose();
        worker?.Dispose();

        if (frontRenderTexture != null) Destroy(frontRenderTexture);
        if (frontTexture != null) Destroy(frontTexture);
    }
}
