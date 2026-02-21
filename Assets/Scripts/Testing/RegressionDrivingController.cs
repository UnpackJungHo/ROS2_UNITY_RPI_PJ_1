using UnityEngine;
using Unity.Sentis;

/// <summary>
/// Speed-Aware 회귀(Regression) 자율주행 컨트롤러 (Single View)
///
/// 분류 모델(AutonomousDrivingController)과 달리 7개 클래스가 아닌
/// steering [-1,1], throttle [0,1] 연속값을 직접 예측합니다.
///
/// 기능:
/// - P키: 자율주행 모드 토글
/// - WASD: 자율주행 중 사람 개입 (자동으로 수동 모드 전환)
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
    public CollisionWarningPublisher collisionWarningPublisher;

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

    // 텐서
    private TensorFloat frontInputTensor;
    private TensorFloat speedInputTensor;

    // ImageNet 정규화 상수
    private readonly float[] mean = { 0.485f, 0.456f, 0.406f };
    private readonly float[] std = { 0.229f, 0.224f, 0.225f };

    private float lastInferenceTime;
    private bool isModelLoaded = false;
    private bool warnedMissingCamera = false;
    private bool warnedMissingWorker = false;

    void Start()
    {
        AutoFindReferences();
        InitializeRenderTextures();
        LoadModel();

        Debug.Log($"[RegressionDriving] Speed-Aware Regression Controller Initialized");
        //Debug.Log($"  '{toggleKey}' 키: 자율주행 모드 토글");
        //Debug.Log($"  WASD: 자율주행 중 개입 (자동 수동 모드 전환)");
        //Debug.Log($"  개입 후 {autoResumeDelay}초 뒤 자동 AI 모드 복귀");
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
        if (collisionWarningPublisher == null)
            collisionWarningPublisher = FindObjectOfType<CollisionWarningPublisher>();
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
            // CPU 백엔드 사용: GPU 비동기 파이프라인에서 MakeReadable()이 이전 프레임
            // 출력 버퍼를 반환하는 문제(stale -0.384/0.185)를 방지한다.
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

    void Update()
    {
        if (frontCamera == null && cameraPublisher != null)
        {
            frontCamera = cameraPublisher.GetCamera();
            warnedMissingCamera = frontCamera == null;
        }

        if (Input.GetKeyDown(toggleKey))
        {
            // 수동 / 자동 모드 토글
            ToggleAutonomousMode();
        }

        if (isAutonomousMode)
        {
            if (IsManualInputDetected()) // 입력 들어오면
            {
                StartIntervention(); // 개입 시작 - AI 제어 중단, 수동 모드로 전환
            }

            if (isInterventionActive)
            {
                interventionTimer += Time.deltaTime;

                if (IsManualInputDetected())
                {
                    interventionTimer = 0f;
                }

                if (interventionTimer >= autoResumeDelay) // 0.5초 동안 입력 없으면 개입 종료 - AI 모드 복귀
                {
                    EndIntervention();
                }
            }
            else
            {
                if (EnsureInferenceReady() && Time.time - lastInferenceTime >= inferenceInterval)
                {
                    if (RunInference()) // 추론 시작
                        lastInferenceTime = Time.time;
                }

                if (!predictionOnlyMode)
                    ApplyAIControl(); // 차량 센서(레이더, 초음파 센서) 및 신호등에 따른 주행 제어 적용
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

    /// <summary>
    /// 현재 개입 상태인지 반환 (DrivingDataCollectorV2 연동용)
    /// </summary>
    public bool IsInterventionActive()
    {
        return isInterventionActive;
    }

    /// <summary>
    /// AI가 예측한 steering 값 반환
    /// </summary>
    public float GetPredictedSteering()
    {
        return predictedSteering;
    }

    /// <summary>
    /// AI가 예측한 throttle 값 반환
    /// </summary>
    public float GetPredictedThrottle()
    {
        return predictedThrottle;
    }

    public float GetAppliedSteering()
    {
        return appliedSteering;
    }

    public float GetAppliedThrottle()
    {
        return appliedThrottle;
    }

    public int GetInterventionCount()
    {
        return interventionCount;
    }

    public float GetInterventionRemainingTime()
    {
        return Mathf.Max(0f, autoResumeDelay - interventionTimer);
    }

    public bool IsModelLoaded()
    {
        return isModelLoaded;
    }

    public float GetCurrentSpeedMS()
    {
        return wheelController != null ? wheelController.GetSpeedMS() : 0f;
    }

    /// <summary>
    /// 에피소드 재시작 시 회귀 컨트롤러 내부 상태를 초기화하고,
    /// stale 예측값 재사용을 막기 위해 필요 시 즉시 1회 추론을 수행한다.
    /// </summary>
    public void ResetForEpisodeRestart(bool forceInference)
    {
        isInterventionActive = false;
        interventionTimer = 0f;

        // [DEBUG] 리셋 직전 값 기록
        Debug.Log($"[RegressionReset] BEFORE reset: steer={predictedSteering:F4}, throttle={predictedThrottle:F4} | isAutonomous={isAutonomousMode}, forceInference={forceInference}");

        predictedSteering = 0f;
        predictedThrottle = 0f;
        appliedSteering = 0f;
        appliedThrottle = 0f;

        // 다음 Update에서 바로 추론이 가능하도록 타이머를 되감는다.
        lastInferenceTime = Time.time - Mathf.Max(0.001f, inferenceInterval);

        if (frontCamera == null && cameraPublisher != null)
            frontCamera = cameraPublisher.GetCamera();

        // [DEBUG] 카메라 위치 (TeleportRoot 후 즉시 읽은 값)
        if (frontCamera != null)
            Debug.Log($"[RegressionReset] Camera world pos at reset: {frontCamera.transform.position} | cam={frontCamera.name}");
        else
            Debug.LogWarning("[RegressionReset] frontCamera is NULL at reset!");

        if (!forceInference)
        {
            Debug.Log("[RegressionReset] forceInference=false → skipping immediate RunInference. predictedSteering/Throttle = 0.");
            return;
        }

        if (!isAutonomousMode)
        {
            Debug.LogWarning("[RegressionReset] forceInference=true 이지만 isAutonomousMode=false → RunInference 건너뜀!");
            return;
        }

        if (!EnsureInferenceReady())
        {
            Debug.LogWarning("[RegressionReset] forceInference=true 이지만 EnsureInferenceReady()=false → RunInference 건너뜀!");
            return;
        }

        Debug.Log($"[RegressionReset] RunInference 즉시 호출 (forceInference=true). 카메라 pos={frontCamera?.transform.position}");
        bool inferenceOk = RunInference();
        Debug.Log($"[RegressionReset] AFTER forceInference: ok={inferenceOk}, steer={predictedSteering:F4}, throttle={predictedThrottle:F4}");
    }

    bool EnsureInferenceReady()
    {
        if (cameraPublisher == null)
            cameraPublisher = FindObjectOfType<CameraPublisher>();

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

    bool RunInference() // 카메라 이미지 캡처 → 텐서 변환 → 모델 추론 → 제어값 적용
    {
        if (!EnsureInferenceReady())
            return false;

        try
        {
            // 1. 카메라 이미지 캡처
            // [DEBUG] 추론 시작 시 카메라 위치 기록
            Vector3 camPos = frontCamera != null ? frontCamera.transform.position : Vector3.zero;
            CaptureCamera(frontCamera, frontRenderTexture, frontTexture);

            // 2. 이미지 텐서 생성
            frontInputTensor = TextureToTensor(frontTexture, frontImageHeight, frontImageWidth);

            // 3. 속도 텐서 생성 (정규화)
            float currentSpeed = wheelController != null ? wheelController.GetSpeedMS() : 0f;
            float normalizedSpeed = currentSpeed / speedNormalize;
            speedInputTensor = new TensorFloat(new TensorShape(1, 1), new float[] { normalizedSpeed });

            // 4. 추론 실행
            worker.SetInput("front_image", frontInputTensor);
            worker.SetInput("speed", speedInputTensor);
            worker.Execute();

            // 5. 결과 읽기 - [steering, throttle] 연속값
            TensorFloat outputTensor = worker.PeekOutput("output") as TensorFloat;
            if (outputTensor != null)
            {
                outputTensor.MakeReadable();
                // 모델 출력: steering [-1, 1], throttle [0, 1]
                predictedSteering = outputTensor[0];
                predictedThrottle = outputTensor[1];

                // 직접 제어값으로 사용
                appliedSteering = predictedSteering;
                appliedThrottle = predictedThrottle;

                // [DEBUG] 추론 결과 로그 (camPos와 함께 확인)
                //Debug.Log($"[RegressionInference] camPos={camPos} speed={currentSpeed:F3} → steer={predictedSteering:F4}, throttle={predictedThrottle:F4}");
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
        finally
        {
            // 텐서 정리
            frontInputTensor?.Dispose();
            speedInputTensor?.Dispose();
        }
    }

    void CaptureCamera(Camera cam, RenderTexture rt, Texture2D tex)
    {
        RenderTexture originalTarget = cam.targetTexture;
        cam.targetTexture = rt;
        cam.Render();

        RenderTexture.active = rt;
        tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        tex.Apply();
        RenderTexture.active = null;

        cam.targetTexture = originalTarget;
    }

    TensorFloat TextureToTensor(Texture2D texture, int height, int width)
    {
        Color[] pixels = texture.GetPixels();
        float[] tensorData = new float[3 * height * width];

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int srcY = height - 1 - y;
                int pixelIdx = srcY * width + x;
                Color pixel = pixels[pixelIdx];

                tensorData[0 * height * width + y * width + x] = (pixel.r - mean[0]) / std[0];
                tensorData[1 * height * width + y * width + x] = (pixel.g - mean[1]) / std[1];
                tensorData[2 * height * width + y * width + x] = (pixel.b - mean[2]) / std[2];
            }
        }

        return new TensorFloat(new TensorShape(1, 3, height, width), tensorData);
    }

    void ApplyAIControl()
    {
        if (wheelController == null) return;

        float currentSpeed = wheelController.GetSpeedMS();
        float adjustedThrottle = appliedThrottle;
        float brakeCommand = 0f;

        // 최대 속도 초과 시 감속
        if (currentSpeed >= maxSpeedLimit)
        {
            adjustedThrottle = Mathf.Min(adjustedThrottle, 0f);
        }

        if (enableSafetyOverride)
        {
            // 1) 센서 기반 충돌 오버라이드
            if (collisionWarningPublisher != null)
            {
                CollisionWarningPublisher.WarningLevel level = collisionWarningPublisher.GetWarningLevel();
                if (level >= CollisionWarningPublisher.WarningLevel.EmergencyStop)
                {
                    adjustedThrottle = 0f;
                    brakeCommand = 1f;
                }
                else if (level >= CollisionWarningPublisher.WarningLevel.Brake)
                {
                    adjustedThrottle = 0f;
                    brakeCommand = Mathf.Max(brakeCommand, brakeLevelBrake);
                }
                else if (level >= CollisionWarningPublisher.WarningLevel.Warning)
                {
                    adjustedThrottle = Mathf.Min(adjustedThrottle, 0.15f);
                    if (Mathf.Abs(currentSpeed) >= Mathf.Max(0f, warningBrakeMinSpeed))
                        brakeCommand = Mathf.Max(brakeCommand, warningBrake);
                }
                else if (level >= CollisionWarningPublisher.WarningLevel.SlowDown)
                {
                    adjustedThrottle = Mathf.Min(adjustedThrottle, appliedThrottle * slowDownThrottleScale);
                }
            }

            // 2) 신호등 기반 오버라이드
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
