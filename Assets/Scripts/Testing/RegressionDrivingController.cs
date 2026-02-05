using UnityEngine;
using Unity.Sentis;
using TMPro;

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

    [Header("UI References")]
    public TextMeshProUGUI uiModeText;
    public TextMeshProUGUI uiActionText;
    public TextMeshProUGUI uiControlText;
    public TextMeshProUGUI uiStatsText;
    public TextMeshProUGUI uiGuideText;

    [Header("References")]
    [Tooltip("차량 컨트롤러")]
    public WheelTest wheelController;

    [Tooltip("CameraPublisher (Front View 카메라)")]
    public CameraPublisher cameraPublisher;

    [Tooltip("데이터 수집기 (DAgger 연동)")]
    public DrivingDataCollectorV2 dataCollector;

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

    [Header("Debug (Read Only)")]
    [SerializeField] private float predictedSteering = 0f;
    [SerializeField] private float predictedThrottle = 0f;
    [SerializeField] private float appliedSteering = 0f;
    [SerializeField] private float appliedThrottle = 0f;
    [SerializeField] private bool isInterventionActive = false;
    [SerializeField] private float interventionTimer = 0f;
    [SerializeField] private int interventionCount = 0;

    // Sentis
    private Model runtimeModel;
    private Worker worker;

    // 카메라 및 렌더링
    private Camera frontCamera;
    private RenderTexture frontRenderTexture;
    private Texture2D frontTexture;

    // 텐서
    private Tensor<float> frontInputTensor;
    private Tensor<float> speedInputTensor;

    // ImageNet 정규화 상수
    private readonly float[] mean = { 0.485f, 0.456f, 0.406f };
    private readonly float[] std = { 0.229f, 0.224f, 0.225f };

    private float lastInferenceTime;
    private bool isModelLoaded = false;

    void Start()
    {
        AutoFindReferences();
        InitializeRenderTextures();
        LoadModel();

        Debug.Log($"[RegressionDriving] Speed-Aware Regression Controller Initialized");
        Debug.Log($"  '{toggleKey}' 키: 자율주행 모드 토글");
        Debug.Log($"  WASD: 자율주행 중 개입 (자동 수동 모드 전환)");
        Debug.Log($"  개입 후 {autoResumeDelay}초 뒤 자동 AI 모드 복귀");
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
            runtimeModel = ModelLoader.Load(modelAsset);
            worker = new Worker(runtimeModel, BackendType.GPUCompute);

            isModelLoaded = true;
            Debug.Log("[RegressionDriving] Regression 모델 로드 완료");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[RegressionDriving] 모델 로드 실패: {e.Message}");
            isModelLoaded = false;
        }
    }

    void Update()
    {
        if (frontCamera == null && cameraPublisher != null)
            frontCamera = cameraPublisher.GetCamera();

        if (Input.GetKeyDown(toggleKey))
        {
            ToggleAutonomousMode();
        }

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
                if (isModelLoaded && Time.time - lastInferenceTime >= inferenceInterval)
                {
                    RunInference();
                    lastInferenceTime = Time.time;
                }

                ApplyAIControl();
            }
        }

        UpdateUI();
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

    void RunInference()
    {
        if (frontCamera == null) return;

        // 1. 카메라 이미지 캡처
        CaptureCamera(frontCamera, frontRenderTexture, frontTexture);

        // 2. 이미지 텐서 생성
        frontInputTensor = TextureToTensor(frontTexture, frontImageHeight, frontImageWidth);

        // 3. 속도 텐서 생성 (정규화)
        float currentSpeed = wheelController != null ? wheelController.GetSpeedMS() : 0f;
        float normalizedSpeed = currentSpeed / speedNormalize;
        speedInputTensor = new Tensor<float>(new TensorShape(1, 1), new float[] { normalizedSpeed });

        // 4. 추론 실행
        worker.SetInput("front_image", frontInputTensor);
        worker.SetInput("speed", speedInputTensor);
        worker.Schedule();

        // 5. 결과 읽기 - [steering, throttle] 연속값
        using (Tensor<float> outputTensor = worker.PeekOutput("output") as Tensor<float>)
        {
            if (outputTensor != null)
            {
                using (Tensor<float> cpuTensor = outputTensor.ReadbackAndClone())
                {
                    // 모델 출력: steering [-1, 1], throttle [0, 1]
                    predictedSteering = cpuTensor[0];
                    predictedThrottle = cpuTensor[1];

                    // 직접 제어값으로 사용
                    appliedSteering = predictedSteering;
                    appliedThrottle = predictedThrottle;
                }
            }
        }

        // 텐서 정리
        frontInputTensor?.Dispose();
        speedInputTensor?.Dispose();
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

    Tensor<float> TextureToTensor(Texture2D texture, int height, int width)
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

        return new Tensor<float>(new TensorShape(1, 3, height, width), tensorData);
    }

    void ApplyAIControl()
    {
        if (wheelController == null) return;

        float currentSpeed = wheelController.GetSpeedMS();
        float adjustedThrottle = appliedThrottle;

        // 최대 속도 초과 시 감속
        if (currentSpeed >= maxSpeedLimit)
        {
            adjustedThrottle = Mathf.Min(adjustedThrottle, 0f);
        }

        wheelController.SetSteering(appliedSteering);
        wheelController.SetThrottle(adjustedThrottle);
    }

    void OnDestroy()
    {
        frontInputTensor?.Dispose();
        speedInputTensor?.Dispose();
        worker?.Dispose();

        if (frontRenderTexture != null) Destroy(frontRenderTexture);
        if (frontTexture != null) Destroy(frontTexture);
    }

    void UpdateUI()
    {
        float speed = wheelController != null ? wheelController.GetSpeedMS() : 0f;

        if (isAutonomousMode)
        {
            if (isInterventionActive)
            {
                if (uiModeText != null)
                    uiModeText.text = $"<color=yellow> INTERVENTION (#{interventionCount})</color>";

                if (uiActionText != null)
                {
                    float remaining = autoResumeDelay - interventionTimer;
                    uiActionText.text = $"Return to AI in: {remaining:F1}s\nWASD Manual Control...";
                }

                if (uiControlText != null)
                    uiControlText.text = "";
            }
            else
            {
                if (uiModeText != null)
                    uiModeText.text = "<color=#00FF00>● AUTONOMOUS (Regression)</color>";

                if (uiActionText != null)
                    uiActionText.text = $"Steer: <color=#00FFFF>{predictedSteering:F3}</color> | Throt: <color=#00FFFF>{predictedThrottle:F3}</color>";

                if (uiControlText != null)
                    uiControlText.text = $"Applied → Steer: {appliedSteering:F2} | Throt: {appliedThrottle:F2}";
            }

            if (uiStatsText != null)
                uiStatsText.text = $"Speed: {speed:F2} m/s | Interventions: {interventionCount}";

            if (uiGuideText != null)
                uiGuideText.text = "<color=grey>WASD: Intervention | P: Stop Auto</color>";
        }
        else
        {
            if (uiModeText != null)
                uiModeText.text = "<color=yellow>○ MANUAL MODE</color>";

            if (uiActionText != null)
                uiActionText.text = $"[{toggleKey}] Start Autonomous Mode";

            if (uiControlText != null)
            {
                if (interventionCount > 0)
                    uiControlText.text = $"<color=#00FFFF>Total Interventions: {interventionCount}</color>";
                else
                    uiControlText.text = "";
            }

            if (uiStatsText != null)
            {
                if (!isModelLoaded)
                    uiStatsText.text = "<color=red>! Model Not Loaded</color>";
                else
                    uiStatsText.text = "";
            }

            if (uiGuideText != null)
                uiGuideText.text = "";
        }
    }
}
