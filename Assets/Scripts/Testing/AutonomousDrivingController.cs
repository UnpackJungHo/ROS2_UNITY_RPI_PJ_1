using UnityEngine;
using Unity.Sentis;
using TMPro;

/// <summary>
/// Speed-Aware 자율주행 컨트롤러 (Single View)
///
/// 기능:
/// - P키: 자율주행 모드 토글
/// - WASD: 자율주행 중 사람 개입 (자동으로 수동 모드 전환)
/// - 3초 후 자동으로 AI 모드 복귀
/// - DrivingDataCollectorV2 연동하여 개입 데이터 수집
///
/// 모델 입력: front_image + speed
/// 모델 출력: 7개 클래스 logits
/// </summary>
public class AutonomousDrivingController : MonoBehaviour
{
    // 클래스 정의
    public enum KeyAction
    {
        FORWARD = 0,
        FORWARD_LEFT = 1,
        FORWARD_RIGHT = 2,
        LEFT = 3,
        RIGHT = 4,
        BACKWARD = 5,
        NONE = 6
    }

    private static readonly string[] ActionNames = {
        "FORWARD", "FWD+LEFT", "FWD+RIGHT", "LEFT", "RIGHT", "BACKWARD", "NONE"
    };

    [Header("AI Model")]
    [Tooltip("ONNX 분류 모델 파일 (Speed-Aware Single View)")]
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

    [Tooltip("속도 정규화 기준값 (학습 시 설정과 동일하게)")]
    public float speedNormalize = 5.0f;

    [Header("Control Settings")]
    [Tooltip("전진 시 throttle 값")]
    [Range(0.1f, 1.0f)]
    public float forwardThrottle = 0.8f;

    [Tooltip("조향 시 steering 강도")]
    [Range(0.1f, 1.0f)]
    public float steeringStrength = 1.0f;

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
    [SerializeField] private int predictedClass = -1;
    [SerializeField] private string predictedAction = "NONE";
    [SerializeField] private float confidence = 0f;
    [SerializeField] private float appliedSteering = 0f;
    [SerializeField] private float appliedThrottle = 0f;
    [SerializeField] private bool isInterventionActive = false;
    [SerializeField] private float interventionTimer = 0f;
    [SerializeField] private int interventionCount = 0;

    // Sentis 관련
    private Model runtimeModel;
    private Worker worker;

    // 카메라 및 렌더링
    private Camera frontCamera;
    private RenderTexture frontRenderTexture;
    private Texture2D frontTexture;

    // 텐서
    private Tensor<float> frontInputTensor;
    private Tensor<float> speedInputTensor;

    // 정규화 상수 (ImageNet)
    private readonly float[] mean = { 0.485f, 0.456f, 0.406f };
    private readonly float[] std = { 0.229f, 0.224f, 0.225f };

    private float lastInferenceTime;
    private bool isModelLoaded = false;

    void Start()
    {
        AutoFindReferences();
        InitializeRenderTextures();
        LoadModel();

        Debug.Log($"[AutonomousDriving] Speed-Aware Single View Controller Initialized");
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
            Debug.LogError("[AutonomousDriving] CameraPublisher를 찾을 수 없습니다!");
        if (wheelController == null)
            Debug.LogError("[AutonomousDriving] WheelTest를 찾을 수 없습니다!");
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
            Debug.LogError("[AutonomousDriving] Model Asset이 할당되지 않았습니다!");
            return;
        }

        try
        {
            runtimeModel = ModelLoader.Load(modelAsset);
            worker = new Worker(runtimeModel, BackendType.GPUCompute);

            isModelLoaded = true;
            Debug.Log("[AutonomousDriving] Speed-Aware Single View 모델 로드 완료");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[AutonomousDriving] 모델 로드 실패: {e.Message}");
            isModelLoaded = false;
        }
    }

    void Update()
    {
        if (frontCamera == null && cameraPublisher != null)
            frontCamera = cameraPublisher.GetCamera();

        // P키: 자율주행 모드 토글
        if (Input.GetKeyDown(toggleKey))
        {
            ToggleAutonomousMode();
        }

        // 자율주행 모드일 때
        if (isAutonomousMode)
        {
            // WASD 입력 감지 → 개입 시작
            if (IsManualInputDetected())
            {
                StartIntervention();
            }

            // 개입 타이머 처리
            if (isInterventionActive)
            {
                interventionTimer += Time.deltaTime;

                // 키를 놓으면 타이머 시작, 키를 누르면 타이머 리셋
                if (IsManualInputDetected())
                {
                    interventionTimer = 0f;
                }

                // N초 후 AI 모드 자동 복귀
                if (interventionTimer >= autoResumeDelay)
                {
                    EndIntervention();
                }
            }
            else
            {
                // AI 주행 중 - 추론 실행
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

        Debug.Log($"[AutonomousDriving] 자율주행 모드: {(isAutonomousMode ? "ON" : "OFF")}");
    }

    /// <summary>
    /// AI가 예측한 클래스 번호 반환 (DrivingDataCollectorV2 연동용)
    /// </summary>
    public int GetPredictedClass()
    {
        return predictedClass;
    }

    /// <summary>
    /// 현재 개입 상태인지 반환 (DrivingDataCollectorV2 연동용)
    /// </summary>
    public bool IsInterventionActive()
    {
        return isInterventionActive;
    }

    void StartIntervention()
    {
        if (isInterventionActive) return;

        isInterventionActive = true;
        interventionTimer = 0f;
        interventionCount++;

        // 사람이 직접 제어
        if (wheelController != null)
            wheelController.externalControlEnabled = false;

        Debug.Log($"[AutonomousDriving] 🔴 개입 #{interventionCount} 시작 - 수동 모드");
    }

    void EndIntervention()
    {
        isInterventionActive = false;
        interventionTimer = 0f;

        // AI 제어 재개
        if (wheelController != null)
            wheelController.externalControlEnabled = true;

        Debug.Log($"[AutonomousDriving] 🟢 개입 종료 - AI 모드 복귀");
    }

    void RunInference()
    {
        if (frontCamera == null) return;

        // 1. 카메라 이미지 캡처 (Front)
        CaptureCamera(frontCamera, frontRenderTexture, frontTexture);

        // 2. 텐서 생성
        frontInputTensor = TextureToTensor(frontTexture, frontImageHeight, frontImageWidth);

        // 3. 속도 텐서 생성 (정규화)
        float currentSpeed = wheelController != null ? wheelController.GetSpeedMS() : 0f;
        float normalizedSpeed = currentSpeed / speedNormalize;
        speedInputTensor = new Tensor<float>(new TensorShape(1, 1), new float[] { normalizedSpeed });

        // 4. 추론 실행 - ONNX 입력 이름과 매칭
        worker.SetInput("front_image", frontInputTensor);
        worker.SetInput("speed", speedInputTensor);

        worker.Schedule();

        // 5. 결과 읽기 - 7개 클래스 logits
        using (Tensor<float> outputTensor = worker.PeekOutput("logits") as Tensor<float>)
        {
            if (outputTensor != null)
            {
                using (Tensor<float> cpuTensor = outputTensor.ReadbackAndClone())
                {
                    // 디버그: 로짓 값 출력
                    string logitsStr = "";
                    for (int i = 0; i < 7; i++)
                        logitsStr += $"{cpuTensor[i]:F2}, ";
                    Debug.Log($"[AI] Logits: [{logitsStr}]");

                    // Softmax 적용하여 확률로 변환 + argmax
                    float maxVal = float.MinValue;
                    float sumExp = 0f;
                    int bestClass = 0;

                    // max 찾기 (수치 안정성)
                    for (int i = 0; i < 7; i++)
                    {
                        if (cpuTensor[i] > maxVal)
                        {
                            maxVal = cpuTensor[i];
                            bestClass = i;
                        }
                    }

                    // softmax 계산
                    for (int i = 0; i < 7; i++)
                    {
                        sumExp += Mathf.Exp(cpuTensor[i] - maxVal);
                    }

                    confidence = 1f / sumExp;  // 최대 클래스의 확률
                    predictedClass = bestClass;
                    predictedAction = ActionNames[bestClass];

                    // 클래스를 steering/throttle로 변환
                    ClassToControl((KeyAction)bestClass);
                }
            }
        }

        // 텐서 정리
        frontInputTensor?.Dispose();
        speedInputTensor?.Dispose();
    }

    /// <summary>
    /// 예측된 클래스를 steering/throttle 제어값으로 변환
    /// </summary>
    void ClassToControl(KeyAction action)
    {
        switch (action)
        {
            case KeyAction.FORWARD:
                appliedSteering = 0f;
                appliedThrottle = forwardThrottle;
                break;
            case KeyAction.FORWARD_LEFT:
                appliedSteering = -steeringStrength;
                appliedThrottle = forwardThrottle;
                break;
            case KeyAction.FORWARD_RIGHT:
                appliedSteering = steeringStrength;
                appliedThrottle = forwardThrottle;
                break;
            case KeyAction.LEFT:
                appliedSteering = -steeringStrength;
                appliedThrottle = 0f;
                break;
            case KeyAction.RIGHT:
                appliedSteering = steeringStrength;
                appliedThrottle = 0f;
                break;
            case KeyAction.BACKWARD:
                appliedSteering = 0f;
                appliedThrottle = -forwardThrottle;
                break;
            case KeyAction.NONE:
                appliedSteering = 0f;
                appliedThrottle = 0f;
                break;
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

    Tensor<float> TextureToTensor(Texture2D texture, int height, int width)
    {
        Color[] pixels = texture.GetPixels();
        float[] tensorData = new float[3 * height * width];

        // 픽셀 데이터를 텐서로 변환 (HWC → CHW, ImageNet 정규화)
        // Unity GetPixels()는 좌하단부터 읽으므로 y축 반전 필요
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                // Unity는 좌하단 기준, Python/학습은 좌상단 기준이므로 y 반전
                int srcY = height - 1 - y;
                int pixelIdx = srcY * width + x;
                Color pixel = pixels[pixelIdx];

                // CHW 형식: [C, H, W] - 채널별로 연속 배치
                // R 채널
                tensorData[0 * height * width + y * width + x] = (pixel.r - mean[0]) / std[0];
                // G 채널
                tensorData[1 * height * width + y * width + x] = (pixel.g - mean[1]) / std[1];
                // B 채널
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
                    uiModeText.text = "<color=#00FF00>● AUTONOMOUS (Single View)</color>";

                if (uiActionText != null)
                    uiActionText.text = $"Action: <color=#00FFFF>{predictedAction}</color> ({confidence * 100:F1}%)";

                if (uiControlText != null)
                    uiControlText.text = $"Steer: {appliedSteering:F2} | Throt: {appliedThrottle:F2}";
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
