using UnityEngine;
using Unity.Sentis;
using System.Collections;

/// <summary>
/// ONNX 모델을 사용한 자율주행 컨트롤러
/// Front View + Top View 듀얼 뷰 모델 지원
///
/// 사용법:
/// 1. Unity Package Manager에서 "Sentis" 패키지 설치
/// 2. ONNX 모델 파일을 Assets/Models/ONNX에 배치
/// 3. 이 스크립트를 차량에 추가하고 참조 설정
/// 4. P키로 자율주행 모드 토글
/// </summary>
public class AutonomousDrivingController : MonoBehaviour
{
    [Header("AI Model")]
    [Tooltip("ONNX 모델 파일 (.onnx → .sentis)")]
    public ModelAsset modelAsset;

    [Header("References")]
    [Tooltip("차량 컨트롤러")]
    public WheelTest wheelController;

    [Tooltip("CameraPublisher (Front View 카메라)")]
    public CameraPublisher cameraPublisher;

    [Tooltip("TopView 대상 오브젝트 (차량 base_link)")]
    public Transform topViewTarget;

    [Header("Image Settings (학습 시 설정과 동일하게)")]
    public int frontImageWidth = 200;
    public int frontImageHeight = 66;
    public int topViewImageSize = 128;
    public float topViewHeight = 8f;

    [Header("Inference Settings")]
    [Tooltip("추론 주기 (초)")]
    public float inferenceInterval = 0.1f;

    [Tooltip("출력 스무딩 (0=즉시 적용, 1=이전값 유지)")]
    [Range(0f, 0.95f)]
    public float outputSmoothing = 0.3f;

    [Header("Control")]
    public KeyCode toggleKey = KeyCode.P;
    public bool isAutonomousMode = false;

    [Header("Safety")]
    [Tooltip("최대 속도 제한 (m/s)")]
    public float maxSpeedLimit = 3f;

    [Tooltip("최소 속도 유지 (m/s)")]
    public float minSpeed = 0.5f;

    [Header("Debug (Read Only)")]
    [SerializeField] private float predictedSteering;
    [SerializeField] private float predictedThrottle;
    [SerializeField] private float smoothedSteering;
    [SerializeField] private float smoothedThrottle;

    // Sentis 관련
    private Model runtimeModel;
    private Worker worker;

    // 카메라 및 렌더링
    private Camera frontCamera;
    private Camera topViewCamera;
    private GameObject topViewCameraObj;

    private RenderTexture frontRenderTexture;
    private RenderTexture topRenderTexture;
    private Texture2D frontTexture;
    private Texture2D topTexture;

    // 텐서
    private Tensor<float> frontInputTensor;
    private Tensor<float> topInputTensor;

    // 정규화 상수 (ImageNet)
    private readonly float[] mean = { 0.485f, 0.456f, 0.406f };
    private readonly float[] std = { 0.229f, 0.224f, 0.225f };

    private float lastInferenceTime;
    private bool isModelLoaded = false;

    void Start()
    {
        // 참조 자동 찾기
        AutoFindReferences();

        // 렌더 텍스처 초기화
        InitializeRenderTextures();

        // TopView 카메라 생성
        CreateTopViewCamera();

        // 모델 로드
        LoadModel();

        Debug.Log($"[AutonomousDriving] Initialized");
        Debug.Log($"  '{toggleKey}' 키: 자율주행 모드 토글");
    }

    void AutoFindReferences()
    {
        if (cameraPublisher == null)
        {
            cameraPublisher = FindObjectOfType<CameraPublisher>();
        }

        if (wheelController == null)
        {
            wheelController = GetComponent<WheelTest>();
            if (wheelController == null)
            {
                wheelController = GetComponentInParent<WheelTest>();
            }
            if (wheelController == null)
            {
                wheelController = FindObjectOfType<WheelTest>();
            }
        }

        if (topViewTarget == null && wheelController != null)
        {
            topViewTarget = wheelController.transform;
        }

        // 검증
        if (cameraPublisher == null)
        {
            Debug.LogError("[AutonomousDriving] CameraPublisher를 찾을 수 없습니다!");
        }
        if (wheelController == null)
        {
            Debug.LogError("[AutonomousDriving] WheelTest를 찾을 수 없습니다!");
        }
    }

    void InitializeRenderTextures()
    {
        frontRenderTexture = new RenderTexture(frontImageWidth, frontImageHeight, 24);
        frontTexture = new Texture2D(frontImageWidth, frontImageHeight, TextureFormat.RGB24, false);

        topRenderTexture = new RenderTexture(topViewImageSize, topViewImageSize, 24);
        topTexture = new Texture2D(topViewImageSize, topViewImageSize, TextureFormat.RGB24, false);
    }

    void CreateTopViewCamera()
    {
        topViewCameraObj = new GameObject("AutonomousDriving_TopViewCamera");
        topViewCamera = topViewCameraObj.AddComponent<Camera>();
        topViewCamera.nearClipPlane = 0.1f;
        topViewCamera.farClipPlane = 100f;
        topViewCamera.fieldOfView = 60f;
        topViewCamera.enabled = false;
        topViewCamera.targetTexture = topRenderTexture;
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
            Debug.Log("[AutonomousDriving] 모델 로드 완료");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[AutonomousDriving] 모델 로드 실패: {e.Message}");
            isModelLoaded = false;
        }
    }

    void Update()
    {
        // Front Camera 지연 획득
        if (frontCamera == null && cameraPublisher != null)
        {
            frontCamera = cameraPublisher.GetCamera();
        }

        // TopView 카메라 위치 업데이트
        if (topViewTarget != null && topViewCamera != null)
        {
            UpdateTopViewCamera();
        }

        // 자율주행 모드 토글
        if (Input.GetKeyDown(toggleKey))
        {
            isAutonomousMode = !isAutonomousMode;

            // WheelTest의 외부 제어 모드 설정
            if (wheelController != null)
            {
                wheelController.externalControlEnabled = isAutonomousMode;
            }

            Debug.Log($"[AutonomousDriving] 자율주행 모드: {(isAutonomousMode ? "ON" : "OFF")}");
        }

        // 자율주행 모드일 때만 추론 실행
        if (isAutonomousMode && isModelLoaded)
        {
            if (Time.time - lastInferenceTime >= inferenceInterval)
            {
                RunInference();
                lastInferenceTime = Time.time;
            }

            // 차량 제어 적용
            ApplyControl();
        }
    }

    void UpdateTopViewCamera()
    {
        Vector3 targetPosition = topViewTarget.position;
        topViewCamera.transform.position = targetPosition + Vector3.up * topViewHeight;
        float yRotation = topViewTarget.eulerAngles.y;
        topViewCamera.transform.rotation = Quaternion.Euler(90f, yRotation, 0f);
    }

    void RunInference()
    {
        if (frontCamera == null || topViewCamera == null) return;

        // 1. 카메라 이미지 캡처
        CaptureImages();

        // 2. 텐서 생성
        CreateInputTensors();

        // 3. 추론 실행
        worker.Schedule(frontInputTensor, topInputTensor);

        // 4. 결과 읽기 (GPU → CPU 복사)
        using (Tensor<float> steeringOutput = worker.PeekOutput("steering") as Tensor<float>)
        using (Tensor<float> throttleOutput = worker.PeekOutput("throttle") as Tensor<float>)
        {
            if (steeringOutput != null && throttleOutput != null)
            {
                // ReadbackAndClone으로 CPU 텐서로 복사
                using (Tensor<float> steeringCPU = steeringOutput.ReadbackAndClone())
                using (Tensor<float> throttleCPU = throttleOutput.ReadbackAndClone())
                {
                    predictedSteering = steeringCPU[0];
                    predictedThrottle = throttleCPU[0];

                    // 스무딩 적용
                    smoothedSteering = Mathf.Lerp(predictedSteering, smoothedSteering, outputSmoothing);
                    smoothedThrottle = Mathf.Lerp(predictedThrottle, smoothedThrottle, outputSmoothing);
                }
            }
        }

        // 텐서 정리
        frontInputTensor?.Dispose();
        topInputTensor?.Dispose();
    }

    void CaptureImages()
    {
        // Front View 캡처
        CaptureCamera(frontCamera, frontRenderTexture, frontTexture);

        // Top View 캡처
        CaptureCamera(topViewCamera, topRenderTexture, topTexture);
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

    void CreateInputTensors()
    {
        // Front tensor: [1, 3, H, W]
        frontInputTensor = TextureToTensor(frontTexture, frontImageHeight, frontImageWidth);

        // Top tensor: [1, 3, H, W]
        topInputTensor = TextureToTensor(topTexture, topViewImageSize, topViewImageSize);
    }

    Tensor<float> TextureToTensor(Texture2D texture, int height, int width)
    {
        Color[] pixels = texture.GetPixels();
        float[] tensorData = new float[3 * height * width];

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                // 이미지 상하 반전 (Unity 좌표계 → 표준 이미지 좌표계)
                int srcIdx = (height - 1 - y) * width + x;
                Color pixel = pixels[srcIdx];

                // RGB 채널별로 정규화 적용 (ImageNet 정규화)
                // CHW 포맷: [C, H, W]
                int baseIdx = y * width + x;
                tensorData[0 * height * width + baseIdx] = (pixel.r - mean[0]) / std[0]; // R
                tensorData[1 * height * width + baseIdx] = (pixel.g - mean[1]) / std[1]; // G
                tensorData[2 * height * width + baseIdx] = (pixel.b - mean[2]) / std[2]; // B
            }
        }

        return new Tensor<float>(new TensorShape(1, 3, height, width), tensorData);
    }

    void ApplyControl()
    {
        if (wheelController == null) return;

        // 속도 제어
        float currentSpeed = wheelController.GetSpeedMS();

        // 속도 제한 적용
        float adjustedThrottle = smoothedThrottle;
        if (currentSpeed >= maxSpeedLimit)
        {
            adjustedThrottle = Mathf.Min(adjustedThrottle, 0f);
        }
        else if (currentSpeed < minSpeed && adjustedThrottle < 0.3f)
        {
            // 최소 속도 유지
            adjustedThrottle = Mathf.Max(adjustedThrottle, 0.3f);
        }

        // 차량 제어 적용
        wheelController.SetSteering(smoothedSteering);
        wheelController.SetThrottle(adjustedThrottle);
    }

    void OnDestroy()
    {
        // 리소스 정리
        frontInputTensor?.Dispose();
        topInputTensor?.Dispose();
        worker?.Dispose();

        if (frontRenderTexture != null) Destroy(frontRenderTexture);
        if (topRenderTexture != null) Destroy(topRenderTexture);
        if (frontTexture != null) Destroy(frontTexture);
        if (topTexture != null) Destroy(topTexture);
        if (topViewCameraObj != null) Destroy(topViewCameraObj);
    }

    void OnGUI()
    {
        // 상태 표시 UI
        GUI.Box(new Rect(10, 220, 280, 100), "");

        GUIStyle style = new GUIStyle(GUI.skin.label);
        style.fontSize = 14;
        style.fontStyle = FontStyle.Bold;

        if (isAutonomousMode)
        {
            style.normal.textColor = Color.green;
            GUI.Label(new Rect(20, 225, 260, 25), "● AUTONOMOUS MODE", style);

            style.normal.textColor = Color.white;
            style.fontStyle = FontStyle.Normal;
            GUI.Label(new Rect(20, 250, 260, 20), $"Steering: {smoothedSteering:F3}", style);
            GUI.Label(new Rect(20, 270, 260, 20), $"Throttle: {smoothedThrottle:F3}", style);

            float speed = wheelController != null ? wheelController.GetSpeedMS() : 0f;
            GUI.Label(new Rect(20, 290, 260, 20), $"Speed: {speed:F2} m/s", style);
        }
        else
        {
            style.normal.textColor = Color.yellow;
            GUI.Label(new Rect(20, 225, 260, 25), "○ MANUAL MODE", style);

            style.normal.textColor = Color.white;
            style.fontStyle = FontStyle.Normal;
            GUI.Label(new Rect(20, 250, 260, 20), $"[{toggleKey}] 자율주행 시작", style);

            if (!isModelLoaded)
            {
                style.normal.textColor = Color.red;
                GUI.Label(new Rect(20, 270, 260, 20), "! 모델이 로드되지 않음", style);
            }
        }
    }
}
