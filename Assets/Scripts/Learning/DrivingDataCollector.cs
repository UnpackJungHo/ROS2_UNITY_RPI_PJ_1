using UnityEngine;
using System;
using System.IO;
using System.Collections.Generic;

/// <summary>
/// 딥러닝 모방학습용 주행 데이터 수집기
/// Front View + Top View 이미지를 동시에 저장
///
/// 사용법:
/// - R키: 녹화 시작/중지
/// - T키: 데이터 저장
/// </summary>
public class DrivingDataCollector : MonoBehaviour
{
    [Header("References")]
    [Tooltip("CameraPublisher 참조 (Front View 카메라 자동 획득)")]
    public CameraPublisher cameraPublisher;

    [Tooltip("차량 컨트롤러")]
    public WheelTest wheelController;

    [Tooltip("TopView 대상 오브젝트 (차량 base_link)")]
    public Transform topViewTarget;

    [Header("TopView Settings")]
    [Tooltip("TopView 카메라 높이")]
    public float topViewHeight = 8f;

    [Tooltip("TopView도 함께 저장")]
    public bool captureTopView = true;

    [Header("Image Settings")]
    [Tooltip("Front View 이미지 너비")]
    public int frontImageWidth = 200;

    [Tooltip("Front View 이미지 높이 (PilotNet: 66)")]
    public int frontImageHeight = 66;

    [Tooltip("Top View 이미지 크기")]
    public int topViewImageSize = 128;

    [Header("Collection Settings")]
    public KeyCode recordKey = KeyCode.R;
    public KeyCode saveKey = KeyCode.T;
    public float captureInterval = 0.1f;  // 10 FPS

    [Header("Status (Read Only)")]
    public bool isRecording = false;
    public int frameCount = 0;

    // Front View 카메라 (CameraPublisher에서 가져옴)
    private Camera frontCamera;

    // Top View 카메라 (동적 생성)
    private Camera topViewCamera;
    private GameObject topViewCameraObj;

    // 렌더 텍스처
    private RenderTexture frontRenderTexture;
    private RenderTexture topRenderTexture;
    private Texture2D frontCaptureTexture;
    private Texture2D topCaptureTexture;

    // 데이터 저장
    private string sessionFolder;
    private List<DrivingFrame> frameBuffer = new List<DrivingFrame>();
    private float lastCaptureTime;
    private float recordingStartTime;

    [Serializable]
    public class DrivingFrame
    {
        public string frontImagePath;
        public string topImagePath;
        public float steering;      // -1 ~ 1
        public float throttle;      // -1 ~ 1
        public float speed;         // m/s
        public float timestamp;
    }

    void Start()
    {
        // 렌더 텍스처 초기화
        frontRenderTexture = new RenderTexture(frontImageWidth, frontImageHeight, 24);
        frontCaptureTexture = new Texture2D(frontImageWidth, frontImageHeight, TextureFormat.RGB24, false);

        if (captureTopView)
        {
            topRenderTexture = new RenderTexture(topViewImageSize, topViewImageSize, 24);
            topCaptureTexture = new Texture2D(topViewImageSize, topViewImageSize, TextureFormat.RGB24, false);

            // TopView 카메라 동적 생성
            CreateTopViewCamera();
        }

        // 참조 자동 찾기
        AutoFindReferences();

        // 검증
        ValidateReferences();

        Debug.Log($"[DataCollector] Ready!");
        Debug.Log($"  '{recordKey}' 키: 녹화 시작/중지");
        Debug.Log($"  '{saveKey}' 키: 데이터 저장");
        Debug.Log($"  Front: {frontImageWidth}x{frontImageHeight}, Top: {(captureTopView ? $"{topViewImageSize}x{topViewImageSize}" : "비활성화")}");
    }

    void AutoFindReferences()
    {
        // CameraPublisher 자동 찾기
        if (cameraPublisher == null)
        {
            cameraPublisher = FindObjectOfType<CameraPublisher>();
        }

        // Front Camera 가져오기
        if (cameraPublisher != null)
        {
            frontCamera = cameraPublisher.GetCamera();
        }

        // WheelTest 자동 찾기
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

        // TopView 대상 자동 찾기
        if (topViewTarget == null && wheelController != null)
        {
            topViewTarget = wheelController.transform;
        }
    }

    void ValidateReferences()
    {
        // Front Camera는 CameraPublisher.Start()에서 생성되므로
        // 여기서는 검증하지 않고 Update()에서 재시도함
        if (cameraPublisher == null)
        {
            Debug.LogError("[DataCollector] CameraPublisher를 찾을 수 없습니다!");
        }
        if (wheelController == null)
        {
            Debug.LogError("[DataCollector] WheelTest를 찾을 수 없습니다!");
        }
        if (captureTopView && topViewTarget == null)
        {
            Debug.LogWarning("[DataCollector] TopView 대상이 없습니다. TopView 캡처가 비활성화됩니다.");
            captureTopView = false;
        }
    }

    void CreateTopViewCamera()
    {
        topViewCameraObj = new GameObject("DataCollector_TopViewCamera");
        topViewCamera = topViewCameraObj.AddComponent<Camera>();
        topViewCamera.nearClipPlane = 0.1f;
        topViewCamera.farClipPlane = 100f;
        topViewCamera.fieldOfView = 60f;
        topViewCamera.enabled = false;  // 화면에 렌더링하지 않음
        topViewCamera.targetTexture = topRenderTexture;

        Debug.Log("[DataCollector] TopView 카메라 생성됨");
    }

    void Update()
    {
        // CameraPublisher에서 카메라가 늦게 생성될 수 있으므로 재확인
        if (frontCamera == null && cameraPublisher != null)
        {
            frontCamera = cameraPublisher.GetCamera();
        }

        // TopView 카메라 위치 업데이트
        if (captureTopView && topViewTarget != null && topViewCamera != null)
        {
            UpdateTopViewCamera();
        }

        // 녹화 토글
        if (Input.GetKeyDown(recordKey))
        {
            if (!isRecording)
            {
                StartRecording();
            }
            else
            {
                StopRecording();
            }
        }

        // 저장
        if (Input.GetKeyDown(saveKey))
        {
            SaveDataset();
        }

        // 녹화 중이면 캡처
        if (isRecording && Time.time - lastCaptureTime >= captureInterval)
        {
            CaptureFrame();
            lastCaptureTime = Time.time;
        }
    }

    void UpdateTopViewCamera()
    {
        Vector3 targetPosition = topViewTarget.position;
        topViewCamera.transform.position = targetPosition + Vector3.up * topViewHeight;
        // 차량 방향에 맞춰 회전 (차량이 항상 위쪽을 향하도록)
        float yRotation = topViewTarget.eulerAngles.y;
        topViewCamera.transform.rotation = Quaternion.Euler(90f, yRotation, 0f);
    }

    void StartRecording()
    {
        // 세션 폴더 생성
        string basePath = Path.Combine(Application.dataPath, "..", "TrainingData");
        sessionFolder = Path.Combine(basePath, $"session_{DateTime.Now:yyyyMMdd_HHmmss}");

        Directory.CreateDirectory(sessionFolder);
        Directory.CreateDirectory(Path.Combine(sessionFolder, "front"));

        if (captureTopView)
        {
            Directory.CreateDirectory(Path.Combine(sessionFolder, "top"));
        }

        frameBuffer.Clear();
        frameCount = 0;
        isRecording = true;
        lastCaptureTime = Time.time;
        recordingStartTime = Time.time;

        Debug.Log($"[DataCollector] ● 녹화 시작: {sessionFolder}");
    }

    void StopRecording()
    {
        isRecording = false;
        float duration = Time.time - recordingStartTime;
        Debug.Log($"[DataCollector] ■ 녹화 중지: {frameCount} frames ({duration:F1}초)");
        Debug.Log($"  '{saveKey}' 키를 눌러 저장하세요.");
    }

    void CaptureFrame()
    {
        if (frontCamera == null) return;

        // 현재 입력값 가져오기
        float steering = Input.GetAxis("Horizontal");
        float throttle = Input.GetAxis("Vertical");
        float speed = wheelController != null ? wheelController.GetSpeedMS() : 0f;

        // 정지 상태 스킵 (노이즈 방지)
        if (Mathf.Abs(speed) < 0.1f && Mathf.Abs(throttle) < 0.1f)
        {
            return;
        }

        string frameNum = $"{frameCount:D6}";

        // Front View 캡처
        string frontImageName = $"frame_{frameNum}.jpg";
        string frontImagePath = Path.Combine(sessionFolder, "front", frontImageName);
        CaptureCamera(frontCamera, frontRenderTexture, frontCaptureTexture, frontImagePath);

        // Top View 캡처
        string topImageName = "";
        if (captureTopView && topViewCamera != null)
        {
            topImageName = $"frame_{frameNum}.jpg";
            string topImagePath = Path.Combine(sessionFolder, "top", topImageName);
            CaptureCamera(topViewCamera, topRenderTexture, topCaptureTexture, topImagePath);
        }

        // 프레임 데이터 저장
        DrivingFrame frame = new DrivingFrame
        {
            frontImagePath = $"front/{frontImageName}",
            topImagePath = captureTopView ? $"top/{topImageName}" : "",
            steering = steering,
            throttle = throttle,
            speed = speed,
            timestamp = Time.time - recordingStartTime
        };

        frameBuffer.Add(frame);
        frameCount++;

        // 100 프레임마다 로그
        if (frameCount % 100 == 0)
        {
            Debug.Log($"[DataCollector] {frameCount} frames captured");
        }
    }

    void CaptureCamera(Camera cam, RenderTexture rt, Texture2D tex, string path)
    {
        // 카메라의 원래 타겟 저장
        RenderTexture originalTarget = cam.targetTexture;

        // 렌더 텍스처로 렌더링
        cam.targetTexture = rt;
        cam.Render();

        // 텍스처로 복사
        RenderTexture.active = rt;
        tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        tex.Apply();
        RenderTexture.active = null;

        // 원래 타겟 복원
        cam.targetTexture = originalTarget;

        // JPG로 저장
        byte[] bytes = tex.EncodeToJPG(85);
        File.WriteAllBytes(path, bytes);
    }

    void SaveDataset()
    {
        if (frameBuffer.Count == 0)
        {
            Debug.LogWarning("[DataCollector] 저장할 데이터가 없습니다!");
            return;
        }

        // CSV 저장
        string csvPath = Path.Combine(sessionFolder, "driving_log.csv");
        using (StreamWriter writer = new StreamWriter(csvPath))
        {
            // 헤더
            if (captureTopView)
            {
                writer.WriteLine("front_image,top_image,steering,throttle,speed,timestamp");
            }
            else
            {
                writer.WriteLine("front_image,steering,throttle,speed,timestamp");
            }

            // 데이터
            foreach (var frame in frameBuffer)
            {
                if (captureTopView)
                {
                    writer.WriteLine($"{frame.frontImagePath},{frame.topImagePath},{frame.steering:F6},{frame.throttle:F6},{frame.speed:F6},{frame.timestamp:F6}");
                }
                else
                {
                    writer.WriteLine($"{frame.frontImagePath},{frame.steering:F6},{frame.throttle:F6},{frame.speed:F6},{frame.timestamp:F6}");
                }
            }
        }

        // 메타데이터 JSON 저장
        string metaPath = Path.Combine(sessionFolder, "metadata.json");
        string metaJson = $@"{{
    ""total_frames"": {frameBuffer.Count},
    ""front_image_size"": {{ ""width"": {frontImageWidth}, ""height"": {frontImageHeight} }},
    ""top_image_size"": {{ ""width"": {topViewImageSize}, ""height"": {topViewImageSize} }},
    ""top_view_enabled"": {captureTopView.ToString().ToLower()},
    ""top_view_height"": {topViewHeight},
    ""capture_fps"": {1f / captureInterval:F1},
    ""created"": ""{DateTime.Now:yyyy-MM-dd HH:mm:ss}""
}}";
        File.WriteAllText(metaPath, metaJson);

        Debug.Log($"[DataCollector] ✓ 저장 완료!");
        Debug.Log($"  경로: {csvPath}");
        Debug.Log($"  프레임: {frameBuffer.Count}");
    }

    void OnDestroy()
    {
        // 렌더 텍스처 정리
        if (frontRenderTexture != null) Destroy(frontRenderTexture);
        if (topRenderTexture != null) Destroy(topRenderTexture);
        if (frontCaptureTexture != null) Destroy(frontCaptureTexture);
        if (topCaptureTexture != null) Destroy(topCaptureTexture);

        // TopView 카메라 오브젝트 정리
        if (topViewCameraObj != null) Destroy(topViewCameraObj);
    }

    void OnGUI()
    {
        // 배경 박스
        GUI.Box(new Rect(10, 130, 250, 80), "");

        GUIStyle style = new GUIStyle(GUI.skin.label);
        style.fontSize = 14;
        style.fontStyle = FontStyle.Bold;

        if (isRecording)
        {
            style.normal.textColor = Color.red;
            GUI.Label(new Rect(20, 135, 230, 25), $"● REC: {frameCount} frames", style);

            float duration = Time.time - recordingStartTime;
            style.normal.textColor = Color.white;
            GUI.Label(new Rect(20, 155, 230, 20), $"Time: {duration:F1}s", style);

            float speed = wheelController != null ? wheelController.GetSpeedMS() : 0f;
            GUI.Label(new Rect(20, 175, 230, 20), $"Speed: {speed:F2} m/s", style);
        }
        else
        {
            style.normal.textColor = Color.white;
            GUI.Label(new Rect(20, 135, 230, 20), $"[{recordKey}] 녹화 시작", style);
            GUI.Label(new Rect(20, 155, 230, 20), $"[{saveKey}] 저장", style);

            if (frameBuffer.Count > 0)
            {
                style.normal.textColor = Color.yellow;
                GUI.Label(new Rect(20, 175, 230, 20), $"미저장: {frameBuffer.Count} frames", style);
            }
        }
    }
}
