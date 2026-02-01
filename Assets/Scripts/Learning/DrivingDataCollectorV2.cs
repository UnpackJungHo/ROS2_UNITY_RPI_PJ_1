using UnityEngine;
using System;
using System.IO;
using System.Collections.Generic;
using TMPro;

/// <summary>
/// 분류 기반 주행 데이터 수집기 V2
///
/// 변경점:
/// - steering/throttle 연속값 대신 키 입력 클래스 기록
/// - 분류 문제에 적합한 데이터 형식
///
/// 키 입력 클래스:
///   0: FORWARD (W)
///   1: FORWARD_LEFT (W+A)
///   2: FORWARD_RIGHT (W+D)
///   3: LEFT (A) - 관성 좌회전
///   4: RIGHT (D) - 관성 우회전
///   5: BACKWARD (S)
///   6: NONE - 관성 주행
///
/// 사용법:
/// - R키: 녹화 시작/중지
/// - T키: 데이터 저장
/// </summary>
public class DrivingDataCollectorV2 : MonoBehaviour
{
    // 키 입력 클래스 정의
    public enum KeyAction
    {
        FORWARD = 0,        // W만
        FORWARD_LEFT = 1,   // W + A
        FORWARD_RIGHT = 2,  // W + D
        LEFT = 3,           // A만 (관성 좌회전)
        RIGHT = 4,          // D만 (관성 우회전)
        BACKWARD = 5,       // S
        NONE = 6            // 아무것도 안 누름 (관성 주행)
    }

    public static readonly string[] KeyActionNames = {
        "FORWARD", "FORWARD_LEFT", "FORWARD_RIGHT",
        "LEFT", "RIGHT", "BACKWARD", "NONE"
    };

    [Header("UI References")]
    public TextMeshProUGUI uiStatusText;
    public TextMeshProUGUI uiInfoText;
    public TextMeshProUGUI uiActionText;
    public TextMeshProUGUI uiSpeedText;

    [Header("References")]
    [Tooltip("CameraPublisher 참조 (Front View 카메라 자동 획득)")]
    public CameraPublisher cameraPublisher;

    [Tooltip("차량 컨트롤러")]
    public WheelTest wheelController;

    [Tooltip("TopView 대상 오브젝트 (차량 base_link)")]
    public Transform topViewTarget;

    [Header("DAgger Settings")]
    [Tooltip("자율주행 컨트롤러 (AI 모드 및 개입 상태 확인)")]
    public AutonomousDrivingController aiController;

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
    public float captureInterval = 0.05f;  // 20 FPS

    [Header("Status (Read Only)")]
    public bool isRecording = false;
    public int frameCount = 0;
    public KeyAction currentAction = KeyAction.NONE;

    // Front View 카메라
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
    private List<DrivingFrameV2> frameBuffer = new List<DrivingFrameV2>();
    private float lastCaptureTime;
    private float recordingStartTime;

    [Serializable]
    public class DrivingFrameV2
    {
        public string frontImagePath;
        public string topImagePath;
        public int keyAction;           // KeyAction enum의 int 값
        public string keyActionName;    // 가독성을 위한 이름
        public float speed;             // m/s
        public float timestamp;
        public bool isIntervention;     // DAgger: 전문가 개입 프레임 여부
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
            CreateTopViewCamera();
        }

        AutoFindReferences();
        ValidateReferences();

        Debug.Log($"[DataCollectorV2] Ready! (분류 기반)");
        Debug.Log($"  '{recordKey}' 키: 녹화 시작/중지");
        Debug.Log($"  '{saveKey}' 키: 데이터 저장");
        Debug.Log($"  클래스: {string.Join(", ", KeyActionNames)}");
    }

    void AutoFindReferences()
    {
        if (cameraPublisher == null)
            cameraPublisher = FindObjectOfType<CameraPublisher>();

        if (cameraPublisher != null)
            frontCamera = cameraPublisher.GetCamera();

        if (wheelController == null)
        {
            wheelController = GetComponent<WheelTest>();
            if (wheelController == null)
                wheelController = GetComponentInParent<WheelTest>();
            if (wheelController == null)
                wheelController = FindObjectOfType<WheelTest>();
        }

        if (topViewTarget == null && wheelController != null)
            topViewTarget = wheelController.transform;
    }

    void ValidateReferences()
    {
        if (cameraPublisher == null)
            Debug.LogError("[DataCollectorV2] CameraPublisher를 찾을 수 없습니다!");
        if (wheelController == null)
            Debug.LogError("[DataCollectorV2] WheelTest를 찾을 수 없습니다!");
        if (captureTopView && topViewTarget == null)
        {
            Debug.LogWarning("[DataCollectorV2] TopView 대상이 없습니다. TopView 캡처 비활성화.");
            captureTopView = false;
        }
    }

    void CreateTopViewCamera()
    {
        topViewCameraObj = new GameObject("DataCollectorV2_TopViewCamera");
        topViewCamera = topViewCameraObj.AddComponent<Camera>();
        topViewCamera.nearClipPlane = 0.1f;
        topViewCamera.farClipPlane = 100f;
        topViewCamera.fieldOfView = 60f;
        topViewCamera.enabled = false;
        topViewCamera.targetTexture = topRenderTexture;
    }

    void Update()
    {
        if (frontCamera == null && cameraPublisher != null)
            frontCamera = cameraPublisher.GetCamera();

        if (captureTopView && topViewTarget != null && topViewCamera != null)
            UpdateTopViewCamera();

        // 녹화 토글
        if (Input.GetKeyDown(recordKey))
        {
            if (!isRecording)
                StartRecording();
            else
                StopRecording();
        }

        // 저장
        if (Input.GetKeyDown(saveKey))
            SaveDataset();

        // Update currentAction for UI display (Record or not)
        currentAction = GetCurrentKeyAction();

        // 녹화 중이면 캡처
        if (isRecording && Time.time - lastCaptureTime >= captureInterval)
        {
            CaptureFrame();
            lastCaptureTime = Time.time;
        }

        UpdateUI();
    }

    /// <summary>
    /// 현재 액션을 반환
    /// - 자율주행 모드(개입 아님): AI 예측 액션 반환
    /// - 수동/개입 모드: 키보드 입력 반환
    /// </summary>
    KeyAction GetCurrentKeyAction()
    {
        // 자율주행 모드 + 개입 아님 = AI가 제어 중
        if (aiController != null && aiController.isAutonomousMode && !aiController.IsInterventionActive())
        {
            // AI의 예측된 클래스를 반환
            int predictedClass = aiController.GetPredictedClass();
            if (predictedClass >= 0 && predictedClass < 7)
            {
                return (KeyAction)predictedClass;
            }
        }

        // 수동 모드 또는 개입 중 - 키보드 입력 확인
        bool w = Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow);
        bool a = Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow);
        bool s = Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow);
        bool d = Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow);

        // 우선순위: 전진 조합 > 후진 > 좌/우 단독 > NONE
        if (w && a) return KeyAction.FORWARD_LEFT;
        if (w && d) return KeyAction.FORWARD_RIGHT;
        if (w) return KeyAction.FORWARD;
        if (s) return KeyAction.BACKWARD;
        if (a) return KeyAction.LEFT;
        if (d) return KeyAction.RIGHT;

        return KeyAction.NONE;
    }

    void UpdateTopViewCamera()
    {
        Vector3 targetPosition = topViewTarget.position;
        topViewCamera.transform.position = targetPosition + Vector3.up * topViewHeight;
        float yRotation = topViewTarget.eulerAngles.y;
        topViewCamera.transform.rotation = Quaternion.Euler(90f, yRotation, 0f);
    }

    void StartRecording()
    {
        string basePath = Path.Combine(Application.dataPath, "..", "TrainingData");
        sessionFolder = Path.Combine(basePath, $"session_{DateTime.Now:yyyyMMdd_HHmmss}");

        Directory.CreateDirectory(sessionFolder);
        Directory.CreateDirectory(Path.Combine(sessionFolder, "front"));

        if (captureTopView)
            Directory.CreateDirectory(Path.Combine(sessionFolder, "top"));

        frameBuffer.Clear();
        frameCount = 0;
        isRecording = true;
        lastCaptureTime = Time.time;
        recordingStartTime = Time.time;

        Debug.Log($"[DataCollectorV2] ● 녹화 시작: {sessionFolder}");
    }

    void StopRecording()
    {
        isRecording = false;
        float duration = Time.time - recordingStartTime;
        Debug.Log($"[DataCollectorV2] ■ 녹화 중지: {frameCount} frames ({duration:F1}초)");
        Debug.Log($"  '{saveKey}' 키를 눌러 저장하세요.");
    }

    void CaptureFrame()
    {
        if (frontCamera == null) return;

        KeyAction action = GetCurrentKeyAction();
        float speed = wheelController != null ? wheelController.GetSpeedMS() : 0f;

        // NONE + 정지 상태는 스킵 (의미없는 데이터)
        if (action == KeyAction.NONE && speed < 0.1f)
            return;

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
        // DAgger: AI 컨트롤러가 개입 상태인지 확인
        bool isIntervening = (aiController != null && aiController.isAutonomousMode &&
                              (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.A) ||
                               Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.D) ||
                               Input.GetKey(KeyCode.UpArrow) || Input.GetKey(KeyCode.LeftArrow) ||
                               Input.GetKey(KeyCode.DownArrow) || Input.GetKey(KeyCode.RightArrow)));

        DrivingFrameV2 frame = new DrivingFrameV2
        {
            frontImagePath = $"front/{frontImageName}",
            topImagePath = captureTopView ? $"top/{topImageName}" : "",
            keyAction = (int)action,
            keyActionName = KeyActionNames[(int)action],
            speed = speed,
            timestamp = Time.time - recordingStartTime,
            isIntervention = isIntervening
        };

        frameBuffer.Add(frame);
        frameCount++;

        if (frameCount % 100 == 0)
            Debug.Log($"[DataCollectorV2] {frameCount} frames captured");
    }

    void CaptureCamera(Camera cam, RenderTexture rt, Texture2D tex, string path)
    {
        RenderTexture originalTarget = cam.targetTexture;

        cam.targetTexture = rt;
        cam.Render();

        RenderTexture.active = rt;
        tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        tex.Apply();
        RenderTexture.active = null;

        cam.targetTexture = originalTarget;

        byte[] bytes = tex.EncodeToJPG(85);
        File.WriteAllBytes(path, bytes);
    }

    void SaveDataset()
    {
        if (frameBuffer.Count == 0)
        {
            Debug.LogWarning("[DataCollectorV2] 저장할 데이터가 없습니다!");
            return;
        }

        // CSV 저장 (intervention 컴럼 추가)
        string csvPath = Path.Combine(sessionFolder, "driving_log.csv");
        using (StreamWriter writer = new StreamWriter(csvPath))
        {
            // 헤더
            if (captureTopView)
                writer.WriteLine("front_image,top_image,key_action,key_action_name,speed,timestamp,intervention");
            else
                writer.WriteLine("front_image,key_action,key_action_name,speed,timestamp,intervention");

            // 데이터
            foreach (var frame in frameBuffer)
            {
                int intervention = frame.isIntervention ? 1 : 0;
                if (captureTopView)
                {
                    writer.WriteLine($"{frame.frontImagePath},{frame.topImagePath},{frame.keyAction},{frame.keyActionName},{frame.speed:F6},{frame.timestamp:F6},{intervention}");
                }
                else
                {
                    writer.WriteLine($"{frame.frontImagePath},{frame.keyAction},{frame.keyActionName},{frame.speed:F6},{frame.timestamp:F6},{intervention}");
                }
            }
        }

        // 클래스 분포 계산
        int[] classCounts = new int[7];
        float totalSpeed = 0f;
        float minSpeed = float.MaxValue;
        float maxSpeed = float.MinValue;

        foreach (var frame in frameBuffer)
        {
            classCounts[frame.keyAction]++;
            totalSpeed += frame.speed;
            minSpeed = Mathf.Min(minSpeed, frame.speed);
            maxSpeed = Mathf.Max(maxSpeed, frame.speed);
        }

        int total = frameBuffer.Count;
        float avgSpeed = totalSpeed / total;

        // 메타데이터 JSON 저장
        string metaPath = Path.Combine(sessionFolder, "metadata.json");
        string metaJson = $@"{{
    ""version"": ""v2_classification"",
    ""total_frames"": {total},
    ""num_classes"": 7,
    ""class_names"": [""FORWARD"", ""FORWARD_LEFT"", ""FORWARD_RIGHT"", ""LEFT"", ""RIGHT"", ""BACKWARD"", ""NONE""],
    ""front_image_size"": {{ ""width"": {frontImageWidth}, ""height"": {frontImageHeight} }},
    ""top_image_size"": {{ ""width"": {topViewImageSize}, ""height"": {topViewImageSize} }},
    ""top_view_enabled"": {captureTopView.ToString().ToLower()},
    ""top_view_height"": {topViewHeight},
    ""capture_fps"": {1f / captureInterval:F1},
    ""created"": ""{DateTime.Now:yyyy-MM-dd HH:mm:ss}"",
    ""class_distribution"": {{
        ""FORWARD"": {{ ""count"": {classCounts[0]}, ""ratio"": {(float)classCounts[0] / total * 100f:F1} }},
        ""FORWARD_LEFT"": {{ ""count"": {classCounts[1]}, ""ratio"": {(float)classCounts[1] / total * 100f:F1} }},
        ""FORWARD_RIGHT"": {{ ""count"": {classCounts[2]}, ""ratio"": {(float)classCounts[2] / total * 100f:F1} }},
        ""LEFT"": {{ ""count"": {classCounts[3]}, ""ratio"": {(float)classCounts[3] / total * 100f:F1} }},
        ""RIGHT"": {{ ""count"": {classCounts[4]}, ""ratio"": {(float)classCounts[4] / total * 100f:F1} }},
        ""BACKWARD"": {{ ""count"": {classCounts[5]}, ""ratio"": {(float)classCounts[5] / total * 100f:F1} }},
        ""NONE"": {{ ""count"": {classCounts[6]}, ""ratio"": {(float)classCounts[6] / total * 100f:F1} }}
    }},
    ""speed_stats"": {{
        ""min"": {minSpeed:F4},
        ""max"": {maxSpeed:F4},
        ""avg"": {avgSpeed:F4}
    }}
}}";
        File.WriteAllText(metaPath, metaJson);

        // 콘솔 출력
        Debug.Log($"[DataCollectorV2] ✓ 저장 완료!");
        Debug.Log($"  경로: {csvPath}");
        Debug.Log($"  프레임: {total}");
        Debug.Log($"  ═══════════════════════════════════════");
        Debug.Log($"  [클래스 분포]");
        for (int i = 0; i < 7; i++)
        {
            float ratio = (float)classCounts[i] / total * 100f;
            string bar = new string('█', (int)(ratio / 5));
            Debug.Log($"    {KeyActionNames[i],-14}: {classCounts[i],5} ({ratio,5:F1}%) {bar}");
        }
        Debug.Log($"  ═══════════════════════════════════════");
        Debug.Log($"  평균 속도: {avgSpeed:F2} m/s");
    }

    void OnDestroy()
    {
        if (frontRenderTexture != null) Destroy(frontRenderTexture);
        if (topRenderTexture != null) Destroy(topRenderTexture);
        if (frontCaptureTexture != null) Destroy(frontCaptureTexture);
        if (topCaptureTexture != null) Destroy(topCaptureTexture);
        if (topViewCameraObj != null) Destroy(topViewCameraObj);
    }

    void UpdateUI()
    {
        // 1. Status Text (Main State)
        if (uiStatusText != null)
        {
            if (isRecording)
                uiStatusText.text = $"<color=red>● REC: {frameCount}</color>";
            else
                uiStatusText.text = $"[{recordKey}] Rec  [{saveKey}] Save";
        }

        // 2. Info Text (Time / Unsaved)
        if (uiInfoText != null)
        {
            if (isRecording)
            {
                float duration = Time.time - recordingStartTime;
                uiInfoText.text = $"Time: {duration:F1}s";
            }
            else
            {
                if (frameBuffer.Count > 0)
                    uiInfoText.text = $"<color=yellow>Unsaved: {frameBuffer.Count}</color>";
                else
                    uiInfoText.text = "Ready";
            }
        }

        // 3. Action Text
        if (uiActionText != null)
        {
            uiActionText.text = $"Action: <color=#00FFFF>{KeyActionNames[(int)currentAction]}</color>";
        }

        // 4. Speed Text
        if (uiSpeedText != null)
        {
            float speed = wheelController != null ? wheelController.GetSpeedMS() : 0f;
            uiSpeedText.text = $"Speed: {speed:F2} m/s";
        }
    }
}
