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
    public float captureInterval = 0.05f;  // 20 FPS

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

        // ============================================================
        // 데이터 품질 분석 (steering 분포)
        // ============================================================
        // 이 트랙은 D자 모양으로 완만한 커브가 대부분입니다.
        // 완만한 커브도 "커브"로 인식하도록 기준을 세분화했습니다.
        //
        // 분류 기준:
        //   직진:       |steering| < 0.03  (거의 핸들 안 돌림)
        //   완만한 커브: 0.03 <= |steering| < 0.15
        //   중간 커브:   0.15 <= |steering| < 0.35
        //   급커브:      |steering| >= 0.35
        // ============================================================

        int strongLeft = 0;     // steering < -0.35  (급 좌회전)
        int mediumLeft = 0;     // -0.35 <= steering < -0.15 (중간 좌회전)
        int gentleLeft = 0;     // -0.15 <= steering < -0.03 (완만한 좌회전)
        int straight = 0;       // -0.03 <= steering <= 0.03 (직진)
        int gentleRight = 0;    // 0.03 < steering <= 0.15 (완만한 우회전)
        int mediumRight = 0;    // 0.15 < steering <= 0.35 (중간 우회전)
        int strongRight = 0;    // steering > 0.35 (급 우회전)

        float minSteering = float.MaxValue;
        float maxSteering = float.MinValue;
        float sumSteering = 0f;

        float minThrottle = float.MaxValue;
        float maxThrottle = float.MinValue;
        float sumThrottle = 0f;

        float minSpeed = float.MaxValue;
        float maxSpeed = float.MinValue;
        float sumSpeed = 0f;

        foreach (var frame in frameBuffer)
        {
            float s = frame.steering;
            float t = frame.throttle;
            float spd = frame.speed;

            // Steering 분포 계산 (세분화된 기준)
            if (s < -0.35f) strongLeft++;
            else if (s < -0.15f) mediumLeft++;
            else if (s < -0.03f) gentleLeft++;
            else if (s <= 0.03f) straight++;
            else if (s <= 0.15f) gentleRight++;
            else if (s <= 0.35f) mediumRight++;
            else strongRight++;

            // 통계
            minSteering = Mathf.Min(minSteering, s);
            maxSteering = Mathf.Max(maxSteering, s);
            sumSteering += s;

            minThrottle = Mathf.Min(minThrottle, t);
            maxThrottle = Mathf.Max(maxThrottle, t);
            sumThrottle += t;

            minSpeed = Mathf.Min(minSpeed, spd);
            maxSpeed = Mathf.Max(maxSpeed, spd);
            sumSpeed += spd;
        }

        int total = frameBuffer.Count;
        float avgSteering = sumSteering / total;
        float avgThrottle = sumThrottle / total;
        float avgSpeed = sumSpeed / total;

        // 커브 데이터 합계
        int totalLeft = strongLeft + mediumLeft + gentleLeft;
        int totalRight = strongRight + mediumRight + gentleRight;
        int totalCurve = totalLeft + totalRight;

        // 품질 평가 (이 트랙에 맞게 조정)
        // D자 트랙: 완만한 커브가 대부분이므로 직진 기준을 엄격하게
        float straightRatio = (float)straight / total * 100f;
        float curveRatio = (float)totalCurve / total * 100f;
        float leftRatio = (float)totalLeft / total * 100f;
        float rightRatio = (float)totalRight / total * 100f;

        string qualityGrade;
        string qualityNote;

        // 품질 기준 (D자 트랙 기준)
        // - 직진이 20% 이하: EXCELLENT
        // - 직진이 35% 이하: GOOD
        // - 직진이 50% 이하: FAIR
        // - 직진이 50% 초과: POOR
        if (straightRatio <= 20f)
        {
            qualityGrade = "EXCELLENT";
            qualityNote = "커브 데이터가 매우 충분합니다!";
        }
        else if (straightRatio <= 35f)
        {
            qualityGrade = "GOOD";
            qualityNote = "양호한 데이터입니다.";
        }
        else if (straightRatio <= 50f)
        {
            qualityGrade = "FAIR";
            qualityNote = "직진 구간에서 속도를 더 높이거나, 커브에서 더 천천히 주행하세요.";
        }
        else
        {
            qualityGrade = "POOR";
            qualityNote = "직진 데이터가 너무 많습니다! 커브에서 아주 천천히 주행하세요.";
        }

        // 좌우 균형 체크
        string balanceNote = "";
        if (totalCurve > 0)
        {
            float leftOfCurve = (float)totalLeft / totalCurve * 100f;
            if (leftOfCurve < 20f)
            {
                balanceNote = " [주의] 좌회전 데이터 부족 - 반시계 방향 주행 추가 권장";
            }
            else if (leftOfCurve > 80f)
            {
                balanceNote = " [주의] 우회전 데이터 부족 - 시계 방향 주행 추가 권장";
            }
        }

        // 메타데이터 JSON 저장 (품질 정보 포함)
        string metaPath = Path.Combine(sessionFolder, "metadata.json");
        string metaJson = $@"{{
    ""total_frames"": {total},
    ""front_image_size"": {{ ""width"": {frontImageWidth}, ""height"": {frontImageHeight} }},
    ""top_image_size"": {{ ""width"": {topViewImageSize}, ""height"": {topViewImageSize} }},
    ""top_view_enabled"": {captureTopView.ToString().ToLower()},
    ""top_view_height"": {topViewHeight},
    ""capture_fps"": {1f / captureInterval:F1},
    ""created"": ""{DateTime.Now:yyyy-MM-dd HH:mm:ss}"",
    ""data_quality"": {{
        ""grade"": ""{qualityGrade}"",
        ""note"": ""{qualityNote}{balanceNote}""
    }},
    ""steering_summary"": {{
        ""straight_ratio"": {straightRatio:F1},
        ""curve_ratio"": {curveRatio:F1},
        ""left_ratio"": {leftRatio:F1},
        ""right_ratio"": {rightRatio:F1}
    }},
    ""steering_distribution"": {{
        ""strong_left"": {{ ""count"": {strongLeft}, ""ratio"": {(float)strongLeft / total * 100f:F1}, ""range"": ""< -0.35"" }},
        ""medium_left"": {{ ""count"": {mediumLeft}, ""ratio"": {(float)mediumLeft / total * 100f:F1}, ""range"": ""-0.35 ~ -0.15"" }},
        ""gentle_left"": {{ ""count"": {gentleLeft}, ""ratio"": {(float)gentleLeft / total * 100f:F1}, ""range"": ""-0.15 ~ -0.03"" }},
        ""straight"": {{ ""count"": {straight}, ""ratio"": {straightRatio:F1}, ""range"": ""-0.03 ~ 0.03"" }},
        ""gentle_right"": {{ ""count"": {gentleRight}, ""ratio"": {(float)gentleRight / total * 100f:F1}, ""range"": ""0.03 ~ 0.15"" }},
        ""medium_right"": {{ ""count"": {mediumRight}, ""ratio"": {(float)mediumRight / total * 100f:F1}, ""range"": ""0.15 ~ 0.35"" }},
        ""strong_right"": {{ ""count"": {strongRight}, ""ratio"": {(float)strongRight / total * 100f:F1}, ""range"": ""> 0.35"" }}
    }},
    ""steering_stats"": {{
        ""min"": {minSteering:F4},
        ""max"": {maxSteering:F4},
        ""avg"": {avgSteering:F4}
    }},
    ""throttle_stats"": {{
        ""min"": {minThrottle:F4},
        ""max"": {maxThrottle:F4},
        ""avg"": {avgThrottle:F4}
    }},
    ""speed_stats"": {{
        ""min"": {minSpeed:F4},
        ""max"": {maxSpeed:F4},
        ""avg"": {avgSpeed:F4}
    }}
}}";
        File.WriteAllText(metaPath, metaJson);

        // 콘솔 출력 (품질 정보 포함)
        Debug.Log($"[DataCollector] ✓ 저장 완료!");
        Debug.Log($"  경로: {csvPath}");
        Debug.Log($"  프레임: {total}");
        Debug.Log($"  ═══════════════════════════════════════");
        Debug.Log($"  [데이터 품질: {qualityGrade}]");
        Debug.Log($"  {qualityNote}");
        if (!string.IsNullOrEmpty(balanceNote))
        {
            Debug.Log($"  {balanceNote}");
        }
        Debug.Log($"  ═══════════════════════════════════════");
        Debug.Log($"  [요약] 직진: {straightRatio:F1}% | 커브: {curveRatio:F1}%");
        Debug.Log($"         좌회전: {leftRatio:F1}% | 우회전: {rightRatio:F1}%");
        Debug.Log($"  ───────────────────────────────────────");
        Debug.Log($"  [상세 Steering 분포]");
        Debug.Log($"    급좌회전   (< -0.35): {strongLeft,5} ({(float)strongLeft / total * 100f:F1}%)");
        Debug.Log($"    중간좌회전 (-0.35~-0.15): {mediumLeft,5} ({(float)mediumLeft / total * 100f:F1}%)");
        Debug.Log($"    완만좌회전 (-0.15~-0.03): {gentleLeft,5} ({(float)gentleLeft / total * 100f:F1}%)");
        Debug.Log($"    직진       (-0.03~0.03): {straight,5} ({straightRatio:F1}%)");
        Debug.Log($"    완만우회전 (0.03~0.15): {gentleRight,5} ({(float)gentleRight / total * 100f:F1}%)");
        Debug.Log($"    중간우회전 (0.15~0.35): {mediumRight,5} ({(float)mediumRight / total * 100f:F1}%)");
        Debug.Log($"    급우회전   (> 0.35): {strongRight,5} ({(float)strongRight / total * 100f:F1}%)");
        Debug.Log($"  ───────────────────────────────────────");
        Debug.Log($"  평균 속도: {avgSpeed:F2} m/s");
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
