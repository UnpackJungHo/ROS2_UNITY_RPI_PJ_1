using UnityEngine;
using System;
using System.IO;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Threading;
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

    [Header("DAgger Settings")]
    [Tooltip("자율주행 컨트롤러 (AI 모드 및 개입 상태 확인)")]
    public AutonomousDrivingController aiController;

    [Header("Image Settings")]
    [Tooltip("JPEG 품질 (front_1, front_2에 적용)")]
    [Range(0, 100)]
    public int jpegQuality = 85;

    [Header("Collection Settings")]
    public KeyCode recordKey = KeyCode.R;
    public KeyCode saveKey = KeyCode.T;
    public float captureInterval = 0.05f;  // 20 FPS
    [Tooltip("W 키를 누르면 자동으로 녹화 시작")]
    public bool autoStartOnMove = true;

    [Header("Status (Read Only)")]
    public bool isRecording = false;
    public int frameCount = 0;
    public KeyAction currentAction = KeyAction.NONE;

    // Front View 카메라
    private Camera frontCamera;

    // ============================================================
    // 4가지 이미지 조건용 렌더 텍스처
    // front_1: 200x66 (JPEG 85)
    // front_2: 320x120 (JPEG 85)
    // front_3: 200x66 (PNG)
    // front_4: 320x120 (PNG)
    // ============================================================
    private RenderTexture rt_small;      // 200x66
    private RenderTexture rt_large;      // 320x120
    private Texture2D tex_small;         // 200x66
    private Texture2D tex_large;         // 320x120

    private const int SMALL_WIDTH = 200;
    private const int SMALL_HEIGHT = 66;
    private const int LARGE_WIDTH = 320;
    private const int LARGE_HEIGHT = 120;

    // 데이터 저장
    private string sessionFolder;
    private List<DrivingFrameV2> frameBuffer = new List<DrivingFrameV2>();
    private float lastCaptureTime;
    private float recordingStartTime;

    // 비동기 파일 I/O
    private struct WriteTask
    {
        public string path;
        public byte[] data;
    }
    private ConcurrentQueue<WriteTask> writeQueue = new ConcurrentQueue<WriteTask>();
    private Thread writeThread;
    private volatile bool isWriteThreadRunning;

    // 입력 캐싱 (Update → FixedUpdate 전달용)
    private bool cachedIsIntervening;

    public class DrivingFrameV2
    {
        // 분류 학습용
        public string frontImagePath;
        public int keyAction;           // KeyAction enum의 int 값
        public string keyActionName;    // 가독성을 위한 이름
        public float speed;             // m/s
        public float timestamp;
        public bool isIntervention;     // DAgger: 전문가 개입 프레임 여부

        // 회귀 학습용 (조향/가속 연속값)
        public float steeringInput;     // 조향 입력 [-1, 1]
        public float throttleInput;     // 가속 입력 [-1, 1]
        public float brakeInput;        // 브레이크 입력 [0, 1]
        public float steeringAngle;     // 실제 조향각 (도)

        // 추가 상태 정보 (고급 학습용)
        public float acceleration;      // 현재 가속도 (m/s²)
        public float motorRPM;          // 모터 RPM
        public float slipRatio;         // 평균 슬립률
    }

    void Start()
    {
        // ============================================================
        // 4가지 조건용 렌더 텍스처 초기화
        // ============================================================
        // Small (200x66)
        rt_small = new RenderTexture(SMALL_WIDTH, SMALL_HEIGHT, 24);
        tex_small = new Texture2D(SMALL_WIDTH, SMALL_HEIGHT, TextureFormat.RGB24, false);
        
        // Large (320x120)
        rt_large = new RenderTexture(LARGE_WIDTH, LARGE_HEIGHT, 24);
        tex_large = new Texture2D(LARGE_WIDTH, LARGE_HEIGHT, TextureFormat.RGB24, false);

        AutoFindReferences();
        ValidateReferences();

        // 비동기 파일 쓰기 스레드 시작
        isWriteThreadRunning = true;
        writeThread = new Thread(WriteThreadLoop);
        writeThread.IsBackground = true;
        writeThread.Start();

        Debug.Log($"[DataCollectorV2] Ready! (분류 기반, 4가지 이미지 조건)");
        Debug.Log($"  front_1: {SMALL_WIDTH}x{SMALL_HEIGHT} JPEG(Q{jpegQuality})");
        Debug.Log($"  front_2: {LARGE_WIDTH}x{LARGE_HEIGHT} JPEG(Q{jpegQuality})");
        Debug.Log($"  front_3: {SMALL_WIDTH}x{SMALL_HEIGHT} PNG");
        Debug.Log($"  front_4: {LARGE_WIDTH}x{LARGE_HEIGHT} PNG");
        Debug.Log($"  '{recordKey}' 키: 녹화 시작/중지");
        Debug.Log($"  '{saveKey}' 키: 데이터 저장");
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
    }

    void ValidateReferences()
    {
        if (cameraPublisher == null)
            Debug.LogError("[DataCollectorV2] CameraPublisher를 찾을 수 없습니다!");
        if (wheelController == null)
            Debug.LogError("[DataCollectorV2] WheelTest를 찾을 수 없습니다!");
    }

    void Update()
    {
        if (frontCamera == null && cameraPublisher != null)
            frontCamera = cameraPublisher.GetCamera();

        // 녹화 토글
        if (Input.GetKeyDown(recordKey))
        {
            if (!isRecording)
                StartRecording();
            else
                StopRecording();
        }

        // W키 누를 시 자동 녹화 시작 (autoStartOnMove가 true이고 녹화 중이 아닐 때)
        if (autoStartOnMove && !isRecording)
        {
            if (Input.GetKeyDown(KeyCode.W) || Input.GetKeyDown(KeyCode.UpArrow))
            {
                StartRecording();
            }
        }

        // 저장
        if (Input.GetKeyDown(saveKey))
            SaveDataset();

        // 입력 상태 캐싱 (FixedUpdate에서 사용)
        currentAction = GetCurrentKeyAction();
        cachedIsIntervening = aiController != null && aiController.isAutonomousMode &&
                              (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.A) ||
                               Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.D) ||
                               Input.GetKey(KeyCode.UpArrow) || Input.GetKey(KeyCode.LeftArrow) ||
                               Input.GetKey(KeyCode.DownArrow) || Input.GetKey(KeyCode.RightArrow));

        UpdateUI();
    }

    void FixedUpdate()
    {
        if (isRecording && Time.fixedTime - lastCaptureTime >= captureInterval)
        {
            CaptureFrame();
            lastCaptureTime = Time.fixedTime;
        }
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

    void StartRecording()
    {
        string basePath = Path.Combine(Application.dataPath, "..", "TrainingData");
        sessionFolder = Path.Combine(basePath, $"session_{DateTime.Now:yyyyMMdd_HHmmss}");

        Directory.CreateDirectory(sessionFolder);
        
        // 4가지 이미지 조건별 폴더 생성
        Directory.CreateDirectory(Path.Combine(sessionFolder, "front_1"));  // 200x66 JPEG
        Directory.CreateDirectory(Path.Combine(sessionFolder, "front_2"));  // 320x120 JPEG
        Directory.CreateDirectory(Path.Combine(sessionFolder, "front_3"));  // 200x66 PNG
        Directory.CreateDirectory(Path.Combine(sessionFolder, "front_4"));  // 320x120 PNG

        frameBuffer.Clear();
        frameCount = 0;
        isRecording = true;
        lastCaptureTime = Time.time;
        recordingStartTime = Time.time;

        Debug.Log($"[DataCollectorV2] ● 녹화 시작: {sessionFolder}");
        Debug.Log($"  4가지 이미지 조건으로 동시 저장");
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

        KeyAction action = currentAction;  // Update()에서 캐싱된 입력 사용
        float speed = wheelController != null ? wheelController.GetSpeedMS() : 0f;

        // NONE + 정지 상태는 스킵 (의미없는 데이터)
        if (action == KeyAction.NONE && speed < 0.01f)
            return;

        string frameNum = $"{frameCount:D6}";
        RenderTexture originalTarget = frontCamera.targetTexture;

        // 텍스처 캡처 (메인 스레드에서 수행)
        CaptureToTexture(frontCamera, rt_small, tex_small);
        CaptureToTexture(frontCamera, rt_large, tex_large);

        // 인코딩 (메인 스레드에서 수행 - Unity API 제약)
        byte[] jpgSmall = tex_small.EncodeToJPG(jpegQuality);
        byte[] jpgLarge = tex_large.EncodeToJPG(jpegQuality);
        byte[] pngSmall = tex_small.EncodeToPNG();
        byte[] pngLarge = tex_large.EncodeToPNG();

        frontCamera.targetTexture = originalTarget;

        // 비동기 파일 쓰기 큐에 추가 (백그라운드 스레드에서 처리)
        writeQueue.Enqueue(new WriteTask { path = Path.Combine(sessionFolder, "front_1", $"frame_{frameNum}.jpg"), data = jpgSmall });
        writeQueue.Enqueue(new WriteTask { path = Path.Combine(sessionFolder, "front_2", $"frame_{frameNum}.jpg"), data = jpgLarge });
        writeQueue.Enqueue(new WriteTask { path = Path.Combine(sessionFolder, "front_3", $"frame_{frameNum}.png"), data = pngSmall });
        writeQueue.Enqueue(new WriteTask { path = Path.Combine(sessionFolder, "front_4", $"frame_{frameNum}.png"), data = pngLarge });

        // WheelTest에서 회귀 학습용 데이터 추출
        float steeringInput = wheelController != null ? wheelController.GetSteeringInput() : 0f;
        float throttleInput = wheelController != null ? wheelController.GetThrottleInput() : 0f;
        float brakeInput = wheelController != null ? wheelController.GetBrakeInput() : 0f;
        float steeringAngle = wheelController != null ? wheelController.GetSteeringAngle() : 0f;
        float acceleration = wheelController != null ? wheelController.GetAcceleration() : 0f;
        float motorRPM = wheelController != null ? wheelController.GetMotorRPM() : 0f;

        DrivingFrameV2 frame = new DrivingFrameV2
        {
            // 분류 학습용
            frontImagePath = $"front_1/frame_{frameNum}.jpg",
            keyAction = (int)action,
            keyActionName = KeyActionNames[(int)action],
            speed = speed,
            timestamp = Time.fixedTime - recordingStartTime,
            isIntervention = cachedIsIntervening,

            // 회귀 학습용
            steeringInput = steeringInput,
            throttleInput = throttleInput,
            brakeInput = brakeInput,
            steeringAngle = steeringAngle,

            // 추가 상태 정보
            acceleration = acceleration,
            motorRPM = motorRPM
        };

        frameBuffer.Add(frame);
        frameCount++;

        if (frameCount % 100 == 0)
            Debug.Log($"[DataCollectorV2] {frameCount} frames captured (x4 formats)");
    }

    /// <summary>
    /// 카메라 렌더링 결과를 텍스처로 캡처
    /// </summary>
    void CaptureToTexture(Camera cam, RenderTexture rt, Texture2D tex)
    {
        cam.targetTexture = rt;
        cam.Render();

        RenderTexture.active = rt;
        tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        tex.Apply();
        RenderTexture.active = null;
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

        byte[] bytes = tex.EncodeToPNG();
        File.WriteAllBytes(path, bytes);
    }

    void SaveDataset()
    {
        if (frameBuffer.Count == 0)
        {
            Debug.LogWarning("[DataCollectorV2] 저장할 데이터가 없습니다!");
            return;
        }

        // 남은 이미지 파일 쓰기 완료 대기
        while (!writeQueue.IsEmpty)
            Thread.Sleep(10);

        // CSV 저장 (분류 + 회귀 + 상태 정보 포함)
        string csvPath = Path.Combine(sessionFolder, "driving_log.csv");
        using (StreamWriter writer = new StreamWriter(csvPath))
        {
            // 헤더 (분류 + 회귀 + 추가 상태)
            writer.WriteLine("front_image,key_action,key_action_name,speed,timestamp,intervention," +
                           "steering_input,throttle_input,brake_input,steering_angle," +
                           "acceleration,motor_rpm,slip_ratio");

            // 데이터
            foreach (var frame in frameBuffer)
            {
                int intervention = frame.isIntervention ? 1 : 0;
                writer.WriteLine($"{frame.frontImagePath},{frame.keyAction},{frame.keyActionName}," +
                               $"{frame.speed:F6},{frame.timestamp:F6},{intervention}," +
                               $"{frame.steeringInput:F6},{frame.throttleInput:F6},{frame.brakeInput:F6},{frame.steeringAngle:F6}," +
                               $"{frame.acceleration:F6},{frame.motorRPM:F2},{frame.slipRatio:F6}");
            }
        }

        // 클래스 분포 및 통계 계산
        int[] classCounts = new int[7];
        float totalSpeed = 0f, minSpeed = float.MaxValue, maxSpeed = float.MinValue;
        float totalSteering = 0f, minSteering = float.MaxValue, maxSteering = float.MinValue;
        float totalThrottle = 0f, minThrottle = float.MaxValue, maxThrottle = float.MinValue;
        float totalAccel = 0f, minAccel = float.MaxValue, maxAccel = float.MinValue;

        foreach (var frame in frameBuffer)
        {
            classCounts[frame.keyAction]++;

            // 속도 통계
            totalSpeed += frame.speed;
            minSpeed = Mathf.Min(minSpeed, frame.speed);
            maxSpeed = Mathf.Max(maxSpeed, frame.speed);

            // 조향 통계
            totalSteering += frame.steeringInput;
            minSteering = Mathf.Min(minSteering, frame.steeringInput);
            maxSteering = Mathf.Max(maxSteering, frame.steeringInput);

            // 가속 통계
            totalThrottle += frame.throttleInput;
            minThrottle = Mathf.Min(minThrottle, frame.throttleInput);
            maxThrottle = Mathf.Max(maxThrottle, frame.throttleInput);

            // 가속도 통계
            totalAccel += frame.acceleration;
            minAccel = Mathf.Min(minAccel, frame.acceleration);
            maxAccel = Mathf.Max(maxAccel, frame.acceleration);
        }

        int total = frameBuffer.Count;
        float avgSpeed = totalSpeed / total;
        float avgSteering = totalSteering / total;
        float avgThrottle = totalThrottle / total;
        float avgAccel = totalAccel / total;

        // 메타데이터 JSON 저장
        string metaPath = Path.Combine(sessionFolder, "metadata.json");
        string metaJson = $@"{{
    ""version"": ""v3_unified"",
    ""description"": ""분류 + 회귀 통합 데이터셋"",
    ""total_frames"": {total},
    ""num_classes"": 7,
    ""class_names"": [""FORWARD"", ""FORWARD_LEFT"", ""FORWARD_RIGHT"", ""LEFT"", ""RIGHT"", ""BACKWARD"", ""NONE""],
    ""image_conditions"": {{
        ""front_1"": {{ ""width"": {SMALL_WIDTH}, ""height"": {SMALL_HEIGHT}, ""format"": ""JPEG"", ""quality"": {jpegQuality} }},
        ""front_2"": {{ ""width"": {LARGE_WIDTH}, ""height"": {LARGE_HEIGHT}, ""format"": ""JPEG"", ""quality"": {jpegQuality} }},
        ""front_3"": {{ ""width"": {SMALL_WIDTH}, ""height"": {SMALL_HEIGHT}, ""format"": ""PNG"" }},
        ""front_4"": {{ ""width"": {LARGE_WIDTH}, ""height"": {LARGE_HEIGHT}, ""format"": ""PNG"" }}
    }},
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
    ""acceleration_stats"": {{
        ""min"": {minAccel:F4},
        ""max"": {maxAccel:F4},
        ""avg"": {avgAccel:F4}
    }}
}}";
        File.WriteAllText(metaPath, metaJson);

        // 콘솔 출력
        Debug.Log($"[DataCollectorV3] ✓ 저장 완료! (분류 + 회귀 통합)");
        Debug.Log($"  경로: {csvPath}");
        Debug.Log($"  프레임: {total}");
        Debug.Log($"  ═══════════════════════════════════════");
        Debug.Log($"  [클래스 분포 - 분류 학습용]");
        for (int i = 0; i < 7; i++)
        {
            float ratio = (float)classCounts[i] / total * 100f;
            string bar = new string('█', (int)(ratio / 5));
            Debug.Log($"    {KeyActionNames[i],-14}: {classCounts[i],5} ({ratio,5:F1}%) {bar}");
        }
        Debug.Log($"  ═══════════════════════════════════════");
        Debug.Log($"  [회귀 학습용 데이터 통계]");
        Debug.Log($"    Steering: [{minSteering:F2}, {maxSteering:F2}] avg={avgSteering:F3}");
        Debug.Log($"    Throttle: [{minThrottle:F2}, {maxThrottle:F2}] avg={avgThrottle:F3}");
        Debug.Log($"    Speed:    [{minSpeed:F2}, {maxSpeed:F2}] avg={avgSpeed:F2} m/s");
        Debug.Log($"    Accel:    [{minAccel:F2}, {maxAccel:F2}] avg={avgAccel:F2} m/s²");
        Debug.Log($"  ═══════════════════════════════════════");
    }

    void OnDestroy()
    {
        // 쓰기 스레드 종료 및 잔여 큐 처리 대기
        isWriteThreadRunning = false;
        if (writeThread != null && writeThread.IsAlive)
            writeThread.Join(3000);

        if (rt_small != null) Destroy(rt_small);
        if (rt_large != null) Destroy(rt_large);
        if (tex_small != null) Destroy(tex_small);
        if (tex_large != null) Destroy(tex_large);
    }

    void WriteThreadLoop()
    {
        while (isWriteThreadRunning || !writeQueue.IsEmpty)
        {
            if (writeQueue.TryDequeue(out WriteTask task))
            {
                try
                {
                    File.WriteAllBytes(task.path, task.data);
                }
                catch (Exception e)
                {
                    Debug.LogError($"[DataCollectorV2] 파일 쓰기 실패: {task.path}\n{e.Message}");
                }
            }
            else
            {
                Thread.Sleep(1);
            }
        }
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
