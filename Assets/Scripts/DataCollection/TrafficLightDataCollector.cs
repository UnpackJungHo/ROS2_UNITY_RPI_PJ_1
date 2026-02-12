using UnityEngine;
using System.IO;
using System.Collections.Generic;

/// <summary>
/// Unity 시뮬레이션에서 신호등 데이터 자동 수집 + 자동 라벨링
///
/// 동작: F9로 수집 시작/종료, 1초마다 카메라 이미지 캡처 + YOLO 라벨 자동 생성
/// 출력: python/datasets/directlyDataset/images/*.jpg + labels/*.txt (YOLO 포맷)
/// 클래스: 0=Green, 1=Red, 2=Yellow (기존 Roboflow 데이터셋과 동일)
///
/// 사용법:
/// 1. 빈 GameObject에 이 스크립트 추가
/// 2. Inspector에서 cameraPublisher 할당
/// 3. Play → 항상 디버그 박스 표시됨 → F9로 수집 시작/종료
/// </summary>
public class TrafficLightDataCollector : MonoBehaviour
{
    [Header("References")]
    public CameraPublisher cameraPublisher;

    [Header("Collection Settings")]
    public float captureInterval = 1f;
    public KeyCode toggleKey = KeyCode.F9;
    public string savePath = "python/datasets/directlyDataset";

    [Tooltip("신호등이 없는 프레임도 저장 (배경 네거티브 샘플)")]
    public bool saveEmptyFrames = false;

    [Tooltip("최소 바운딩 박스 크기 (뷰포트 비율, 0~1)")]
    public float minBoxViewport = 0.015f;

    [Header("Debug")]
    [Tooltip("게임 뷰에 바운딩 박스 실시간 표시")]
    public bool showDebugBoxes = true;

    [Header("Status")]
    [SerializeField] private bool isCollecting = false;
    [SerializeField] private int imageCount = 0;
    [SerializeField] private int labeledCount = 0;

    private Camera cam;
    private TrafficLight[] trafficLights;
    private float lastCaptureTime;
    private string fullSavePath;

    // OnGUI 디버그용: 매 프레임 갱신되는 바운딩 박스 리스트
    private readonly List<DebugBox> debugBoxes = new List<DebugBox>();

    private struct DebugBox
    {
        public Rect viewportRect; // (x, y, w, h) in viewport 0~1, y=bottom-up
        public string label;
        public Color color;
    }

    private readonly Dictionary<string, int> classMap = new Dictionary<string, int>
    {
        {"green", 0}, {"red", 1}, {"yellow", 2}
    };

    private readonly Dictionary<string, Color> colorMap = new Dictionary<string, Color>
    {
        {"green", Color.green}, {"red", Color.red}, {"yellow", Color.yellow}
    };

    void Start()
    {
        fullSavePath = Path.Combine(Application.dataPath, "..", savePath);
        Directory.CreateDirectory(Path.Combine(fullSavePath, "images"));
        Directory.CreateDirectory(Path.Combine(fullSavePath, "labels"));

        string imgDir = Path.Combine(fullSavePath, "images");
        imageCount = Directory.GetFiles(imgDir, "*.jpg").Length;

        trafficLights = FindObjectsOfType<TrafficLight>();

        Debug.Log($"[DataCollector] === INIT ===");
        Debug.Log($"[DataCollector] TrafficLights found: {trafficLights.Length}");
        Debug.Log($"[DataCollector] Save: {fullSavePath}  |  Existing: {imageCount}");
        for (int i = 0; i < trafficLights.Length; i++)
        {
            var tl = trafficLights[i];
            Debug.Log($"[DataCollector]   [{i}] {tl.gameObject.name}" +
                      $"  signalImage={tl.signalImage != null}" +
                      $"  fitTarget={tl.fitTargetRenderer != null}");
        }
        Debug.Log($"[DataCollector] Press {toggleKey} to toggle recording");
    }

    void LateUpdate()
    {
        // 카메라 참조 갱신
        if (cam == null && cameraPublisher != null)
            cam = cameraPublisher.GetCamera();

        // F9 토글
        if (Input.GetKeyDown(toggleKey))
        {
            isCollecting = !isCollecting;
            if (isCollecting)
            {
                trafficLights = FindObjectsOfType<TrafficLight>(); // 갱신
                if (cam == null)
                {
                    Debug.LogError("[DataCollector] Camera is NULL!");
                    isCollecting = false;
                    return;
                }
            }
            Debug.Log($"[DataCollector] {(isCollecting ? "REC START" : "REC STOP")}  total={imageCount}  labeled={labeledCount}");
        }

        // 디버그 박스 항상 갱신 (수집 여부와 무관)
        if (showDebugBoxes)
            UpdateDebugBoxes();

        // 수집
        if (isCollecting && Time.time - lastCaptureTime >= captureInterval)
        {
            CaptureAndLabel();
            lastCaptureTime = Time.time;
        }
    }

    // ========== 디버그 박스 갱신 (매 프레임) ==========
    void UpdateDebugBoxes()
    {
        debugBoxes.Clear();
        if (cam == null || trafficLights == null) return;

        foreach (var tl in trafficLights)
        {
            Renderer renderer = GetRenderer(tl);
            if (renderer == null) continue;

            Rect? vpRect = CalcViewportRect(renderer);
            if (vpRect == null) continue;

            Color c = colorMap.ContainsKey(tl.currentColor) ? colorMap[tl.currentColor] : Color.gray;
            debugBoxes.Add(new DebugBox
            {
                viewportRect = vpRect.Value,
                label = $"{tl.currentColor.ToUpper()} ({tl.gameObject.name})",
                color = c
            });
        }
    }

    // ========== Viewport 좌표로 바운딩 박스 계산 ==========
    Rect? CalcViewportRect(Renderer renderer)
    {
        Bounds bounds = renderer.bounds;

        // 중심이 카메라 뒤면 즉시 스킵
        Vector3 vpCenter = cam.WorldToViewportPoint(bounds.center);
        if (vpCenter.z <= 0) return null;

        // 8 코너 투영
        Vector3[] corners = GetBoundsCorners(bounds);
        float minX = float.MaxValue, maxX = float.MinValue;
        float minY = float.MaxValue, maxY = float.MinValue;
        int visibleCount = 0;

        foreach (var corner in corners)
        {
            Vector3 vp = cam.WorldToViewportPoint(corner);
            if (vp.z <= 0) continue;
            visibleCount++;

            minX = Mathf.Min(minX, vp.x);
            maxX = Mathf.Max(maxX, vp.x);
            minY = Mathf.Min(minY, vp.y);
            maxY = Mathf.Max(maxY, vp.y);
        }

        if (visibleCount == 0) return null;

        // 뷰포트 범위로 클램핑 (0~1)
        minX = Mathf.Clamp01(minX);
        maxX = Mathf.Clamp01(maxX);
        minY = Mathf.Clamp01(minY);
        maxY = Mathf.Clamp01(maxY);

        float w = maxX - minX;
        float h = maxY - minY;
        if (w < minBoxViewport || h < minBoxViewport) return null;

        return new Rect(minX, minY, w, h);
    }

    Renderer GetRenderer(TrafficLight tl)
    {
        // signalImage(SpriteRenderer) 우선, 없으면 fitTargetRenderer(패널) 사용
        if (tl.signalImage != null) return tl.signalImage;
        if (tl.fitTargetRenderer != null) return tl.fitTargetRenderer;
        return null;
    }

    // ========== 캡처 + 라벨 저장 ==========
    void CaptureAndLabel()
    {
        RenderTexture rt = cameraPublisher.GetRenderTexture();
        if (cam == null || rt == null) return;

        int width = rt.width;
        int height = rt.height;

        // ★ RT를 먼저 설정해야 뷰포트 좌표가 RT 비율(4:3)과 일치함
        RenderTexture prevTarget = cam.targetTexture;
        Rect prevRect = cam.rect; // 기존 레터박스 Rect 저장

        cam.targetTexture = rt;
        cam.rect = new Rect(0, 0, 1, 1); // 캡처할 때는 전체 화면(1:1) 비율로 복원

        // 라벨 생성 (이제 뷰포트 계산이 RT 비율 기준 + 전체 화면 Rect 기준)
        List<string> labels = new List<string>();
        foreach (var tl in trafficLights)
        {
            Renderer renderer = GetRenderer(tl);
            if (renderer == null) continue;

            Rect? vpRect = CalcViewportRect(renderer);
            if (vpRect == null) continue;

            Rect r = vpRect.Value;

            // YOLO 포맷: class centerX centerY width height (0~1)
            // Viewport Y: 0=하단, 1=상단 → YOLO Y: 0=상단, 1=하단 → 반전
            float yoloCX = r.x + r.width / 2f;
            float yoloCY = 1f - (r.y + r.height / 2f);
            float yoloW = r.width;
            float yoloH = r.height;

            if (!classMap.TryGetValue(tl.currentColor, out int classId)) continue;

            labels.Add($"{classId} {yoloCX:F6} {yoloCY:F6} {yoloW:F6} {yoloH:F6}");
        }

        if (labels.Count == 0 && !saveEmptyFrames)
        {
            cam.targetTexture = prevTarget;
            cam.rect = prevRect; // 복원
            return;
        }

        // RT에 최신 프레임 렌더링
        cam.Render();

        RenderTexture.active = rt;
        Texture2D tex = new Texture2D(width, height, TextureFormat.RGB24, false);
        tex.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        tex.Apply();
        RenderTexture.active = null;

        // 저장
        string fileName = $"unity_{imageCount:D6}";
        File.WriteAllBytes(
            Path.Combine(fullSavePath, "images", fileName + ".jpg"),
            tex.EncodeToJPG(90));
        File.WriteAllText(
            Path.Combine(fullSavePath, "labels", fileName + ".txt"),
            string.Join("\n", labels));

        if (labels.Count > 0) labeledCount++;
        imageCount++;

        // 원래 상태 복원
        cam.targetTexture = prevTarget;
        cam.rect = prevRect;

        Destroy(tex);

        if (imageCount % 10 == 0)
            Debug.Log($"[DataCollector] {imageCount} captured ({labeledCount} labeled)");
    }

    Vector3[] GetBoundsCorners(Bounds bounds)
    {
        Vector3 c = bounds.center;
        Vector3 e = bounds.extents;
        return new Vector3[]
        {
            c + new Vector3(-e.x, -e.y, -e.z),
            c + new Vector3(-e.x, -e.y,  e.z),
            c + new Vector3(-e.x,  e.y, -e.z),
            c + new Vector3(-e.x,  e.y,  e.z),
            c + new Vector3( e.x, -e.y, -e.z),
            c + new Vector3( e.x, -e.y,  e.z),
            c + new Vector3( e.x,  e.y, -e.z),
            c + new Vector3( e.x,  e.y,  e.z),
        };
    }

    // ========== OnGUI: 디버그 시각화 ==========
    void OnGUI()
    {
        // 수집 상태 표시
        if (isCollecting)
        {
            GUI.color = Color.red;
            GUIStyle recStyle = new GUIStyle(GUI.skin.label) { fontSize = 16, fontStyle = FontStyle.Bold };
            GUI.Label(new Rect(10, 10, 500, 30),
                $"● REC  |  {imageCount} images  ({labeledCount} labeled)", recStyle);
        }

        if (!showDebugBoxes || debugBoxes.Count == 0 || cam == null) return;
        
        // Front View 모드 확인: targetTexture가 null일 때만 화면에 그리는 중임
        // (TopView나 BackView는 RT에만 그리고 있으므로 여기서는 안 보이게 해야 함)
        if (cam.targetTexture != null) return;

        // 디버그 박스 정보 표시
        GUI.color = Color.white;
        GUI.Label(new Rect(10, isCollecting ? 40 : 10, 300, 20),
            $"[Debug] Visible TLs: {debugBoxes.Count}");

        Rect camRect = cam.rect; // 현재 카메라의 Viewport Rect (0.71 등 레터박스 영역)

        foreach (var box in debugBoxes)
        {
            // Viewport → GUI 좌표 변환
            
            // 1. 박스의 실제 화면상 Viewport X 좌표
            // camRect.x (카메라 시작점) + box.x (카메라 내부 상대좌표) * camRect.width (카메라 너비 비율)
            float screenVpX = camRect.x + (box.viewportRect.x * camRect.width);
            
            // 2. 박스의 실제 화면상 Viewport Y 좌표 (Bottom-Left 기준)
            float screenVpY = camRect.y + (box.viewportRect.y * camRect.height);
            
            // 3. 박스의 실제 화면상 너비와 높이 비율
            float screenVpW = box.viewportRect.width * camRect.width;
            float screenVpH = box.viewportRect.height * camRect.height;

            // 4. GUI 좌표계로 변환 (Top-Left 기준, 픽셀 단위)
            float guiX = screenVpX * Screen.width;
            float guiY = (1f - (screenVpY + screenVpH)) * Screen.height; // Y축 반전 주의 (Top 기준 좌표 = 전체 1 - (Bottom 기준 Y + H))
            float guiW = screenVpW * Screen.width;
            float guiH = screenVpH * Screen.height;

            Rect guiRect = new Rect(guiX, guiY, guiW, guiH);

            // 박스 외곽선
            DrawOutline(guiRect, box.color, 2f);

            // 라벨
            GUI.color = box.color;
            GUIStyle style = new GUIStyle(GUI.skin.label) { fontSize = 14, fontStyle = FontStyle.Bold };
            GUI.Label(new Rect(guiRect.x, guiRect.y - 22, 300, 22), box.label, style);
        }
    }

    void DrawOutline(Rect rect, Color color, float t)
    {
        GUI.color = color;
        GUI.DrawTexture(new Rect(rect.x, rect.y, rect.width, t), Texture2D.whiteTexture);             // top
        GUI.DrawTexture(new Rect(rect.x, rect.yMax - t, rect.width, t), Texture2D.whiteTexture);      // bottom
        GUI.DrawTexture(new Rect(rect.x, rect.y, t, rect.height), Texture2D.whiteTexture);            // left
        GUI.DrawTexture(new Rect(rect.xMax - t, rect.y, t, rect.height), Texture2D.whiteTexture);     // right
    }
}
