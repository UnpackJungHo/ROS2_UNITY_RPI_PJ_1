using UnityEngine;
using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

/// <summary>
/// DART 데이터 증강 기반 규칙 리플레이 컨트롤러
///
/// 기존 세션의 액션 시퀀스를 변형(perturbation)하여
/// 규칙 기반으로 재주행, 새로운 학습 데이터를 생성합니다.
///
/// 사용법:
/// 1. Inspector에서 sessionIndex 설정 (0~N)
/// 2. 차량을 트랙 시작점에 배치
/// 3. R키 → 녹화 시작 (DrivingDataCollectorV2)
/// 4. K키 → 변형된 액션으로 자동 주행 시작
/// 5. 주행 종료 시 자동 정지 (또는 K키로 수동 정지)
/// 6. R키 → 녹화 중지, T키 → 저장
/// </summary>
public class RuleBasedReplayController : MonoBehaviour
{
    // DrivingDataCollectorV2의 KeyAction과 동일
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

    [Header("References")]
    [Tooltip("차량 컨트롤러")]
    public WheelTest wheelController;

    [Header("Session Selection")]
    [Tooltip("리플레이할 세션 인덱스 (0-based)")]
    public int sessionIndex = 0;

    [Tooltip("선택된 세션명 (읽기전용)")]
    [SerializeField] private string selectedSessionName = "";

    [Tooltip("사용 가능한 세션 수 (읽기전용)")]
    [SerializeField] private int totalSessions = 0;

    [Header("Replay Settings")]
    [Tooltip("리플레이 토글 키")]
    public KeyCode replayToggleKey = KeyCode.K;

    [Tooltip("재생 간격 (초) - 원본과 동일하게 20FPS")]
    public float replayInterval = 0.05f;

    [Header("Control Settings")]
    [Tooltip("전진 throttle 값")]
    [Range(0.1f, 1.0f)]
    public float forwardThrottle = 0.8f;

    [Tooltip("조향 강도")]
    [Range(0.1f, 1.0f)]
    public float steeringStrength = 1.0f;

    [Header("DART Perturbation")]
    [Tooltip("각 프레임의 변형 확률 (0~1)")]
    [Range(0f, 0.5f)]
    public float perturbationProbability = 0.12f;

    [Tooltip("최대 연속 변형 프레임 수")]
    public int maxConsecutivePerturbed = 3;

    [Header("Debug (Read Only)")]
    [SerializeField] private bool isReplaying = false;
    [SerializeField] private int currentFrameIndex = 0;
    [SerializeField] private int totalFrames = 0;
    [SerializeField] private int perturbedCount = 0;
    [SerializeField] private KeyAction currentAction = KeyAction.NONE;
    [SerializeField] private bool currentFramePerturbed = false;

    // 공개 프로퍼티 (DrivingDataCollectorV2 연동용)
    public bool IsReplaying => isReplaying;
    public DrivingDataCollectorV2.KeyAction CurrentAction =>
        (DrivingDataCollectorV2.KeyAction)(int)currentAction;

    // 내부 데이터
    private List<string> sessionPaths = new List<string>();
    private List<KeyAction> originalActions = new List<KeyAction>();
    private List<KeyAction> perturbedActions = new List<KeyAction>();
    private List<bool> perturbedFlags = new List<bool>();
    private Coroutine replayCoroutine;

    // 제어값
    private float appliedSteering = 0f;
    private float appliedThrottle = 0f;

    void Start()
    {
        if (wheelController == null)
        {
            wheelController = GetComponent<WheelTest>();
            if (wheelController == null)
                wheelController = GetComponentInParent<WheelTest>();
            if (wheelController == null)
                wheelController = FindObjectOfType<WheelTest>();
        }

        if (wheelController == null)
            Debug.LogError("[ReplayController] WheelTest를 찾을 수 없습니다!");

        RefreshSessionList();
    }

    void Update()
    {
        // 세션 인덱스 변경 시 이름 업데이트
        if (sessionPaths.Count > 0)
        {
            sessionIndex = Mathf.Clamp(sessionIndex, 0, sessionPaths.Count - 1);
            selectedSessionName = Path.GetFileName(sessionPaths[sessionIndex]);
        }

        // 리플레이 토글
        if (Input.GetKeyDown(replayToggleKey))
        {
            if (!isReplaying)
                StartReplay();
            else
                StopReplay();
        }

        // 리플레이 중 제어 적용
        if (isReplaying && wheelController != null)
        {
            ApplyControl();
        }
    }

    /// <summary>
    /// TrainingDataV2/session_* 디렉토리 스캔 및 정렬
    /// </summary>
    public void RefreshSessionList()
    {
        sessionPaths.Clear();

        string basePath = Path.Combine(Application.dataPath, "..", "TrainingData");
        if (!Directory.Exists(basePath))
        {
            Debug.LogWarning("[ReplayController] TrainingData 폴더가 없습니다.");
            totalSessions = 0;
            return;
        }

        string[] dirs = Directory.GetDirectories(basePath, "session_*");
        // 이름순 정렬
        Array.Sort(dirs);

        // driving_log.csv가 있는 세션만 포함
        foreach (string dir in dirs)
        {
            string csvPath = Path.Combine(dir, "driving_log.csv");
            if (File.Exists(csvPath))
                sessionPaths.Add(dir);
        }

        totalSessions = sessionPaths.Count;

        if (totalSessions > 0)
        {
            sessionIndex = Mathf.Clamp(sessionIndex, 0, totalSessions - 1);
            selectedSessionName = Path.GetFileName(sessionPaths[sessionIndex]);
            Debug.Log($"[ReplayController] {totalSessions}개 세션 발견");
        }
        else
        {
            Debug.LogWarning("[ReplayController] 사용 가능한 세션이 없습니다.");
        }
    }

    /// <summary>
    /// CSV 파싱 (key_action 컬럼만 사용)
    /// </summary>
    bool LoadSession(string path)
    {
        originalActions.Clear();

        string csvPath = Path.Combine(path, "driving_log.csv");
        if (!File.Exists(csvPath))
        {
            Debug.LogError($"[ReplayController] CSV 파일을 찾을 수 없습니다: {csvPath}");
            return false;
        }

        try
        {
            string[] lines = File.ReadAllLines(csvPath);
            if (lines.Length < 2)
            {
                Debug.LogError("[ReplayController] CSV 파일이 비어있습니다.");
                return false;
            }

            // 헤더에서 key_action 컬럼 인덱스 찾기
            string[] headers = lines[0].Split(',');
            int actionIndex = -1;
            for (int i = 0; i < headers.Length; i++)
            {
                if (headers[i].Trim() == "key_action")
                {
                    actionIndex = i;
                    break;
                }
            }

            if (actionIndex < 0)
            {
                Debug.LogError("[ReplayController] CSV에 key_action 컬럼이 없습니다.");
                return false;
            }

            // 데이터 행 파싱
            for (int i = 1; i < lines.Length; i++)
            {
                string line = lines[i].Trim();
                if (string.IsNullOrEmpty(line)) continue;

                string[] cols = line.Split(',');
                if (cols.Length <= actionIndex)
                {
                    Debug.LogWarning($"[ReplayController] 라인 {i + 1} 파싱 실패, 스킵");
                    continue;
                }

                if (int.TryParse(cols[actionIndex], out int actionVal) && actionVal >= 0 && actionVal <= 6)
                {
                    originalActions.Add((KeyAction)actionVal);
                }
                else
                {
                    Debug.LogWarning($"[ReplayController] 라인 {i + 1}: 잘못된 key_action 값 '{cols[actionIndex]}', NONE으로 대체");
                    originalActions.Add(KeyAction.NONE);
                }
            }

            Debug.Log($"[ReplayController] 세션 로드 완료: {originalActions.Count} 프레임");
            return originalActions.Count > 0;
        }
        catch (Exception e)
        {
            Debug.LogError($"[ReplayController] CSV 파싱 실패: {e.Message}");
            return false;
        }
    }

    /// <summary>
    /// DART 알고리즘으로 액션 변형 적용
    /// - 각 프레임 perturbationProbability 확률로 변형
    /// - maxConsecutivePerturbed 프레임 이상 연속 변형 금지
    /// - 로드 시점에 한번 적용
    /// </summary>
    void ApplyPerturbation()
    {
        perturbedActions.Clear();
        perturbedFlags.Clear();
        perturbedCount = 0;

        int consecutivePerturbed = 0;

        for (int i = 0; i < originalActions.Count; i++)
        {
            KeyAction original = originalActions[i];
            KeyAction result = original;
            bool wasPerturbed = false;

            // 변형 가능한 액션이고, 확률 조건 충족 시
            if (consecutivePerturbed < maxConsecutivePerturbed &&
                UnityEngine.Random.value < perturbationProbability)
            {
                KeyAction neighbor = GetNeighborAction(original);
                if (neighbor != original)
                {
                    result = neighbor;
                    wasPerturbed = true;
                    consecutivePerturbed++;
                    perturbedCount++;
                }
                else
                {
                    consecutivePerturbed = 0;
                }
            }
            else
            {
                consecutivePerturbed = 0;
            }

            perturbedActions.Add(result);
            perturbedFlags.Add(wasPerturbed);
        }

        float perturbRatio = (float)perturbedCount / originalActions.Count * 100f;
        Debug.Log($"[ReplayController] 변형 적용: {perturbedCount}/{originalActions.Count} 프레임 ({perturbRatio:F1}%)");
    }

    /// <summary>
    /// 이웃 액션 반환 (DART perturbation rule)
    /// </summary>
    KeyAction GetNeighborAction(KeyAction action)
    {
        switch (action)
        {
            case KeyAction.FORWARD:
                // FORWARD → {FORWARD_LEFT, FORWARD_RIGHT} 중 랜덤
                return UnityEngine.Random.value < 0.5f ? KeyAction.FORWARD_LEFT : KeyAction.FORWARD_RIGHT;

            case KeyAction.FORWARD_LEFT:
                // FORWARD_LEFT → {FORWARD, LEFT} 중 랜덤
                return UnityEngine.Random.value < 0.5f ? KeyAction.FORWARD : KeyAction.LEFT;

            case KeyAction.FORWARD_RIGHT:
                // FORWARD_RIGHT → {FORWARD, RIGHT} 중 랜덤
                return UnityEngine.Random.value < 0.5f ? KeyAction.FORWARD : KeyAction.RIGHT;

            case KeyAction.LEFT:
                // LEFT → FORWARD_LEFT
                return KeyAction.FORWARD_LEFT;

            case KeyAction.RIGHT:
                // RIGHT → FORWARD_RIGHT
                return KeyAction.FORWARD_RIGHT;

            case KeyAction.BACKWARD:
                // BACKWARD → 변형 없음
                return KeyAction.BACKWARD;

            case KeyAction.NONE:
                // NONE → 변형 없음
                return KeyAction.NONE;

            default:
                return action;
        }
    }

    /// <summary>
    /// 리플레이 시작
    /// </summary>
    void StartReplay()
    {
        if (wheelController == null)
        {
            Debug.LogError("[ReplayController] WheelTest가 없어 리플레이를 시작할 수 없습니다.");
            return;
        }

        // 다른 컨트롤러가 이미 외부 제어 사용 중이면 거부
        if (wheelController.externalControlEnabled)
        {
            Debug.LogWarning("[ReplayController] 다른 컨트롤러가 외부 제어를 사용 중입니다. 먼저 비활성화하세요.");
            return;
        }

        if (sessionPaths.Count == 0)
        {
            RefreshSessionList();
            if (sessionPaths.Count == 0)
            {
                Debug.LogError("[ReplayController] 사용 가능한 세션이 없습니다.");
                return;
            }
        }

        sessionIndex = Mathf.Clamp(sessionIndex, 0, sessionPaths.Count - 1);
        string sessionPath = sessionPaths[sessionIndex];

        if (!LoadSession(sessionPath))
        {
            Debug.LogError("[ReplayController] 세션 로드 실패. 리플레이를 시작하지 않습니다.");
            return;
        }

        ApplyPerturbation();

        // 외부 제어 활성화
        wheelController.externalControlEnabled = true;
        isReplaying = true;
        currentFrameIndex = 0;
        totalFrames = perturbedActions.Count;

        Debug.Log($"[ReplayController] ▶ 리플레이 시작: {selectedSessionName} ({totalFrames} frames, {perturbedCount} perturbed)");

        replayCoroutine = StartCoroutine(ReplayCoroutine());
    }

    /// <summary>
    /// 리플레이 정지
    /// </summary>
    void StopReplay()
    {
        if (replayCoroutine != null)
        {
            StopCoroutine(replayCoroutine);
            replayCoroutine = null;
        }

        isReplaying = false;
        currentAction = KeyAction.NONE;
        currentFramePerturbed = false;

        // 제어 초기화
        if (wheelController != null)
        {
            wheelController.SetSteering(0f);
            wheelController.SetThrottle(0f);
            wheelController.externalControlEnabled = false;
        }

        appliedSteering = 0f;
        appliedThrottle = 0f;

        Debug.Log($"[ReplayController] ■ 리플레이 정지 (프레임 {currentFrameIndex}/{totalFrames})");
    }

    /// <summary>
    /// 프레임 순차 진행 코루틴
    /// </summary>
    IEnumerator ReplayCoroutine()
    {
        while (currentFrameIndex < perturbedActions.Count)
        {
            AdvanceFrame();
            yield return new WaitForSeconds(replayInterval);
        }

        // 마지막 프레임 도달 시 자동 정지
        Debug.Log("[ReplayController] 리플레이 완료 (마지막 프레임 도달)");
        StopReplay();
    }

    /// <summary>
    /// 현재 프레임의 액션 적용 및 다음 프레임으로 이동
    /// </summary>
    void AdvanceFrame()
    {
        if (currentFrameIndex >= perturbedActions.Count) return;

        currentAction = perturbedActions[currentFrameIndex];
        currentFramePerturbed = perturbedFlags[currentFrameIndex];

        ClassToControl(currentAction);

        currentFrameIndex++;
    }

    /// <summary>
    /// 액션 → steering/throttle 변환
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

    /// <summary>
    /// 차량 제어 적용
    /// </summary>
    void ApplyControl()
    {
        wheelController.SetSteering(appliedSteering);
        wheelController.SetThrottle(appliedThrottle);
    }

    void OnDestroy()
    {
        if (isReplaying)
            StopReplay();
    }

    void OnGUI()
    {
        // Y=350 위치에 리플레이 상태 표시
        GUI.Box(new Rect(10, 350, 300, 120), "");

        GUIStyle style = new GUIStyle(GUI.skin.label);
        style.fontSize = 14;
        style.fontStyle = FontStyle.Bold;

        if (isReplaying)
        {
            style.normal.textColor = Color.magenta;
            string perturbMarker = currentFramePerturbed ? " [P]" : "";
            GUI.Label(new Rect(20, 355, 280, 25), $"▶ REPLAY{perturbMarker}", style);

            style.fontStyle = FontStyle.Normal;
            style.normal.textColor = Color.white;
            GUI.Label(new Rect(20, 375, 280, 20), $"Session: {selectedSessionName}", style);
            GUI.Label(new Rect(20, 395, 280, 20), $"Frame: {currentFrameIndex}/{totalFrames}", style);

            style.normal.textColor = currentFramePerturbed ? Color.yellow : Color.cyan;
            GUI.Label(new Rect(20, 415, 280, 20), $"Action: {ActionNames[(int)currentAction]}", style);

            style.normal.textColor = Color.white;
            float speed = wheelController != null ? wheelController.GetSpeedMS() : 0f;
            GUI.Label(new Rect(20, 435, 280, 20), $"Speed: {speed:F2} m/s | Perturbed: {perturbedCount}", style);
        }
        else
        {
            style.normal.textColor = Color.white;
            GUI.Label(new Rect(20, 355, 280, 20), $"[{replayToggleKey}] 리플레이 시작", style);

            style.fontStyle = FontStyle.Normal;
            if (totalSessions > 0)
            {
                style.normal.textColor = Color.gray;
                GUI.Label(new Rect(20, 375, 280, 20), $"Session [{sessionIndex}]: {selectedSessionName}", style);
                GUI.Label(new Rect(20, 395, 280, 20), $"총 {totalSessions}개 세션 | 변형확률: {perturbationProbability * 100:F0}%", style);
            }
            else
            {
                style.normal.textColor = Color.red;
                GUI.Label(new Rect(20, 375, 280, 20), "세션 없음 (TrainingDataV2 확인)", style);
            }
        }
    }
}
