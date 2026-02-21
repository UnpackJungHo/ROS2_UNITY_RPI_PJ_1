using System;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// 강화학습 에피소드 종료/평가기.
/// - 성공: FinishLineGate 통과
/// - 실패: 위험단계(5,6) 정지 지속, 충돌, 타임아웃
/// - 종료 시 점수 산출 + 쓰레기 학습 여부 라벨링 + CSV 로깅
/// </summary>
public class RLEpisodeEvaluator : MonoBehaviour
{
    /// <summary>에피소드 종료 유형을 나타내는 열거형.</summary>
    public enum TerminalType
    {
        None = 0,            // 종료되지 않음 (에피소드 진행 중)
        Finish = 1,          // 성공: 결승선(FinishLineGate) 통과
        FailRiskStop = 2,    // 실패: 위험 단계에서 저속 정지가 유지됨 (장애물 앞 교착)
        FailCollision = 3,   // 실패: 다른 오브젝트와 물리적 충돌 발생
        FailTimeout = 4,     // 실패: 제한 시간 초과
        FailStuck = 5        // 실패: 일정 시간 동안 거의 이동하지 못함 (경로 이탈/멈춤)
    }

    [Header("References")]
    /// <summary>경로 진행률 기반 보상을 계산하는 프로바이더 (path progress reward shaping).</summary>
    public ProgressRewardProvider progressRewardProvider;
    /// <summary>전방 장애물과의 TTC(Time-To-Collision) 기반 경고 레벨을 발행하는 퍼블리셔.</summary>
    public CollisionWarningPublisher collisionWarningPublisher;
    /// <summary>Ackermann 조향/스로틀/브레이크를 제어하는 차량 컨트롤러.</summary>
    public WheelTest wheelController;
    /// <summary>신호등 주행 결정을 제공하는 엔진. 종료 시점의 traffic 상태 로그에 사용.</summary>
    public TrafficLightDecisionEngine trafficLightDecisionEngine;
    /// <summary>회귀 모델(신경망) 기반 자율주행 컨트롤러. 에피소드 종료 시 자율 모드를 끌 때 사용.</summary>
    public RegressionDrivingController regressionDrivingController;
    /// <summary>Residual RL Agent. 종료 시점의 base/delta/final 제어값 진단 로그에 사용.</summary>
    public AutoDriverRLAgent autoDriverRLAgent;

    [Header("Episode Control")]
    /// <summary>Start() 시 씬 내 참조 컴포넌트를 자동으로 탐색할지 여부.</summary>
    public bool autoFindReferences = true;
    /// <summary>Start() 시 에피소드를 즉시 시작할지 여부.</summary>
    public bool autoBeginOnStart = true;
    /// <summary>에피소드 종료 시 차량을 강제 정지(브레이크 100%)시킬지 여부.</summary>
    public bool stopVehicleOnTerminal = true;
    /// <summary>에피소드 종료 시 자율주행 모드를 비활성화할지 여부.</summary>
    public bool disableAutonomousOnTerminal = true;
    /// <summary>에피소드 최대 허용 시간(초). 0 이하이면 타임아웃 비활성.</summary>
    [Tooltip("0 이하이면 타임아웃 비활성")]
    public float maxEpisodeSeconds = 120f;

    [Header("Failure Rules")]
    /// <summary>이 경고 레벨(예: Brake=5) 이상에서 저속 정지가 유지되면 FailRiskStop으로 판정.</summary>
    [Tooltip("이 레벨 이상에서 저속 정지가 유지되면 실패")]
    public CollisionWarningPublisher.WarningLevel dangerLevelThreshold = CollisionWarningPublisher.WarningLevel.Brake;
    /// <summary>위험 레벨 + 정지 상태가 이 시간(초) 이상 지속되면 FailRiskStop 종료.</summary>
    [Tooltip("위험레벨 + 정지 상태가 이 시간 지속되면 FailRiskStop")]
    public float dangerStopHoldSeconds = 0.7f;
    /// <summary>차량이 "정지" 상태로 간주되는 속도 임계값(m/s). 이 이하이면 정지로 판정.</summary>
    public float stoppedSpeedThreshold = 0.05f;
    /// <summary>경로 대비 횡방향 오차(lateral error) 허용 최대값(m). 0이면 비활성, 초과 시 실패.</summary>
    [Tooltip("0보다 크면 횡오차로 실패 처리(옵션)")]
    public float maxLateralErrorForFailure = 0f;
    /// <summary>충돌 실패를 판정할 레이어 마스크. 해당 레이어의 오브젝트와 충돌해야 실패 처리.</summary>
    public LayerMask collisionFailLayers = ~0;
    /// <summary>충돌 시 상대 속도(m/s)가 이 값 미만이면 경미한 접촉으로 무시.</summary>
    [Tooltip("상대 충돌속도가 이 값 이상일 때만 충돌 실패로 인정")]
    public float collisionMinRelativeSpeed = 0.1f;

    [Header("Stuck Detection")]
    /// <summary>에피소드 시작 직후 모델 로드/초기 추론 대기를 위한 유예 시간(초). 이 기간에는 stuck 감지 비활성.</summary>
    [Tooltip("에피소드 시작 후 이 시간(초)이 지나야 stuck 감지 시작")]
    public float stuckGracePeriod = 5f;
    /// <summary>슬라이딩 윈도우 길이(초). 이 시간 동안 이동거리를 누적하여 stuck 여부 판정.</summary>
    [Tooltip("이 시간(초) 동안 이동거리가 stuckDistanceThreshold 미만이면 FailStuck")]
    public float stuckTimeWindow = 10f;
    /// <summary>stuckTimeWindow 내 최소 이동거리(m). 미달 시 차량이 멈춘 것(stuck)으로 판정.</summary>
    [Tooltip("stuckTimeWindow 동안 최소 이동거리 (m)")]
    public float stuckDistanceThreshold = 1f;

    [Header("Scoring")]
    /// <summary>성공(Finish) 시 보상에 가산되는 보너스 점수.</summary>
    public float finishBonus = 20f;
    /// <summary>실패 시 공통으로 차감되는 기본 페널티.</summary>
    public float baseFailurePenalty = 10f;
    /// <summary>FailRiskStop(위험 정지) 실패 시 추가 차감 페널티.</summary>
    public float riskStopExtraPenalty = 6f;
    /// <summary>FailCollision(물리 충돌) 실패 시 추가 차감 페널티.</summary>
    public float collisionExtraPenalty = 10f;
    /// <summary>FailTimeout(시간 초과) 실패 시 추가 차감 페널티.</summary>
    public float timeoutExtraPenalty = 4f;
    /// <summary>FailStuck(이동 불능) 실패 시 추가 차감 페널티.</summary>
    public float stuckExtraPenalty = 8f;
    /// <summary>실패 에피소드에서 경로 진행률이 이 비율 미만이면 "쓰레기 학습(trash)"으로 라벨링.</summary>
    [Tooltip("실패 시 progress ratio가 이 값 미만이면 쓰레기 학습으로 라벨")]
    [Range(0f, 1f)] public float lowProgressTrashThreshold = 0.1f;

    [Header("Episode Report CSV")]
    /// <summary>에피소드 종료 시 결과를 CSV 파일에 자동 저장할지 여부.</summary>
    public bool saveEpisodeReportCsv = true;
    /// <summary>CSV 파일명. Assets/Resources/RL/ 하위에 생성됨.</summary>
    public string csvFileName = "rl_episode_report.csv";
    [Tooltip("true면 실행 세션마다 csvFileName 뒤에 타임스탬프를 붙여 파일을 분리 저장")]
    public bool appendSessionTimestampToCsv = true;
    [Tooltip("선택: 실행 세션 라벨(파일명 suffix). run-id를 수동으로 넣고 싶을 때 사용")]
    public string csvSessionLabel = "";

    [Header("Debug (Read Only)")]
    /// <summary>현재까지 실행된 에피소드 누적 인덱스 (1부터 시작).</summary>
    [SerializeField] private int episodeIndex = 0;
    /// <summary>현재 에피소드가 진행 중인지 여부.</summary>
    [SerializeField] private bool episodeActive = false;
    /// <summary>종료 조건(성공 또는 실패)에 도달했는지 여부.</summary>
    [SerializeField] private bool terminalReached = false;
    /// <summary>현재 에피소드가 성공(Finish)으로 종료되었는지 여부.</summary>
    [SerializeField] private bool episodeSuccess = false;
    /// <summary>현재 에피소드가 "쓰레기 학습"으로 라벨링되었는지 여부. 학습 데이터 필터링에 사용.</summary>
    [SerializeField] private bool isTrashEpisode = false;
    /// <summary>에피소드가 어떤 유형으로 종료되었는지 (Finish, FailRiskStop 등).</summary>
    [SerializeField] private TerminalType terminalType = TerminalType.None;
    /// <summary>종료 사유를 사람이 읽을 수 있는 문자열로 기록 (로그/CSV용).</summary>
    [SerializeField] private string terminalReason = "None";
    /// <summary>쓰레기 학습 판정 사유 문자열 (해당 없으면 "None").</summary>
    [SerializeField] private string trashReason = "None";
    /// <summary>에피소드 시작 후 경과 시간(초).</summary>
    [SerializeField] private float elapsedSeconds = 0f;
    /// <summary>최종 에피소드 점수 = rewardBase ± 보너스/페널티. 정책 평가 지표.</summary>
    [SerializeField] private float episodeScore = 0f;
    /// <summary>ProgressRewardProvider가 산출한 누적 보상 (보너스/페널티 적용 전 원시 보상).</summary>
    [SerializeField] private float episodeRewardBase = 0f;
    /// <summary>위험 레벨 + 정지 상태가 연속으로 유지된 시간(초). 임계값 초과 시 FailRiskStop.</summary>
    [SerializeField] private float dangerStoppedDuration = 0f;
    /// <summary>에피소드 내 위험 레벨(dangerLevelThreshold 이상)에 머문 총 누적 시간(초).</summary>
    [SerializeField] private float timeAtDangerLevel = 0f;
    /// <summary>에피소드 내 발생한 물리 충돌 횟수.</summary>
    [SerializeField] private int collisionCount = 0;
    /// <summary>마지막으로 충돌한 게임오브젝트 이름 (디버그용).</summary>
    [SerializeField] private string lastCollisionObjectName = "None";
    /// <summary>마지막 충돌의 상대 속도(m/s). 충돌 심각도 판단 지표.</summary>
    [SerializeField] private float lastCollisionRelativeSpeed = 0f;
    /// <summary>에피소드 중 관측된 최고 경고 레벨 (Safe → ... → Brake 순).</summary>
    [SerializeField] private CollisionWarningPublisher.WarningLevel maxWarningLevel =
        CollisionWarningPublisher.WarningLevel.Safe;
    /// <summary>에피소드 중 관측된 최소 TTC(Time-To-Collision, 초). 장애물 근접도 지표.</summary>
    [SerializeField] private float minObservedTtc = float.PositiveInfinity;
    /// <summary>stuck 감지용 슬라이딩 윈도우 경과 타이머(초).</summary>
    [SerializeField] private float stuckTimer = 0f;
    /// <summary>stuck 감지 윈도우 내 누적 이동거리(m).</summary>
    [SerializeField] private float stuckDistanceAccum = 0f;
    [SerializeField] private string resolvedCsvFileName = "rl_episode_report.csv";
    /// <summary>stuck 감지를 위해 이전 프레임의 차량 월드 위치를 저장.</summary>
    private Vector3 stuckLastPosition;

    /// <summary>에피소드 종료 시 외부 구독자에게 알리는 이벤트. 리플레이 버퍼 저장, 모델 업데이트 등에 활용.</summary>
    public event Action<RLEpisodeEvaluator> OnEpisodeTerminated;

    void Start()
    {
        if (autoFindReferences)
            AutoFindReferences();

        ResolveCsvFileName();

        if (autoBeginOnStart)
        {
            // When an ML-Agent exists, its OnEpisodeBegin should be the single owner
            // of episode lifecycle to avoid startup double-begin.
            AutoDriverRLAgent managedAgent = FindObjectOfType<AutoDriverRLAgent>();
            if (managedAgent != null)
                return;

            BeginEpisode();
        }
    }

    void Update()
    {
        if (!episodeActive || terminalReached)
            return;

        float dt = Time.deltaTime;
        elapsedSeconds += dt;
        UpdateRiskMetrics(dt);

        if (maxEpisodeSeconds > 0f && elapsedSeconds >= maxEpisodeSeconds)
        {
            SetTerminal(
                TerminalType.FailTimeout,
                $"Timeout({elapsedSeconds:F1}s >= {maxEpisodeSeconds:F1}s)"
            );
            return;
        }

        if (maxLateralErrorForFailure > 0f && progressRewardProvider != null)
        {
            float lateral = progressRewardProvider.GetCurrentLateralError();
            if (lateral > maxLateralErrorForFailure)
            {
                SetTerminal(
                    TerminalType.FailRiskStop,
                    $"OffPath(lateral={lateral:F2}m > {maxLateralErrorForFailure:F2}m)"
                );
                return;
            }
        }

        UpdateStuckDetection(dt);
    }

    void AutoFindReferences()
    {
        if (progressRewardProvider == null)
            progressRewardProvider = FindObjectOfType<ProgressRewardProvider>();
        if (collisionWarningPublisher == null)
            collisionWarningPublisher = FindObjectOfType<CollisionWarningPublisher>();
        if (wheelController == null)
            wheelController = FindObjectOfType<WheelTest>();
        if (trafficLightDecisionEngine == null)
            trafficLightDecisionEngine = FindObjectOfType<TrafficLightDecisionEngine>();
        if (regressionDrivingController == null)
            regressionDrivingController = FindObjectOfType<RegressionDrivingController>();
        if (autoDriverRLAgent == null)
            autoDriverRLAgent = FindObjectOfType<AutoDriverRLAgent>();
    }

    /// <summary>
    /// 새 에피소드를 시작한다. 모든 런타임 상태(점수, 타이머, 충돌 카운트 등)를
    /// 초기화하고 ProgressRewardProvider의 보상 상태도 리셋한다.
    /// </summary>
    public void BeginEpisode()
    {
        episodeIndex++;
        episodeActive = true;
        terminalReached = false;
        episodeSuccess = false;
        isTrashEpisode = false;
        terminalType = TerminalType.None;
        terminalReason = "None";
        trashReason = "None";
        elapsedSeconds = 0f;
        episodeScore = 0f;
        episodeRewardBase = 0f;
        dangerStoppedDuration = 0f;
        timeAtDangerLevel = 0f;
        collisionCount = 0;
        lastCollisionObjectName = "None";
        lastCollisionRelativeSpeed = 0f;
        maxWarningLevel = CollisionWarningPublisher.WarningLevel.Safe;
        minObservedTtc = float.PositiveInfinity;
        stuckTimer = 0f;
        stuckDistanceAccum = 0f;
        stuckLastPosition = wheelController != null ? wheelController.transform.position : transform.position;

        if (progressRewardProvider != null)
            progressRewardProvider.ResetRewardState();
    }

    /// <summary>
    /// 매 프레임 호출되어 위험 지표를 갱신한다.
    /// TTC 최솟값, 위험 레벨 체류 시간, 위험 정지 지속 시간을 추적하며,
    /// 위험 정지가 dangerStopHoldSeconds를 초과하면 FailRiskStop으로 에피소드를 종료한다.
    /// </summary>
    void UpdateRiskMetrics(float dt)
    {
        if (collisionWarningPublisher == null)
            return;

        CollisionWarningPublisher.WarningLevel level = collisionWarningPublisher.GetWarningLevel();
        if (level > maxWarningLevel)
            maxWarningLevel = level;

        if (level >= dangerLevelThreshold)
        {
            timeAtDangerLevel += dt;
        }

        float ttc = collisionWarningPublisher.GetTimeToCollision();
        if (!float.IsInfinity(ttc))
            minObservedTtc = Mathf.Min(minObservedTtc, ttc);

        float speed = wheelController != null ? Mathf.Abs(wheelController.GetSpeedMS()) : 0f;
        bool isDangerStop = level >= dangerLevelThreshold && speed <= stoppedSpeedThreshold;

        if (isDangerStop)
            dangerStoppedDuration += dt;
        else
            dangerStoppedDuration = 0f;

        if (dangerStoppedDuration >= dangerStopHoldSeconds)
        {
            SetTerminal(
                TerminalType.FailRiskStop,
                $"DangerStop(level={(int)level}, hold={dangerStoppedDuration:F2}s, speed={speed:F2}m/s)"
            );
        }
    }

    /// <summary>
    /// 차량의 이동 불능(stuck) 상태를 감지한다.
    /// 슬라이딩 윈도우(stuckTimeWindow) 동안 누적 이동거리가 stuckDistanceThreshold 미만이면
    /// FailStuck으로 에피소드를 종료한다. 에피소드 초기 grace period 동안은 감지를 건너뛴다.
    /// </summary>
    void UpdateStuckDetection(float dt)
    {
        if (stuckTimeWindow <= 0f)
            return;

        // 에피소드 초기에는 모델 로드/추론 시작까지 시간이 필요하므로 grace period 적용
        if (elapsedSeconds < stuckGracePeriod)
            return;

        Vector3 currentPos = wheelController != null ? wheelController.transform.position : transform.position;
        stuckDistanceAccum += Vector3.Distance(currentPos, stuckLastPosition);
        stuckLastPosition = currentPos;
        stuckTimer += dt;

        if (stuckTimer >= stuckTimeWindow)
        {
            if (stuckDistanceAccum < stuckDistanceThreshold)
            {
                SetTerminal(
                    TerminalType.FailStuck,
                    $"Stuck(moved={stuckDistanceAccum:F2}m in {stuckTimer:F1}s < {stuckDistanceThreshold:F1}m)"
                );
                return;
            }

            // 윈도우 리셋
            stuckTimer = 0f;
            stuckDistanceAccum = 0f;
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (!episodeActive || terminalReached || collision == null)
            return;

        int otherLayerMask = 1 << collision.collider.gameObject.layer;
        if ((collisionFailLayers.value & otherLayerMask) == 0)
            return;

        float relativeSpeed = collision.relativeVelocity.magnitude;
        if (relativeSpeed < collisionMinRelativeSpeed)
            return;

        collisionCount++;
        lastCollisionObjectName = collision.collider != null ? collision.collider.name : "Unknown";
        lastCollisionRelativeSpeed = relativeSpeed;
        SetTerminal(
            TerminalType.FailCollision,
            $"Collision(obj={collision.collider.name}, relV={relativeSpeed:F2}m/s)"
        );
    }

    /// <summary>
    /// FinishLineGate 트리거에서 호출되어 결승선 통과(성공)를 알린다.
    /// enterSigned/exitSigned는 게이트 평면 기준 진입·이탈 부호 거리로, 정방향 통과를 검증한다.
    /// </summary>
    public void NotifyFinishCrossed(string gateName, float enterSigned, float exitSigned)
    {
        if (!episodeActive || terminalReached)
            return;

        SetTerminal(
            TerminalType.Finish,
            $"Finish(gate={gateName}, enter={enterSigned:F3}, exit={exitSigned:F3})"
        );
    }

    /// <summary>
    /// 에피소드를 종료 처리하는 핵심 메서드.
    /// 1) 종료 유형과 사유 기록  2) 누적 보상 + 보너스/페널티로 최종 점수 산출
    /// 3) 쓰레기 학습 여부 라벨링  4) 차량 정지 및 자율주행 비활성화
    /// 5) CSV 로깅  6) OnEpisodeTerminated 이벤트 발행
    /// </summary>
    void SetTerminal(TerminalType type, string reason)
    {
        if (terminalReached)
            return;

        terminalReached = true;
        episodeActive = false;
        terminalType = type;
        episodeSuccess = type == TerminalType.Finish;
        terminalReason = string.IsNullOrEmpty(reason) ? type.ToString() : reason;

        episodeRewardBase = progressRewardProvider != null ? progressRewardProvider.GetCumulativeReward() : 0f;
        episodeScore = episodeRewardBase;

        if (episodeSuccess)
        {
            episodeScore += finishBonus;
            isTrashEpisode = false;
            trashReason = "None";
        }
        else
        {
            episodeScore -= baseFailurePenalty;
            switch (type)
            {
                case TerminalType.FailRiskStop:
                    episodeScore -= riskStopExtraPenalty;
                    break;
                case TerminalType.FailCollision:
                    episodeScore -= collisionExtraPenalty;
                    break;
                case TerminalType.FailTimeout:
                    episodeScore -= timeoutExtraPenalty;
                    break;
                case TerminalType.FailStuck:
                    episodeScore -= stuckExtraPenalty;
                    break;
            }

            float progressRatio = progressRewardProvider != null ? progressRewardProvider.GetPathProgressRatio() : 0f;
            bool lowProgress = progressRatio < lowProgressTrashThreshold;
            bool highRiskStop = type == TerminalType.FailRiskStop;
            bool collisionFail = type == TerminalType.FailCollision;
            bool stuckFail = type == TerminalType.FailStuck;

            isTrashEpisode = highRiskStop || collisionFail || stuckFail || lowProgress;
            if (collisionFail)
                trashReason = "Trash: Collision termination";
            else if (highRiskStop)
                trashReason = "Trash: Danger level(5/6) stop termination";
            else if (stuckFail)
                trashReason = $"Trash: Stuck ({stuckDistanceAccum:F2}m in {stuckTimeWindow:F1}s)";
            else if (lowProgress)
                trashReason = $"Trash: Low progress ratio ({progressRatio:F2} < {lowProgressTrashThreshold:F2})";
            else
                trashReason = "None";
        }

        string terminalDiag = BuildTerminalDiagnosticLine();

        if (stopVehicleOnTerminal && wheelController != null)
        {
            wheelController.SetSteering(0f);
            wheelController.SetThrottle(0f);
            wheelController.SetBrake(1f);
        }

        if (disableAutonomousOnTerminal && regressionDrivingController != null)
        {
            regressionDrivingController.isAutonomousMode = false;
            if (wheelController != null)
                wheelController.externalControlEnabled = false;
        }

        if (saveEpisodeReportCsv)
            AppendEpisodeCsv();

        Debug.Log(
            $"[RLEpisode] End #{episodeIndex} | type={terminalType} | success={episodeSuccess} | " +
            $"score={episodeScore:F3} | rewardBase={episodeRewardBase:F3} | trash={isTrashEpisode} | " +
            $"collisionObj={lastCollisionObjectName} | collisionRelV={lastCollisionRelativeSpeed:F2}m/s | reason={terminalReason}"
        );
        Debug.Log($"[RLEpisode][TerminalDiag] {terminalDiag}");

        OnEpisodeTerminated?.Invoke(this);
    }

    /// <summary>
    /// 에피소드 결과를 CSV 파일에 한 행으로 추가한다.
    /// 진행 보상, 구역 보상, 안전 페널티, 교통 페널티 등 세분화된 보상 항목과
    /// TTC, 경고 레벨, 경로 진행률 등 주행 품질 지표를 함께 기록한다.
    /// </summary>
    void AppendEpisodeCsv()
    {
        try
        {
            string dir = Path.Combine(Application.dataPath, "Resources", "RL");
            if (!Directory.Exists(dir))
                Directory.CreateDirectory(dir);
            if (string.IsNullOrEmpty(resolvedCsvFileName))
                ResolveCsvFileName();

            string path = Path.Combine(dir, resolvedCsvFileName);
            bool writeHeader = !File.Exists(path);
            using (var sw = new StreamWriter(path, true))
            {
                if (writeHeader)
                {
                    sw.WriteLine(
                        "utc_time,episode_index,terminal_type,success,trash,trash_reason,score,reward_base," +
                        "reward_progress,reward_zone,penalty_safety,penalty_traffic,elapsed_sec,progress_ratio," +
                        "path_s,total_path,lateral_error,max_warning_level,time_at_danger,danger_stop_hold,collision_count,min_ttc,terminal_reason"
                    );
                }

                float rewardProgress = progressRewardProvider != null ? progressRewardProvider.GetCumulativeProgressReward() : 0f;
                float rewardZone = progressRewardProvider != null ? progressRewardProvider.GetCumulativeZoneReward() : 0f;
                float penaltySafety = progressRewardProvider != null ? progressRewardProvider.GetCumulativeSafetyPenalty() : 0f;
                float penaltyTraffic = progressRewardProvider != null ? progressRewardProvider.GetCumulativeTrafficPenalty() : 0f;
                float progressRatio = progressRewardProvider != null ? progressRewardProvider.GetPathProgressRatio() : 0f;
                float pathS = progressRewardProvider != null ? progressRewardProvider.GetCurrentPathS() : 0f;
                float totalPath = progressRewardProvider != null ? progressRewardProvider.GetTotalPathLength() : 0f;
                float lateral = progressRewardProvider != null ? progressRewardProvider.GetCurrentLateralError() : 0f;
                string minTtcStr = float.IsInfinity(minObservedTtc) ? "inf" : F(minObservedTtc);

                sw.WriteLine(
                    $"{DateTime.UtcNow:O},{episodeIndex},{terminalType},{episodeSuccess},{isTrashEpisode}," +
                    $"{Esc(trashReason)},{F(episodeScore)},{F(episodeRewardBase)}," +
                    $"{F(rewardProgress)},{F(rewardZone)},{F(penaltySafety)},{F(penaltyTraffic)}," +
                    $"{F(elapsedSeconds)},{F(progressRatio)},{F(pathS)},{F(totalPath)},{F(lateral)}," +
                    $"{(int)maxWarningLevel},{F(timeAtDangerLevel)},{F(dangerStoppedDuration)},{collisionCount},{minTtcStr},{Esc(terminalReason)}"
                );
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[RLEpisode] CSV 저장 실패: {e.Message}");
        }
    }

    string BuildTerminalDiagnosticLine()
    {
        float speed = wheelController != null ? wheelController.GetSpeedMS() : 0f;
        float wheelSteer = wheelController != null ? wheelController.GetSteeringInput() : 0f;
        float wheelThrottle = wheelController != null ? wheelController.GetThrottleInput() : 0f;
        float wheelBrake = wheelController != null ? wheelController.GetBrakeInput() : 0f;

        CollisionWarningPublisher.WarningLevel warningLevel = collisionWarningPublisher != null
            ? collisionWarningPublisher.GetWarningLevel()
            : CollisionWarningPublisher.WarningLevel.Safe;
        float ttc = collisionWarningPublisher != null
            ? collisionWarningPublisher.GetTimeToCollision()
            : float.PositiveInfinity;
        float obstacleDist = collisionWarningPublisher != null
            ? collisionWarningPublisher.GetDistanceToObstacle()
            : float.PositiveInfinity;
        float pathObstacleDist = collisionWarningPublisher != null
            ? collisionWarningPublisher.GetPathDistanceToObstacle()
            : float.PositiveInfinity;
        float sideObstacleDist = collisionWarningPublisher != null
            ? collisionWarningPublisher.GetSideDistanceToObstacle()
            : float.PositiveInfinity;
        (string source, string sensor, float distance) closestObstacleInfo = collisionWarningPublisher != null
            ? collisionWarningPublisher.GetClosestObstacleInfo()
            : ("None", "None", float.PositiveInfinity);
        float closestUltraConfidence = collisionWarningPublisher != null
            ? collisionWarningPublisher.CurrentSensorData.ultrasonicClosestConfidence
            : 0f;
        string ttcText = float.IsInfinity(ttc)
            ? "inf"
            : (float.IsNaN(ttc) ? "nan" : ttc.ToString("F3", CultureInfo.InvariantCulture));
        string obstacleDistText = float.IsInfinity(obstacleDist)
            ? "inf"
            : obstacleDist.ToString("F3", CultureInfo.InvariantCulture);
        string pathObstacleDistText = float.IsInfinity(pathObstacleDist)
            ? "inf"
            : pathObstacleDist.ToString("F3", CultureInfo.InvariantCulture);
        string sideObstacleDistText = float.IsInfinity(sideObstacleDist)
            ? "inf"
            : sideObstacleDist.ToString("F3", CultureInfo.InvariantCulture);

        TrafficLightDecisionEngine.TrafficDecision trafficDecision = trafficLightDecisionEngine != null
            ? trafficLightDecisionEngine.GetDecision()
            : TrafficLightDecisionEngine.TrafficDecision.Go;
        bool shouldStop = trafficLightDecisionEngine != null && trafficLightDecisionEngine.ShouldStop();

        float progressRatio = progressRewardProvider != null ? progressRewardProvider.GetPathProgressRatio() : 0f;
        float pathS = progressRewardProvider != null ? progressRewardProvider.GetCurrentPathS() : 0f;
        float lateral = progressRewardProvider != null ? progressRewardProvider.GetCurrentLateralError() : 0f;
        float zoneScore = progressRewardProvider != null ? progressRewardProvider.GetCurrentZoneScore() : 0f;
        float zoneScale = progressRewardProvider != null ? progressRewardProvider.GetLastZoneProgressScale() : 1f;
        float rewardStep = progressRewardProvider != null ? progressRewardProvider.GetLastStepReward() : 0f;
        float rawDeltaS = progressRewardProvider != null ? progressRewardProvider.GetLastRawDeltaS() : 0f;
        float usedDeltaS = progressRewardProvider != null ? progressRewardProvider.GetLastUsedDeltaS() : 0f;
        bool deltaSClamped = progressRewardProvider != null && progressRewardProvider.WasLastDeltaSClamped();

        if (autoDriverRLAgent == null)
            autoDriverRLAgent = FindObjectOfType<AutoDriverRLAgent>();

        string rlDebug = autoDriverRLAgent != null
            ? autoDriverRLAgent.GetActionDebugSummary()
            : "base=(0.000,0.000) delta=(0.000,0.000) final=(0.000,0.000,0.000) stepReward=0.000";

        string regressionDebug = regressionDrivingController != null
            ? $"regBase=({regressionDrivingController.GetPredictedSteering():F3},{regressionDrivingController.GetPredictedThrottle():F3}) " +
              $"regApplied=({regressionDrivingController.GetAppliedSteering():F3},{regressionDrivingController.GetAppliedThrottle():F3}) " +
              $"regAuto={regressionDrivingController.isAutonomousMode}"
            : "regBase=(0.000,0.000) regApplied=(0.000,0.000) regAuto=False";

        return
            $"ep={episodeIndex} type={terminalType} reason={terminalReason} " +
            $"speed={speed:F3}m/s movedWin={stuckDistanceAccum:F3}/{stuckDistanceThreshold:F3}m " +
            $"warning={(int)warningLevel}({warningLevel}) ttc={ttcText}s " +
            $"obs={obstacleDistText}(path:{pathObstacleDistText},side:{sideObstacleDistText}) " +
            $"closest={closestObstacleInfo.source}-{closestObstacleInfo.sensor}@{(float.IsInfinity(closestObstacleInfo.distance) ? "inf" : closestObstacleInfo.distance.ToString("F3", CultureInfo.InvariantCulture))} " +
            $"ultraConf={closestUltraConfidence:F2} traffic={trafficDecision} stop={shouldStop} " +
            $"wheel=({wheelSteer:F3},{wheelThrottle:F3},{wheelBrake:F3}) " +
            $"progress=(s:{pathS:F3},ratio:{progressRatio:F3},lat:{lateral:F3}) " +
            $"zone=(score:{zoneScore:F3},scale:{zoneScale:F3}) " +
            $"dS(raw:{rawDeltaS:F3},used:{usedDeltaS:F3},clamped:{deltaSClamped}) stepReward={rewardStep:F3} " +
            $"{regressionDebug} rl[{rlDebug}]";
    }

    void ResolveCsvFileName()
    {
        string configured = string.IsNullOrWhiteSpace(csvFileName) ? "rl_episode_report.csv" : csvFileName.Trim();
        string ext = Path.GetExtension(configured);
        if (string.IsNullOrEmpty(ext))
            ext = ".csv";

        string baseName = Path.GetFileNameWithoutExtension(configured);
        if (string.IsNullOrWhiteSpace(baseName))
            baseName = "rl_episode_report";

        string suffix = "";
        if (appendSessionTimestampToCsv)
            suffix += "_" + DateTime.Now.ToString("yyyyMMdd_HHmmss");

        if (!string.IsNullOrWhiteSpace(csvSessionLabel))
            suffix += "_" + SanitizeFileToken(csvSessionLabel);

        resolvedCsvFileName = baseName + suffix + ext;

        if (saveEpisodeReportCsv)
        {
            string dir = Path.Combine(Application.dataPath, "Resources", "RL");
            string path = Path.Combine(dir, resolvedCsvFileName);
            Debug.Log($"[RLEpisode] CSV output: {path}");
        }
    }

    string SanitizeFileToken(string input)
    {
        if (string.IsNullOrWhiteSpace(input))
            return "session";

        StringBuilder sb = new StringBuilder(input.Length);
        for (int i = 0; i < input.Length; i++)
        {
            char c = input[i];
            if (char.IsLetterOrDigit(c) || c == '_' || c == '-')
                sb.Append(c);
        }

        return sb.Length > 0 ? sb.ToString() : "session";
    }

    string F(float value) => value.ToString("F6", CultureInfo.InvariantCulture);

    string Esc(string text)
    {
        if (string.IsNullOrEmpty(text))
            return "\"\"";
        return "\"" + text.Replace("\"", "\"\"") + "\"";
    }

    public bool IsEpisodeActive() => episodeActive;
    public bool IsTerminalReached() => terminalReached;
    public bool IsEpisodeSuccess() => episodeSuccess;
    public bool IsTrash() => isTrashEpisode;
    public TerminalType GetTerminalType() => terminalType;
    public string GetTerminalReason() => terminalReason;
    public string GetTrashReason() => trashReason;
    public float GetEpisodeScore() => episodeScore;
    public float GetEpisodeRewardBase() => episodeRewardBase;
    public float GetElapsedSeconds() => elapsedSeconds;
    public int GetEpisodeIndex() => episodeIndex;
    public int GetCollisionCount() => collisionCount;
    public float GetTimeAtDangerLevel() => timeAtDangerLevel;
    public CollisionWarningPublisher.WarningLevel GetMaxWarningLevel() => maxWarningLevel;
    public float GetMinObservedTtc() => minObservedTtc;
}
