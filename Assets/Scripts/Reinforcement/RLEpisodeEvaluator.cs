using System;
using System.Globalization;
using System.IO;
using UnityEngine;

/// <summary>
/// 강화학습 에피소드 종료/평가기.
/// - 성공: FinishLineGate 통과
/// - 실패: 위험단계(5,6) 정지 지속, 충돌, 타임아웃
/// - 종료 시 점수 산출 + 쓰레기 학습 여부 라벨링 + CSV 로깅
/// </summary>
public class RLEpisodeEvaluator : MonoBehaviour
{
    public enum TerminalType
    {
        None = 0,
        Finish = 1,
        FailRiskStop = 2,
        FailCollision = 3,
        FailTimeout = 4
    }

    [Header("References")]
    public ProgressRewardProvider progressRewardProvider;
    public CollisionWarningPublisher collisionWarningPublisher;
    public WheelTest wheelController;
    public RegressionDrivingController regressionDrivingController;

    [Header("Episode Control")]
    public bool autoFindReferences = true;
    public bool autoBeginOnStart = true;
    public bool stopVehicleOnTerminal = true;
    public bool disableAutonomousOnTerminal = true;
    [Tooltip("0 이하이면 타임아웃 비활성")]
    public float maxEpisodeSeconds = 120f;

    [Header("Failure Rules")]
    [Tooltip("이 레벨 이상에서 저속 정지가 유지되면 실패")]
    public CollisionWarningPublisher.WarningLevel dangerLevelThreshold = CollisionWarningPublisher.WarningLevel.Brake;
    [Tooltip("위험레벨 + 정지 상태가 이 시간 지속되면 FailRiskStop")]
    public float dangerStopHoldSeconds = 0.7f;
    public float stoppedSpeedThreshold = 0.05f;
    [Tooltip("0보다 크면 횡오차로 실패 처리(옵션)")]
    public float maxLateralErrorForFailure = 0f;
    public LayerMask collisionFailLayers = ~0;
    [Tooltip("상대 충돌속도가 이 값 이상일 때만 충돌 실패로 인정")]
    public float collisionMinRelativeSpeed = 0.1f;

    [Header("Scoring")]
    public float finishBonus = 20f;
    public float baseFailurePenalty = 10f;
    public float riskStopExtraPenalty = 6f;
    public float collisionExtraPenalty = 10f;
    public float timeoutExtraPenalty = 4f;
    [Tooltip("실패 시 progress ratio가 이 값 미만이면 쓰레기 학습으로 라벨")]
    [Range(0f, 1f)] public float lowProgressTrashThreshold = 0.1f;

    [Header("Episode Report CSV")]
    public bool saveEpisodeReportCsv = true;
    public string csvFileName = "rl_episode_report.csv";

    [Header("Debug (Read Only)")]
    [SerializeField] private int episodeIndex = 0;
    [SerializeField] private bool episodeActive = false;
    [SerializeField] private bool terminalReached = false;
    [SerializeField] private bool episodeSuccess = false;
    [SerializeField] private bool isTrashEpisode = false;
    [SerializeField] private TerminalType terminalType = TerminalType.None;
    [SerializeField] private string terminalReason = "None";
    [SerializeField] private string trashReason = "None";
    [SerializeField] private float elapsedSeconds = 0f;
    [SerializeField] private float episodeScore = 0f;
    [SerializeField] private float episodeRewardBase = 0f;
    [SerializeField] private float dangerStoppedDuration = 0f;
    [SerializeField] private float timeAtDangerLevel = 0f;
    [SerializeField] private int collisionCount = 0;
    [SerializeField] private CollisionWarningPublisher.WarningLevel maxWarningLevel =
        CollisionWarningPublisher.WarningLevel.Safe;
    [SerializeField] private float minObservedTtc = float.PositiveInfinity;

    public event Action<RLEpisodeEvaluator> OnEpisodeTerminated;

    void Start()
    {
        if (autoFindReferences)
            AutoFindReferences();

        if (autoBeginOnStart)
            BeginEpisode();
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
            }
        }
    }

    void AutoFindReferences()
    {
        if (progressRewardProvider == null)
            progressRewardProvider = FindObjectOfType<ProgressRewardProvider>();
        if (collisionWarningPublisher == null)
            collisionWarningPublisher = FindObjectOfType<CollisionWarningPublisher>();
        if (wheelController == null)
            wheelController = FindObjectOfType<WheelTest>();
        if (regressionDrivingController == null)
            regressionDrivingController = FindObjectOfType<RegressionDrivingController>();
    }

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
        maxWarningLevel = CollisionWarningPublisher.WarningLevel.Safe;
        minObservedTtc = float.PositiveInfinity;

        if (progressRewardProvider != null)
            progressRewardProvider.ResetRewardState();
    }

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
        SetTerminal(
            TerminalType.FailCollision,
            $"Collision(obj={collision.collider.name}, relV={relativeSpeed:F2}m/s)"
        );
    }

    /// <summary>
    /// FinishLineGate에서 호출.
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
            }

            float progressRatio = progressRewardProvider != null ? progressRewardProvider.GetPathProgressRatio() : 0f;
            bool lowProgress = progressRatio < lowProgressTrashThreshold;
            bool highRiskStop = type == TerminalType.FailRiskStop;
            bool collisionFail = type == TerminalType.FailCollision;

            isTrashEpisode = highRiskStop || collisionFail || lowProgress;
            if (collisionFail)
                trashReason = "Trash: Collision termination";
            else if (highRiskStop)
                trashReason = "Trash: Danger level(5/6) stop termination";
            else if (lowProgress)
                trashReason = $"Trash: Low progress ratio ({progressRatio:F2} < {lowProgressTrashThreshold:F2})";
            else
                trashReason = "None";
        }

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
            $"score={episodeScore:F3} | rewardBase={episodeRewardBase:F3} | trash={isTrashEpisode} | reason={terminalReason}"
        );

        OnEpisodeTerminated?.Invoke(this);
    }

    void AppendEpisodeCsv()
    {
        try
        {
            string path = Path.Combine(Application.persistentDataPath, csvFileName);
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
