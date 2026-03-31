using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// RL 에피소드 종료 시 요약 CSV를 기록하는 로거.
/// RLEpisodeEvaluator.OnEpisodeTerminated 이벤트에 구독하여
/// 에피소드당 1줄만 기록 → I/O 최소, 학습 성능 영향 없음.
///
/// 멀티 에이전트: RosTopicNamespace에서 네임스페이스를 읽어
/// 에이전트별 개별 CSV 파일을 생성한다 (파일 잠금 충돌 방지).
/// </summary>
public class RLEpisodeCsvLogger : MonoBehaviour
{
    [Header("References")]
    public RLEpisodeEvaluator episodeEvaluator;
    public ProgressRewardProvider progressRewardProvider;
    public AutoDriverRLAgent autoDriverRLAgent;

    [Header("Settings")]
    [Tooltip("CSV 저장 폴더 (프로젝트 루트 기준 상대경로 또는 절대경로)")]
    public string outputFolder = "Logs/RL";
    [Tooltip("파일명 접두사. 실행 시 네임스페이스+타임스탬프가 붙음")]
    public string filePrefix = "rl_episodes";
    [Tooltip("true면 자동으로 부모 계층에서 레퍼런스 탐색")]
    public bool autoFindReferences = true;

    private StreamWriter writer;
    private string filePath;
    private bool headerWritten = false;
    private string agentNamespace = "";

    void Start()
    {
        ResolveNamespace();

        if (autoFindReferences)
            AutoFindReferences();

        InitCsvFile();
        Subscribe();
    }

    void OnDestroy()
    {
        Unsubscribe();
        CloseFile();
    }

    void ResolveNamespace()
    {
        var ns = GetComponentInParent<RosTopicNamespace>();
        if (ns != null && !string.IsNullOrEmpty(ns.namespacePrefix))
        {
            // "/amr0" → "amr0", "/amr15" → "amr15"
            agentNamespace = ns.namespacePrefix.TrimStart('/').TrimEnd('/');
        }
        else
        {
            agentNamespace = gameObject.GetInstanceID().ToString();
        }
    }

    void AutoFindReferences()
    {
        // 멀티 에이전트 환경: FindObjectOfType 사용 금지, 부모 계층만 탐색
        if (episodeEvaluator == null)
            episodeEvaluator = GetComponentInParent<RLEpisodeEvaluator>();
        if (progressRewardProvider == null)
            progressRewardProvider = GetComponentInParent<ProgressRewardProvider>();
        if (autoDriverRLAgent == null)
            autoDriverRLAgent = GetComponentInParent<AutoDriverRLAgent>();
    }

    void InitCsvFile()
    {
        string folder = Path.IsPathRooted(outputFolder)
            ? outputFolder
            : Path.Combine(Application.dataPath, "..", outputFolder);

        if (!Directory.Exists(folder))
            Directory.CreateDirectory(folder);

        string timestamp = System.DateTime.Now.ToString("yyyyMMdd_HHmmss");
        string fileName = $"{filePrefix}_{agentNamespace}_{timestamp}.csv";
        filePath = Path.Combine(folder, fileName);

        writer = new StreamWriter(filePath, false, Encoding.UTF8, 4096);
        writer.AutoFlush = false;

        WriteHeader();
        Debug.Log($"[RLEpisodeCsvLogger:{agentNamespace}] CSV logging to: {filePath}");
    }

    void WriteHeader()
    {
        if (headerWritten) return;

        writer.WriteLine(string.Join(",", new[]
        {
            "agent",
            "episode",
            "elapsed_s",
            "terminal_type",
            "terminal_reason",
            "cumulative_reward",
            "progress_reward",
            "heading_reward",
            "lateral_reward",
            "safety_penalty",
            "traffic_penalty",
            "terminal_reward",
            "collision_count",
            "max_warning_level",
            "min_ttc",
            "time_at_danger"
        }));
        writer.Flush();
        headerWritten = true;
    }

    void Subscribe()
    {
        if (episodeEvaluator != null)
            episodeEvaluator.OnEpisodeTerminated += OnEpisodeTerminated;
    }

    void Unsubscribe()
    {
        if (episodeEvaluator != null)
            episodeEvaluator.OnEpisodeTerminated -= OnEpisodeTerminated;
    }

    void OnEpisodeTerminated(RLEpisodeEvaluator evaluator)
    {
        if (writer == null) return;

        float terminalReward = 0f;
        if (autoDriverRLAgent != null)
        {
            terminalReward = evaluator.IsEpisodeSuccess()
                ? autoDriverRLAgent.successTerminalReward
                : autoDriverRLAgent.failureTerminalPenalty;
        }

        float progressReward = 0f;
        float headingReward = 0f;
        float lateralReward = 0f;
        float safetyPenalty = 0f;
        float trafficPenalty = 0f;
        float cumulativeReward = 0f;

        if (progressRewardProvider != null)
        {
            progressReward = progressRewardProvider.GetCumulativeProgressReward();
            headingReward = progressRewardProvider.GetCumulativeHeadingReward();
            lateralReward = progressRewardProvider.GetCumulativeLateralReward();
            safetyPenalty = progressRewardProvider.GetCumulativeSafetyPenalty();
            trafficPenalty = progressRewardProvider.GetCumulativeTrafficPenalty();
            cumulativeReward = progressRewardProvider.GetCumulativeReward();
        }

        string line = string.Join(",", new[]
        {
            agentNamespace,
            evaluator.GetEpisodeIndex().ToString(),
            evaluator.GetElapsedSeconds().ToString("F1"),
            evaluator.GetTerminalType().ToString(),
            Escape(evaluator.GetTerminalReason()),
            cumulativeReward.ToString("F3"),
            progressReward.ToString("F3"),
            headingReward.ToString("F3"),
            lateralReward.ToString("F3"),
            safetyPenalty.ToString("F3"),
            trafficPenalty.ToString("F3"),
            terminalReward.ToString("F3"),
            evaluator.GetCollisionCount().ToString(),
            evaluator.GetMaxWarningLevel().ToString(),
            evaluator.GetMinObservedTtc().ToString("F2"),
            evaluator.GetTimeAtDangerLevel().ToString("F2")
        });

        writer.WriteLine(line);
        writer.Flush();
    }

    static string Escape(string value)
    {
        if (string.IsNullOrEmpty(value)) return "";
        if (value.Contains(",") || value.Contains("\"") || value.Contains("\n"))
            return "\"" + value.Replace("\"", "\"\"") + "\"";
        return value;
    }

    void CloseFile()
    {
        if (writer != null)
        {
            writer.Flush();
            writer.Close();
            writer = null;
        }
    }
}
