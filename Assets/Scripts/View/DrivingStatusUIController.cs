using UnityEngine;
using TMPro;

/// <summary>
/// 주행 관련 UI 텍스트를 한 곳에서 갱신하는 전용 컨트롤러.
/// - CollisionWarningPublisher
/// - DrivingDataCollectorV2
/// - RegressionDrivingController
/// </summary>
public class DrivingStatusUIController : MonoBehaviour
{
    [Header("Source References")]
    public CollisionWarningPublisher collisionWarningPublisher;
    public DrivingDataCollectorV2 dataCollector;
    public RegressionDrivingController regressionController;
    public WheelTest wheelController;
    public TrafficLightStateSubscriber trafficLightSubscriber;
    public TrafficLightDecisionEngine trafficLightDecisionEngine;
    public ProgressRewardProvider progressRewardProvider;
    public RLEpisodeEvaluator rlEpisodeEvaluator;

    [Header("Auto Find")]
    public bool autoFindReferences = true;

    [Header("Collision Warning UI")]
    public TextMeshProUGUI collisionWarningLevelText;
    public TextMeshProUGUI collisionTtcText;
    public TextMeshProUGUI collisionClosestSensorText;
    public TextMeshProUGUI collisionDistanceText;

    [Header("Data Collector UI")]
    public TextMeshProUGUI collectorStatusText;
    public TextMeshProUGUI collectorInfoText;
    public TextMeshProUGUI collectorActionText;
    public TextMeshProUGUI collectorSpeedText;

    [Header("Regression Controller UI")]
    public TextMeshProUGUI regressionModeText;
    public TextMeshProUGUI regressionActionText;
    public TextMeshProUGUI regressionControlText;
    public TextMeshProUGUI regressionStatsText;
    public TextMeshProUGUI regressionGuideText;

    [Header("Traffic Light UI")]
    public TextMeshProUGUI trafficLightStateText;
    public TextMeshProUGUI trafficLightConfidenceText;
    public TextMeshProUGUI trafficLightDecisionValueText;
    public TextMeshProUGUI trafficLightDecisionRelevantText;
    public TextMeshProUGUI trafficLightDecisionReasonText;
    public TextMeshProUGUI trafficLightDecisionConfText;
    public TextMeshProUGUI trafficLightDecisionAreaText;
    public TextMeshProUGUI trafficLightDecisionLaneText;
    public TextMeshProUGUI trafficLightDecisionStopSrcText;
    public TextMeshProUGUI trafficLightDistanceTfText;
    public TextMeshProUGUI trafficLightDistancePerceptionText;
    public TextMeshProUGUI trafficLightDistanceFusedText;
    public TextMeshProUGUI trafficLightDistanceSourceText;
    public TextMeshProUGUI trafficLightControlText;

    [Header("Progress Reward UI")]
    public TextMeshProUGUI progressZoneNameText;
    public TextMeshProUGUI progressZoneScoreText;
    public TextMeshProUGUI progressStepRewardText;
    public TextMeshProUGUI progressCumulativeRewardText;
    public TextMeshProUGUI progressPathText;
    public TextMeshProUGUI progressLateralErrorText;

    [Header("RL Episode UI")]
    public TextMeshProUGUI rlEpisodeStatusText;
    public TextMeshProUGUI rlEpisodeScoreText;
    public TextMeshProUGUI rlEpisodeReasonText;
    public TextMeshProUGUI rlEpisodeTrashText;

    void Start()
    {
        if (autoFindReferences)
            AutoFindReferences();
    }

    void Update()
    {
        if (autoFindReferences &&
            (collisionWarningPublisher == null || dataCollector == null || regressionController == null ||
             wheelController == null ||
             trafficLightSubscriber == null || trafficLightDecisionEngine == null || progressRewardProvider == null ||
             rlEpisodeEvaluator == null))
        {
            AutoFindReferences();
        }

        UpdateCollisionWarningUI();
        UpdateDataCollectorUI();
        UpdateRegressionUI();
        UpdateTrafficLightUI();
        UpdateProgressRewardUI();
        UpdateRLEpisodeUI();
    }

    void AutoFindReferences()
    {
        if (collisionWarningPublisher == null)
            collisionWarningPublisher = FindObjectOfType<CollisionWarningPublisher>();

        if (dataCollector == null)
            dataCollector = FindObjectOfType<DrivingDataCollectorV2>();

        if (regressionController == null)
            regressionController = FindObjectOfType<RegressionDrivingController>();

        if (wheelController == null)
            wheelController = FindObjectOfType<WheelTest>();

        if (trafficLightSubscriber == null)
            trafficLightSubscriber = FindObjectOfType<TrafficLightStateSubscriber>();

        if (trafficLightDecisionEngine == null)
            trafficLightDecisionEngine = FindObjectOfType<TrafficLightDecisionEngine>();

        if (progressRewardProvider == null)
            progressRewardProvider = FindObjectOfType<ProgressRewardProvider>();

        if (rlEpisodeEvaluator == null)
            rlEpisodeEvaluator = FindObjectOfType<RLEpisodeEvaluator>();
    }

    void UpdateCollisionWarningUI()
    {
        if (collisionWarningPublisher == null) return;

        CollisionWarningPublisher.WarningLevel level = collisionWarningPublisher.GetWarningLevel();
        float ttc = collisionWarningPublisher.GetTimeToCollision();
        (string source, string sensor, float distance) info = collisionWarningPublisher.GetClosestObstacleInfo();

        if (collisionWarningLevelText != null)
        {
            collisionWarningLevelText.text = $"위험도: {level} ({(int)level})";
        }

        if (collisionTtcText != null)
        {
            collisionTtcText.text = float.IsInfinity(ttc) ? "TTC: ∞" : $"TTC: {ttc:F2}s";
        }

        if (collisionClosestSensorText != null)
        {
            string distanceStr = float.IsInfinity(info.distance) ? "∞" : $"{info.distance:F2}m";
            string sensorInfo = info.source == "None" ? "None" : $"{info.source}-{info.sensor}";
            collisionClosestSensorText.text = $"최근접 센서: {sensorInfo}";
        }

        if (collisionDistanceText != null)
        {
            string distanceStr = float.IsInfinity(info.distance) ? "∞" : $"{info.distance:F2}m";
            collisionDistanceText.text = $"거리: {distanceStr}";
        }
    }

    void UpdateDataCollectorUI()
    {
        if (dataCollector == null) return;

        if (collectorStatusText != null)
        {
            if (dataCollector.isRecording)
                collectorStatusText.text = $"<color=red>● REC: {dataCollector.frameCount}</color>";
            else
                collectorStatusText.text = $"[{dataCollector.recordKey}] Rec  [{dataCollector.saveKey}] Save";
        }

        if (collectorInfoText != null)
        {
            if (dataCollector.isRecording)
            {
                float duration = dataCollector.GetRecordingDuration();
                collectorInfoText.text = $"Time: {duration:F1}s";
            }
            else
            {
                int bufferedCount = dataCollector.GetBufferedFrameCount();
                collectorInfoText.text = bufferedCount > 0
                    ? $"<color=yellow>Unsaved: {bufferedCount}</color>"
                    : "Ready";
            }
        }

        if (collectorActionText != null)
        {
            collectorActionText.text = $"Action: <color=#00FFFF>{dataCollector.GetCurrentActionName()}</color>";
        }

        if (collectorSpeedText != null)
        {
            float speed = dataCollector.GetCurrentSpeedMS();
            collectorSpeedText.text = $"Speed: {speed:F2} m/s";
        }
    }

    void UpdateRegressionUI()
    {
        if (regressionController == null) return;

        float speed = regressionController.GetCurrentSpeedMS();
        int interventionCount = regressionController.GetInterventionCount();
        bool residualMode = regressionController.predictionOnlyMode;

        if (regressionController.isAutonomousMode)
        {
            if (regressionController.IsInterventionActive())
            {
                if (regressionModeText != null)
                    regressionModeText.text = $"<color=yellow> INTERVENTION (#{interventionCount})</color>";

                if (regressionActionText != null)
                {
                    float remaining = regressionController.GetInterventionRemainingTime();
                    regressionActionText.text = $"Return to AI in: {remaining:F1}s\nWASD Manual Control...";
                }

                if (regressionControlText != null)
                    regressionControlText.text = "";
            }
            else
            {
                if (regressionModeText != null)
                    regressionModeText.text = residualMode
                        ? "<color=#00FF00>● AUTONOMOUS (Regression Base + RL Final)</color>"
                        : "<color=#00FF00>● AUTONOMOUS (Regression)</color>";

                if (regressionActionText != null)
                {
                    regressionActionText.text =
                        $"Base Steer: <color=#00FFFF>{regressionController.GetPredictedSteering():F3}</color> | " +
                        $"Base Throt: <color=#00FFFF>{regressionController.GetPredictedThrottle():F3}</color>";
                }

                if (regressionControlText != null)
                {
                    if (residualMode)
                    {
                        float finalSteer = wheelController != null
                            ? wheelController.GetSteeringInput()
                            : regressionController.GetAppliedSteering();
                        float finalThrottle = wheelController != null
                            ? wheelController.GetThrottleInput()
                            : regressionController.GetAppliedThrottle();
                        float finalBrake = wheelController != null ? wheelController.GetBrakeInput() : 0f;

                        regressionControlText.text =
                            $"Final → Steer: {finalSteer:F2} | Throt: {finalThrottle:F2} | Brake: {finalBrake:F2}";
                    }
                    else
                    {
                        regressionControlText.text =
                            $"Applied → Steer: {regressionController.GetAppliedSteering():F2} | " +
                            $"Throt: {regressionController.GetAppliedThrottle():F2}";
                    }
                }
            }

            if (regressionStatsText != null)
            {
                string warningInfo = collisionWarningPublisher != null
                    ? $" | Warn: {collisionWarningPublisher.GetWarningLevel()}"
                    : "";
                regressionStatsText.text = $"Speed: {speed:F2} m/s | Interventions: {interventionCount}{warningInfo}";
            }

            if (regressionGuideText != null)
                regressionGuideText.text = "<color=grey>WASD: Intervention | P: Stop Auto</color>";
        }
        else
        {
            if (regressionModeText != null)
                regressionModeText.text = "<color=yellow>○ MANUAL MODE</color>";

            if (regressionActionText != null)
                regressionActionText.text = $"[{regressionController.toggleKey}] Start Autonomous Mode";

            if (regressionControlText != null)
            {
                if (interventionCount > 0)
                    regressionControlText.text = $"<color=#00FFFF>Total Interventions: {interventionCount}</color>";
                else
                    regressionControlText.text = "";
            }

            if (regressionStatsText != null)
            {
                if (!regressionController.IsModelLoaded())
                    regressionStatsText.text = "<color=red>! Model Not Loaded</color>";
                else
                    regressionStatsText.text = "";
            }

            if (regressionGuideText != null)
                regressionGuideText.text = "";
        }
    }

    void UpdateTrafficLightUI()
    {
        if (trafficLightSubscriber == null && trafficLightDecisionEngine == null)
        {
            if (trafficLightStateText != null) trafficLightStateText.text = "신호상태: none -> none (raw: none)";
            if (trafficLightConfidenceText != null) trafficLightConfidenceText.text = "안정 상태비율: 0%, Raw Conf: 0.00";
            if (trafficLightDecisionValueText != null) trafficLightDecisionValueText.text = "신호판단: N/A";
            if (trafficLightDecisionRelevantText != null) trafficLightDecisionRelevantText.text = "Relevant=N/A";
            if (trafficLightDecisionReasonText != null) trafficLightDecisionReasonText.text = "Reason: No Decision Engine";
            if (trafficLightDecisionConfText != null) trafficLightDecisionConfText.text = "Gate Conf=0.00";
            if (trafficLightDecisionAreaText != null) trafficLightDecisionAreaText.text = "Gate Area=0.0000";
            if (trafficLightDecisionLaneText != null) trafficLightDecisionLaneText.text = "Gate LaneOk=false";
            if (trafficLightDecisionStopSrcText != null) trafficLightDecisionStopSrcText.text = "Gate StopSrc=none";
            if (trafficLightDistanceTfText != null) trafficLightDistanceTfText.text = "정지선거리: Tf(OFF)=∞";
            if (trafficLightDistancePerceptionText != null) trafficLightDistancePerceptionText.text = "Perception=∞";
            if (trafficLightDistanceFusedText != null) trafficLightDistanceFusedText.text = "Fused=∞";
            if (trafficLightDistanceSourceText != null) trafficLightDistanceSourceText.text = "Src=none";
            if (trafficLightControlText != null) trafficLightControlText.text = "권장제어: ThrottleScale=1.00, Brake=0.00";
            return;
        }

        string prev = trafficLightSubscriber != null ? trafficLightSubscriber.GetPreviousState() : "none";
        string current = trafficLightSubscriber != null ? trafficLightSubscriber.GetCurrentState() : "none";
        string raw = trafficLightSubscriber != null ? trafficLightSubscriber.GetRawState() : "none";
        float stableRatio = trafficLightSubscriber != null ? trafficLightSubscriber.GetStableStateRatio() : 0f;
        float rawConf = trafficLightSubscriber != null ? trafficLightSubscriber.GetRawConfidence() : 0f;

        if (trafficLightStateText != null)
            trafficLightStateText.text = $"신호상태: {prev} -> {current} (raw: {raw})";

        if (trafficLightConfidenceText != null)
            trafficLightConfidenceText.text = $"안정 상태비율: {stableRatio:P0}, Raw Conf: {rawConf:F2}";

        if (trafficLightDecisionEngine == null)
        {
            if (trafficLightDecisionValueText != null) trafficLightDecisionValueText.text = "신호판단: N/A";
            if (trafficLightDecisionRelevantText != null) trafficLightDecisionRelevantText.text = "Relevant=N/A";
            if (trafficLightDecisionReasonText != null) trafficLightDecisionReasonText.text = "Reason: No Decision Engine";
            if (trafficLightDecisionConfText != null) trafficLightDecisionConfText.text = "Gate Conf=0.00";
            if (trafficLightDecisionAreaText != null) trafficLightDecisionAreaText.text = "Gate Area=0.0000";
            if (trafficLightDecisionLaneText != null) trafficLightDecisionLaneText.text = "Gate LaneOk=false";
            if (trafficLightDecisionStopSrcText != null) trafficLightDecisionStopSrcText.text = "Gate StopSrc=none";
            if (trafficLightDistanceTfText != null) trafficLightDistanceTfText.text = "정지선거리: Tf(OFF)=∞";
            if (trafficLightDistancePerceptionText != null) trafficLightDistancePerceptionText.text = "Perception=∞";
            if (trafficLightDistanceFusedText != null) trafficLightDistanceFusedText.text = "Fused=∞";
            if (trafficLightDistanceSourceText != null) trafficLightDistanceSourceText.text = "Src=none";
            if (trafficLightControlText != null) trafficLightControlText.text = "권장제어: No Decision Engine";
            return;
        }

        bool relevant = trafficLightDecisionEngine.IsSignalRelevant();
        var decision = trafficLightDecisionEngine.GetDecision();
        string reason = trafficLightDecisionEngine.GetDecisionReason();
        float gateConf = trafficLightDecisionEngine.GetGateStateConfidence();
        float gateArea = trafficLightDecisionEngine.GetGateBBoxArea();
        bool gateLaneOk = trafficLightDecisionEngine.GetGateLaneOk();
        string gateStopSrc = trafficLightDecisionEngine.GetStopLineDistanceSource();
        float stopLineDist = trafficLightDecisionEngine.GetStopLineDistance();
        float stopLinePerceptionDist = trafficLightDecisionEngine.GetStopLineDistancePerception();
        float fusedDist = trafficLightDecisionEngine.GetDecisionDistance();
        string stopLineMode = trafficLightDecisionEngine.HasStopLineTransform() ? "ON" : "OFF";
        string stopLineSource = trafficLightDecisionEngine.GetStopLineDistanceSource();
        float throttleScale = trafficLightDecisionEngine.GetRecommendedThrottleScale();
        float brake = trafficLightDecisionEngine.GetRecommendedBrake();

        if (trafficLightDecisionValueText != null)
            trafficLightDecisionValueText.text = $"신호판단: {decision}";

        if (trafficLightDecisionRelevantText != null)
            trafficLightDecisionRelevantText.text = $"Relevant={relevant}";

        if (trafficLightDecisionReasonText != null)
            trafficLightDecisionReasonText.text = $"Reason: {reason}";

        if (trafficLightDecisionConfText != null)
            trafficLightDecisionConfText.text = $"Gate Conf={gateConf:F2}";

        if (trafficLightDecisionAreaText != null)
            trafficLightDecisionAreaText.text = $"Gate Area={gateArea:F4}";

        if (trafficLightDecisionLaneText != null)
            trafficLightDecisionLaneText.text = $"Gate LaneOk={gateLaneOk}";

        if (trafficLightDecisionStopSrcText != null)
            trafficLightDecisionStopSrcText.text = $"Gate StopSrc={gateStopSrc}";

        if (trafficLightDistanceTfText != null)
            trafficLightDistanceTfText.text = $"정지선거리: Tf({stopLineMode})={FormatDistance(stopLineDist)}";

        if (trafficLightDistancePerceptionText != null)
            trafficLightDistancePerceptionText.text = $"Perception={FormatDistance(stopLinePerceptionDist)}";

        if (trafficLightDistanceFusedText != null)
            trafficLightDistanceFusedText.text = $"Fused={FormatDistance(fusedDist)}";

        if (trafficLightDistanceSourceText != null)
            trafficLightDistanceSourceText.text = $"Src={stopLineSource}";

        if (trafficLightControlText != null)
            trafficLightControlText.text = $"권장제어: ThrottleScale={throttleScale:F2}, Brake={brake:F2}";
    }

    void UpdateProgressRewardUI()
    {
        if (progressRewardProvider == null)
        {
            if (progressZoneNameText != null) progressZoneNameText.text = "현재 Zone: None";
            if (progressZoneScoreText != null) progressZoneScoreText.text = "Zone Score: 0.00 (Active: 0)";
            if (progressStepRewardText != null) progressStepRewardText.text = "Step Reward: 0.0000";
            if (progressCumulativeRewardText != null) progressCumulativeRewardText.text = "누적 보상: 0.0000";
            if (progressPathText != null) progressPathText.text = "경로 진행: 0.00m / 0.00m (0.0%)";
            if (progressLateralErrorText != null) progressLateralErrorText.text = "횡 오차: 0.00m";
            return;
        }

        string zoneName = progressRewardProvider.GetCurrentZoneName();
        int activeZoneCount = progressRewardProvider.GetActiveZoneCount();
        float zoneScore = progressRewardProvider.GetCurrentZoneScore();
        float stepReward = progressRewardProvider.GetLastStepReward();
        float cumulative = progressRewardProvider.GetCumulativeReward();
        float pathS = progressRewardProvider.GetCurrentPathS();
        float totalPath = progressRewardProvider.GetTotalPathLength();
        float progressRatio = progressRewardProvider.GetPathProgressRatio();
        float lateralError = progressRewardProvider.GetCurrentLateralError();

        if (progressZoneNameText != null)
            progressZoneNameText.text = $"현재 Zone: {zoneName}";

        if (progressZoneScoreText != null)
            progressZoneScoreText.text = $"Zone Score: {zoneScore:F2} (Active: {activeZoneCount})";

        if (progressStepRewardText != null)
            progressStepRewardText.text = $"Step Reward: {stepReward:F4}";

        if (progressCumulativeRewardText != null)
            progressCumulativeRewardText.text = $"누적 보상: {cumulative:F4}";

        if (progressPathText != null)
            progressPathText.text = $"경로 진행: {pathS:F2}m / {totalPath:F2}m ({progressRatio * 100f:F1}%)";

        if (progressLateralErrorText != null)
            progressLateralErrorText.text = $"횡 오차: {lateralError:F2}m";
    }

    void UpdateRLEpisodeUI()
    {
        if (rlEpisodeEvaluator == null)
        {
            if (rlEpisodeStatusText != null) rlEpisodeStatusText.text = "Episode: N/A";
            if (rlEpisodeScoreText != null) rlEpisodeScoreText.text = "Episode Score: 0.000";
            if (rlEpisodeReasonText != null) rlEpisodeReasonText.text = "Terminal Reason: None";
            if (rlEpisodeTrashText != null) rlEpisodeTrashText.text = "Trash: false";
            return;
        }

        string terminal = rlEpisodeEvaluator.GetTerminalType().ToString();
        bool success = rlEpisodeEvaluator.IsEpisodeSuccess();
        bool terminalReached = rlEpisodeEvaluator.IsTerminalReached();
        bool isActive = rlEpisodeEvaluator.IsEpisodeActive();
        float score = rlEpisodeEvaluator.GetEpisodeScore();
        int episodeIndex = rlEpisodeEvaluator.GetEpisodeIndex();
        string reason = rlEpisodeEvaluator.GetTerminalReason();
        bool trash = rlEpisodeEvaluator.IsTrash();
        string trashReason = rlEpisodeEvaluator.GetTrashReason();

        if (rlEpisodeStatusText != null)
        {
            string status = isActive ? "Running" : (terminalReached ? "Terminated" : "Idle");
            rlEpisodeStatusText.text =
                $"Episode #{episodeIndex}: {status}, Success={success}, Terminal={terminal}";
        }

        if (rlEpisodeScoreText != null)
            rlEpisodeScoreText.text = $"Episode Score: {score:F3}";

        if (rlEpisodeReasonText != null)
            rlEpisodeReasonText.text = $"Terminal Reason: {reason}";

        if (rlEpisodeTrashText != null)
            rlEpisodeTrashText.text = $"Trash: {trash}, Why: {trashReason}";
    }

    string FormatDistance(float distance)
    {
        return float.IsInfinity(distance) ? "∞" : $"{distance:F2}m";
    }
}
