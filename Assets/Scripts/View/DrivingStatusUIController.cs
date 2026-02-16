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

    void Start()
    {
        if (autoFindReferences)
            AutoFindReferences();
    }

    void Update()
    {
        if (autoFindReferences &&
            (collisionWarningPublisher == null || dataCollector == null || regressionController == null))
        {
            AutoFindReferences();
        }

        UpdateCollisionWarningUI();
        UpdateDataCollectorUI();
        UpdateRegressionUI();
    }

    void AutoFindReferences()
    {
        if (collisionWarningPublisher == null)
            collisionWarningPublisher = FindObjectOfType<CollisionWarningPublisher>();

        if (dataCollector == null)
            dataCollector = FindObjectOfType<DrivingDataCollectorV2>();

        if (regressionController == null)
            regressionController = FindObjectOfType<RegressionDrivingController>();
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
                    regressionModeText.text = "<color=#00FF00>● AUTONOMOUS (Regression)</color>";

                if (regressionActionText != null)
                {
                    regressionActionText.text =
                        $"Steer: <color=#00FFFF>{regressionController.GetPredictedSteering():F3}</color> | " +
                        $"Throt: <color=#00FFFF>{regressionController.GetPredictedThrottle():F3}</color>";
                }

                if (regressionControlText != null)
                {
                    regressionControlText.text =
                        $"Applied → Steer: {regressionController.GetAppliedSteering():F2} | " +
                        $"Throt: {regressionController.GetAppliedThrottle():F2}";
                }
            }

            if (regressionStatsText != null)
                regressionStatsText.text = $"Speed: {speed:F2} m/s | Interventions: {interventionCount}";

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
}
