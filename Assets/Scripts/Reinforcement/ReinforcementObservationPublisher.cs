using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// Publish RL observation features so external ROS policy can match training-time inputs.
/// </summary>
public class ReinforcementObservationPublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    public string lateralErrorTopic = "/rl/lateral_error";
    public string signedLateralErrorTopic = "/rl/signed_lateral_error";
    public string headingErrorDegTopic = "/rl/heading_error_deg";
    public string progressRatioTopic = "/rl/progress_ratio";
    public float publishRate = 20f;

    [Header("References")]
    public ProgressRewardProvider progressRewardProvider;
    public AutoDriverRLAgent autoDriverRLAgent;
    public bool autoFindReferences = true;

    [Header("Debug")]
    public bool showDebugInfo = false;
    public float debugLogIntervalSec = 1f;
    [SerializeField] private string resolvedLateralErrorTopic = "";
    [SerializeField] private string resolvedSignedLateralErrorTopic = "";
    [SerializeField] private string resolvedHeadingErrorDegTopic = "";
    [SerializeField] private string resolvedProgressRatioTopic = "";

    private ROSConnection ros;
    private float publishInterval = 0.05f;
    private float lastPublishTime = -999f;
    private float lastDebugLogTime = -999f;

    void Start()
    {
        if (autoFindReferences)
            AutoFindReferences();

        ros = ROSConnection.GetOrCreateInstance();
        publishInterval = 1f / Mathf.Max(1f, publishRate);

        resolvedLateralErrorTopic = RosTopicNamespace.Resolve(gameObject, lateralErrorTopic);
        resolvedSignedLateralErrorTopic = RosTopicNamespace.Resolve(gameObject, signedLateralErrorTopic);
        resolvedHeadingErrorDegTopic = RosTopicNamespace.Resolve(gameObject, headingErrorDegTopic);
        resolvedProgressRatioTopic = RosTopicNamespace.Resolve(gameObject, progressRatioTopic);

        ros.RegisterPublisher<Float32Msg>(resolvedLateralErrorTopic);
        ros.RegisterPublisher<Float32Msg>(resolvedSignedLateralErrorTopic);
        ros.RegisterPublisher<Float32Msg>(resolvedHeadingErrorDegTopic);
        ros.RegisterPublisher<Float32Msg>(resolvedProgressRatioTopic);

        Debug.Log(
            "[RLObsPub] Started | " +
            $"lat={resolvedLateralErrorTopic} signed_lat={resolvedSignedLateralErrorTopic} " +
            $"heading={resolvedHeadingErrorDegTopic} progress={resolvedProgressRatioTopic}"
        );
    }

    void Update()
    {
        if (Time.time - lastPublishTime < publishInterval)
            return;

        if (autoFindReferences && (progressRewardProvider == null || autoDriverRLAgent == null))
            AutoFindReferences();

        PublishObservationTopics();
        lastPublishTime = Time.time;
    }

    void AutoFindReferences()
    {
        if (progressRewardProvider == null)
            progressRewardProvider = GetComponent<ProgressRewardProvider>() ??
                                     GetComponentInParent<ProgressRewardProvider>() ??
                                     FindObjectOfType<ProgressRewardProvider>();

        if (autoDriverRLAgent == null)
            autoDriverRLAgent = GetComponent<AutoDriverRLAgent>() ??
                                GetComponentInParent<AutoDriverRLAgent>() ??
                                FindObjectOfType<AutoDriverRLAgent>();
    }

    void PublishObservationTopics()
    {
        float lateralErrorAbs = 0f;
        float progressRatio = 0f;
        float signedLateralError = 0f;
        float headingErrorDeg = 0f;

        if (progressRewardProvider != null)
        {
            progressRewardProvider.RefreshTrackingState();
            lateralErrorAbs = Mathf.Max(0f, progressRewardProvider.GetCurrentLateralError());
            progressRatio = Mathf.Clamp01(progressRewardProvider.GetPathProgressRatio());
        }

        if (autoDriverRLAgent != null)
            headingErrorDeg = autoDriverRLAgent.GetCurrentHeadingErrorDeg(out signedLateralError);

        PublishFloat(resolvedLateralErrorTopic, lateralErrorAbs);
        PublishFloat(resolvedSignedLateralErrorTopic, signedLateralError);
        PublishFloat(resolvedHeadingErrorDegTopic, headingErrorDeg);
        PublishFloat(resolvedProgressRatioTopic, progressRatio);

        if (showDebugInfo && (Time.time - lastDebugLogTime) >= Mathf.Max(0.1f, debugLogIntervalSec))
        {
            lastDebugLogTime = Time.time;
            Debug.Log(
                "[RLObsPub] " +
                $"lat={lateralErrorAbs:F3}m signed_lat={signedLateralError:F3}m " +
                $"heading={headingErrorDeg:F3}deg progress={progressRatio:F3}"
            );
        }
    }

    void PublishFloat(string topic, float value)
    {
        if (ros == null || string.IsNullOrWhiteSpace(topic))
            return;

        Float32Msg msg = new Float32Msg { data = float.IsFinite(value) ? value : 0f };
        ros.Publish(topic, msg);
    }
}
