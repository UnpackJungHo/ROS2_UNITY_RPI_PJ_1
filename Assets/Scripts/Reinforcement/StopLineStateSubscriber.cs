using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// ROS2 정지선 구독자
/// - /stop_line/state: detected / none
/// - /stop_line/perception: raw/stable/confidence/line geometry
/// </summary>
public class StopLineStateSubscriber : MonoBehaviour
{
    [Header("ROS Settings")]
    public string stateTopicName = "/stop_line/state";
    public string perceptionTopicName = "/stop_line/perception";

    [Header("State (Read Only)")]
    [SerializeField] private string currentState = "none";
    [SerializeField] private string previousState = "none";
    [SerializeField] private bool rawDetected = false;
    [SerializeField] private bool stableDetected = false;
    [SerializeField] private float confidence = 0f;
    [SerializeField] private Vector2 linePoint1Normalized = new Vector2(-1f, -1f);
    [SerializeField] private Vector2 linePoint2Normalized = new Vector2(-1f, -1f);
    [SerializeField] private float yCenterNormalized = -1f;
    [SerializeField] private float distanceFromBottomNormalized = -1f;
    [SerializeField] private int stripeCount = 0;

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<StringMsg>(stateTopicName, OnState);
        ros.Subscribe<Float32MultiArrayMsg>(perceptionTopicName, OnPerception);

        Debug.Log($"[StopLine] Subscribing state={stateTopicName}, perception={perceptionTopicName}");
    }

    private void OnState(StringMsg msg)
    {
        string newState = msg.data;
        if (newState != currentState)
        {
            previousState = currentState;
            currentState = newState;
        }
    }

    private void OnPerception(Float32MultiArrayMsg msg)
    {
        if (msg == null || msg.data == null || msg.data.Length < 10)
            return;

        rawDetected = msg.data[0] > 0.5f;
        stableDetected = msg.data[1] > 0.5f;
        confidence = Mathf.Clamp01(msg.data[2]);

        linePoint1Normalized = new Vector2(msg.data[3], msg.data[4]);
        linePoint2Normalized = new Vector2(msg.data[5], msg.data[6]);
        yCenterNormalized = msg.data[7];
        distanceFromBottomNormalized = msg.data[8];
        stripeCount = Mathf.Max(0, Mathf.RoundToInt(msg.data[9]));

        string perceptionState = stableDetected ? "detected" : "none";
        if (perceptionState != currentState)
        {
            previousState = currentState;
            currentState = perceptionState;
        }
    }

    public string GetCurrentState() => currentState;
    public string GetPreviousState() => previousState;
    public bool IsDetected() => currentState == "detected";
    public bool IsRawDetected() => rawDetected;
    public bool IsStableDetected() => stableDetected;
    public float GetConfidence() => confidence;
    public Vector2 GetLinePoint1Normalized() => linePoint1Normalized;
    public Vector2 GetLinePoint2Normalized() => linePoint2Normalized;
    public float GetYCenterNormalized() => yCenterNormalized;
    public float GetDistanceFromBottomNormalized() => distanceFromBottomNormalized;
    public int GetStripeCount() => stripeCount;
}
