using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// ROS2 /traffic_light/state 토픽 구독자
/// RPi5에서 YOLOv8n으로 탐지한 신호등 상태를 수신하여 디버깅
///
/// 수신 값: "red", "yellow", "green", "none"
/// </summary>
public class TrafficLightStateSubscriber : MonoBehaviour
{
    [Header("ROS Settings")]
    public string topicName = "/traffic_light/state";

    [Header("State (Read Only)")]
    [SerializeField] private string currentState = "none";
    [SerializeField] private string previousState = "none";

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<StringMsg>(topicName, OnTrafficLightState);

        Debug.Log($"[TrafficLight] Subscribing to {topicName}");
    }

    private void OnTrafficLightState(StringMsg msg)
    {
        string newState = msg.data;

        if (newState != currentState)
        {
            Debug.Log($"[TrafficLight] State changed: {currentState} -> {newState}");
            previousState = currentState;
            currentState = newState;
        }
    }

    // ========== 외부 접근용 메서드 ==========

    /// <summary>현재 신호등 상태 반환</summary>
    public string GetCurrentState()
    {
        return currentState;
    }

    /// <summary>빨간불 여부</summary>
    public bool IsRed()
    {
        return currentState == "red";
    }

    /// <summary>초록불 여부</summary>
    public bool IsGreen()
    {
        return currentState == "green";
    }

    /// <summary>노란불 여부</summary>
    public bool IsYellow()
    {
        return currentState == "yellow";
    }

    /// <summary>신호등 미탐지 여부</summary>
    public bool IsNone()
    {
        return currentState == "none";
    }
}
