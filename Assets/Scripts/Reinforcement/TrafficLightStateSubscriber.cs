using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// ROS2 신호등 구독자
/// - /traffic_light/state: 안정화된 문자열 상태
/// - /traffic_light/perception: 상태 신뢰도/박스 정보(Float32MultiArray)
/// </summary>
public class TrafficLightStateSubscriber : MonoBehaviour
{
    [Header("ROS Settings")]
    public string stateTopicName = "/traffic_light/state";
    public string perceptionTopicName = "/traffic_light/perception";

    [Header("State (Read Only)")]
    [SerializeField] private string currentState = "none";
    [SerializeField] private string previousState = "none";
    [SerializeField] private float stableStateRatio = 0f;
    [SerializeField] private string rawState = "none";
    [SerializeField] private float rawConfidence = 0f;
    [SerializeField] private Vector2 bboxCenterNormalized = new Vector2(-1f, -1f);
    [SerializeField] private float bboxAreaNormalized = 0f;

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<StringMsg>(stateTopicName, OnTrafficLightState);
        ros.Subscribe<Float32MultiArrayMsg>(perceptionTopicName, OnTrafficLightPerception);

        Debug.Log($"[TrafficLight] Subscribing state={stateTopicName}, perception={perceptionTopicName}");
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

    private void OnTrafficLightPerception(Float32MultiArrayMsg msg)
    {
        if (msg == null || msg.data == null || msg.data.Length < 7)
            return;

        // data[0]: stable_state_id
        // data[1]: stable_state_ratio
        // data[2]: raw_state_id
        // data[3]: raw_confidence
        // data[4]: bbox_center_x_norm
        // data[5]: bbox_center_y_norm
        // data[6]: bbox_area_norm
        int stableStateId = Mathf.RoundToInt(msg.data[0]);
        stableStateRatio = Mathf.Clamp01(msg.data[1]);
        int rawStateId = Mathf.RoundToInt(msg.data[2]);
        rawConfidence = Mathf.Clamp01(msg.data[3]);
        bboxCenterNormalized = new Vector2(msg.data[4], msg.data[5]);
        bboxAreaNormalized = Mathf.Max(0f, msg.data[6]);

        string stableState = StateIdToString(stableStateId);
        rawState = StateIdToString(rawStateId);

        // perception state와 문자열 state가 드물게 어긋날 때 보정
        if (!string.IsNullOrEmpty(stableState) && stableState != currentState)
        {
            previousState = currentState;
            currentState = stableState;
        }
    }

    private string StateIdToString(int id)
    {
        return id switch
        {
            1 => "red",
            2 => "yellow",
            3 => "green",
            _ => "none"
        };
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

    public float GetStateConfidence()
    {
        // Backward compatibility:
        // 기존 호출부에서 사용하던 GetStateConfidence()는
        // 실제로는 "stable state ratio" 값을 의미합니다.
        return stableStateRatio;
    }

    public string GetPreviousState()
    {
        return previousState;
    }

    public float GetStableStateRatio()
    {
        return stableStateRatio;
    }

    public string GetRawState()
    {
        return rawState;
    }

    public float GetRawConfidence()
    {
        return rawConfidence;
    }

    public Vector2 GetBBoxCenterNormalized()
    {
        return bboxCenterNormalized;
    }

    public float GetBBoxAreaNormalized()
    {
        return bboxAreaNormalized;
    }
}
