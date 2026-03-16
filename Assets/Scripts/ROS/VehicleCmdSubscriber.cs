using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

/// <summary>
/// ROS geometry_msgs/Twist 구독 브리지.
/// 수신한 명령을 VehicleCmdController에 전달한다.
/// </summary>
public class VehicleCmdSubscriber : MonoBehaviour
{
    [Header("ROS Settings")]
    [Tooltip("Twist 제어 토픽. RosTopicNamespace가 있으면 prefix가 자동 적용됨")]
    public string cmdTopicName = "/vehicle/cmd";

    [Header("Controller Reference")]
    [Tooltip("같은 GameObject 또는 부모에서 자동 탐색")]
    public VehicleCmdController controller;

    // TrainTestModeSwitcher 호환용 플래그 (Controller에서 실제 처리)
    public bool useRegressionAutonomyGate
    {
        get => controller != null ? controller.useRegressionAutonomyGate : _useRegressionAutonomyGate;
        set
        {
            _useRegressionAutonomyGate = value;
            if (controller != null) controller.useRegressionAutonomyGate = value;
        }
    }
    [Header("Compatibility Flags")]
    [Tooltip("true면 RegressionDrivingController.isAutonomousMode(P키)일 때만 외부 cmd 적용")]
    [SerializeField] private bool _useRegressionAutonomyGate = true;

    [Header("Debug (Read Only)")]
    [SerializeField] private string resolvedTopicName = "";

    private ROSConnection ros;

    void Start()
    {
        if (controller == null)
            controller = GetComponent<VehicleCmdController>() ?? GetComponentInParent<VehicleCmdController>();

        if (controller == null)
        {
            Debug.LogError("[VehicleCmdSubscriber] VehicleCmdController를 찾지 못했습니다.");
            enabled = false;
            return;
        }

        // 호환 플래그 동기화
        controller.useRegressionAutonomyGate = _useRegressionAutonomyGate;

        ros = ROSConnection.GetOrCreateInstance();
        resolvedTopicName = RosTopicNamespace.Resolve(gameObject, cmdTopicName);
        ros.Subscribe<TwistMsg>(resolvedTopicName, OnCmdReceived);
        Debug.Log($"[VehicleCmdSubscriber] Subscribed: {resolvedTopicName}");
    }

    void OnCmdReceived(TwistMsg msg)
    {
        if (msg == null || controller == null) return;
        controller.SetCommand((float)msg.linear.x, (float)msg.angular.z);
    }
}
