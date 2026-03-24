using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// SingleUltrasonicSensor의 ROS 토픽 입출력 브리지.
/// - Raw 토픽 발행 (개별 센서 데이터)
/// - 외부 토픽 구독 (실차 센서 연동)
/// Logic은 SingleUltrasonicSensor에 위임하고, 이 클래스는 ROS I/O만 담당한다.
/// </summary>
public class SingleUltrasonicSensorRosBridge : MonoBehaviour
{
    [Header("Sensor Reference")]
    [Tooltip("같은 GameObject 또는 부모에서 자동 탐색")]
    public SingleUltrasonicSensor sensor;

    [Header("External Topic Input (실차 센서 토픽 연동)")]
    [Tooltip("true면 레이캐스트 대신 개별 센서 ROS 토픽을 우선 사용")]
    public bool useExternalTopicInput = false;
    [Tooltip("true면 inputTopicName이 비어있을 때 sensorPosition 기반 기본 토픽을 사용")]
    public bool autoTopicFromSensorPosition = true;
    [Tooltip("개별 초음파 입력 토픽 (std_msgs/Float32MultiArray: [distance_m, confidence(optional), angle_deg(optional)])")]
    public string inputTopicName = "";
    [Tooltip("이 시간(초) 이상 새 메시지가 없으면 stale로 판단")]
    public float externalDataTimeoutSec = 0.5f;
    [Tooltip("외부 입력이 stale일 때 시뮬레이션 레이캐스트로 fallback")]
    public bool fallbackToRaycastWhenExternalStale = true;
    [Tooltip("confidence가 메시지에 없을 때 사용할 기본값")]
    [Range(0f, 1f)]
    public float defaultExternalConfidence = 1f;

    [Header("External Topic Debug (Read Only)")]
    [SerializeField] private string resolvedInputTopicName = "";
    [SerializeField] private bool hasExternalMessage = false;
    [SerializeField] private float lastExternalMessageTime = -999f;

    [Header("Raw Topic Output (개별 센서 발행)")]
    [Tooltip("true면 이 센서의 raw 토픽을 발행")]
    public bool publishRawTopic = true;
    [Tooltip("true면 outputTopicName이 비어있을 때 sensorPosition 기반 기본 토픽을 사용")]
    public bool autoOutputTopicFromSensorPosition = true;
    [Tooltip("개별 초음파 출력 토픽 (std_msgs/Float32MultiArray: [distance_m, confidence, angle_deg])")]
    public string outputTopicName = "";

    [Header("Raw Topic Output Debug (Read Only)")]
    [SerializeField] private string resolvedOutputTopicName = "";

    private ROSConnection ros;
    private bool rawPublisherReady = false;

    void Start()
    {
        if (sensor == null)
            sensor = GetComponent<SingleUltrasonicSensor>() ?? GetComponentInParent<SingleUltrasonicSensor>();

        if (sensor == null)
        {
            Debug.LogError($"[UltrasonicRosBridge] SingleUltrasonicSensor를 찾지 못했습니다.");
            enabled = false;
            return;
        }

        if (useExternalTopicInput)
            SetupExternalTopicInput();
        if (publishRawTopic)
            SetupRawTopicOutput();
    }

    void Update()
    {
        if (sensor == null) return;

        if (useExternalTopicInput)
        {
            bool externalFresh = hasExternalMessage &&
                                 (Time.time - lastExternalMessageTime) <= Mathf.Max(0.02f, externalDataTimeoutSec);
            sensor.SetExternalInputState(externalFresh, fallbackToRaycastWhenExternalStale);
        }
        else
        {
            sensor.SetExternalInputState(false, true);
        }

        if (publishRawTopic && rawPublisherReady)
            PublishRawTopic();
    }

    void SetupExternalTopicInput()
    {
        ros = ROSConnection.GetOrCreateInstance();

        if (autoTopicFromSensorPosition && string.IsNullOrWhiteSpace(inputTopicName))
            inputTopicName = GetDefaultTopicName();

        if (string.IsNullOrWhiteSpace(inputTopicName))
        {
            Debug.LogWarning($"[UltrasonicRosBridge-{sensor.SensorName}] inputTopicName이 비어 있어 외부 입력 모드를 비활성화합니다.");
            useExternalTopicInput = false;
            return;
        }

        resolvedInputTopicName = RosTopicNamespace.Resolve(gameObject, inputTopicName);
        ros.Subscribe<Float32MultiArrayMsg>(resolvedInputTopicName, OnExternalData);
        Debug.Log($"[UltrasonicRosBridge-{sensor.SensorName}] External topic subscribed: {resolvedInputTopicName}");
    }

    void SetupRawTopicOutput()
    {
        ros = ROSConnection.GetOrCreateInstance();

        if (autoOutputTopicFromSensorPosition && string.IsNullOrWhiteSpace(outputTopicName))
            outputTopicName = GetDefaultTopicName();

        if (string.IsNullOrWhiteSpace(outputTopicName))
        {
            Debug.LogWarning($"[UltrasonicRosBridge-{sensor.SensorName}] outputTopicName이 비어 있어 raw 발행을 비활성화합니다.");
            return;
        }

        resolvedOutputTopicName = RosTopicNamespace.Resolve(gameObject, outputTopicName);
        if (!string.IsNullOrWhiteSpace(resolvedInputTopicName) && resolvedInputTopicName == resolvedOutputTopicName)
        {
            Debug.LogWarning($"[UltrasonicRosBridge-{sensor.SensorName}] input/output topic이 동일({resolvedOutputTopicName})하여 self-loop 방지를 위해 raw 발행을 비활성화합니다.");
            return;
        }

        ros.RegisterPublisher<Float32MultiArrayMsg>(resolvedOutputTopicName);
        rawPublisherReady = true;
    }

    string GetDefaultTopicName()
    {
        if (sensor == null) return "/ultrasonic/unknown";
        return sensor.sensorPosition switch
        {
            SingleUltrasonicSensor.SensorPosition.FrontLeft => "/ultrasonic/fl",
            SingleUltrasonicSensor.SensorPosition.FrontRight => "/ultrasonic/fr",
            SingleUltrasonicSensor.SensorPosition.FrontCenter => "/ultrasonic/fc",
            SingleUltrasonicSensor.SensorPosition.RearLeft => "/ultrasonic/rl",
            SingleUltrasonicSensor.SensorPosition.RearRight => "/ultrasonic/rr",
            SingleUltrasonicSensor.SensorPosition.RearCenter => "/ultrasonic/rc",
            SingleUltrasonicSensor.SensorPosition.SideLeft => "/ultrasonic/sl",
            SingleUltrasonicSensor.SensorPosition.SideRight => "/ultrasonic/sr",
            _ => "/ultrasonic/unknown"
        };
    }

    void PublishRawTopic()
    {
        if (ros == null || sensor == null) return;

        Float32MultiArrayMsg msg = new Float32MultiArrayMsg
        {
            data = new float[]
            {
                sensor.HasDetection ? sensor.Distance : -1f,
                sensor.Confidence,
                sensor.DetectedAngle
            }
        };
        ros.Publish(resolvedOutputTopicName, msg);
    }

    void OnExternalData(Float32MultiArrayMsg msg)
    {
        hasExternalMessage = true;
        lastExternalMessageTime = Time.time;

        if (sensor != null)
            sensor.ApplyExternalData(msg != null ? msg.data : null);
    }
}
