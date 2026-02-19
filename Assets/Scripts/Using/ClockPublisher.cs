using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Rosgraph;

/// <summary>
/// ROS 2 시뮬레이션 시간을 동기화하기 위해 /clock 토픽을 발행하는 클래스.
/// use_sim_time=true 설정 시 필수입니다.
/// </summary>
public class ClockPublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    public string clockTopic = "/clock";
    public float publishRate = 100f; // 100Hz recommended for smooth check

    private ROSConnection ros;
    private float timeElapsed;
    private float publishInterval;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        clockTopic = RosTopicNamespace.Resolve(gameObject, clockTopic);
        ros.RegisterPublisher<ClockMsg>(clockTopic);
        publishInterval = 1.0f / publishRate;
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed >= publishInterval)
        {
            PublishClock();
            // Debug.Log($"[Clock] Publishing time: {Time.time}");
            timeElapsed = 0;
        }
    }

    void PublishClock()
    {
        var tfTime = ConvertToRosTime(Time.time);
        
        ClockMsg clockMsg = new ClockMsg
        {
            clock = tfTime
        };

        ros.Publish(clockTopic, clockMsg);
    }

    private TimeMsg ConvertToRosTime(float time)
    {
        int sec = (int)time;
        uint nanosec = (uint)((time - sec) * 1e9);
        return new TimeMsg { sec = sec, nanosec = nanosec };
    }
}
