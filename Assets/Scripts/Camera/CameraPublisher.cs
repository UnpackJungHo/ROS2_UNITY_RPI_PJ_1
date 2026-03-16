using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

/// <summary>
/// CameraRenderer의 ROS 토픽 발행 브리지.
/// ImageMsg를 발행한다.
/// </summary>
public class CameraPublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    public string topicName = "/camera/image_raw";
    public string frameId = "camera_link";
    public float publishRate = 10f;

    [Header("Renderer Reference")]
    [Tooltip("같은 GameObject 또는 부모에서 자동 탐색")]
    public CameraRenderer cameraRenderer;

    [Header("ROS Publish Control")]
    [Tooltip("false면 카메라 렌더링은 유지하되 ROS 발행만 차단 (Train 모드 등)")]
    public bool rosPublishingEnabled = true;

    [Header("Performance (Multi-Vehicle)")]
    [Tooltip("다중 차량 시 발행 시점을 분산")]
    public bool enablePublishStagger = true;

    private ROSConnection ros;
    private float publishInterval;
    private float lastPublishTime;

    void Start()
    {
        if (cameraRenderer == null)
            cameraRenderer = GetComponent<CameraRenderer>() ?? GetComponentInParent<CameraRenderer>();

        if (cameraRenderer == null)
        {
            Debug.LogError("[CameraPublisher] CameraRenderer를 찾지 못했습니다.");
            enabled = false;
            return;
        }

        ros = ROSConnection.GetOrCreateInstance();
        topicName = RosTopicNamespace.Resolve(gameObject, topicName);
        ros.RegisterPublisher<ImageMsg>(topicName, queue_size: 1);

        publishInterval = 1f / publishRate;
        lastPublishTime = Time.time;

        if (enablePublishStagger)
        {
            float stagger = Mathf.Abs(gameObject.GetInstanceID() % 97) / 97f * publishInterval;
            lastPublishTime = Time.time + stagger - publishInterval;
        }
    }

    void Update()
    {
        if (Time.time - lastPublishTime >= publishInterval)
        {
            if (rosPublishingEnabled)
                PublishImage();
            lastPublishTime = Time.time;
        }
    }

    void PublishImage()
    {
        if (cameraRenderer == null) return;

        byte[] rgbData = cameraRenderer.ReadRgbData();
        if (rgbData == null) return;

        ImageMsg imageMsg = new ImageMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1) * 1e9)
                },
                frame_id = frameId
            },
            height = (uint)cameraRenderer.imageHeight,
            width = (uint)cameraRenderer.imageWidth,
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)(cameraRenderer.imageWidth * 3),
            data = rgbData
        };

        ros.Publish(topicName, imageMsg);
    }

    // ========== 하위 호환용 프로퍼티/메서드 (VehicleViewProvider, PolicyCameraPublisher 등에서 사용) ==========

    public Transform cameraTransform => cameraRenderer != null ? cameraRenderer.cameraTransform : null;
    public float cameraXRotation => cameraRenderer != null ? cameraRenderer.cameraXRotation : 0f;

    public Camera GetCamera()
    {
        return cameraRenderer != null ? cameraRenderer.GetCamera() : null;
    }

    public RenderTexture GetRenderTexture()
    {
        return cameraRenderer != null ? cameraRenderer.GetRenderTexture() : null;
    }

    public void RestoreRenderTexture()
    {
        if (cameraRenderer != null)
            cameraRenderer.RestoreRenderTexture();
    }

    public void RenderToScreen()
    {
        if (cameraRenderer != null)
            cameraRenderer.RenderToScreen();
    }
}
