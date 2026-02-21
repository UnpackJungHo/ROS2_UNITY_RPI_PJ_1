using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

/// <summary>
/// 정책 추론 전용 카메라 토픽 퍼블리셔.
/// 기존 CameraPublisher가 사용하는 실제 카메라를 그대로 재사용하되,
/// 정책 입력 해상도(기본 200x66)로 별도 렌더링해서 발행한다.
/// </summary>
public class PolicyCameraPublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    public string topicName = "/camera/policy/image_raw";
    public string frameId = "camera_link";
    public float publishRate = 20f;

    [Header("Policy Camera Settings")]
    public int imageWidth = 200;
    public int imageHeight = 66;

    [Header("Source Camera")]
    public CameraPublisher sourceCameraPublisher;
    public bool autoFindSourceCameraPublisher = true;

    private ROSConnection ros;
    private Camera sourceCamera;
    private RenderTexture renderTexture;
    private Texture2D texture2D;
    private float publishInterval;
    private float lastPublishTime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        topicName = RosTopicNamespace.Resolve(gameObject, topicName);
        ros.RegisterPublisher<ImageMsg>(topicName);

        if (autoFindSourceCameraPublisher && sourceCameraPublisher == null)
            sourceCameraPublisher = GetComponent<CameraPublisher>()
                ?? GetComponentInParent<CameraPublisher>()
                ?? FindObjectOfType<CameraPublisher>();

        renderTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.ARGB32);
        renderTexture.Create();
        texture2D = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);

        publishInterval = 1f / Mathf.Max(1f, publishRate);
        lastPublishTime = Time.time;
    }

    void Update()
    {
        if (sourceCamera == null && sourceCameraPublisher != null)
            sourceCamera = sourceCameraPublisher.GetCamera();

        if (sourceCamera == null)
            return;

        if (Time.time - lastPublishTime >= publishInterval)
        {
            PublishImage();
            lastPublishTime = Time.time;
        }
    }

    void PublishImage()
    {
        if (sourceCamera == null || renderTexture == null)
            return;

        RenderTexture prevTarget = sourceCamera.targetTexture;
        sourceCamera.targetTexture = renderTexture;
        sourceCamera.Render();

        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        texture2D.Apply();
        RenderTexture.active = null;
        sourceCamera.targetTexture = prevTarget;

        byte[] imageData = texture2D.GetRawTextureData();
        byte[] rgbData = ConvertToRGB(imageData);

        ImageMsg imageMsg = new ImageMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1f) * 1e9f)
                },
                frame_id = frameId
            },
            height = (uint)imageHeight,
            width = (uint)imageWidth,
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)(imageWidth * 3),
            data = rgbData
        };

        ros.Publish(topicName, imageMsg);
    }

    byte[] ConvertToRGB(byte[] rgbData)
    {
        byte[] flippedData = new byte[imageWidth * imageHeight * 3];

        for (int y = 0; y < imageHeight; y++)
        {
            int srcRow = (imageHeight - 1 - y) * imageWidth * 3;
            int dstRow = y * imageWidth * 3;
            for (int x = 0; x < imageWidth * 3; x++)
                flippedData[dstRow + x] = rgbData[srcRow + x];
        }

        return flippedData;
    }

    void OnDestroy()
    {
        if (renderTexture != null)
        {
            renderTexture.Release();
            Destroy(renderTexture);
        }

        if (texture2D != null)
            Destroy(texture2D);
    }
}
