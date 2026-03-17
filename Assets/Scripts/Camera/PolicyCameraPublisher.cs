using UnityEngine;
using UnityEngine.Rendering;
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
    public float publishRate = 10f;

    [Header("Policy Camera Settings")]
    public int imageWidth = 200;
    public int imageHeight = 66;

    [Header("Source Camera")]
    public CameraPublisher sourceCameraPublisher;
    public bool autoFindSourceCameraPublisher = true;

    [Header("Performance (Multi-Vehicle)")]
    [Tooltip("다중 차량 시 발행 시점을 분산")]
    public bool enablePublishStagger = true;

    private ROSConnection ros;
    private Camera sourceCamera;
    private RenderTexture renderTexture;
    private float publishInterval;
    private float lastPublishTime;
    private byte[] flippedDataBuffer;

    // AsyncGPUReadback 상태 (GPU stall 제거)
    private byte[] latestPolicyRgbData;
    private bool hasPolicyData = false;
    private bool policyReadbackInProgress = false;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        topicName = RosTopicNamespace.Resolve(gameObject, topicName);
        ros.RegisterPublisher<ImageMsg>(topicName, queue_size: 1);

        if (autoFindSourceCameraPublisher && sourceCameraPublisher == null)
            sourceCameraPublisher = GetComponent<CameraPublisher>()
                ?? GetComponentInParent<CameraPublisher>()
                ?? FindObjectOfType<CameraPublisher>();

        renderTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.ARGB32);
        renderTexture.Create();
        flippedDataBuffer = new byte[imageWidth * imageHeight * 3];

        publishInterval = 1f / Mathf.Max(1f, publishRate);
        lastPublishTime = Time.time;

        if (enablePublishStagger)
        {
            float stagger = Mathf.Abs(gameObject.GetInstanceID() % 97) / 97f * publishInterval;
            lastPublishTime = Time.time + stagger - publishInterval;
        }
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
        if (renderTexture == null)
            return;

        // CameraPublisher의 렌더 결과를 정책 해상도로 다운스케일 (추가 씬 렌더 불필요)
        RenderTexture sourceRT = sourceCameraPublisher != null ? sourceCameraPublisher.GetRenderTexture() : null;
        if (sourceRT != null)
        {
            Graphics.Blit(sourceRT, renderTexture);
        }
        else if (sourceCamera != null)
        {
            // fallback: 소스 RT가 없으면 직접 렌더
            RenderTexture prevTarget = sourceCamera.targetTexture;
            sourceCamera.targetTexture = renderTexture;
            sourceCamera.Render();
            sourceCamera.targetTexture = prevTarget;
        }
        else
        {
            return;
        }

        // ReadPixels → AsyncGPUReadback (GPU stall 제거)
        if (!policyReadbackInProgress)
        {
            policyReadbackInProgress = true;
            AsyncGPUReadback.Request(renderTexture, 0, TextureFormat.RGB24, OnPolicyReadbackComplete);
        }

        // 이전 프레임 완료 데이터로 발행 (1프레임 지연, 10Hz 기준 영향 없음)
        if (!hasPolicyData)
            return;

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
            data = latestPolicyRgbData
        };

        ros.Publish(topicName, imageMsg);
    }

    void OnPolicyReadbackComplete(AsyncGPUReadbackRequest request)
    {
        policyReadbackInProgress = false;

        if (request.hasError)
            return;

        var data = request.GetData<byte>();
        if (latestPolicyRgbData == null || latestPolicyRgbData.Length != data.Length)
            latestPolicyRgbData = new byte[data.Length];

        int rowBytes = imageWidth * 3;
        for (int y = 0; y < imageHeight; y++)
        {
            int srcRow = (imageHeight - 1 - y) * rowBytes;
            int dstRow = y * rowBytes;
            for (int x = 0; x < rowBytes; x++)
                latestPolicyRgbData[dstRow + x] = data[srcRow + x];
        }

        hasPolicyData = true;
    }

    byte[] ConvertToRGB(byte[] rgbData)
    {
        int rowBytes = imageWidth * 3;

        for (int y = 0; y < imageHeight; y++)
        {
            int srcRow = (imageHeight - 1 - y) * rowBytes;
            int dstRow = y * rowBytes;
            System.Buffer.BlockCopy(rgbData, srcRow, flippedDataBuffer, dstRow, rowBytes);
        }

        return flippedDataBuffer;
    }

    void OnDestroy()
    {
        if (renderTexture != null)
        {
            renderTexture.Release();
            Destroy(renderTexture);
        }
    }
}
