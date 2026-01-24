using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

public class CameraPublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    public string topicName = "/camera/image_raw";
    public string frameId = "camera_link";
    public float publishRate = 10f;

    [Header("Camera Settings")]
    public int imageWidth = 640;
    public int imageHeight = 480;

    [Header("Auto-Find Settings")]
    public bool autoFindCameraLink = true;
    public string cameraLinkName = "camera_link";

    private ROSConnection ros;
    private Camera cam;
    private RenderTexture renderTexture;
    private Texture2D texture2D;
    private float publishInterval;
    private float lastPublishTime;
    private Transform cameraTransform;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);

        if (autoFindCameraLink)
        {
            FindCameraLink();
        }

        SetupCamera();

        publishInterval = 1f / publishRate;
        lastPublishTime = Time.time;
    }

    void FindCameraLink()
    {
        cameraTransform = FindChildRecursive(transform, cameraLinkName);
        if (cameraTransform != null)
        {
            Debug.Log($"[CameraPublisher] Found camera link: {cameraLinkName}");
        }
        else
        {
            Debug.LogWarning($"[CameraPublisher] Camera link not found: {cameraLinkName}, using this object");
            cameraTransform = transform;
        }
    }

    Transform FindChildRecursive(Transform parent, string name)
    {
        foreach (Transform child in parent)
        {
            if (child.name == name)
                return child;

            Transform found = FindChildRecursive(child, name);
            if (found != null)
                return found;
        }
        return null;
    }

    void SetupCamera()
    {
        GameObject camObj;
        if (cameraTransform != null && cameraTransform != transform)
        {
            camObj = cameraTransform.gameObject;
        }
        else
        {
            camObj = gameObject;
        }

        cam = camObj.GetComponent<Camera>();
        if (cam == null)
        {
            cam = camObj.AddComponent<Camera>();
        }

        cam.nearClipPlane = 0.1f;
        cam.farClipPlane = 100f;
        cam.fieldOfView = 60f;

        renderTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.ARGB32);
        renderTexture.Create();

        texture2D = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);

        cam.targetTexture = renderTexture;
    }

    void Update()
    {
        if (Time.time - lastPublishTime >= publishInterval)
        {
            PublishImage();
            lastPublishTime = Time.time;
        }
    }

    void PublishImage()
    {
        if (cam == null || renderTexture == null) return;

        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        texture2D.Apply();
        RenderTexture.active = null;

        byte[] imageData = texture2D.GetRawTextureData();
        byte[] rgbData = ConvertToRGB(imageData);

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
        int pixelCount = imageWidth * imageHeight;
        byte[] flippedData = new byte[pixelCount * 3];

        for (int y = 0; y < imageHeight; y++)
        {
            int srcRow = (imageHeight - 1 - y) * imageWidth * 3;
            int dstRow = y * imageWidth * 3;

            for (int x = 0; x < imageWidth * 3; x++)
            {
                flippedData[dstRow + x] = rgbData[srcRow + x];
            }
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
        {
            Destroy(texture2D);
        }
    }
}
