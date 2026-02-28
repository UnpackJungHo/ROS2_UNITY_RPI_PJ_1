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
    public float publishRate = 30f;

    [Header("Camera Settings")]
    public int imageWidth = 640;
    public int imageHeight = 480;

    [Tooltip("카메라 X축 회전 각도 (피치)")]
    [Range(-90f, 90f)]
    public float cameraXRotation = 0f;

    [Header("Camera Reference")]
    [Tooltip("카메라가 부착된 Transform을 직접 할당하세요 (camera_link)")]
    public Transform cameraTransform;

    [Header("FrontView Camera Source")]
    [Tooltip("FrontView/ROS 퍼블리싱에 사용할 카메라. 비워두면 Main Camera를 자동 사용합니다.")]
    public Camera frontViewCamera;

    [Header("Camera Stabilization (흔들림 방지)")]
    [Tooltip("카메라 안정화 활성화 - 물리 시뮬레이션 흔들림 방지")]
    public bool enableStabilization = true;

    [Tooltip("위치 안정화 강도 (낮을수록 부드러움)")]
    [Range(0.01f, 0.5f)]
    public float positionSmoothTime = 0.08f;

    [Tooltip("회전 안정화 강도 (낮을수록 부드러움)")]
    [Range(0.01f, 0.5f)]
    public float rotationSmoothTime = 0.06f;

    [Tooltip("수직 흔들림 추가 안정화")]
    public bool stabilizeVertical = true;

    [Tooltip("롤(좌우 기울기) 안정화")]
    public bool stabilizeRoll = true;

    private ROSConnection ros;
    private Camera cam;
    private RenderTexture renderTexture;
    private Texture2D texture2D;
    private float publishInterval;
    private float lastPublishTime;
    private CameraStabilizer stabilizer;
    private bool createdStabilizerAtRuntime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        topicName = RosTopicNamespace.Resolve(gameObject, topicName);
        ros.RegisterPublisher<ImageMsg>(topicName);

        if (cameraTransform == null)
        {
            Debug.LogError("[CameraPublisher] cameraTransform이 할당되지 않았습니다! Inspector에서 camera_link를 할당하세요.");
            cameraTransform = transform;
        }

        SetupCamera();

        publishInterval = 1f / publishRate;
        lastPublishTime = Time.time;
    }

    void SetupCamera()
    {
        ResolveFrontViewCamera();
        if (cam == null)
        {
            Debug.LogError("[CameraPublisher] 사용할 Camera를 찾지 못했습니다. frontViewCamera 또는 Main Camera를 확인하세요.");
            return;
        }

        cam.nearClipPlane = 0.1f;
        cam.farClipPlane = 100f;
        cam.fieldOfView = 60f;

        SetupStabilization();
        SetupRenderResources();

        int vehicleLayer = LayerMask.NameToLayer("RLVehicle");
        if (vehicleLayer >= 0)
            cam.cullingMask &= ~(1 << vehicleLayer);
    }

    void ResolveFrontViewCamera()
    {
        if (frontViewCamera == null && Camera.main != null)
            frontViewCamera = Camera.main;

        if (frontViewCamera == null)
        {
            GameObject mainCameraObj = GameObject.Find("Main Camera");
            if (mainCameraObj != null)
                frontViewCamera = mainCameraObj.GetComponent<Camera>();
        }

        if (frontViewCamera == null && cameraTransform != null)
            frontViewCamera = cameraTransform.GetComponent<Camera>();

        cam = frontViewCamera;
    }

    void SetupStabilization()
    {
        if (cam == null)
            return;

        if (enableStabilization)
        {
            stabilizer = cam.GetComponent<CameraStabilizer>();
            if (stabilizer == null)
            {
                stabilizer = cam.gameObject.AddComponent<CameraStabilizer>();
                createdStabilizerAtRuntime = true;
            }

            stabilizer.enabled = true;
            stabilizer.targetTransform = cameraTransform;
            stabilizer.positionSmoothTime = positionSmoothTime;
            stabilizer.rotationSmoothTime = rotationSmoothTime;
            stabilizer.stabilizeVertical = stabilizeVertical;
            stabilizer.stabilizeRoll = stabilizeRoll;
            stabilizer.stabilizePitch = false;
            stabilizer.targetXRotation = cameraXRotation;
        }
        else
        {
            if (stabilizer != null)
                stabilizer.enabled = false;

            if (cameraTransform != null)
            {
                cam.transform.position = cameraTransform.position;
                cam.transform.rotation = cameraTransform.rotation * Quaternion.Euler(cameraXRotation, 0f, 0f);
            }
        }
    }

    void SetupRenderResources()
    {
        if (renderTexture != null)
        {
            renderTexture.Release();
            Destroy(renderTexture);
            renderTexture = null;
        }

        if (texture2D != null)
        {
            Destroy(texture2D);
            texture2D = null;
        }

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
        if (cam == null || renderTexture == null || texture2D == null)
            return;

        RenderTexture prevTarget = cam.targetTexture;

        cam.targetTexture = renderTexture;
        cam.Render();

        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        texture2D.Apply();
        RenderTexture.active = null;

        cam.targetTexture = prevTarget;

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
                flippedData[dstRow + x] = rgbData[srcRow + x];
        }

        return flippedData;
    }

    // ========== 외부 접근용 메서드들 ==========

    public Camera GetCamera()
    {
        return cam;
    }

    public RenderTexture GetRenderTexture()
    {
        return renderTexture;
    }

    public void RestoreRenderTexture()
    {
        if (cam != null && renderTexture != null)
            cam.targetTexture = renderTexture;
    }

    public void RenderToScreen()
    {
        if (cam != null)
            cam.targetTexture = null;
    }

    void OnDestroy()
    {
        if (renderTexture != null)
        {
            renderTexture.Release();
            Destroy(renderTexture);
            renderTexture = null;
        }

        if (texture2D != null)
        {
            Destroy(texture2D);
            texture2D = null;
        }

        if (createdStabilizerAtRuntime && stabilizer != null)
            Destroy(stabilizer);
    }
}
