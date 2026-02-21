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
        if (enableStabilization)
        {
            // 안정화 모드: 물리와 분리된 새 카메라 오브젝트 생성
            SetupStabilizedCamera();
        }
        else
        {
            // 기존 방식: cameraTransform에 직접 카메라 추가
            SetupDirectCamera();
        }
    }

    /// <summary>
    /// 안정화된 카메라 설정 - 물리 오브젝트와 분리하여 흔들림 방지
    /// </summary>
    void SetupStabilizedCamera()
    {
        // 1. 새로운 독립 카메라 오브젝트 생성 (물리 계층과 분리)
        GameObject stabilizedCamObj = new GameObject("StabilizedCamera_Publisher");
        
        // 2. 카메라 컴포넌트 추가
        cam = stabilizedCamObj.AddComponent<Camera>();
        cam.nearClipPlane = 0.1f;
        cam.farClipPlane = 100f;
        cam.fieldOfView = 60f;

        // 3. 안정화 스크립트 추가 및 설정
        stabilizer = stabilizedCamObj.AddComponent<CameraStabilizer>();
        stabilizer.targetTransform = cameraTransform;
        stabilizer.positionSmoothTime = positionSmoothTime;
        stabilizer.rotationSmoothTime = rotationSmoothTime;
        stabilizer.stabilizeVertical = stabilizeVertical;
        stabilizer.stabilizeRoll = stabilizeRoll;
        stabilizer.stabilizePitch = false; // 피치는 전방 시야 확보를 위해 기본 off
        stabilizer.targetXRotation = cameraXRotation; // X축 회전 각도 전달

        // 4. 렌더 텍스처 설정
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.ARGB32);
        renderTexture.Create();
        texture2D = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        cam.targetTexture = renderTexture;

        // 5. 차체(RLVehicle 레이어) 제외 - 카메라가 자기 폴/본체를 찍지 않도록
        int vehicleLayer = LayerMask.NameToLayer("RLVehicle");
        if (vehicleLayer >= 0)
            cam.cullingMask &= ~(1 << vehicleLayer);

        Debug.Log("[CameraPublisher] Stabilized camera created - physics shake will be filtered");
    }

    /// <summary>
    /// 기존 방식의 카메라 설정 - cameraTransform에 직접 부착
    /// </summary>
    void SetupDirectCamera()
    {
        GameObject camObj = cameraTransform.gameObject;

        cam = camObj.GetComponent<Camera>();
        if (cam == null)
        {
            cam = camObj.AddComponent<Camera>();
        }

        cam.nearClipPlane = 0.1f;
        cam.farClipPlane = 100f;
        cam.fieldOfView = 60f;

        // X축 회전 적용
        cam.transform.localRotation = Quaternion.Euler(cameraXRotation, 0f, 0f);

        renderTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.ARGB32);
        renderTexture.Create();

        texture2D = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);

        cam.targetTexture = renderTexture;

        // 차체(RLVehicle 레이어) 제외 - 카메라 폴/본체가 찍히지 않도록
        int vehicleLayerDirect = LayerMask.NameToLayer("RLVehicle");
        if (vehicleLayerDirect >= 0)
            cam.cullingMask &= ~(1 << vehicleLayerDirect);
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

        // 현재 targetTexture 상태 보존 (RenderToScreen 모드일 수 있음)
        RenderTexture prevTarget = cam.targetTexture;

        // 일시적으로 RT에 렌더링 → 캡처 → 원래 상태 복원
        cam.targetTexture = renderTexture;
        cam.Render();

        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        texture2D.Apply();
        RenderTexture.active = null;

        // 원래 상태 복원 (null이면 화면 직접 렌더링 유지)
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
            {
                flippedData[dstRow + x] = rgbData[srcRow + x];
            }
        }

        return flippedData;
    }

    // ========== 외부 접근용 메서드들 (AMRViewController 등에서 사용) ==========

    /// <summary>
    /// 실제 사용 중인 카메라 반환 (안정화 모드/일반 모드 모두 지원)
    /// </summary>
    public Camera GetCamera()
    {
        return cam;
    }

    /// <summary>
    /// 현재 사용 중인 RenderTexture 반환
    /// </summary>
    public RenderTexture GetRenderTexture()
    {
        return renderTexture;
    }

    /// <summary>
    /// RenderTexture를 카메라에 다시 할당 (FrontView에서 복귀 시 사용)
    /// </summary>
    public void RestoreRenderTexture()
    {
        if (cam != null && renderTexture != null)
        {
            cam.targetTexture = renderTexture;
        }
    }

    /// <summary>
    /// RenderTexture 해제하여 화면에 직접 렌더링 (FrontView 모드)
    /// </summary>
    public void RenderToScreen()
    {
        if (cam != null)
        {
            cam.targetTexture = null;
        }
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
        // 안정화 모드로 생성된 카메라 오브젝트 정리
        if (stabilizer != null && stabilizer.gameObject != null)
        {
            Destroy(stabilizer.gameObject);
        }
    }
}
