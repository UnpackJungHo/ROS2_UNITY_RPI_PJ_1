using UnityEngine;

/// <summary>
/// 카메라 설정, RenderTexture 관리, Stabilizer를 담당하는 로직 컴포넌트.
/// ROS 발행은 CameraPublisher가 담당한다.
/// </summary>
public class CameraRenderer : MonoBehaviour
{
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
    [Tooltip("FrontView에 사용할 카메라. 비워두면 Main Camera를 자동 사용합니다.")]
    public Camera frontViewCamera;

    [Tooltip("true면 Main Camera를 직접 재사용하지 않고 런타임 전용 front camera를 생성합니다.")]
    public bool preferDedicatedRuntimeCamera = true;

    [Header("Performance (Multi-Vehicle)")]
    [Tooltip("다중 차량 시 렌더 시점을 분산하여 프레임당 GPU 부하를 평활화")]
    public bool enableRenderStagger = true;

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

    [Header("Render Rate")]
    public float renderRate = 10f;

    private Camera cam;
    private RenderTexture renderTexture;
    private Texture2D texture2D;
    private CameraStabilizer stabilizer;
    private bool createdStabilizerAtRuntime;
    private bool createdRuntimeFrontCameraAtRuntime;
    private float renderInterval;
    private float lastRenderTime;
    private byte[] flippedDataBuffer;

    public Camera GetCamera() => cam;
    public RenderTexture GetRenderTexture() => renderTexture;

    void Start()
    {
        if (cameraTransform == null)
        {
            Debug.LogError("[CameraRenderer] cameraTransform이 할당되지 않았습니다! Inspector에서 camera_link를 할당하세요.");
            cameraTransform = transform;
        }

        SetupCamera();

        renderInterval = 1f / renderRate;
        lastRenderTime = Time.time;

        if (enableRenderStagger)
        {
            float stagger = Mathf.Abs(gameObject.GetInstanceID() % 97) / 97f * renderInterval;
            lastRenderTime = Time.time + stagger - renderInterval;
        }
    }

    void SetupCamera()
    {
        Camera originalFrontCamera = frontViewCamera;
        ResolveFrontViewCamera();
        if (cam == null)
        {
            Debug.LogError("[CameraRenderer] 사용할 Camera를 찾지 못했습니다. frontViewCamera 또는 Main Camera를 확인하세요.");
            return;
        }

        cam.nearClipPlane = 0.1f;
        cam.farClipPlane = 100f;
        cam.fieldOfView = 60f;

        SetupStabilization();
        SetupRenderResources();

        int vehicleLayer = LayerMask.NameToLayer("RLVehicle");
        if (vehicleLayer >= 0)
        {
            cam.cullingMask &= ~(1 << vehicleLayer);
            if (originalFrontCamera != null && originalFrontCamera != cam)
                originalFrontCamera.cullingMask &= ~(1 << vehicleLayer);
        }
    }

    void ResolveFrontViewCamera()
    {
        Camera templateCamera = frontViewCamera != null ? frontViewCamera : Camera.main;

        if (preferDedicatedRuntimeCamera && cameraTransform != null)
        {
            frontViewCamera = GetOrCreateRuntimeFrontCamera(templateCamera);
            cam = frontViewCamera;
            return;
        }

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
            Object.Destroy(renderTexture);
            renderTexture = null;
        }

        if (texture2D != null)
        {
            Object.Destroy(texture2D);
            texture2D = null;
        }

        renderTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.ARGB32);
        renderTexture.Create();

        texture2D = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        flippedDataBuffer = new byte[imageWidth * imageHeight * 3];
        cam.targetTexture = renderTexture;
        cam.enabled = false;
    }

    Camera GetOrCreateRuntimeFrontCamera(Camera templateCamera)
    {
        string runtimeCameraName = $"{gameObject.name}_{gameObject.GetInstanceID()}_PublishedFrontCamera";
        GameObject runtimeCameraObject = GameObject.Find(runtimeCameraName);

        if (runtimeCameraObject == null)
        {
            runtimeCameraObject = new GameObject(runtimeCameraName);
            createdRuntimeFrontCameraAtRuntime = true;
        }

        Camera runtimeCamera = runtimeCameraObject.GetComponent<Camera>();
        if (runtimeCamera == null)
            runtimeCamera = runtimeCameraObject.AddComponent<Camera>();

        runtimeCamera.enabled = false;
        runtimeCamera.depth = -50f;

        if (templateCamera != null)
        {
            runtimeCamera.nearClipPlane = templateCamera.nearClipPlane;
            runtimeCamera.farClipPlane = templateCamera.farClipPlane;
            runtimeCamera.fieldOfView = templateCamera.fieldOfView;
            runtimeCamera.clearFlags = templateCamera.clearFlags;
            runtimeCamera.backgroundColor = templateCamera.backgroundColor;
            runtimeCamera.cullingMask = templateCamera.cullingMask;
        }

        return runtimeCamera;
    }

    void Update()
    {
        if (Time.time - lastRenderTime >= renderInterval)
        {
            RenderFrame();
            lastRenderTime = Time.time;
        }
    }

    /// <summary>
    /// 카메라를 렌더링하여 RenderTexture에 결과를 저장한다.
    /// </summary>
    public void RenderFrame()
    {
        if (cam == null || renderTexture == null)
            return;

        RenderTexture prevTarget = cam.targetTexture;
        cam.targetTexture = renderTexture;
        cam.Render();
        cam.targetTexture = prevTarget;
    }

    /// <summary>
    /// RenderTexture에서 RGB 데이터를 읽어 반환한다.
    /// </summary>
    public byte[] ReadRgbData()
    {
        if (cam == null || renderTexture == null || texture2D == null)
            return null;

        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        texture2D.Apply();
        RenderTexture.active = null;

        byte[] imageData = texture2D.GetRawTextureData();
        return ConvertToRGB(imageData);
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

        if (createdRuntimeFrontCameraAtRuntime && cam != null)
            Destroy(cam.gameObject);
    }
}
