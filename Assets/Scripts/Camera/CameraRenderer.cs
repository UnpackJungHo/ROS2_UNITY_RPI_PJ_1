using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// 카메라 설정, RenderTexture 관리를 담당하는 로직 컴포넌트.
/// ROS 발행은 CameraPublisher가 담당한다.
/// </summary>
public class CameraRenderer : MonoBehaviour
{
    [Header("Camera Settings")]
    public int imageWidth = 200;
    public int imageHeight = 66;

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

    [Header("Render Rate")]
    public float renderRate = 10f;

    private Camera cam;
    private RenderTexture renderTexture;
    private bool createdRuntimeFrontCameraAtRuntime;
    private float renderInterval;
    private float lastRenderTime;
    private byte[] flippedDataBuffer;

    // AsyncGPUReadback 상태 (GPU stall 제거)
    private byte[] latestRgbData;
    private bool hasValidData = false;
    private bool readbackInProgress = false;

    public Camera GetCamera() => cam;
    public RenderTexture GetRenderTexture() => renderTexture;

    /// <summary>
    /// 런타임에 렌더 해상도를 변경한다. RenderTexture/Texture2D를 재생성한다.
    /// </summary>
    public void SetResolution(int width, int height)
    {
        if (imageWidth == width && imageHeight == height)
            return;

        imageWidth = width;
        imageHeight = height;

        if (cam != null)
            SetupRenderResources();
    }

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
            Debug.Log("[CameraRenderer] frontViewCamera가 비어있어 카메라 렌더링을 건너뜁니다.");
            return;
        }

        cam.nearClipPlane = 0.1f;
        cam.farClipPlane = 100f;
        cam.fieldOfView = 60f;

        if (cameraTransform != null)
        {
            cam.transform.position = cameraTransform.position;
            cam.transform.rotation = cameraTransform.rotation * Quaternion.Euler(cameraXRotation, 0f, 0f);
        }

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
        if (frontViewCamera == null)
        {
            cam = null;
            return;
        }

        if (preferDedicatedRuntimeCamera && cameraTransform != null)
        {
            frontViewCamera = GetOrCreateRuntimeFrontCamera(frontViewCamera);
            cam = frontViewCamera;
            return;
        }

        cam = frontViewCamera;
    }

    void SetupRenderResources()
    {
        if (renderTexture != null)
        {
            renderTexture.Release();
            Object.Destroy(renderTexture);
            renderTexture = null;
        }

        renderTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.ARGB32);
        renderTexture.Create();

        flippedDataBuffer = new byte[imageWidth * imageHeight * 3];
        cam.targetTexture = renderTexture;
        cam.enabled = false;

        // 해상도 변경 시 비동기 상태 초기화
        hasValidData = false;
        readbackInProgress = false;
        latestRgbData = null;
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
    /// RenderTexture에서 RGB 데이터를 비동기로 요청하고 이전 프레임의 완료된 데이터를 반환한다.
    /// GPU stall 없이 1프레임 지연된 데이터를 제공한다 (10Hz 캡처 기준 영향 없음).
    /// </summary>
    public byte[] ReadRgbData()
    {
        if (cam == null || renderTexture == null)
            return null;

        if (!readbackInProgress)
        {
            readbackInProgress = true;
            AsyncGPUReadback.Request(renderTexture, 0, TextureFormat.RGB24, OnAsyncReadbackComplete);
        }

        return hasValidData ? latestRgbData : null;
    }

    void OnAsyncReadbackComplete(AsyncGPUReadbackRequest request)
    {
        readbackInProgress = false;

        if (request.hasError)
            return;

        var data = request.GetData<byte>();
        if (latestRgbData == null || latestRgbData.Length != data.Length)
            latestRgbData = new byte[data.Length];

        // 수직 반전 복사 (GPU 기본값은 하단 원점, ConvertToRGB 로직 통합)
        int rowBytes = imageWidth * 3;
        for (int y = 0; y < imageHeight; y++)
        {
            int srcRow = (imageHeight - 1 - y) * rowBytes;
            int dstRow = y * rowBytes;
            for (int x = 0; x < rowBytes; x++)
                latestRgbData[dstRow + x] = data[srcRow + x];
        }

        hasValidData = true;
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

        if (createdRuntimeFrontCameraAtRuntime && cam != null)
            Destroy(cam.gameObject);
    }
}
