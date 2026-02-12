using UnityEngine;
using UnityEngine.UI;

public class AMRViewController : MonoBehaviour
{
    [Header("Camera References")]
    [Tooltip("CameraPublisher가 사용하는 카메라")]
    public CameraPublisher cameraPublisher;

    [Header("TopView Settings")]
    [Tooltip("TopView에서 바라볼 대상 오브젝트")]
    public GameObject topViewTarget;

    [Tooltip("대상 오브젝트로부터 Y축 높이")]
    public float topViewHeight = 5f;

    [Header("BackView Settings")]
    [Tooltip("BackView에서 대상 오브젝트로부터 뒤쪽 거리")]
    public float backViewDistance = 3f;

    [Tooltip("BackView에서 대상 오브젝트로부터 Y축 높이")]
    public float backViewHeight = 1.5f;

    [Tooltip("카메라 이동 제동 시간 (낮을수록 빠름, 높을수록 부드러움)")]
    public float smoothTime = 0.1f;

    private Vector3 currentVelocity = Vector3.zero;

    [Header("UI Settings")]
    public Button front_view_button;
    public Button top_view_button;
    public Button back_view_button;
    
    [Tooltip("오른쪽 패널의 RectTransform을 연결하세요. 패널을 제외한 나머지 영역이 카메라 뷰가 됩니다.")]
    public RectTransform rightPanel;

    [Header("Viewport & FOV Settings")]
    // viewContainer는 이제 동적으로 계산되므로 Inspector에서 숨김 (또는 디버그용으로 볼 수 있음)
    private Rect viewContainer = new Rect(0f, 0f, 0.71f, 1f);

    [Tooltip("유지하고 싶은 화면 비율 (가로/세로). 예: 1.7778 (16:9)")]
    public float targetAspectRatio = 1.7778f;

    [Tooltip("카메라 시야각 (기본값: 60)")]
    public float explicitFov = 60f;

    private Camera topViewCamera;
    private Camera backViewCamera;
    private Camera publisherCamera;
    private Camera backgroundCamera; // 검은 배경용 카메라
    private ViewMode currentViewMode = ViewMode.TopView;

    private enum ViewMode
    {
        TopView,
        BackView,
        PublisherCamera
    }

    void Start()
    {
        CreateBackgroundCamera(); // 배경 카메라 생성

        // TopView용 카메라 동적 생성
        CreateTopViewCamera();

        // BackView용 카메라 동적 생성
        CreateBackViewCamera();

        // 기본 뷰로 시작
        SetViewMode(ViewMode.TopView);

        top_view_button.onClick.AddListener(() => SetViewMode(ViewMode.TopView));
        front_view_button.onClick.AddListener(() => SetViewMode(ViewMode.PublisherCamera));
        back_view_button.onClick.AddListener(() => SetViewMode(ViewMode.BackView));
    }

    void CreateBackgroundCamera()
    {
        if (backgroundCamera != null) return;

        GameObject bgObj = new GameObject("Background_Camera");
        backgroundCamera = bgObj.AddComponent<Camera>();
        backgroundCamera.depth = -10; // 다른 카메라보다 뒤에 렌더링
        backgroundCamera.clearFlags = CameraClearFlags.SolidColor;
        backgroundCamera.backgroundColor = Color.black;
        backgroundCamera.cullingMask = 0; // 아무것도 렌더링하지 않음 (배경색만)
        backgroundCamera.rect = viewContainer;
    }

    void CreateTopViewCamera()
    {
        GameObject topViewCamObj = new GameObject("TopView_Camera");
        topViewCamera = topViewCamObj.AddComponent<Camera>();
        topViewCamera.nearClipPlane = 0.1f;
        topViewCamera.farClipPlane = 100f;
        topViewCamera.fieldOfView = explicitFov;
        topViewCamera.enabled = false;
    }

    void CreateBackViewCamera()
    {
        GameObject backViewCamObj = new GameObject("BackView_Camera");
        backViewCamera = backViewCamObj.AddComponent<Camera>();
        backViewCamera.nearClipPlane = 0.1f;
        backViewCamera.farClipPlane = 100f;
        backViewCamera.fieldOfView = explicitFov;
        backViewCamera.enabled = false;
    }

    void LateUpdate()
    {
        // 0. RightPanel을 기준으로 ViewContainer 동적 계산
        if (rightPanel != null)
        {
            UpdateViewContainerFromPanel();
        }

        // 1. 적절한 뷰포트 계산 (화면 비율 유지)
        Rect letterboxRect = CalculateLetterboxRect();

        // 2. 현재 활성화된 카메라에 뷰포트 및 FOV 적용
        Camera currentCam = GetCurrentActiveCamera();
        if (currentCam != null)
        {
            currentCam.rect = letterboxRect;
            currentCam.fieldOfView = explicitFov;
        }

        // 3. 배경 카메라 뷰포트 업데이트 (컨테이너 크기 변경 대응)
        if (backgroundCamera != null)
        {
            backgroundCamera.rect = viewContainer;
        }

        // 4. 카메라 위치 업데이트 로직
        if (currentViewMode == ViewMode.TopView && topViewTarget != null && topViewCamera != null)
        {
            UpdateTopViewCamera();
        }

        if (currentViewMode == ViewMode.BackView && topViewTarget != null && backViewCamera != null)
        {
            UpdateBackViewCamera();
        }
    }

    void UpdateViewContainerFromPanel()
    {
        // UI 패널의 월드 코너 좌표 가져오기
        // corners: [0]좌하, [1]좌상, [2]우상, [3]우하
        Vector3[] corners = new Vector3[4];
        rightPanel.GetWorldCorners(corners);

        // 패널의 왼쪽 가장자리 X 좌표 (Screen Space)
        // Canvas Mode가 Overlay라면 WorldCorner가 곧 Screen 좌표와 유사하거나 비례함
        // Camera.WorldToScreenPoint가 필요할 수도 있지만, Overlay Canvas라면 transform.position이 픽셀 단위임.
        
        // 하지만 Canvas Scaler가 있을 수 있으므로 안전하게 처리:
        // Overlay Canvas의 경우 WorldCorners는 Screen Coordinates와 맵핑됨.
        
        float panelLeftX = corners[0].x; // 패널의 왼쪽 끝 X 좌표
        
        // 전체 화면 너비 대비 비율 계산 (0~1)
        float normalizedWidth = Mathf.Clamp01(panelLeftX / Screen.width);

        // ViewContainer 업데이트 (왼쪽 0부터 패널 시작점까지)
        viewContainer = new Rect(0f, 0f, normalizedWidth, 1f);
    }

    Rect CalculateLetterboxRect()
    {
        float sw = Screen.width;
        float sh = Screen.height;
        
        // 컨테이너의 실제 화면상 비율 계산
        float containerRatio = (viewContainer.width * sw) / (viewContainer.height * sh);
        
        Rect rect = new Rect(0, 0, 0, 0);

        if (containerRatio > targetAspectRatio)
        {
            // 컨테이너가 더 넓음 -> 높이를 맞추고 좌우에 여백 (Pillarbox)
            // (하지만 현재 설정상으로는 컨테이너가 좁으므로 이 분기는 잘 안 탈 것임)
            rect.height = viewContainer.height;
            rect.width = rect.height * (sh / sw) * targetAspectRatio;
            rect.y = viewContainer.y;
            rect.x = viewContainer.x + (viewContainer.width - rect.width) * 0.5f;
        }
        else
        {
            // 컨테이너가 더 좁거나 김 -> 너비를 맞추고 위아래 여백 (Letterbox)
            // (사용자의 케이스: 16:9 카메라를 좁은 0.71폭에 넣으려 함)
            rect.width = viewContainer.width;
            rect.height = rect.width * (sw / sh) / targetAspectRatio;
            rect.x = viewContainer.x;
            rect.y = viewContainer.y + (viewContainer.height - rect.height) * 0.5f;
        }

        return rect;
    }

    Camera GetCurrentActiveCamera()
    {
        switch (currentViewMode)
        {
            case ViewMode.TopView: return topViewCamera;
            case ViewMode.BackView: return backViewCamera;
            case ViewMode.PublisherCamera: return publisherCamera;
        }
        return null;
    }

    void UpdateTopViewCamera()
    {
        Vector3 targetPosition = topViewTarget.transform.position;
        topViewCamera.transform.position = targetPosition + Vector3.up * topViewHeight;
        topViewCamera.transform.rotation = Quaternion.Euler(90f, 0f, 0f); // 아래를 바라봄
    }

    void UpdateBackViewCamera()
    {
        Vector3 targetPosition = topViewTarget.transform.position;
        Vector3 targetForward = topViewTarget.transform.forward;

        // 대상 오브젝트의 뒤쪽에 카메라 배치 목표 지점 계산
        Vector3 desiredPosition = targetPosition - targetForward * backViewDistance + Vector3.up * backViewHeight;
        
        // SmoothDamp를 사용하여 부드럽게 이동
        backViewCamera.transform.position = Vector3.SmoothDamp(backViewCamera.transform.position, desiredPosition, ref currentVelocity, smoothTime);
        
        // 대상의 약간 위쪽을 바라보게 함
        backViewCamera.transform.LookAt(targetPosition + Vector3.up * backViewHeight * 0.5f);
    }

    void SetViewMode(ViewMode mode)
    {
        currentViewMode = mode;

        // 모든 카메라 비활성화
        if (topViewCamera != null) topViewCamera.enabled = false;
        if (backViewCamera != null) backViewCamera.enabled = false;

        if (publisherCamera != null && cameraPublisher != null)
        {
            cameraPublisher.RestoreRenderTexture();
            publisherCamera.enabled = false; // 일단 끔 (LateUpdate에서 켜지는 게 아니라 활성화 상태만 관리)
        }

        // 선택된 모드의 카메라 활성화
        switch (mode)
        {
            case ViewMode.TopView:
                if (topViewCamera != null && topViewTarget != null)
                {
                    UpdateTopViewCamera();
                    topViewCamera.enabled = true;
                }
                else if (topViewTarget == null)
                {
                    Debug.LogWarning("[AMRViewController] TopView 대상 오브젝트가 설정되지 않았습니다!");
                    return;
                }
                RestorePublisherCameraRenderTexture();
                break;

            case ViewMode.BackView:
                if (backViewCamera != null && topViewTarget != null)
                {
                    UpdateBackViewCamera();
                    backViewCamera.enabled = true;
                }
                else if (topViewTarget == null)
                {
                    Debug.LogWarning("[AMRViewController] BackView 대상 오브젝트가 설정되지 않았습니다!");
                    return;
                }
                RestorePublisherCameraRenderTexture();
                break;

            case ViewMode.PublisherCamera:
                if (cameraPublisher != null)
                {
                    publisherCamera = GetPublisherCamera();
                    if (publisherCamera != null)
                    {
                        cameraPublisher.RenderToScreen();
                        publisherCamera.enabled = true;
                        // Rect 설정은 LateUpdate에서 매 프레임 수행됨
                    }
                }
                else
                {
                    Debug.LogWarning("[AMRViewController] CameraPublisher가 설정되지 않았습니다!");
                    return;
                }
                break;
        }
    }

    Camera GetPublisherCamera()
    {
        // CameraPublisher의 GetCamera() 메서드 사용 (안정화 모드 지원)
        if (cameraPublisher != null)
        {
            return cameraPublisher.GetCamera();
        }
        return null;
    }

    void RestorePublisherCameraRenderTexture()
    {
        // CameraPublisher의 API를 통해 RenderTexture 모드로 복원
        if (cameraPublisher != null)
        {
            cameraPublisher.RestoreRenderTexture();
            Camera pubCam = GetPublisherCamera();
            if (pubCam != null)
            {
                pubCam.enabled = true; // ROS 퍼블리싱을 위해 활성화 유지
            }
        }
    }

    void OnDestroy()
    {
        // 배경 카메라 정리
        if (backgroundCamera != null) Destroy(backgroundCamera.gameObject);

        // 동적으로 생성한 TopView 카메라 정리
        if (topViewCamera != null) Destroy(topViewCamera.gameObject);

        // 동적으로 생성한 BackView 카메라 정리
        if (backViewCamera != null) Destroy(backViewCamera.gameObject);

        // Publisher Camera를 원래 상태로 복원
        RestorePublisherCameraRenderTexture();
    }
}
