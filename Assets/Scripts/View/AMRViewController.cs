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

    private Camera topViewCamera;
    private Camera backViewCamera;
    private Camera publisherCamera;
    private ViewMode currentViewMode = ViewMode.TopView;

    private enum ViewMode
    {
        TopView,
        BackView,
        PublisherCamera
    }

    void Start()
    {
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

    void CreateTopViewCamera()
    {
        GameObject topViewCamObj = new GameObject("TopView_Camera");
        topViewCamera = topViewCamObj.AddComponent<Camera>();
        topViewCamera.nearClipPlane = 0.1f;
        topViewCamera.farClipPlane = 100f;
        topViewCamera.fieldOfView = 60f;
        topViewCamera.enabled = false;
    }

    void CreateBackViewCamera()
    {
        GameObject backViewCamObj = new GameObject("BackView_Camera");
        backViewCamera = backViewCamObj.AddComponent<Camera>();
        backViewCamera.nearClipPlane = 0.1f;
        backViewCamera.farClipPlane = 100f;
        backViewCamera.fieldOfView = 60f;
        backViewCamera.enabled = false;
    }

    void LateUpdate()
    {
        // TopView 모드일 때 카메라 위치 업데이트
        if (currentViewMode == ViewMode.TopView && topViewTarget != null && topViewCamera != null)
        {
            UpdateTopViewCamera();
        }

        // BackView 모드일 때 카메라 위치 업데이트
        if (currentViewMode == ViewMode.BackView && topViewTarget != null && backViewCamera != null)
        {
            UpdateBackViewCamera();
        }
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
        if (topViewCamera != null)
        {
            topViewCamera.enabled = false;
        }

        if (backViewCamera != null)
        {
            backViewCamera.enabled = false;
        }

        if (publisherCamera != null && cameraPublisher != null)
        {
            // CameraPublisher의 API를 통해 RenderTexture 복원
            cameraPublisher.RestoreRenderTexture();
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
                // CameraPublisher 카메라는 RenderTexture로 유지
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
                // CameraPublisher 카메라는 RenderTexture로 유지
                RestorePublisherCameraRenderTexture();
                break;

            case ViewMode.PublisherCamera:
                if (cameraPublisher != null)
                {
                    publisherCamera = GetPublisherCamera();
                    if (publisherCamera != null)
                    {
                        // CameraPublisher의 API를 통해 화면에 직접 렌더링
                        cameraPublisher.RenderToScreen();
                        publisherCamera.enabled = true;
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
        // 동적으로 생성한 TopView 카메라 정리
        if (topViewCamera != null)
        {
            Destroy(topViewCamera.gameObject);
        }

        // 동적으로 생성한 BackView 카메라 정리
        if (backViewCamera != null)
        {
            Destroy(backViewCamera.gameObject);
        }

        // Publisher Camera를 원래 상태로 복원
        RestorePublisherCameraRenderTexture();
    }
}
