using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using UnityEngine.EventSystems;
using TMPro;

public class AMRViewController : MonoBehaviour
{
    [Header("Camera References")]
    [Tooltip("FrontView 카메라 소스를 제공하는 CameraPublisher")]
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

    [Header("Camera Tab Style")]
    [Tooltip("탭 버튼 Hover/Pressed 스타일을 복사할 기준 버튼. 비워두면 DrivingTrainerUI/Center/StartButton을 자동 탐색합니다.")]
    public Button cameraTabStyleSourceButton;

    [Tooltip("선택된 탭의 배경색")]
    public Color selectedTabColor = new Color(0.141f, 0.208f, 0.561f, 1f);

    [Tooltip("선택되지 않은 탭의 배경색")]
    public Color unselectedTabColor = Color.white;

    [Tooltip("선택된 탭의 텍스트 색")]
    public Color selectedTabTextColor = Color.white;

    [Tooltip("선택되지 않은 탭의 텍스트 색")]
    public Color unselectedTabTextColor = new Color(0.49f, 0.53f, 0.62f, 1f);

    [Tooltip("카메라 출력 영역으로 사용할 Viewport RectTransform")]
    public RectTransform viewportRect;

    [Tooltip("Viewport 하위 CameraFeed RawImage")]
    public RawImage viewportRawImage;

    [Header("Viewport & FOV Settings")]
    [Tooltip("Viewport 출력 비율 (예: 16:9 = 1.7778)")]
    public float targetAspectRatio = 1.7778f;

    [Tooltip("카메라 시야각")]
    public float explicitFov = 60f;

    [Header("Viewport Feed Offset")]
    [Tooltip("Viewport 내부 카메라 화면 오프셋(px). X 음수면 왼쪽, 양수면 오른쪽으로 이동합니다.")]
    public Vector2 viewportFeedOffset = Vector2.zero;

    private AspectRatioFitter viewportAspectFitter;

    private Camera topViewCamera;
    private Camera backViewCamera;
    private Camera publisherCamera;
    private Camera displayKeepAliveCamera;

    private RenderTexture viewportRenderTexture;
    private int viewportTextureWidth = -1;
    private int viewportTextureHeight = -1;
    private bool loggedMissingViewport = false;
    private bool tabStyleInitialized = false;
    private Color tabHoverColor = new Color(0.396f, 0.443f, 0.698f, 1f);
    private Color tabPressedColor = new Color(0.592f, 0.635f, 0.847f, 1f);
    private float tabColorMultiplier = 1f;
    private float tabFadeDuration = 0.08f;

    private ViewMode currentViewMode = ViewMode.TopView;

    private enum ViewMode
    {
        TopView,
        BackView,
        PublisherCamera
    }

    void Start()
    {
        EnsureDisplayKeepAliveCamera();
        ResolveViewportReferences();
        InitializeCameraTabStyle();

        CreateTopViewCamera();
        CreateBackViewCamera();
        EnsureViewportRenderTexture();

        SetViewMode(ViewMode.TopView);

        if (top_view_button != null)
            top_view_button.onClick.AddListener(() => SetViewMode(ViewMode.TopView));

        if (front_view_button != null)
            front_view_button.onClick.AddListener(() => SetViewMode(ViewMode.PublisherCamera));

        if (back_view_button != null)
            back_view_button.onClick.AddListener(() => SetViewMode(ViewMode.BackView));

        BindCameraTabHoverEvents();
    }

    void LateUpdate()
    {
        EnsureDisplayKeepAliveCamera();
        ResolveViewportReferences();
        EnsureViewportRenderTexture();

        Camera currentCam = GetCurrentActiveCamera();
        if (currentCam != null)
            currentCam.fieldOfView = explicitFov;

        if (currentViewMode == ViewMode.TopView && topViewTarget != null && topViewCamera != null)
            UpdateTopViewCamera();

        if (currentViewMode == ViewMode.BackView && topViewTarget != null && backViewCamera != null)
            UpdateBackViewCamera();

        if (currentViewMode == ViewMode.PublisherCamera && viewportRawImage != null && cameraPublisher != null)
        {
            if (viewportRenderTexture != null && viewportRawImage.texture != viewportRenderTexture)
                viewportRawImage.texture = viewportRenderTexture;

            UpdateFeedAspectFromTexture(viewportRenderTexture);
        }
    }

    void ResolveViewportReferences()
    {
        if (viewportRect == null)
            viewportRect = FindViewportRectTransform();

        if (viewportRect == null)
        {
            if (!loggedMissingViewport)
            {
                Debug.LogWarning("[AMRViewController] Viewport RectTransform을 찾지 못했습니다. 경로를 확인하세요.");
                loggedMissingViewport = true;
            }
            return;
        }

        EnsureViewportFeedChild();
        ConfigureViewportFeed();
    }

    RectTransform FindViewportRectTransform()
    {
        string[] candidatePaths =
        {
            "Canvas_1920x1080/SimulationMonitorUI/Content/MainPane/Viewport",
            "Canvas_1920x1080/SimulationMonitorUI/Content/MainPanel/Viewport"
        };

        for (int i = 0; i < candidatePaths.Length; i++)
        {
            GameObject target = GameObject.Find(candidatePaths[i]);
            if (target == null)
                continue;

            RectTransform targetRect = target.GetComponent<RectTransform>();
            if (targetRect != null)
                return targetRect;
        }

        return null;
    }

    void EnsureViewportFeedChild()
    {
        if (viewportRect == null)
            return;

        Transform feedTransform = viewportRect.Find("CameraFeed");
        if (feedTransform == null)
        {
            GameObject feedGo = new GameObject("CameraFeed", typeof(RectTransform), typeof(RawImage), typeof(AspectRatioFitter));
            feedTransform = feedGo.transform;
            feedTransform.SetParent(viewportRect, false);
        }

        viewportRawImage = feedTransform.GetComponent<RawImage>();
        if (viewportRawImage == null)
            viewportRawImage = feedTransform.gameObject.AddComponent<RawImage>();

        viewportAspectFitter = feedTransform.GetComponent<AspectRatioFitter>();
        if (viewportAspectFitter == null)
            viewportAspectFitter = feedTransform.gameObject.AddComponent<AspectRatioFitter>();

        // FitInParent는 anchoredPosition을 중앙으로 강제해 오프셋 적용을 방해하므로 비활성화한다.
        viewportAspectFitter.enabled = false;

        RawImage viewportRootRawImage = viewportRect.GetComponent<RawImage>();
        if (viewportRootRawImage != null)
            viewportRootRawImage.enabled = false;

        RectMask2D mask = viewportRect.GetComponent<RectMask2D>();
        if (mask == null)
            viewportRect.gameObject.AddComponent<RectMask2D>();
    }

    void EnsureDisplayKeepAliveCamera()
    {
        if (displayKeepAliveCamera == null)
        {
            GameObject existing = GameObject.Find("DisplayKeepAliveCamera");
            if (existing != null)
                displayKeepAliveCamera = existing.GetComponent<Camera>();

            if (displayKeepAliveCamera == null)
            {
                GameObject keepAliveObj = new GameObject("DisplayKeepAliveCamera");
                displayKeepAliveCamera = keepAliveObj.AddComponent<Camera>();
            }
        }

        if (displayKeepAliveCamera == null)
            return;

        displayKeepAliveCamera.targetTexture = null;
        displayKeepAliveCamera.clearFlags = CameraClearFlags.Depth;
        displayKeepAliveCamera.cullingMask = 0;
        displayKeepAliveCamera.depth = -1000f;
        displayKeepAliveCamera.enabled = true;
    }

    void ConfigureViewportFeed()
    {
        if (viewportRawImage == null)
            return;

        viewportRawImage.color = Color.white;
        viewportRawImage.raycastTarget = false;

        RectTransform feedRect = viewportRawImage.rectTransform;
        feedRect.anchorMin = new Vector2(0.5f, 0.5f);
        feedRect.anchorMax = new Vector2(0.5f, 0.5f);
        feedRect.pivot = new Vector2(0.5f, 0.5f);
        feedRect.localScale = Vector3.one;
        feedRect.localEulerAngles = Vector3.zero;

        ApplyFeedLayout(GetPreferredAspectRatio());
    }

    float GetPreferredAspectRatio()
    {
        return targetAspectRatio > 0f ? targetAspectRatio : (16f / 9f);
    }

    void UpdateFeedAspectFromTexture(Texture texture)
    {
        float aspect;
        if (texture != null && texture.height > 0)
            aspect = Mathf.Max(0.01f, (float)texture.width / texture.height);
        else
            aspect = GetPreferredAspectRatio();

        ApplyFeedLayout(aspect);
    }

    void ApplyFeedLayout(float aspect)
    {
        if (viewportRect == null || viewportRawImage == null)
            return;

        aspect = Mathf.Max(0.01f, aspect);

        Vector2 parentSize = viewportRect.rect.size;
        if (parentSize.x <= 0.01f || parentSize.y <= 0.01f)
            return;

        float width = parentSize.x;
        float height = width / aspect;

        if (height > parentSize.y)
        {
            height = parentSize.y;
            width = height * aspect;
        }

        RectTransform feedRect = viewportRawImage.rectTransform;
        feedRect.sizeDelta = new Vector2(width, height);
        feedRect.anchoredPosition = viewportFeedOffset;
    }

    void EnsureViewportRenderTexture()
    {
        if (viewportRect == null)
            return;

        Vector3[] corners = new Vector3[4];
        viewportRect.GetWorldCorners(corners);
        int viewportWidth = Mathf.Max(2, Mathf.RoundToInt(Mathf.Abs(corners[3].x - corners[0].x)));
        int viewportHeight = Mathf.Max(2, Mathf.RoundToInt(Mathf.Abs(corners[1].y - corners[0].y)));

        float aspect = GetPreferredAspectRatio();

        int width = viewportWidth;
        int height = Mathf.Max(2, Mathf.RoundToInt(width / aspect));
        if (height > viewportHeight)
        {
            height = viewportHeight;
            width = Mathf.Max(2, Mathf.RoundToInt(height * aspect));
        }

        if (viewportRenderTexture != null && width == viewportTextureWidth && height == viewportTextureHeight)
            return;

        ReleaseViewportRenderTexture();

        viewportRenderTexture = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
        viewportRenderTexture.Create();
        viewportTextureWidth = width;
        viewportTextureHeight = height;

        if (topViewCamera != null)
            topViewCamera.targetTexture = viewportRenderTexture;

        if (backViewCamera != null)
            backViewCamera.targetTexture = viewportRenderTexture;

        if (publisherCamera != null && currentViewMode == ViewMode.PublisherCamera)
            publisherCamera.targetTexture = viewportRenderTexture;

        if (currentViewMode != ViewMode.PublisherCamera && viewportRawImage != null)
        {
            viewportRawImage.texture = viewportRenderTexture;
            UpdateFeedAspectFromTexture(viewportRenderTexture);
        }
    }

    void ReleaseViewportRenderTexture()
    {
        if (viewportRenderTexture == null)
            return;

        if (topViewCamera != null && topViewCamera.targetTexture == viewportRenderTexture)
            topViewCamera.targetTexture = null;

        if (backViewCamera != null && backViewCamera.targetTexture == viewportRenderTexture)
            backViewCamera.targetTexture = null;

        if (publisherCamera != null && publisherCamera.targetTexture == viewportRenderTexture)
            publisherCamera.targetTexture = null;

        viewportRenderTexture.Release();
        Destroy(viewportRenderTexture);
        viewportRenderTexture = null;
        viewportTextureWidth = -1;
        viewportTextureHeight = -1;
    }

    void CreateTopViewCamera()
    {
        GameObject topViewCamObj = new GameObject("TopView_Camera");
        topViewCamera = topViewCamObj.AddComponent<Camera>();
        topViewCamera.nearClipPlane = 0.1f;
        topViewCamera.farClipPlane = 100f;
        topViewCamera.fieldOfView = explicitFov;
        topViewCamera.clearFlags = CameraClearFlags.Skybox;
        topViewCamera.enabled = false;

        if (viewportRenderTexture != null)
            topViewCamera.targetTexture = viewportRenderTexture;
    }

    void CreateBackViewCamera()
    {
        GameObject backViewCamObj = new GameObject("BackView_Camera");
        backViewCamera = backViewCamObj.AddComponent<Camera>();
        backViewCamera.nearClipPlane = 0.1f;
        backViewCamera.farClipPlane = 100f;
        backViewCamera.fieldOfView = explicitFov;
        backViewCamera.clearFlags = CameraClearFlags.Skybox;
        backViewCamera.enabled = false;

        if (viewportRenderTexture != null)
            backViewCamera.targetTexture = viewportRenderTexture;
    }

    Camera GetCurrentActiveCamera()
    {
        switch (currentViewMode)
        {
            case ViewMode.TopView:
                return topViewCamera;
            case ViewMode.BackView:
                return backViewCamera;
            case ViewMode.PublisherCamera:
                return publisherCamera;
            default:
                return null;
        }
    }

    void UpdateTopViewCamera()
    {
        Vector3 targetPosition = topViewTarget.transform.position;
        topViewCamera.transform.position = targetPosition + Vector3.up * topViewHeight;
        topViewCamera.transform.rotation = Quaternion.Euler(90f, 0f, 0f);
    }

    void UpdateBackViewCamera()
    {
        Vector3 targetPosition = topViewTarget.transform.position;
        Vector3 targetForward = topViewTarget.transform.forward;

        Vector3 desiredPosition = targetPosition - targetForward * backViewDistance + Vector3.up * backViewHeight;
        backViewCamera.transform.position = Vector3.SmoothDamp(backViewCamera.transform.position, desiredPosition, ref currentVelocity, smoothTime);
        backViewCamera.transform.LookAt(targetPosition + Vector3.up * backViewHeight * 0.5f);
    }

    void InitializeCameraTabStyle()
    {
        if (tabStyleInitialized)
            return;

        // 현재 씬 디자인의 기본 선택/비선택 색을 우선 반영한다.
        CacheCurrentTabBaseColors();

        Button source = ResolveCameraTabStyleSourceButton();
        if (source != null)
        {
            ColorBlock sourceColors = source.colors;
            tabHoverColor = sourceColors.highlightedColor;
            tabPressedColor = sourceColors.pressedColor;
            tabColorMultiplier = sourceColors.colorMultiplier;
            tabFadeDuration = sourceColors.fadeDuration;
        }

        tabStyleInitialized = true;
    }

    Button ResolveCameraTabStyleSourceButton()
    {
        if (cameraTabStyleSourceButton != null)
            return cameraTabStyleSourceButton;

        Transform startButtonTransform = FindTransformByPath("Canvas_1920x1080/DrivingTrainerUI/Center/StartButton");
        if (startButtonTransform == null)
            return null;

        cameraTabStyleSourceButton = startButtonTransform.GetComponent<Button>();
        return cameraTabStyleSourceButton;
    }

    Transform FindTransformByPath(string path)
    {
        if (string.IsNullOrEmpty(path))
            return null;

        string[] parts = path.Split('/');
        Scene scene = SceneManager.GetActiveScene();
        if (!scene.IsValid())
            return null;

        GameObject[] roots = scene.GetRootGameObjects();
        for (int i = 0; i < roots.Length; i++)
        {
            if (roots[i].name != parts[0])
                continue;

            Transform current = roots[i].transform;
            bool matched = true;
            for (int p = 1; p < parts.Length; p++)
            {
                Transform next = null;
                for (int c = 0; c < current.childCount; c++)
                {
                    Transform child = current.GetChild(c);
                    if (child.name == parts[p])
                    {
                        next = child;
                        break;
                    }
                }

                if (next == null)
                {
                    matched = false;
                    break;
                }

                current = next;
            }

            if (matched)
                return current;
        }

        return null;
    }

    void CacheCurrentTabBaseColors()
    {
        if (front_view_button != null)
        {
            Image selectedImage = front_view_button.GetComponent<Image>();
            if (selectedImage != null)
                selectedTabColor = selectedImage.color;

            TMP_Text selectedLabel = front_view_button.GetComponentInChildren<TMP_Text>(true);
            if (selectedLabel != null)
                selectedTabTextColor = selectedLabel.color;
        }

        if (top_view_button != null)
        {
            Image unselectedImage = top_view_button.GetComponent<Image>();
            if (unselectedImage != null)
                unselectedTabColor = unselectedImage.color;

            TMP_Text unselectedLabel = top_view_button.GetComponentInChildren<TMP_Text>(true);
            if (unselectedLabel != null)
                unselectedTabTextColor = unselectedLabel.color;
        }
    }

    void BindCameraTabHoverEvents()
    {
        BindSingleTabHover(front_view_button, ViewMode.PublisherCamera);
        BindSingleTabHover(top_view_button, ViewMode.TopView);
        BindSingleTabHover(back_view_button, ViewMode.BackView);
    }

    void BindSingleTabHover(Button tabButton, ViewMode tabMode)
    {
        if (tabButton == null)
            return;

        EventTrigger trigger = tabButton.GetComponent<EventTrigger>();
        if (trigger == null)
            trigger = tabButton.gameObject.AddComponent<EventTrigger>();

        if (trigger.triggers == null)
            trigger.triggers = new System.Collections.Generic.List<EventTrigger.Entry>();

        EventTrigger.Entry enterEntry = new EventTrigger.Entry { eventID = EventTriggerType.PointerEnter };
        enterEntry.callback.AddListener(_ => OnTabPointerEnter(tabButton, tabMode));
        trigger.triggers.Add(enterEntry);

        EventTrigger.Entry exitEntry = new EventTrigger.Entry { eventID = EventTriggerType.PointerExit };
        exitEntry.callback.AddListener(_ => OnTabPointerExit(tabButton, tabMode));
        trigger.triggers.Add(exitEntry);
    }

    void OnTabPointerEnter(Button tabButton, ViewMode tabMode)
    {
        if (tabButton == null)
            return;

        // 요구사항: hover 시에도 텍스트를 흰색으로 표시.
        SetTabLabelColor(tabButton, selectedTabTextColor);
    }

    void OnTabPointerExit(Button tabButton, ViewMode tabMode)
    {
        if (tabButton == null)
            return;

        bool isSelected = currentViewMode == tabMode;
        SetTabLabelColor(tabButton, isSelected ? selectedTabTextColor : unselectedTabTextColor);
    }

    void SetTabLabelColor(Button tabButton, Color color)
    {
        TMP_Text label = tabButton.GetComponentInChildren<TMP_Text>(true);
        if (label != null)
            label.color = color;
    }

    void UpdateCameraTabVisuals()
    {
        ApplyTabButtonVisual(front_view_button, currentViewMode == ViewMode.PublisherCamera);
        ApplyTabButtonVisual(top_view_button, currentViewMode == ViewMode.TopView);
        ApplyTabButtonVisual(back_view_button, currentViewMode == ViewMode.BackView);
    }

    void ApplyTabButtonVisual(Button tabButton, bool isSelected)
    {
        if (tabButton == null)
            return;

        Image image = tabButton.GetComponent<Image>();
        if (image != null && tabButton.targetGraphic == null)
            tabButton.targetGraphic = image;
        tabButton.transition = Selectable.Transition.ColorTint;

        ColorBlock colors = tabButton.colors;
        colors.colorMultiplier = tabColorMultiplier;
        colors.fadeDuration = tabFadeDuration;

        if (isSelected)
        {
            colors.normalColor = selectedTabColor;
            colors.highlightedColor = selectedTabColor;
            colors.pressedColor = selectedTabColor;
            colors.selectedColor = selectedTabColor;
            colors.disabledColor = selectedTabColor;
            tabButton.colors = colors;
            tabButton.interactable = false;

            if (image != null)
                image.color = selectedTabColor;
        }
        else
        {
            colors.normalColor = unselectedTabColor;
            colors.highlightedColor = tabHoverColor;
            colors.pressedColor = tabPressedColor;
            colors.selectedColor = tabHoverColor;
            colors.disabledColor = selectedTabColor;
            tabButton.colors = colors;
            tabButton.interactable = true;

            if (image != null)
                image.color = unselectedTabColor;
        }

        TMP_Text label = tabButton.GetComponentInChildren<TMP_Text>(true);
        if (label != null)
            label.color = isSelected ? selectedTabTextColor : unselectedTabTextColor;
    }

    void SetViewMode(ViewMode mode)
    {
        currentViewMode = mode;
        InitializeCameraTabStyle();

        if (topViewCamera != null)
            topViewCamera.enabled = false;

        if (backViewCamera != null)
            backViewCamera.enabled = false;

        if (cameraPublisher != null)
        {
            publisherCamera = cameraPublisher.GetCamera();
            if (publisherCamera != null)
                publisherCamera.enabled = false;
        }

        switch (mode)
        {
            case ViewMode.TopView:
                if (topViewCamera == null || topViewTarget == null)
                {
                    Debug.LogWarning("[AMRViewController] TopView 대상 오브젝트가 설정되지 않았습니다!");
                    return;
                }

                EnsureViewportRenderTexture();
                topViewCamera.targetTexture = viewportRenderTexture;
                UpdateTopViewCamera();
                topViewCamera.enabled = true;

                if (viewportRawImage != null)
                {
                    viewportRawImage.texture = viewportRenderTexture;
                    UpdateFeedAspectFromTexture(viewportRenderTexture);
                }
                break;

            case ViewMode.BackView:
                if (backViewCamera == null || topViewTarget == null)
                {
                    Debug.LogWarning("[AMRViewController] BackView 대상 오브젝트가 설정되지 않았습니다!");
                    return;
                }

                EnsureViewportRenderTexture();
                backViewCamera.targetTexture = viewportRenderTexture;
                UpdateBackViewCamera();
                backViewCamera.enabled = true;

                if (viewportRawImage != null)
                {
                    viewportRawImage.texture = viewportRenderTexture;
                    UpdateFeedAspectFromTexture(viewportRenderTexture);
                }
                break;

            case ViewMode.PublisherCamera:
                if (cameraPublisher == null)
                {
                    Debug.LogWarning("[AMRViewController] CameraPublisher가 설정되지 않았습니다!");
                    return;
                }

                publisherCamera = cameraPublisher.GetCamera();
                if (publisherCamera == null)
                {
                    Debug.LogWarning("[AMRViewController] Publisher Camera를 찾을 수 없습니다!");
                    return;
                }

                EnsureViewportRenderTexture();
                publisherCamera.targetTexture = viewportRenderTexture;
                publisherCamera.fieldOfView = explicitFov;
                publisherCamera.enabled = true;

                if (viewportRawImage != null)
                {
                    viewportRawImage.texture = viewportRenderTexture;
                    UpdateFeedAspectFromTexture(viewportRenderTexture);
                }
                break;
        }

        UpdateCameraTabVisuals();
    }

    void OnDestroy()
    {
        if (displayKeepAliveCamera != null)
            Destroy(displayKeepAliveCamera.gameObject);

        if (topViewCamera != null)
            Destroy(topViewCamera.gameObject);

        if (backViewCamera != null)
            Destroy(backViewCamera.gameObject);

        ReleaseViewportRenderTexture();

        if (cameraPublisher != null)
        {
            cameraPublisher.RestoreRenderTexture();
            Camera pubCam = cameraPublisher.GetCamera();
            if (pubCam != null)
                pubCam.enabled = true;
        }
    }
}
