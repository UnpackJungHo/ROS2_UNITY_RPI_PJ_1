using System.Collections.Generic;
using UnityEngine;

public class AMRViewController : MonoBehaviour
{
    [Header("Display Camera")]
    [Tooltip("Game View output camera. If empty, Main Camera is used.")]
    public Camera displayCamera;

    [Header("Vehicle Discovery")]
    [Tooltip("Automatically discover VehicleViewSource components in the scene.")]
    public bool autoFindVehicleSources = true;

    [Tooltip("Fallback CameraPublisher for the current single-vehicle scene.")]
    public CameraPublisher cameraPublisher;

    [Tooltip("Fallback follow target for the current single-vehicle scene.")]
    public GameObject topViewTarget;

    [Header("Legacy Single Vehicle Offsets")]
    public float topViewHeight = 5f;
    public float backViewDistance = 3f;
    public float backViewHeight = 1.5f;
    public float smoothTime = 0.1f;

    [Header("Selection")]
    [Tooltip("Initial selected vehicle index.")]
    public int defaultSelectedVehicleIndex = 0;

    [Header("View Controls")]
    public ViewMode defaultViewMode = ViewMode.TopView;
    public bool enableKeyboardInput = true;
    public KeyCode frontViewHotkey = KeyCode.Alpha1;
    public KeyCode topViewHotkey = KeyCode.Alpha2;
    public KeyCode backViewHotkey = KeyCode.Alpha3;

    [Header("Display Policy")]
    [Tooltip("Include the vehicle layer in Top/Back view so the selected vehicle is visible.")]
    public bool includeVehicleLayerInTopAndBack = true;

    [Tooltip("Vehicle layer name to add back for Top/Back view.")]
    public string vehicleLayerName = "RLVehicle";

    [Header("Debug")]
    [SerializeField] private string selectedVehicleName = "None";
    [SerializeField] private ViewMode currentViewMode = ViewMode.TopView;
    [SerializeField] private int selectedVehicleIndex = -1;

    private readonly List<VehicleViewSource> discoveredSources = new List<VehicleViewSource>();
    private Vector3 backViewVelocity = Vector3.zero;
    private bool loggedMissingDisplayCamera = false;
    private bool loggedMissingVehicle = false;
    private bool loggedLegacyFallback = false;
    private Camera fallbackDisplayCameraTemplate;

    public enum ViewMode
    {
        TopView = 0,
        BackView = 1,
        FrontView = 2
    }

    void Awake()
    {
        CleanupLegacyChildCameras();
        ResolveDisplayCamera();
        fallbackDisplayCameraTemplate = displayCamera;
        RefreshVehicleSources();
        SelectInitialVehicle();
        currentViewMode = defaultViewMode;
    }

    void Start()
    {
        ApplyCurrentView(forceResetVelocity: true);
        Debug.Log("[AMRViewController] View controls: 1=Front, 2=Top, 3=Back");
    }

    void Update()
    {
        HandleKeyboardInput();
    }

    void LateUpdate()
    {
        if (autoFindVehicleSources && discoveredSources.Count == 0)
        {
            RefreshVehicleSources();
            if (selectedVehicleIndex < 0)
                SelectInitialVehicle();
        }

        ApplyCurrentView(forceResetVelocity: false);
    }

    public void RefreshVehicleSources()
    {
        discoveredSources.Clear();

        VehicleViewSource[] sources = FindObjectsOfType<VehicleViewSource>(true);
        for (int i = 0; i < sources.Length; i++)
        {
            if (sources[i] == null)
                continue;

            discoveredSources.Add(sources[i]);
        }

        discoveredSources.Sort((a, b) => string.CompareOrdinal(a.VehicleId, b.VehicleId));
    }

    public void SelectVehicleByIndex(int index)
    {
        if (discoveredSources.Count == 0)
        {
            selectedVehicleIndex = -1;
            return;
        }

        selectedVehicleIndex = Mathf.Clamp(index, 0, discoveredSources.Count - 1);
        backViewVelocity = Vector3.zero;
        UpdateSelectedVehicleDebugName();
        ApplyCurrentView(forceResetVelocity: true);
    }

    public void SetSelectedVehicle(VehicleViewSource source)
    {
        if (source == null)
        {
            selectedVehicleIndex = -1;
            UpdateSelectedVehicleDebugName();
            return;
        }

        int index = discoveredSources.IndexOf(source);
        if (index >= 0)
        {
            SelectVehicleByIndex(index);
            return;
        }

        discoveredSources.Add(source);
        SelectVehicleByIndex(discoveredSources.Count - 1);
    }

    public void SetViewMode(ViewMode mode)
    {
        currentViewMode = mode;
        ApplyCurrentView(forceResetVelocity: true);
    }

    void HandleKeyboardInput()
    {
        if (!enableKeyboardInput || !Application.isPlaying)
            return;

        if (Input.GetKeyDown(frontViewHotkey))
            SetViewMode(ViewMode.FrontView);
        else if (Input.GetKeyDown(topViewHotkey))
            SetViewMode(ViewMode.TopView);
        else if (Input.GetKeyDown(backViewHotkey))
            SetViewMode(ViewMode.BackView);
    }

    void ResolveDisplayCamera()
    {
        if (displayCamera != null)
        {
            loggedMissingDisplayCamera = false;
            return;
        }

        displayCamera = Camera.main;
        if (displayCamera == null)
            displayCamera = FindObjectOfType<Camera>();

        if (displayCamera == null && !loggedMissingDisplayCamera)
        {
            Debug.LogWarning("[AMRViewController] Display camera was not found.");
            loggedMissingDisplayCamera = true;
        }
    }

    void CleanupLegacyChildCameras()
    {
        DestroyLegacyCameraIfExists("TopView_Camera");
        DestroyLegacyCameraIfExists("BackView_Camera");
    }

    void DestroyLegacyCameraIfExists(string cameraObjectName)
    {
        GameObject legacyCameraObject = GameObject.Find(cameraObjectName);
        if (legacyCameraObject == null)
            return;

        Camera legacyCamera = legacyCameraObject.GetComponent<Camera>();
        if (legacyCamera != null)
            legacyCamera.enabled = false;

        if (Application.isPlaying)
            Destroy(legacyCameraObject);
        else
            DestroyImmediate(legacyCameraObject);
    }

    void SelectInitialVehicle()
    {
        if (discoveredSources.Count > 0)
            SelectVehicleByIndex(defaultSelectedVehicleIndex);
        else
            UpdateSelectedVehicleDebugName();
    }

    void UpdateSelectedVehicleDebugName()
    {
        VehicleViewSource selected = GetSelectedVehicleSource();
        selectedVehicleName = selected != null ? selected.VehicleId : GetLegacyVehicleId();
    }

    VehicleViewSource GetSelectedVehicleSource()
    {
        if (selectedVehicleIndex < 0 || selectedVehicleIndex >= discoveredSources.Count)
            return null;

        return discoveredSources[selectedVehicleIndex];
    }

    void ApplyCurrentView(bool forceResetVelocity)
    {
        ResolveDisplayCamera();
        if (displayCamera == null)
            return;

        VehicleViewSource selectedSource = GetSelectedVehicleSource();
        Camera sourceCamera = ResolveSourceCamera(selectedSource);
        Transform followTarget = ResolveFollowTarget(selectedSource);

        if (followTarget == null)
        {
            if (!loggedMissingVehicle)
            {
                Debug.LogWarning("[AMRViewController] No vehicle view source or follow target is available.");
                loggedMissingVehicle = true;
            }
            return;
        }

        loggedMissingVehicle = false;
        selectedVehicleName = selectedSource != null ? selectedSource.VehicleId : GetLegacyVehicleId();

        ApplyBaseDisplayCameraSettings(sourceCamera);

        switch (currentViewMode)
        {
            case ViewMode.FrontView:
                ApplyFrontView(sourceCamera, followTarget);
                break;

            case ViewMode.TopView:
                ApplyTopView(selectedSource, followTarget, sourceCamera);
                break;

            case ViewMode.BackView:
                ApplyBackView(selectedSource, followTarget, sourceCamera, forceResetVelocity);
                break;
        }
    }

    void ApplyBaseDisplayCameraSettings(Camera sourceCamera)
    {
        displayCamera.enabled = true;
        displayCamera.targetTexture = null;
        displayCamera.depth = 100f;

        Camera template = sourceCamera != null ? sourceCamera : fallbackDisplayCameraTemplate;
        if (template == null)
            return;

        displayCamera.nearClipPlane = template.nearClipPlane;
        displayCamera.farClipPlane = template.farClipPlane;
        displayCamera.fieldOfView = template.fieldOfView;
        displayCamera.clearFlags = template.clearFlags;
        displayCamera.backgroundColor = template.backgroundColor;
        displayCamera.orthographic = false;
    }

    void ApplyFrontView(Camera sourceCamera, Transform followTarget)
    {
        if (sourceCamera != null)
        {
            displayCamera.transform.SetPositionAndRotation(sourceCamera.transform.position, sourceCamera.transform.rotation);
            displayCamera.cullingMask = sourceCamera.cullingMask;
            return;
        }

        displayCamera.transform.SetPositionAndRotation(followTarget.position, followTarget.rotation);
    }

    void ApplyTopView(VehicleViewSource source, Transform followTarget, Camera sourceCamera)
    {
        displayCamera.transform.position = followTarget.position + Vector3.up * GetTopViewHeight(source);
        displayCamera.transform.rotation = Quaternion.Euler(90f, 0f, 0f);
        displayCamera.cullingMask = BuildDisplayCullingMask(sourceCamera, includeVehicle: true);
        backViewVelocity = Vector3.zero;
    }

    void ApplyBackView(VehicleViewSource source, Transform followTarget, Camera sourceCamera, bool forceResetVelocity)
    {
        Vector3 desiredPosition = followTarget.position
            - followTarget.forward * GetBackViewDistance(source)
            + Vector3.up * GetBackViewHeight(source);

        if (forceResetVelocity)
        {
            displayCamera.transform.position = desiredPosition;
            backViewVelocity = Vector3.zero;
        }
        else
        {
            displayCamera.transform.position = Vector3.SmoothDamp(
                displayCamera.transform.position,
                desiredPosition,
                ref backViewVelocity,
                GetSmoothTime(source)
            );
        }

        displayCamera.transform.LookAt(followTarget.position + Vector3.up * (GetBackViewHeight(source) * 0.5f));
        displayCamera.cullingMask = BuildDisplayCullingMask(sourceCamera, includeVehicle: true);
    }

    int BuildDisplayCullingMask(Camera sourceCamera, bool includeVehicle)
    {
        int mask;
        if (sourceCamera != null)
            mask = sourceCamera.cullingMask;
        else if (fallbackDisplayCameraTemplate != null)
            mask = fallbackDisplayCameraTemplate.cullingMask;
        else
            mask = displayCamera.cullingMask;

        if (!includeVehicle || !includeVehicleLayerInTopAndBack)
            return mask;

        int vehicleLayer = LayerMask.NameToLayer(vehicleLayerName);
        if (vehicleLayer >= 0)
            mask |= 1 << vehicleLayer;

        return mask;
    }

    Camera ResolveSourceCamera(VehicleViewSource source)
    {
        if (source != null)
            return source.ResolveFrontCamera();

        if (cameraPublisher != null)
        {
            if (!loggedLegacyFallback)
            {
                Debug.Log("[AMRViewController] Using legacy single-vehicle camera source. Add VehicleViewSource components for multi-vehicle expansion.");
                loggedLegacyFallback = true;
            }

            return cameraPublisher.GetCamera();
        }

        return null;
    }

    Transform ResolveFollowTarget(VehicleViewSource source)
    {
        if (source != null)
            return source.ResolveFollowTarget();

        return topViewTarget != null ? topViewTarget.transform : null;
    }

    float GetTopViewHeight(VehicleViewSource source)
    {
        return source != null ? source.topViewHeight : topViewHeight;
    }

    float GetBackViewDistance(VehicleViewSource source)
    {
        return source != null ? source.backViewDistance : backViewDistance;
    }

    float GetBackViewHeight(VehicleViewSource source)
    {
        return source != null ? source.backViewHeight : backViewHeight;
    }

    float GetSmoothTime(VehicleViewSource source)
    {
        return source != null ? source.smoothTime : smoothTime;
    }

    string GetLegacyVehicleId()
    {
        if (topViewTarget != null)
            return topViewTarget.name;

        return "LegacyVehicle";
    }
}
