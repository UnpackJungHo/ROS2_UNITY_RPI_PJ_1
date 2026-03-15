using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 씬 루트에 배치하여 여러 차량 중 하나를 선택하고,
/// 선택된 차량의 VehicleViewProvider를 통해 Main Camera에 뷰를 표시한다.
///
/// 조작:
///   Z 키       - "robot" 태그 차량 목록에서 순차 전환
///   1/2/3 키   - Front/Top/Back 뷰 전환
/// </summary>
[DisallowMultipleComponent]
public class VehicleSelector : MonoBehaviour
{
    [Header("Display Camera")]
    [Tooltip("Game View 출력 카메라. 비어있으면 Main Camera 자동 사용.")]
    public Camera displayCamera;

    [Header("Selection")]
    [Tooltip("시작 시 첫 번째로 발견된 차량을 자동 선택")]
    public bool autoSelectFirstVehicle = true;

    [Header("View Controls")]
    public VehicleViewProvider.ViewMode defaultViewMode = VehicleViewProvider.ViewMode.TopView;
    public KeyCode frontViewHotkey = KeyCode.Alpha1;
    public KeyCode topViewHotkey = KeyCode.Alpha2;
    public KeyCode backViewHotkey = KeyCode.Alpha3;
    public KeyCode cycleVehicleHotkey = KeyCode.Z;

    [Header("Debug (Read Only)")]
    [SerializeField] private string selectedVehicleName = "None";
    [SerializeField] private VehicleViewProvider.ViewMode currentViewMode;
    [SerializeField] private int discoveredVehicleCount = 0;
    [SerializeField] private int selectedVehicleIndex = -1;

    private VehicleViewProvider selectedVehicle;
    private readonly List<VehicleViewProvider> discoveredProviders = new List<VehicleViewProvider>();
    private Vector3 backViewVelocity = Vector3.zero;

    void Awake()
    {
        ResolveDisplayCamera();
        currentViewMode = defaultViewMode;
    }

    void Start()
    {
        RefreshVehicleProviders();

        if (autoSelectFirstVehicle && selectedVehicle == null && discoveredProviders.Count > 0)
            SelectVehicle(discoveredProviders[0]);
    }

    void Update()
    {
        HandleKeyboardInput();
    }

    void LateUpdate()
    {
        if (selectedVehicle == null)
        {
            if (discoveredProviders.Count == 0)
            {
                RefreshVehicleProviders();
                if (autoSelectFirstVehicle && discoveredProviders.Count > 0)
                    SelectVehicle(discoveredProviders[0]);
            }
            return;
        }

        ResolveDisplayCamera();
        if (displayCamera == null)
            return;

        selectedVehicle.ApplyView(displayCamera, currentViewMode, ref backViewVelocity, false);
    }

    /// <summary>
    /// 씬에서 모든 VehicleViewProvider를 탐색하여 목록을 갱신한다.
    /// </summary>
    public void RefreshVehicleProviders()
    {
        discoveredProviders.Clear();
        VehicleViewProvider[] providers = FindObjectsOfType<VehicleViewProvider>(true);
        discoveredProviders.AddRange(providers);
        discoveredProviders.Sort((a, b) => string.CompareOrdinal(a.VehicleId, b.VehicleId));
        discoveredVehicleCount = discoveredProviders.Count;
    }

    /// <summary>
    /// 지정된 차량을 선택하고 즉시 뷰를 적용한다.
    /// </summary>
    public void SelectVehicle(VehicleViewProvider provider)
    {
        if (provider == null)
            return;

        selectedVehicle = provider;
        selectedVehicleIndex = discoveredProviders.IndexOf(provider);
        backViewVelocity = Vector3.zero;
        selectedVehicleName = provider.VehicleId;

        ResolveDisplayCamera();
        if (displayCamera != null)
            selectedVehicle.ApplyView(displayCamera, currentViewMode, ref backViewVelocity, true);
    }

    /// <summary>
    /// Z 키 입력으로 차량 목록에서 다음 차량으로 순차 전환한다.
    /// </summary>
    void CycleNextVehicle()
    {
        if (discoveredProviders.Count == 0)
        {
            RefreshVehicleProviders();
            if (discoveredProviders.Count == 0)
            {
                Debug.Log("[VehicleSelector] 씬에 차량이 없습니다.");
                return;
            }
        }

        selectedVehicleIndex = (selectedVehicleIndex + 1) % discoveredProviders.Count;
        SelectVehicle(discoveredProviders[selectedVehicleIndex]);

        Debug.Log($"[VehicleSelector] Z 키 전환 → [{selectedVehicleIndex + 1}/{discoveredProviders.Count}] {selectedVehicleName} (뷰: {currentViewMode})");
    }

    /// <summary>
    /// 뷰 모드를 변경하고 즉시 적용한다.
    /// </summary>
    public void SetViewMode(VehicleViewProvider.ViewMode mode)
    {
        currentViewMode = mode;
        backViewVelocity = Vector3.zero;

        if (selectedVehicle != null && displayCamera != null)
            selectedVehicle.ApplyView(displayCamera, currentViewMode, ref backViewVelocity, true);
    }

    void HandleKeyboardInput()
    {
        if (Input.GetKeyDown(cycleVehicleHotkey))
            CycleNextVehicle();
        else if (Input.GetKeyDown(frontViewHotkey))
            SetViewMode(VehicleViewProvider.ViewMode.FrontView);
        else if (Input.GetKeyDown(topViewHotkey))
            SetViewMode(VehicleViewProvider.ViewMode.TopView);
        else if (Input.GetKeyDown(backViewHotkey))
            SetViewMode(VehicleViewProvider.ViewMode.BackView);
    }

    void ResolveDisplayCamera()
    {
        if (displayCamera != null)
            return;

        displayCamera = Camera.main;
    }

    // ──────────────────────────────────────────────
    //  외부 접근용
    // ──────────────────────────────────────────────

    public VehicleViewProvider GetSelectedVehicle() => selectedVehicle;
    public VehicleViewProvider.ViewMode GetCurrentViewMode() => currentViewMode;
    public IReadOnlyList<VehicleViewProvider> GetDiscoveredProviders() => discoveredProviders;
}
