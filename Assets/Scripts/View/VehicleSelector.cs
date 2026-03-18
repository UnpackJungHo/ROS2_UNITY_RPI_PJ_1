using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 씬 루트에 배치하여 여러 차량 중 하나를 선택하고,
/// 선택된 차량의 VehicleViewProvider를 통해 Main Camera에 뷰를 표시한다.
///
/// 조작:
///   Z 키       - 차량 목록 순차 전환 / 자유 관찰 모드에서는 차량 모드 복귀
///   1/2/3 키   - Front/Top/Back 뷰 전환 (차량 모드에서만)
///   G 키       - 자유 관찰 모드 진입
///   자유 관찰 모드: 마우스 우클릭 유지 + WASD/QE 이동, 마우스 시야 회전
///                  Shift 키로 고속 이동
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
    public KeyCode frontViewHotkey  = KeyCode.Alpha1;
    public KeyCode topViewHotkey    = KeyCode.Alpha2;
    public KeyCode backViewHotkey   = KeyCode.Alpha3;
    public KeyCode cycleVehicleHotkey = KeyCode.Z;

    [Header("Free Camera")]
    [Tooltip("자유 관찰 모드 진입 키")]
    public KeyCode freeCameraHotkey = KeyCode.G;

    [Tooltip("기본 이동 속도 (units/s)")]
    public float freeCamMoveSpeed = 10f;

    [Tooltip("Shift 누를 때 이동 속도 배율")]
    public float freeCamFastMultiplier = 3f;

    [Tooltip("마우스 시야 회전 감도")]
    public float freeCamRotateSpeed = 2f;

    [Header("Debug (Read Only)")]
    [SerializeField] private string selectedVehicleName = "None";
    [SerializeField] private VehicleViewProvider.ViewMode currentViewMode;
    [SerializeField] private int discoveredVehicleCount = 0;
    [SerializeField] private int selectedVehicleIndex   = -1;
    [SerializeField] private bool isFreeCameraMode      = false;

    // ── 차량 관측 상태 ──
    private VehicleViewProvider selectedVehicle;
    private readonly List<VehicleViewProvider> discoveredProviders = new List<VehicleViewProvider>();
    private Vector3 backViewVelocity = Vector3.zero;

    // ── 자유 관찰 모드 복귀용 스냅샷 ──
    private VehicleViewProvider       savedVehicle;
    private VehicleViewProvider.ViewMode savedViewMode;
    private int savedVehicleIndex;

    // ── 자유 관찰 카메라 상태 ──
    private float freeCamYaw;
    private float freeCamPitch;

    // ══════════════════════════════════════════════
    //  Unity 생명주기
    // ══════════════════════════════════════════════

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
        if (isFreeCameraMode)
        {
            UpdateFreeCamera();
            return;
        }

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

    // ══════════════════════════════════════════════
    //  입력 처리
    // ══════════════════════════════════════════════

    /// <summary>
    /// 입력 우선순위:
    ///   1. G 키  → 자유 관찰 모드 진입 (어느 모드에서든)
    ///   2. Z 키  → 자유 관찰 모드 중이면 차량 모드 복귀,
    ///              차량 모드 중이면 다음 차량으로 순환
    ///   3. 1/2/3 → 차량 모드에서만 뷰 전환
    /// </summary>
    void HandleKeyboardInput()
    {
        if (Input.GetKeyDown(freeCameraHotkey))
        {
            EnterFreeCameraMode();
            return;
        }

        if (Input.GetKeyDown(cycleVehicleHotkey))
        {
            if (isFreeCameraMode)
                ExitFreeCameraMode();
            else
                CycleNextVehicle();
            return;
        }

        if (!isFreeCameraMode)
        {
            if (Input.GetKeyDown(frontViewHotkey))
                SetViewMode(VehicleViewProvider.ViewMode.FrontView);
            else if (Input.GetKeyDown(topViewHotkey))
                SetViewMode(VehicleViewProvider.ViewMode.TopView);
            else if (Input.GetKeyDown(backViewHotkey))
                SetViewMode(VehicleViewProvider.ViewMode.BackView);
        }
    }

    // ══════════════════════════════════════════════
    //  자유 관찰 모드
    // ══════════════════════════════════════════════

    void EnterFreeCameraMode()
    {
        if (isFreeCameraMode) return;

        // 현재 차량 관측 상태 스냅샷 저장
        savedVehicle       = selectedVehicle;
        savedVehicleIndex  = selectedVehicleIndex;
        savedViewMode      = currentViewMode;

        // 현재 카메라 회전에서 Yaw/Pitch 초기화
        ResolveDisplayCamera();
        if (displayCamera != null)
        {
            Vector3 euler = displayCamera.transform.eulerAngles;
            freeCamYaw   = euler.y;
            freeCamPitch = NormalizeAngle(euler.x); // -180~180 범위로 정규화
        }

        isFreeCameraMode = true;
        Debug.Log("[VehicleSelector] 자유 관찰 모드 진입 (Z키로 차량 모드 복귀)");
    }

    void ExitFreeCameraMode()
    {
        if (!isFreeCameraMode) return;

        isFreeCameraMode = false;

        // 저장된 차량 상태 복원
        if (savedVehicle != null)
        {
            selectedVehicleIndex = savedVehicleIndex;
            currentViewMode      = savedViewMode;
            backViewVelocity     = Vector3.zero;
            SelectVehicle(savedVehicle);
        }
        else if (discoveredProviders.Count > 0)
        {
            SelectVehicle(discoveredProviders[0]);
        }

        Debug.Log($"[VehicleSelector] 차량 관측 모드 복귀 → {selectedVehicleName} (뷰: {currentViewMode})");
    }

    /// <summary>
    /// 자유 관찰 모드의 카메라 이동/회전을 LateUpdate에서 처리한다.
    /// 마우스 우클릭 유지 중에만 회전이 활성화된다.
    /// </summary>
    void UpdateFreeCamera()
    {
        ResolveDisplayCamera();
        if (displayCamera == null) return;

        // ── 마우스 우클릭 유지 시 시야 회전 ──
        if (Input.GetMouseButton(1))
        {
            freeCamYaw   += Input.GetAxis("Mouse X") * freeCamRotateSpeed;
            freeCamPitch -= Input.GetAxis("Mouse Y") * freeCamRotateSpeed;
            freeCamPitch  = Mathf.Clamp(freeCamPitch, -89f, 89f);

            displayCamera.transform.rotation = Quaternion.Euler(freeCamPitch, freeCamYaw, 0f);
        }

        // ── WASD / QE 이동 ──
        float speed = freeCamMoveSpeed * Time.deltaTime;
        if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
            speed *= freeCamFastMultiplier;

        Transform t = displayCamera.transform;
        Vector3 move = Vector3.zero;

        if (Input.GetKey(KeyCode.W)) move += t.forward;
        if (Input.GetKey(KeyCode.S)) move -= t.forward;
        if (Input.GetKey(KeyCode.D)) move += t.right;
        if (Input.GetKey(KeyCode.A)) move -= t.right;
        if (Input.GetKey(KeyCode.E)) move += Vector3.up;
        if (Input.GetKey(KeyCode.Q)) move -= Vector3.up;

        if (move.sqrMagnitude > 0f)
            t.position += move.normalized * speed;
    }

    // ══════════════════════════════════════════════
    //  차량 관측 모드
    // ══════════════════════════════════════════════

    public void RefreshVehicleProviders()
    {
        discoveredProviders.Clear();
        VehicleViewProvider[] providers = FindObjectsOfType<VehicleViewProvider>(true);
        discoveredProviders.AddRange(providers);
        discoveredProviders.Sort((a, b) => string.CompareOrdinal(a.VehicleId, b.VehicleId));
        discoveredVehicleCount = discoveredProviders.Count;
    }

    public void SelectVehicle(VehicleViewProvider provider)
    {
        if (provider == null) return;

        selectedVehicle      = provider;
        selectedVehicleIndex = discoveredProviders.IndexOf(provider);
        backViewVelocity     = Vector3.zero;
        selectedVehicleName  = provider.VehicleId;

        ResolveDisplayCamera();
        if (displayCamera != null)
            selectedVehicle.ApplyView(displayCamera, currentViewMode, ref backViewVelocity, true);
    }

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

    public void SetViewMode(VehicleViewProvider.ViewMode mode)
    {
        currentViewMode  = mode;
        backViewVelocity = Vector3.zero;

        if (selectedVehicle != null && displayCamera != null)
            selectedVehicle.ApplyView(displayCamera, currentViewMode, ref backViewVelocity, true);
    }

    // ══════════════════════════════════════════════
    //  유틸
    // ══════════════════════════════════════════════

    void ResolveDisplayCamera()
    {
        if (displayCamera != null) return;
        displayCamera = Camera.main;
    }

    /// <summary>Unity eulerAngles는 0~360 반환. -180~180으로 정규화한다.</summary>
    static float NormalizeAngle(float angle)
    {
        if (angle > 180f) angle -= 360f;
        return angle;
    }

    // ══════════════════════════════════════════════
    //  외부 접근용
    // ══════════════════════════════════════════════

    public VehicleViewProvider GetSelectedVehicle()                     => selectedVehicle;
    public VehicleViewProvider.ViewMode GetCurrentViewMode()            => currentViewMode;
    public IReadOnlyList<VehicleViewProvider> GetDiscoveredProviders()  => discoveredProviders;
    public bool IsFreeCameraMode                                        => isFreeCameraMode;
}
