using System.Text.RegularExpressions;
using UnityEngine;

/// <summary>
/// 차량별 뷰 제공자. RootAMR 프리팹 내부의 AMRView 오브젝트에 부착.
/// 자신의 CameraPublisher/followTarget을 기반으로 Front/Top/Back 뷰를 제공한다.
///
/// VehicleSelector가 displayCamera를 넘기면 해당 뷰 모드로 카메라를 포지셔닝한다.
/// </summary>
[DisallowMultipleComponent]
public class VehicleViewProvider : MonoBehaviour
{
    public enum ViewMode
    {
        TopView = 0,
        BackView = 1,
        FrontView = 2
    }

    [Header("Identity")]
    public string vehicleId = "Vehicle";

    [Header("References")]
    [Tooltip("차량의 CameraPublisher (Front View 카메라 소스)")]
    public CameraPublisher cameraPublisher;

    [Tooltip("Front View 카메라 직접 지정 (CameraPublisher보다 우선)")]
    public Camera frontViewCameraOverride;

    [Tooltip("Top/Back View가 추적할 타겟 (base_link 등)")]
    public Transform followTarget;

    [Header("Auto Find")]
    public bool autoFindReferences = true;

    [Header("View Offsets")]
    public float topViewHeight = 5f;
    public float backViewDistance = 3f;
    public float backViewHeight = 1.5f;
    public float smoothTime = 0.1f;

    [Header("Display Policy")]
    [Tooltip("Top/Back 뷰에서 차량 레이어를 포함하여 차량이 보이게 할지")]
    public bool includeVehicleLayerInTopAndBack = true;
    public string vehicleLayerName = "RLVehicle";

    public string VehicleId => string.IsNullOrWhiteSpace(vehicleId) ? name : vehicleId;

    private Camera fallbackCameraTemplate;

    void Awake()
    {
        if (autoFindReferences)
            AutoFindReferences();

        AssignVehicleIdFromParent();
        fallbackCameraTemplate = ResolveFrontCamera();
    }

    void AssignVehicleIdFromParent()
    {
        string parentName = transform.parent != null ? transform.parent.name : name;
        var match = Regex.Match(parentName, @"(\d+)$");
        if (match.Success)
            vehicleId = "Vehicle" + match.Groups[1].Value;
    }

    void Reset()
    {
        AutoFindReferences();
    }

    public void AutoFindReferences()
    {
        if (cameraPublisher == null)
            cameraPublisher = GetComponentInChildren<CameraPublisher>(true)
                ?? GetComponentInParent<CameraPublisher>();

        if (followTarget == null)
        {
            Transform parent = transform.parent;
            if (parent != null)
            {
                // RootAMR/FactoryAMR_RL/base_footprint/base_link 경로 탐색
                foreach (Transform child in parent)
                {
                    Transform baseLink = child.Find("base_footprint/base_link");
                    if (baseLink != null)
                    {
                        followTarget = baseLink;
                        break;
                    }
                }
            }
        }

        if (string.IsNullOrWhiteSpace(vehicleId))
            vehicleId = transform.parent != null ? transform.parent.name : name;
    }

    public Camera ResolveFrontCamera()
    {
        if (frontViewCameraOverride != null)
            return frontViewCameraOverride;

        return cameraPublisher != null ? cameraPublisher.GetCamera() : null;
    }

    public Transform ResolveFollowTarget()
    {
        if (followTarget != null)
            return followTarget;

        if (autoFindReferences)
            AutoFindReferences();

        return followTarget;
    }

    /// <summary>
    /// 외부 displayCamera를 지정된 뷰 모드로 포지셔닝한다.
    /// backViewVelocity는 Back View 스무스 댐핑용으로 호출자가 관리한다.
    /// </summary>
    public void ApplyView(Camera displayCamera, ViewMode mode,
        ref Vector3 backViewVelocity, bool forceResetVelocity)
    {
        if (displayCamera == null)
            return;

        Camera sourceCamera = ResolveFrontCamera();
        Transform target = ResolveFollowTarget();

        if (target == null)
            return;

        ApplyBaseDisplayCameraSettings(displayCamera, sourceCamera);

        switch (mode)
        {
            case ViewMode.FrontView:
                ApplyFrontView(displayCamera, sourceCamera, target);
                break;

            case ViewMode.TopView:
                ApplyTopView(displayCamera, sourceCamera, target);
                backViewVelocity = Vector3.zero;
                break;

            case ViewMode.BackView:
                ApplyBackView(displayCamera, sourceCamera, target,
                    ref backViewVelocity, forceResetVelocity);
                break;
        }
    }

    void ApplyBaseDisplayCameraSettings(Camera displayCamera, Camera sourceCamera)
    {
        displayCamera.enabled = true;
        displayCamera.targetTexture = null;
        displayCamera.depth = 100f;

        Camera template = sourceCamera != null ? sourceCamera : fallbackCameraTemplate;
        if (template == null)
            return;

        displayCamera.nearClipPlane = template.nearClipPlane;
        displayCamera.farClipPlane = template.farClipPlane;
        displayCamera.fieldOfView = template.fieldOfView;
        displayCamera.clearFlags = template.clearFlags;
        displayCamera.backgroundColor = template.backgroundColor;
        displayCamera.orthographic = false;
    }

    void ApplyFrontView(Camera displayCamera, Camera sourceCamera, Transform target)
    {
        // camera_link(cameraTransform)를 직접 추적한다.
        // RuntimeFrontCamera + CameraStabilizer 경유 방식은 LateUpdate 실행 순서와
        // AddComponent 초기화 타이밍에 의존하여 FrontView가 고정처럼 보이는 버그를 유발한다.
        if (cameraPublisher != null && cameraPublisher.cameraTransform != null)
        {
            Transform camLink = cameraPublisher.cameraTransform;
            Quaternion rot = camLink.rotation * Quaternion.Euler(cameraPublisher.cameraXRotation, 0f, 0f);
            displayCamera.transform.SetPositionAndRotation(camLink.position, rot);
            if (sourceCamera != null)
                displayCamera.cullingMask = sourceCamera.cullingMask;
            return;
        }

        // 폴백: sourceCamera transform 복사 (frontViewCameraOverride 등 직접 지정된 경우)
        if (sourceCamera != null)
        {
            displayCamera.transform.SetPositionAndRotation(
                sourceCamera.transform.position, sourceCamera.transform.rotation);
            displayCamera.cullingMask = sourceCamera.cullingMask;
            return;
        }

        displayCamera.transform.SetPositionAndRotation(target.position, target.rotation);
    }

    void ApplyTopView(Camera displayCamera, Camera sourceCamera, Transform target)
    {
        displayCamera.transform.position = target.position + Vector3.up * topViewHeight;
        displayCamera.transform.rotation = Quaternion.Euler(90f, 0f, 0f);
        displayCamera.cullingMask = BuildDisplayCullingMask(sourceCamera, includeVehicle: true);
    }

    void ApplyBackView(Camera displayCamera, Camera sourceCamera, Transform target,
        ref Vector3 backViewVelocity, bool forceResetVelocity)
    {
        Vector3 desiredPosition = target.position
            - target.forward * backViewDistance
            + Vector3.up * backViewHeight;

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
                smoothTime);
        }

        displayCamera.transform.LookAt(target.position + Vector3.up * (backViewHeight * 0.5f));
        displayCamera.cullingMask = BuildDisplayCullingMask(sourceCamera, includeVehicle: true);
    }

    int BuildDisplayCullingMask(Camera sourceCamera, bool includeVehicle)
    {
        int mask;
        if (sourceCamera != null)
            mask = sourceCamera.cullingMask;
        else if (fallbackCameraTemplate != null)
            mask = fallbackCameraTemplate.cullingMask;
        else
            mask = ~0;

        if (!includeVehicle || !includeVehicleLayerInTopAndBack)
            return mask;

        int vehicleLayer = LayerMask.NameToLayer(vehicleLayerName);
        if (vehicleLayer >= 0)
            mask |= 1 << vehicleLayer;

        return mask;
    }
}
