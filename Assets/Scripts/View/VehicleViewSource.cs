using UnityEngine;

[DisallowMultipleComponent]
public class VehicleViewSource : MonoBehaviour
{
    [Header("Identity")]
    public string vehicleId = "Vehicle";

    [Header("References")]
    public CameraPublisher cameraPublisher;
    public Camera frontViewCameraOverride;
    public Transform followTarget;

    [Header("Auto Find")]
    public bool autoFindReferences = true;

    [Header("View Offsets")]
    public float topViewHeight = 5f;
    public float backViewDistance = 3f;
    public float backViewHeight = 1.5f;
    public float smoothTime = 0.1f;

    public string VehicleId => string.IsNullOrWhiteSpace(vehicleId) ? name : vehicleId;

    void Awake()
    {
        if (autoFindReferences)
            AutoFindReferences();
    }

    void Reset()
    {
        AutoFindReferences();
    }

    public void AutoFindReferences()
    {
        if (cameraPublisher == null)
            cameraPublisher = GetComponentInChildren<CameraPublisher>(true);

        if (followTarget == null)
        {
            Transform baseLink = transform.Find("base_footprint/base_link");
            if (baseLink != null)
                followTarget = baseLink;
        }

        if (string.IsNullOrWhiteSpace(vehicleId))
            vehicleId = name;
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
}
