using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// 센서 공용 물리 유틸리티.
/// RaycastHit 정렬, self-collider 필터링, 외부 입력 검증 등을 한 곳에서 관리한다.
/// </summary>
public static class SensorPhysicsUtil
{
    private const int IgnoreRaycastLayer = 2;
    private const int UiLayer = 5;
    private const int CameraLayer = 10;

    /// <summary>
    /// RaycastHit를 거리 기준으로 정렬하는 재사용 가능한 Comparer.
    /// Array.Sort에 람다 대신 사용하여 GC 할당을 방지한다.
    /// </summary>
    public static readonly IComparer<RaycastHit> HitDistanceComparer = new HitDistanceComparerImpl();

    private sealed class HitDistanceComparerImpl : IComparer<RaycastHit>
    {
        public int Compare(RaycastHit a, RaycastHit b) => a.distance.CompareTo(b.distance);
    }

    /// <summary>
    /// 콜라이더가 자차(selfRoot) 하위인지 판정한다.
    /// </summary>
    public static bool IsSelfCollider(Collider collider, bool ignoreSelfColliders, Transform selfRoot)
    {
        if (!ignoreSelfColliders || collider == null || selfRoot == null)
            return false;

        Transform ct = collider.transform;
        return ct == selfRoot || ct.IsChildOf(selfRoot);
    }

    /// <summary>
    /// 외부 토픽에서 수신한 거리 값이 유효한지 검증한다.
    /// </summary>
    public static bool IsValidExternalDistance(float distance)
    {
        if (float.IsNaN(distance) || float.IsInfinity(distance))
            return false;
        return distance > 0f;
    }

    /// <summary>
    /// 센서 레이어 마스크에서 물리 감지 대상으로 부적절한 유틸리티 레이어를 제거한다.
    /// Ignore Raycast/UI/Camera는 감지 대상으로 유지할 이유가 거의 없고, 오검출 원인이 되기 쉽다.
    /// </summary>
    public static LayerMask SanitizeDetectionLayerMask(LayerMask mask, Component owner)
    {
        int originalMask = mask.value;
        int sanitizedMask = originalMask &
                            ~(1 << IgnoreRaycastLayer) &
                            ~(1 << UiLayer) &
                            ~(1 << CameraLayer);

        if (sanitizedMask != originalMask && owner != null)
        {
            Debug.LogWarning(
                $"[{owner.GetType().Name}] {owner.name} detectionLayer sanitized: {originalMask} -> {sanitizedMask} " +
                "(removed IgnoreRaycast/UI/Camera layers)"
            );
        }

        mask.value = sanitizedMask;
        return mask;
    }
}
