using UnityEngine;

/// <summary>
/// ProgressRewardProvider가 런타임에 triggerTarget(base_link 등)에 자동 부착하는 프록시.
/// 트리거 이벤트를 ProgressRewardProvider로 중계합니다.
/// </summary>
[AddComponentMenu("")] // Inspector 메뉴에 노출하지 않음
public class ProgressRewardProxy : MonoBehaviour
{
    [HideInInspector] public ProgressRewardProvider owner;

    void OnTriggerEnter(Collider other)
    {
        if (owner != null) owner.NotifyTriggerEnter(other);
    }

    void OnTriggerStay(Collider other)
    {
        if (owner != null) owner.NotifyTriggerStay(other);
    }

    void OnTriggerExit(Collider other)
    {
        if (owner != null) owner.NotifyTriggerExit(other);
    }
}
