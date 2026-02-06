using UnityEngine;

/// <summary>
/// 강화학습 보상 구역 런타임 컴포넌트
/// 차량이 이 Trigger Collider 위에 있을 때 해당 score를 보상으로 사용합니다.
///
/// 사용법 (RL Agent):
///   void OnTriggerStay(Collider other)
///   {
///       RewardZone zone = other.GetComponent<RewardZone>();
///       if (zone != null) reward += zone.score * Time.fixedDeltaTime;
///   }
/// </summary>
public class RewardZone : MonoBehaviour
{
    [Header("Zone Settings")]
    public string zoneName = "Zone";
    public float score = 0f;
}
