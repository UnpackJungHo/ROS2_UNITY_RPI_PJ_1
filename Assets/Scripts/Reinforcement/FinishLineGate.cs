using UnityEngine;

/// <summary>
/// 결승선 게이트 — RLVehicle 레이어 충돌 기반 성공/실패 판정.
///
/// 씬 구조:
///   FinishEpisode  [이 스크립트]
///   ├── SuccessGate  [BoxCollider] ← successGateCollider
///   └── FailGate     [BoxCollider] ← failGateCollider
///
/// 차량: RootAMR > base_link에 BoxCollider (Layer: RLVehicle) 추가 필요.
/// RLEpisodeEvaluator는 차량 계층의 부모에서 GetComponentInParent로 자동 탐색.
/// </summary>
public class FinishLineGate : MonoBehaviour
{
    [Header("Gate Colliders")]
    [SerializeField] private BoxCollider successGateCollider;
    [SerializeField] private BoxCollider failGateCollider;

    private void Start()
    {
        SetupRelay(successGateCollider, isSuccess: true);
        SetupRelay(failGateCollider,    isSuccess: false);
    }

    private void SetupRelay(BoxCollider col, bool isSuccess)
    {
        if (col == null)
        {
            Debug.LogError($"[FinishLineGate] {(isSuccess ? "Success" : "Fail")}GateCollider가 할당되지 않았습니다.");
            return;
        }

        col.isTrigger = true;

        var relay = col.gameObject.GetComponent<GateTriggerRelay>();
        if (relay == null) relay = col.gameObject.AddComponent<GateTriggerRelay>();
        relay.Initialize(this, isSuccess);
    }

    internal void OnVehicleCrossed(bool isSuccess, Collider vehicleCollider)
    {
        var evaluator = vehicleCollider.transform.root.GetComponentInChildren<RLEpisodeEvaluator>();
        if (evaluator == null || evaluator.IsTerminalReached()) return;

        if (isSuccess)
        {
            //Debug.Log($"[FinishLineGate] ★★★ 성공! {vehicleCollider.transform.root.name}");
            evaluator.NotifyFinishCrossed(gameObject.name, 1f, -1f);
        }
        else
        {
            //Debug.Log($"[FinishLineGate] ★ 실패(WrongLane)! {vehicleCollider.transform.root.name}");
            evaluator.NotifyFailCrossed(gameObject.name, 1f, -1f);
        }
    }
}
