using UnityEngine;

/// <summary>
/// 결승선 게이트 — RLVehicle 레이어 충돌 기반 성공/실패 판정.
///
/// 씬 구조:
///   FinishEpisode          [이 스크립트]
///   ├── Checkpoint         [BoxCollider] ← checkpointCollider (중간 체크포인트)
///   ├── SuccessGate        [BoxCollider] ← successGateCollider
///   └── FailGate           [BoxCollider] ← failGateCollider
///
/// 동작:
///   1) 차량이 Checkpoint를 통과 → 해당 차량의 RLEpisodeEvaluator에 통과 플래그 기록
///   2) 차량이 Success/FailGate에 진입 → 체크포인트 통과 여부 확인
///      - 통과 O → 성공/실패 판정 진행
///      - 통과 X → 이벤트 무시 (충돌 없는 것과 동일)
///
/// 차량: RootAMR > base_link에 BoxCollider (Layer: RLVehicle) 추가 필요.
/// RLEpisodeEvaluator는 차량 계층에서 GetComponentInChildren으로 자동 탐색.
/// </summary>
public class FinishLineGate : MonoBehaviour
{
    [Header("Gate Colliders")]
    [SerializeField] private BoxCollider successGateCollider;
    [SerializeField] private BoxCollider failGateCollider;

    [Header("Checkpoint")]
    [Tooltip("중간 체크포인트 콜라이더. 이것을 먼저 통과해야 Success/Fail 게이트가 반응한다. 미할당 시 체크포인트 검증 비활성.")]
    [SerializeField] private BoxCollider checkpointCollider;

    private void Start()
    {
        SetupRelay(successGateCollider, isSuccess: true);
        SetupRelay(failGateCollider,    isSuccess: false);
        SetupCheckpointRelay();
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

    private void SetupCheckpointRelay()
    {
        if (checkpointCollider == null) return;

        checkpointCollider.isTrigger = true;

        var relay = checkpointCollider.gameObject.GetComponent<CheckpointTriggerRelay>();
        if (relay == null) relay = checkpointCollider.gameObject.AddComponent<CheckpointTriggerRelay>();
        relay.Initialize(this);
    }

    /// <summary>
    /// 체크포인트 통과 시 CheckpointTriggerRelay에서 호출.
    /// 해당 차량의 RLEpisodeEvaluator에 통과 플래그를 기록한다.
    /// </summary>
    internal void OnCheckpointPassed(Collider vehicleCollider)
    {
        var evaluator = vehicleCollider.transform.root.GetComponentInChildren<RLEpisodeEvaluator>();
        if (evaluator == null || evaluator.IsTerminalReached()) return;

        // 스폰 직후 체크포인트 게이밍 방지: 최소 이동 거리 미달 시 무시
        if (!evaluator.HasTraveledMinDistanceFromSpawn()) return;

        evaluator.SetCheckpointPassed(true);
        Debug.Log("체크포인트 통과!");
    }

    /// <summary>
    /// Success/Fail 게이트 통과 시 GateTriggerRelay에서 호출.
    /// 체크포인트 미통과 시 이벤트를 무시한다.
    /// </summary>
    internal void OnVehicleCrossed(bool isSuccess, Collider vehicleCollider)
    {
        var evaluator = vehicleCollider.transform.root.GetComponentInChildren<RLEpisodeEvaluator>();
        if (evaluator == null || evaluator.IsTerminalReached()) return;

        // 체크포인트가 설정되어 있고, 아직 통과하지 않았으면 무시
        if (checkpointCollider != null && !evaluator.HasPassedCheckpoint())
        {
            Debug.Log("체크포인트 미 통과 XXXX");
            return;
        }

        if (isSuccess)
        {
            evaluator.NotifyFinishCrossed(gameObject.name, 1f, -1f);
            Debug.Log("성공");
        }
        else{
            evaluator.NotifyFailCrossed(gameObject.name, 1f, -1f);
            Debug.Log("실패");
        }
    }
}
