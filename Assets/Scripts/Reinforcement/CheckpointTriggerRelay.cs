using UnityEngine;

/// <summary>
/// Checkpoint 자식 오브젝트에 런타임으로 부착되는 릴레이.
/// OnTriggerEnter 이벤트를 FinishLineGate.OnCheckpointPassed()로 전달한다.
/// </summary>
[RequireComponent(typeof(BoxCollider))]
public class CheckpointTriggerRelay : MonoBehaviour
{
    private FinishLineGate _gate;
    private int _rlVehicleLayer;

    public void Initialize(FinishLineGate gate)
    {
        _gate = gate;
        _rlVehicleLayer = LayerMask.NameToLayer("RLVehicle");

        if (_rlVehicleLayer == -1)
            Debug.LogError("[CheckpointTriggerRelay] 'RLVehicle' 레이어가 존재하지 않습니다.");
    }

    private void OnTriggerEnter(Collider other)
    {
        if (_gate == null) return;
        if (other.gameObject.layer != _rlVehicleLayer) return;
        if (other.gameObject.name != "base_link") return;

        _gate.OnCheckpointPassed(other);
    }
}
