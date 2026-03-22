using UnityEngine;

/// <summary>
/// SuccessGate / FailGate 자식 오브젝트에 런타임으로 자동 부착되는 릴레이 컴포넌트.
/// OnTriggerEnter 이벤트를 FinishLineGate로 전달한다.
/// </summary>
[RequireComponent(typeof(BoxCollider))]
public class GateTriggerRelay : MonoBehaviour
{
    private FinishLineGate _gate;
    private bool _isSuccess;
    private int  _rlVehicleLayer;

    public void Initialize(FinishLineGate gate, bool isSuccess)
    {
        _gate           = gate;
        _isSuccess      = isSuccess;
        _rlVehicleLayer = LayerMask.NameToLayer("RLVehicle");

        if (_rlVehicleLayer == -1)
            Debug.LogError("[GateTriggerRelay] 'RLVehicle' 레이어가 존재하지 않습니다. Project Settings > Tags and Layers에서 추가하세요.");
    }

    private void OnTriggerEnter(Collider other)
    {
        if (_gate == null) return;
        if (other.gameObject.layer != _rlVehicleLayer)return;
        if (other.gameObject.name != "base_link") return;
        //Debug.Log(other.gameObject.name);
        _gate.OnVehicleCrossed(_isSuccess, other);
    }
}
