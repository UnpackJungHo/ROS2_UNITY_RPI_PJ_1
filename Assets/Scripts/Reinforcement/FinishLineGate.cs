using UnityEngine;

public class FinishLineGate : MonoBehaviour
{
    [Header("Gate 설정")]
    [SerializeField] private BoxCollider gateCollider;
    [SerializeField] private ArticulationBody targetBody;
    
    [Header("RL 연동")]
    [SerializeField] private RLEpisodeEvaluator episodeEvaluator;
    [SerializeField] private bool autoFindEpisodeEvaluator = true;

    private bool _isInside;
    private bool _enteredFromFront;
    private float _logTimer;

    // 통과 방향 축 (자동 감지: box의 가장 얇은 축)
    private int _thinAxis;
    private Vector3 _throughDirection;

    private void Start()
    {
        if (autoFindEpisodeEvaluator && episodeEvaluator == null)
            episodeEvaluator = FindObjectOfType<RLEpisodeEvaluator>();

        if (gateCollider == null || targetBody == null)
        {
            Debug.LogError($"[FinishLineGate] 할당 안됨! gate:{gateCollider} target:{targetBody}");
            return;
        }

        // Box의 가장 얇은 축 = 통과 방향
        Vector3 size = gateCollider.size;
        if (size.x <= size.y && size.x <= size.z) _thinAxis = 0;
        else if (size.y <= size.x && size.y <= size.z) _thinAxis = 1;
        else _thinAxis = 2;

        // 얇은 축에 해당하는 월드 방향
        switch (_thinAxis)
        {
            case 0: _throughDirection = -gateCollider.transform.right; break;
            case 1: _throughDirection = -gateCollider.transform.up; break;
            case 2: _throughDirection = -gateCollider.transform.forward; break;
        }
        /*
        Debug.Log("============ [FinishLineGate] 디버그 시작 ============");
        Debug.Log($"[FinishLineGate] Box size(local): {size}");
        Debug.Log($"[FinishLineGate] 가장 얇은 축: {_thinAxis} (0=x, 1=y, 2=z)");
        Debug.Log($"[FinishLineGate] 월드 right:   {gateCollider.transform.right}");
        Debug.Log($"[FinishLineGate] 월드 up:      {gateCollider.transform.up}");
        Debug.Log($"[FinishLineGate] 월드 forward:  {gateCollider.transform.forward}");
        Debug.Log($"[FinishLineGate] → 통과방향(월드): {_throughDirection}");
        Debug.Log($"[FinishLineGate] Gate 월드위치: {gateCollider.transform.position}");
        Debug.Log($"[FinishLineGate] Gate 회전: {gateCollider.transform.rotation.eulerAngles}");
        Debug.Log("============ [FinishLineGate] 디버그 끝 ============");
        */
    }

    private void FixedUpdate()
    {
        if (gateCollider == null || targetBody == null)
            return;

        Vector3 targetPos = targetBody.transform.position;
        bool overlapping = IsInsideBox(targetPos);
        bool inFront = IsInFront(targetPos);

        _logTimer += Time.fixedDeltaTime;
        if (_logTimer >= 1f)
        {
            _logTimer = 0f;
            float dot = Vector3.Dot(_throughDirection, targetPos - gateCollider.transform.position);
            //Debug.Log($"[FinishLineGate] 내부:{overlapping} | 앞쪽:{inFront} | dot:{dot:F3} | 위치:{targetPos}");
        }

        if (overlapping && !_isInside)
        {
            _isInside = true;
            _enteredFromFront = inFront;
            Debug.Log($"[FinishLineGate] ★ 진입! 앞쪽진입:{_enteredFromFront}");
        }
        else if (!overlapping && _isInside)
        {
            _isInside = false;
            bool exitedToBehind = !inFront;
            Debug.Log($"[FinishLineGate] ★ 퇴출! 앞진입:{_enteredFromFront} 뒤퇴출:{exitedToBehind}");

            if (_enteredFromFront && exitedToBehind)
            {
                Debug.Log("[FinishLineGate] ★★★ 통과 성공! ★★★");
                if (episodeEvaluator != null)
                {
                    float enterSigned = _enteredFromFront ? 1f : -1f;
                    float exitSigned = exitedToBehind ? -1f : 1f;
                    episodeEvaluator.NotifyFinishCrossed(gameObject.name, enterSigned, exitSigned);
                }
            }
            else
                Debug.Log($"[FinishLineGate] 통과 실패 (앞진입:{_enteredFromFront}, 뒤퇴출:{exitedToBehind})");
        }
    }

    private bool IsInsideBox(Vector3 worldPosition)
    {
        Vector3 local = gateCollider.transform.InverseTransformPoint(worldPosition);
        Vector3 halfSize = gateCollider.size * 0.5f;
        Vector3 center = gateCollider.center;

        return Mathf.Abs(local.x - center.x) <= halfSize.x &&
               Mathf.Abs(local.y - center.y) <= halfSize.y &&
               Mathf.Abs(local.z - center.z) <= halfSize.z;
    }

    /// <summary>
    /// 통과 방향(얇은 축) 기준으로 앞쪽이면 true.
    /// </summary>
    private bool IsInFront(Vector3 position)
    {
        Vector3 toTarget = position - gateCollider.transform.position;
        return Vector3.Dot(_throughDirection, toTarget) > 0f;
    }
}
