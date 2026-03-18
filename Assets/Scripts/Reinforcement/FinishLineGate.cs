using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 이중 결승선 게이트 (성공 / 실패) + 16대 멀티에이전트 지원.
///
/// 씬 구조:
///   FinishEpisode  [이 스크립트]
///   ├── SuccessGate  [BoxCollider] ← successGateCollider
///   └── FailGate     [BoxCollider] ← failGateCollider
///
/// 인덱스 매핑: targetBodies[i] ↔ episodeEvaluators[i] (1:1)
/// 인스펙터에서 직접 할당 (autoFind 미지원).
///
/// 판정:
///   successGate 앞→뒤 통과 → episodeEvaluators[i].NotifyFinishCrossed() → 성공
///   failGate    앞→뒤 통과 → episodeEvaluators[i].NotifyFailCrossed()   → 실패(WrongLane)
///
/// 주의: BoxCollider의 isTrigger 설정 불필요 — IsInsideBox는 수동 AABB로 판정한다.
/// </summary>
public class FinishLineGate : MonoBehaviour
{
    [Header("Gate 설정")]
    [SerializeField] private BoxCollider successGateCollider;   // 오른쪽 차선 — 통과 시 성공
    [SerializeField] private BoxCollider failGateCollider;      // 왼쪽 차선  — 통과 시 실패

    [Header("RL 연동 (인덱스 1:1 매핑)")]
    [SerializeField] private List<ArticulationBody> targetBodies = new();
    [SerializeField] private List<RLEpisodeEvaluator> episodeEvaluators = new();

    // ── 게이트별 thin-axis (자식 오브젝트 방향이 다를 수 있으므로 독립 계산) ──
    private int     _successThinAxis;
    private Vector3 _successThroughDirection;
    private int     _failThinAxis;
    private Vector3 _failThroughDirection;

    // ── 차량별 게이트 통과 상태 ──
    private struct GateState
    {
        public bool isInside;
        public bool enteredFromFront;
    }

    private GateState[] _successStates;
    private GateState[] _failStates;
    private int _effectiveCount;

    private void Start()
    {
        // 기본 유효성 검사
        if (successGateCollider == null || failGateCollider == null)
        {
            Debug.LogError("[FinishLineGate] successGateCollider 또는 failGateCollider가 할당되지 않았습니다.");
            return;
        }

        if (targetBodies.Count == 0 || episodeEvaluators.Count == 0)
        {
            Debug.LogError("[FinishLineGate] targetBodies 또는 episodeEvaluators 리스트가 비어 있습니다.");
            return;
        }

        if (targetBodies.Count != episodeEvaluators.Count)
        {
            Debug.LogError($"[FinishLineGate] 리스트 크기 불일치: targetBodies={targetBodies.Count}, " +
                           $"episodeEvaluators={episodeEvaluators.Count}. Min 크기로 순회합니다.");
        }

        _effectiveCount = Mathf.Min(targetBodies.Count, episodeEvaluators.Count);

        // 게이트별 통과 방향 독립 계산 (자식 오브젝트 방향이 다를 수 있음)
        _successThinAxis         = ComputeThinAxis(successGateCollider);
        _successThroughDirection = ComputeThroughDirection(successGateCollider, _successThinAxis);

        _failThinAxis            = ComputeThinAxis(failGateCollider);
        _failThroughDirection    = ComputeThroughDirection(failGateCollider, _failThinAxis);

        // 차량별 상태 배열 초기화
        _successStates = new GateState[_effectiveCount];
        _failStates    = new GateState[_effectiveCount];
    }

    private void FixedUpdate()
    {
        if (successGateCollider == null || failGateCollider == null)
            return;

        for (int i = 0; i < _effectiveCount; i++)
        {
            ArticulationBody body = targetBodies[i];
            RLEpisodeEvaluator evaluator = episodeEvaluators[i];

            if (body == null || evaluator == null)
                continue;

            // 에피소드 비활성(리셋 포함) 시 GateState 초기화.
            // 텔레포트 후 게이트 내부에 스폰되는 경우 오검출 방지.
            if (!evaluator.IsEpisodeActive())
            {
                _successStates[i] = default;
                _failStates[i]    = default;
                continue;
            }

            Vector3 pos = body.transform.position;

            // ── 성공 게이트 ──
            // onCrossed 람다는 UpdateGateState 내에서 동기 호출됨(즉시 실행).
            // enterSigned: 앞에서 진입하면 1f, 뒤에서 진입하면 -1f
            // exitSigned: onCrossed 진입 조건(앞진입+뒤퇴출)이 보장하므로 항상 -1f
            int capturedI = i;  // 람다 클로저 안전용 복사
            UpdateGateState(
                ref _successStates[i],
                pos,
                successGateCollider,
                _successThroughDirection,
                onCrossed: (bool enteredFront) =>
                {
                    float enterSigned = enteredFront ? 1f : -1f;
                    Debug.Log($"[FinishLineGate] ★★★ 성공! vehicle[{capturedI}]");
                    evaluator.NotifyFinishCrossed(gameObject.name, enterSigned, -1f);
                }
            );

            // ── 실패 게이트 ──
            UpdateGateState(
                ref _failStates[i],
                pos,
                failGateCollider,
                _failThroughDirection,
                onCrossed: (bool enteredFront) =>
                {
                    float enterSigned = enteredFront ? 1f : -1f;
                    Debug.Log($"[FinishLineGate] ★ 실패(WrongLane)! vehicle[{capturedI}]");
                    evaluator.NotifyFailCrossed(gameObject.name, enterSigned, -1f);
                }
            );
        }
    }

    /// <summary>
    /// 차량의 게이트 내부/외부 전환을 감지하여 앞→뒤 통과 시 onCrossed 콜백을 동기 호출한다.
    /// onCrossed 인자: 진입 시 앞(front)에서 들어왔는지 여부.
    /// </summary>
    private void UpdateGateState(
        ref GateState state,
        Vector3 targetPos,
        BoxCollider col,
        Vector3 throughDirection,
        System.Action<bool> onCrossed)
    {
        bool overlapping = IsInsideBox(targetPos, col);
        bool inFront     = IsInFront(targetPos, col, throughDirection);

        if (overlapping && !state.isInside)
        {
            state.isInside         = true;
            state.enteredFromFront = inFront;
        }
        else if (!overlapping && state.isInside)
        {
            bool wasEnteredFromFront = state.enteredFromFront;
            state.isInside = false;

            bool exitedToBehind = !inFront;
            if (wasEnteredFromFront && exitedToBehind)
                onCrossed?.Invoke(wasEnteredFromFront);
        }
    }

    // ── 헬퍼 ──

    /// <summary>Box의 가장 얇은 축 인덱스를 반환한다 (0=x, 1=y, 2=z).</summary>
    private static int ComputeThinAxis(BoxCollider col)
    {
        Vector3 size = col.size;
        if (size.x <= size.y && size.x <= size.z) return 0;
        if (size.y <= size.x && size.y <= size.z) return 1;
        return 2;
    }

    /// <summary>얇은 축에 해당하는 월드 방향 벡터를 반환한다.</summary>
    private static Vector3 ComputeThroughDirection(BoxCollider col, int thinAxis)
    {
        return thinAxis switch
        {
            0 => -col.transform.right,
            1 => -col.transform.up,
            _ => -col.transform.forward,
        };
    }

    /// <summary>월드 좌표 worldPosition이 col의 로컬 박스 내부인지 반환한다.</summary>
    private static bool IsInsideBox(Vector3 worldPosition, BoxCollider col)
    {
        Vector3 local    = col.transform.InverseTransformPoint(worldPosition);
        Vector3 halfSize = col.size * 0.5f;
        Vector3 center   = col.center;

        return Mathf.Abs(local.x - center.x) <= halfSize.x &&
               Mathf.Abs(local.y - center.y) <= halfSize.y &&
               Mathf.Abs(local.z - center.z) <= halfSize.z;
    }

    /// <summary>throughDirection 기준으로 position이 col의 앞쪽이면 true.</summary>
    private static bool IsInFront(Vector3 position, BoxCollider col, Vector3 throughDirection)
    {
        Vector3 toTarget = position - col.transform.position;
        return Vector3.Dot(throughDirection, toTarget) > 0f;
    }
}
