using UnityEngine;

/// <summary>
/// base_link에 1개만 부착. 2가지 경로로 충돌을 감지하여 RLEpisodeEvaluator에 중계:
///
/// 1) 자신(base_link)의 OnCollisionEnter:
///    ArticulationBody가 base_link에 있으므로, 하위 콜라이더(Collisions/Box 등)의
///    충돌 콜백도 여기로 전달됨 (Unity 물리 규칙: 콜백은 ArticulationBody 소유자에게).
///
/// 2) 자식 link(별도 ArticulationBody 보유)의 ChildCollisionListener:
///    휠 등 독립 ArticulationBody를 가진 link는 자체적으로 OnCollisionEnter를 받으므로
///    경량 리스너를 런타임에 자동 부착하여 릴레이로 전달.
///
/// base_link의 BoxCollider(isTrigger=true)는 OnCollisionEnter 대상이 아니므로 자연 무시.
/// ignoreSelfColliders=true: 같은 루트(차량) 내 콜라이더끼리의 충돌은 무시.
/// </summary>
public class RLEpisodeCollisionRelay : MonoBehaviour
{
    [Header("References")]
    public RLEpisodeEvaluator episodeEvaluator;
    public VehicleMotionController vehicleMotionController;

    [Header("Settings")]
    public bool autoFindReferences = true;
    [Tooltip("같은 루트 하위의 콜라이더끼리 충돌 시 무시")]
    public bool ignoreSelfColliders = true;

    [Header("Debug (Read Only)")]
    [SerializeField] private int attachedListenerCount = 0;

    private Transform vehicleRoot;

    void Start()
    {
        if (autoFindReferences)
            AutoFindReferences();

        vehicleRoot = transform.root;

        AttachListenersToChildLinks();
    }

    void AutoFindReferences()
    {
        if (episodeEvaluator == null)
            episodeEvaluator = GetComponentInParent<RLEpisodeEvaluator>()
                            ?? transform.root.GetComponentInChildren<RLEpisodeEvaluator>();

        if (vehicleMotionController == null)
            vehicleMotionController = GetComponent<VehicleMotionController>()
                                   ?? GetComponentInParent<VehicleMotionController>();
    }

    // ────────────────────────────────────────
    //  경로 1: base_link 자체의 OnCollisionEnter
    //  → base_link의 ArticulationBody에 연결된 모든 콜라이더
    //    (Collisions/unnamed/Box 등)의 충돌이 여기로 옴
    // ────────────────────────────────────────

    void OnCollisionEnter(Collision collision)
    {
        if (collision == null || collision.collider == null)
            return;

        HandleCollision(collision.collider, collision.relativeVelocity.magnitude);
    }

    // ────────────────────────────────────────
    //  경로 2: 자식 link(독립 ArticulationBody)의 리스너
    //  → 휠 등 별도 ArticulationBody를 가진 link에 자동 부착
    // ────────────────────────────────────────

    /// <summary>
    /// base_link 하위에서 별도 ArticulationBody를 가진 자식 link를 찾아
    /// ChildCollisionListener를 부착한다.
    /// base_link 자체의 ArticulationBody에 연결된 콜라이더는 경로 1로 처리되므로 건너뛴다.
    /// </summary>
    void AttachListenersToChildLinks()
    {
        attachedListenerCount = 0;
        ArticulationBody rootAB = GetComponent<ArticulationBody>();

        // 하위의 모든 ArticulationBody를 찾아, 루트가 아닌 것만 리스너 부착
        ArticulationBody[] childBodies = GetComponentsInChildren<ArticulationBody>(true);
        foreach (var ab in childBodies)
        {
            if (ab == rootAB) continue; // 자기 자신 = 경로 1에서 처리

            // 이 link에 콜라이더가 있거나 하위에 콜라이더가 있는 경우만
            if (ab.GetComponentInChildren<Collider>(true) == null) continue;

            var existing = ab.gameObject.GetComponent<ChildCollisionListener>();
            if (existing != null)
            {
                existing.relay = this;
                attachedListenerCount++;
                continue;
            }

            var listener = ab.gameObject.AddComponent<ChildCollisionListener>();
            listener.relay = this;
            attachedListenerCount++;
        }

        Debug.Log($"[RLEpisodeCollisionRelay] Self OnCollisionEnter active on {gameObject.name}, " +
                  $"+ {attachedListenerCount} child link listeners attached");
    }

    // ────────────────────────────────────────
    //  공통 처리
    // ────────────────────────────────────────

    /// <summary>자식 리스너 또는 자체 OnCollisionEnter에서 호출.</summary>
    public void HandleCollision(Collider otherCollider, float relativeSpeed)
    {
        if (episodeEvaluator == null || otherCollider == null)
            return;

        // 같은 차량 내부 충돌 무시 (휠 ↔ 바디 등)
        if (ignoreSelfColliders && vehicleRoot != null && otherCollider.transform.root == vehicleRoot)
            return;

        // Debug.Log($"[RLEpisodeCollisionRelay] Collision → {otherCollider.name} " +
        //           $"(layer={LayerMask.LayerToName(otherCollider.gameObject.layer)}, relV={relativeSpeed:F2}m/s)");

        episodeEvaluator.NotifyCollision(otherCollider, relativeSpeed);
    }

    /// <summary>
    /// 독립 ArticulationBody를 가진 자식 link에 부착되는 경량 리스너.
    /// </summary>
    [DisallowMultipleComponent]
    public class ChildCollisionListener : MonoBehaviour
    {
        [HideInInspector] public RLEpisodeCollisionRelay relay;

        void OnCollisionEnter(Collision collision)
        {
            if (relay == null || collision == null || collision.collider == null)
                return;

            relay.HandleCollision(collision.collider, collision.relativeVelocity.magnitude);
        }
    }
}
