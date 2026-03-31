using System;
using UnityEngine;

/// <summary>
/// 강화학습 에피소드 종료/평가기.
/// - 성공: FinishLineGate 통과
/// - 실패: 위험단계(5,6) 정지 지속, 충돌, 타임아웃
/// - 종료 시 상태를 고정하고 종료 이벤트를 발행
/// </summary>
public class RLEpisodeEvaluator : MonoBehaviour
{
    /// <summary>에피소드 종료 유형을 나타내는 열거형.</summary>
    public enum TerminalType
    {
        None = 0,            // 종료되지 않음 (에피소드 진행 중)
        Finish = 1,          // 성공: 결승선(FinishLineGate) 통과
        FailRiskStop = 2,    // 실패: 위험 단계에서 저속 정지가 유지됨 (장애물 앞 교착)
        FailCollision = 3,   // 실패: 다른 오브젝트와 물리적 충돌 발생
        FailTimeout = 4,     // 실패: 제한 시간 초과
        FailStuck = 5,       // 실패: 일정 시간 동안 거의 이동하지 못함 (경로 이탈/멈춤)
        FailWrongLane = 6    // 실패: 왼쪽 차선(실패 게이트) 통과
    }

    [Header("References")]
    /// <summary>경로 진행률 기반 보상을 계산하는 프로바이더 (path progress reward shaping).</summary>
    public ProgressRewardProvider progressRewardProvider;
    /// <summary>전방 장애물과의 TTC(Time-To-Collision) 기반 경고 레벨을 발행하는 퍼블리셔.</summary>
    public CollisionWarningEngine collisionWarningEngine;
    /// <summary>Ackermann 조향/스로틀/브레이크를 제어하는 차량 컨트롤러.</summary>
    public VehicleMotionController vehicleMotionController;
    /// <summary>신호등 주행 결정을 제공하는 엔진. 종료 시점의 traffic 상태 로그에 사용.</summary>
    public TrafficLightDecisionEngine trafficLightDecisionEngine;
    /// <summary>회귀 모델(신경망) 기반 자율주행 컨트롤러. 에피소드 종료 시 자율 모드를 끌 때 사용.</summary>
    public RegressionDrivingController regressionDrivingController;
    /// <summary>Residual RL Agent. 종료 시점의 base/delta/final 제어값 진단 로그에 사용.</summary>
    public AutoDriverRLAgent autoDriverRLAgent;

    [Header("Episode Control")]
    /// <summary>Start() 시 에피소드를 즉시 시작할지 여부.</summary>
    public bool autoBeginOnStart = true;
    /// <summary>에피소드 종료 시 차량을 강제 정지(브레이크 100%)시킬지 여부.</summary>
    public bool stopVehicleOnTerminal = true;
    /// <summary>에피소드 최대 허용 시간(초). 0 이하이면 타임아웃 비활성.
    /// 연속 학습 모드에서는 0으로 설정 — max_steps가 학습 한계를 결정함.</summary>
    [Tooltip("0 이하이면 타임아웃 비활성. 연속 학습 시 0으로 설정")]
    public float maxEpisodeSeconds = 0f;
    /// <summary>true면 FinishLineGate 통과 시 에피소드 종료. 연속 학습 시 false로 설정.</summary>
    [Tooltip("false면 결승선 통과 시에도 에피소드를 종료하지 않음 (연속 학습 모드)")]
    public bool enableFinishLineTerminal = false;

    [Header("Failure Rules")]
    /// <summary>이 경고 레벨(예: Brake=5) 이상에서 저속 정지가 유지되면 FailRiskStop으로 판정.</summary>
    [Tooltip("이 레벨 이상에서 저속 정지가 유지되면 실패")]
    public CollisionWarningEngine.WarningLevel dangerLevelThreshold = CollisionWarningEngine.WarningLevel.Brake;
    /// <summary>위험 레벨 + 정지 상태가 이 시간(초) 이상 지속되면 FailRiskStop 종료.</summary>
    [Tooltip("위험레벨 + 정지 상태가 이 시간 지속되면 FailRiskStop")]
    public float dangerStopHoldSeconds = 0.7f;
    /// <summary>차량이 "정지" 상태로 간주되는 속도 임계값(m/s). 이 이하이면 정지로 판정.</summary>
    public float stoppedSpeedThreshold = 0.05f;
    /// <summary>경로 대비 횡방향 오차(lateral error) 허용 최대값(m). 0이면 비활성, 초과 시 실패.</summary>
    [Tooltip("0보다 크면 횡오차로 실패 처리(옵션)")]
    public float maxLateralErrorForFailure = 0f;
    /// <summary>충돌 실패를 판정할 레이어 마스크. 해당 레이어의 오브젝트와 충돌해야 실패 처리.</summary>
    public LayerMask collisionFailLayers = ~0;

    [Header("Checkpoint Anti-Exploit")]
    [Tooltip("체크포인트 유효화에 필요한 스폰 지점 대비 최소 이동 거리(m). 0이면 비활성.")]
    public float checkpointMinDistanceFromSpawn = 5f;

    [Header("Stuck Detection")]
    /// <summary>에피소드 시작 직후 모델 로드/초기 추론 대기를 위한 유예 시간(초). 이 기간에는 stuck 감지 비활성.</summary>
    [Tooltip("에피소드 시작 후 이 시간(초)이 지나야 stuck 감지 시작")]
    public float stuckGracePeriod = 5f;
    /// <summary>슬라이딩 윈도우 길이(초). 이 시간 동안 이동거리를 누적하여 stuck 여부 판정.</summary>
    [Tooltip("이 시간(초) 동안 이동거리가 stuckDistanceThreshold 미만이면 FailStuck")]
    public float stuckTimeWindow = 10f;
    /// <summary>stuckTimeWindow 내 최소 이동거리(m). 미달 시 차량이 멈춘 것(stuck)으로 판정.</summary>
    [Tooltip("stuckTimeWindow 동안 최소 이동거리 (m)")]
    public float stuckDistanceThreshold = 1f;

    [Header("Debug (Read Only)")]
    /// <summary>현재까지 실행된 에피소드 누적 인덱스 (1부터 시작).</summary>
    [SerializeField] private int episodeIndex = 0;
    /// <summary>현재 에피소드가 진행 중인지 여부.</summary>
    [SerializeField] private bool episodeActive = false;
    /// <summary>종료 조건(성공 또는 실패)에 도달했는지 여부.</summary>
    [SerializeField] private bool terminalReached = false;
    /// <summary>현재 에피소드가 성공(Finish)으로 종료되었는지 여부.</summary>
    [SerializeField] private bool episodeSuccess = false;
    /// <summary>중간 체크포인트를 통과했는지 여부. FinishLineGate에서 설정, BeginEpisode에서 리셋.</summary>
    [SerializeField] private bool checkpointPassed = false;
    /// <summary>에피소드가 어떤 유형으로 종료되었는지 (Finish, FailRiskStop 등).</summary>
    [SerializeField] private TerminalType terminalType = TerminalType.None;
    /// <summary>종료 사유를 사람이 읽을 수 있는 문자열로 기록 (로그/CSV용).</summary>
    [SerializeField] private string terminalReason = "None";
    /// <summary>에피소드 시작 후 경과 시간(초).</summary>
    [SerializeField] private float elapsedSeconds = 0f;
    /// <summary>위험 레벨 + 정지 상태가 연속으로 유지된 시간(초). 임계값 초과 시 FailRiskStop.</summary>
    [SerializeField] private float dangerStoppedDuration = 0f;
    /// <summary>에피소드 내 위험 레벨(dangerLevelThreshold 이상)에 머문 총 누적 시간(초).</summary>
    [SerializeField] private float timeAtDangerLevel = 0f;
    /// <summary>에피소드 내 발생한 물리 충돌 횟수.</summary>
    [SerializeField] private int collisionCount = 0;
    /// <summary>마지막으로 충돌한 게임오브젝트 이름 (디버그용).</summary>
    [SerializeField] private string lastCollisionObjectName = "None";
    /// <summary>마지막 충돌의 상대 속도(m/s). 충돌 심각도 판단 지표.</summary>
    [SerializeField] private float lastCollisionRelativeSpeed = 0f;
    /// <summary>에피소드 중 관측된 최고 경고 레벨 (Safe → ... → Brake 순).</summary>
    [SerializeField] private CollisionWarningEngine.WarningLevel maxWarningLevel =
        CollisionWarningEngine.WarningLevel.Safe;
    /// <summary>에피소드 중 관측된 최소 TTC(Time-To-Collision, 초). 장애물 근접도 지표.</summary>
    [SerializeField] private float minObservedTtc = float.PositiveInfinity;
    /// <summary>stuck 감지용 슬라이딩 윈도우 경과 타이머(초).</summary>
    [SerializeField] private float stuckTimer = 0f;
    /// <summary>stuck 감지 윈도우 내 경로 진행 거리(m). pathS 기반.</summary>
    [SerializeField] private float stuckDistanceAccum = 0f;
    /// <summary>stuck 감지를 위해 이전 프레임의 차량 월드 위치를 저장 (fallback용).</summary>
    private Vector3 stuckLastPosition;
    /// <summary>체크포인트 익스플로잇 방지용 에피소드 시작 위치.</summary>
    private Vector3 episodeSpawnPosition;
    /// <summary>stuck 감지 윈도우 시작 시점의 경로 진행거리(pathS). 경로 진행 기반 stuck 판정에 사용.</summary>
    private float stuckPathSAtWindowStart = 0f;

    /// <summary>에피소드 종료 시 외부 구독자에게 알리는 이벤트. 리플레이 버퍼 저장, 모델 업데이트 등에 활용.</summary>
    public event Action<RLEpisodeEvaluator> OnEpisodeTerminated;

    void Start()
    {
        if (autoBeginOnStart)
        {
            if (autoDriverRLAgent == null)
                autoDriverRLAgent = GetComponent<AutoDriverRLAgent>() ?? GetComponentInParent<AutoDriverRLAgent>();
            else
                return;

            BeginEpisode();
        }
    }

    void Update()
    {
        if (!episodeActive || terminalReached)
            return;

        float dt = Time.deltaTime;
        elapsedSeconds += dt;
        UpdateRiskMetrics(dt);

        if (maxEpisodeSeconds > 0f && elapsedSeconds >= maxEpisodeSeconds)
        {
            SetTerminal(
                TerminalType.FailTimeout,
                $"Timeout({elapsedSeconds:F1}s >= {maxEpisodeSeconds:F1}s)"
            );
            return;
        }

        if (maxLateralErrorForFailure > 0f && progressRewardProvider != null)
        {
            float lateral = progressRewardProvider.GetCurrentLateralError();
            if (lateral > maxLateralErrorForFailure)
            {
                SetTerminal(
                    TerminalType.FailRiskStop,
                    $"OffPath(lateral={lateral:F2}m > {maxLateralErrorForFailure:F2}m)"
                );
                return;
            }
        }

        UpdateStuckDetection(dt);
    }

    /// <summary>
    /// 새 에피소드를 시작한다. 모든 런타임 상태(점수, 타이머, 충돌 카운트 등)를
    /// 초기화하고 ProgressRewardProvider의 보상 상태도 리셋한다.
    /// </summary>
    public void BeginEpisode()
    {
        episodeIndex++;
        episodeActive = true;
        terminalReached = false;
        episodeSuccess = false;
        checkpointPassed = false;
        terminalType = TerminalType.None;
        terminalReason = "None";
        elapsedSeconds = 0f;
        dangerStoppedDuration = 0f;
        timeAtDangerLevel = 0f;
        collisionCount = 0;
        lastCollisionObjectName = "None";
        lastCollisionRelativeSpeed = 0f;
        maxWarningLevel = CollisionWarningEngine.WarningLevel.Safe;
        minObservedTtc = float.PositiveInfinity;
        stuckTimer = 0f;
        stuckDistanceAccum = 0f;
        stuckLastPosition = vehicleMotionController != null ? vehicleMotionController.transform.position : transform.position;
        episodeSpawnPosition = stuckLastPosition;
        stuckPathSAtWindowStart = progressRewardProvider != null ? progressRewardProvider.GetCurrentPathS() : 0f;

        if (progressRewardProvider != null)
            progressRewardProvider.ResetRewardState();
    }

    /// <summary>
    /// 매 프레임 호출되어 위험 지표를 갱신한다.
    /// 위험 정지가 dangerStopHoldSeconds를 초과하면 FailRiskStop으로 에피소드를 종료한다.
    /// </summary>
    void UpdateRiskMetrics(float dt)
    {
        if (collisionWarningEngine == null)
            return;

        CollisionWarningEngine.WarningLevel level = collisionWarningEngine.GetWarningLevel();
        if (level > maxWarningLevel)
            maxWarningLevel = level;

        if (level >= dangerLevelThreshold)
        {
            timeAtDangerLevel += dt;
        }

        float obstacleDist = collisionWarningEngine.GetDistanceToObstacle();
        if (!float.IsInfinity(obstacleDist))
            minObservedTtc = Mathf.Min(minObservedTtc, obstacleDist);

        float speed = vehicleMotionController != null ? Mathf.Abs(vehicleMotionController.GetSpeedMS()) : 0f;
        bool isDangerStop = level >= dangerLevelThreshold && speed <= stoppedSpeedThreshold;

        if (isDangerStop)
            dangerStoppedDuration += dt;
        else
            dangerStoppedDuration = 0f;

        if (dangerStoppedDuration >= dangerStopHoldSeconds)
        {
            SetTerminal(
                TerminalType.FailRiskStop,
                $"DangerStop(level={(int)level}, hold={dangerStoppedDuration:F2}s, speed={speed:F2}m/s)"
            );
        }
    }

    /// <summary>
    /// 차량의 이동 불능(stuck) 상태를 감지한다.
    /// 슬라이딩 윈도우(stuckTimeWindow) 동안 경로 진행거리(pathS)가 stuckDistanceThreshold 미만이면
    /// FailStuck으로 에피소드를 종료한다. 에피소드 초기 grace period 동안은 감지를 건너뛴다.
    ///
    /// 경로 진행 기반: 물리 슬라이딩/서스펜션 노이즈와 무관하게 실제 경로 진행만 측정.
    /// progressRewardProvider가 없을 때만 XZ 평면 거리로 fallback.
    /// </summary>
    void UpdateStuckDetection(float dt)
    {
        if (stuckTimeWindow <= 0f)
            return;

        Vector3 currentPos = vehicleMotionController != null ? vehicleMotionController.transform.position : transform.position;
        float currentPathS = progressRewardProvider != null ? progressRewardProvider.GetCurrentPathS() : -1f;

        // Grace period: 윈도우 기준점만 갱신하고 감지하지 않음
        if (elapsedSeconds < stuckGracePeriod)
        {
            stuckLastPosition = currentPos;
            stuckPathSAtWindowStart = currentPathS >= 0f ? currentPathS : 0f;
            return;
        }

        stuckTimer += dt;

        // 진단 표시용: 현재 윈도우 내 경로 진행거리 갱신
        if (currentPathS >= 0f)
            stuckDistanceAccum = currentPathS - stuckPathSAtWindowStart;

        if (stuckTimer >= stuckTimeWindow)
        {
            bool isStuck;
            float movedAmount;

            if (currentPathS >= 0f)
            {
                // 경로 진행(pathS) 기반 판정: 도로 밖 슬라이딩은 진행으로 카운트 안 됨
                movedAmount = currentPathS - stuckPathSAtWindowStart;
                isStuck = movedAmount < stuckDistanceThreshold;
            }
            else
            {
                // fallback: XZ 평면 거리 (progressRewardProvider 없을 때)
                Vector3 delta = currentPos - stuckLastPosition;
                delta.y = 0f;
                movedAmount = delta.magnitude;
                isStuck = movedAmount < stuckDistanceThreshold;
            }

            stuckDistanceAccum = movedAmount;

            if (isStuck)
            {
                SetTerminal(
                    TerminalType.FailStuck,
                    $"Stuck(pathProgress={movedAmount:F2}m in {stuckTimer:F1}s < {stuckDistanceThreshold:F1}m)"
                );
                return;
            }

            // 윈도우 리셋
            stuckTimer = 0f;
            stuckDistanceAccum = 0f;
            stuckPathSAtWindowStart = currentPathS >= 0f ? currentPathS : 0f;
            stuckLastPosition = currentPos;
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision == null)
            return;

        NotifyCollision(collision.collider, collision.relativeVelocity.magnitude);
    }

    /// <summary>
    /// 물리 충돌 또는 트리거 릴레이로부터 전달된 충돌 후보를 실패 조건으로 판정한다.
    /// 반환값은 실제로 FailCollision 종료가 발생했는지 여부다.
    /// </summary>
    public bool NotifyCollision(Collider otherCollider, float relativeSpeed)
    {
        if (!episodeActive || terminalReached || otherCollider == null)
            return false;

        int otherLayerMask = 1 << otherCollider.gameObject.layer;
        if ((collisionFailLayers.value & otherLayerMask) == 0)
            return false;

        collisionCount++;
        lastCollisionObjectName = otherCollider.name;
        lastCollisionRelativeSpeed = relativeSpeed;
        // Debug.Log(
        //     $"[RLEpisodeEvaluator] Collision detected: obj={otherCollider.name}, layer={LayerMask.LayerToName(otherCollider.gameObject.layer)}, relV={relativeSpeed:F2}m/s, count={collisionCount}",
        //     this
        // );
        SetTerminal(
            TerminalType.FailCollision,
            $"Collision(obj={otherCollider.name}, relV={relativeSpeed:F2}m/s)"
        );
        return true;
    }

    /// <summary>
    /// FinishLineGate 트리거에서 호출되어 결승선 통과(성공)를 알린다.
    /// enterSigned/exitSigned는 게이트 평면 기준 진입·이탈 부호 거리로, 정방향 통과를 검증한다.
    /// </summary>
    public void NotifyFinishCrossed(string gateName, float enterSigned, float exitSigned)
    {
        if (!episodeActive || terminalReached)
            return;

        // 연속 학습 모드에서는 결승선 통과를 무시 — 위치 리셋 없이 학습 계속
        if (!enableFinishLineTerminal)
            return;

        SetTerminal(
            TerminalType.Finish,
            $"Finish(gate={gateName}, enter={enterSigned:F3}, exit={exitSigned:F3})"
        );
    }

    /// <summary>
    /// FinishLineGate의 실패 게이트(왼쪽 차선)를 통과했을 때 호출된다.
    /// enableFinishLineTerminal이 true일 때만 에피소드를 FailWrongLane으로 종료한다.
    /// </summary>
    public void NotifyFailCrossed(string gateName, float enterSigned, float exitSigned)
    {
        if (!episodeActive || terminalReached)
            return;

        // 연속 학습 모드(enableFinishLineTerminal=false)에서는 게이트 무시
        if (!enableFinishLineTerminal)
            return;

        SetTerminal(
            TerminalType.FailWrongLane,
            $"WrongLane(gate={gateName}, enter={enterSigned:F3}, exit={exitSigned:F3})"
        );
    }

    /// <summary>
    /// 에피소드를 종료 처리하는 핵심 메서드.
    /// 1) 종료 유형과 사유 기록  2) 차량 정지
    /// 3) OnEpisodeTerminated 이벤트 발행
    /// </summary>
    void SetTerminal(TerminalType type, string reason)
    {
        if (terminalReached)
            return;

        terminalReached = true;
        episodeActive = false;
        terminalType = type;
        episodeSuccess = type == TerminalType.Finish;
        terminalReason = string.IsNullOrEmpty(reason) ? type.ToString() : reason;

        if (stopVehicleOnTerminal && vehicleMotionController != null)
        {
            vehicleMotionController.SetSteering(0f);
            vehicleMotionController.SetThrottle(0f);
            vehicleMotionController.SetBrake(1f);
        }

        OnEpisodeTerminated?.Invoke(this);
    }

    public bool IsEpisodeActive() => episodeActive;
    public bool IsTerminalReached() => terminalReached;
    public bool IsEpisodeSuccess() => episodeSuccess;
    public TerminalType GetTerminalType() => terminalType;
    public string GetTerminalReason() => terminalReason;
    public float GetElapsedSeconds() => elapsedSeconds;
    public int GetEpisodeIndex() => episodeIndex;
    public int GetCollisionCount() => collisionCount;
    public float GetTimeAtDangerLevel() => timeAtDangerLevel;
    public CollisionWarningEngine.WarningLevel GetMaxWarningLevel() => maxWarningLevel;
    public float GetMinObservedTtc() => minObservedTtc;

    public bool HasPassedCheckpoint() => checkpointPassed;
    public void SetCheckpointPassed(bool passed) => checkpointPassed = passed;

    /// <summary>스폰 지점 대비 최소 거리를 이동했는지 확인. 체크포인트 게이밍 방지용.</summary>
    public bool HasTraveledMinDistanceFromSpawn()
    {
        if (checkpointMinDistanceFromSpawn <= 0f) return true;
        Vector3 currentPos = vehicleMotionController != null
            ? vehicleMotionController.transform.position : transform.position;
        Vector3 delta = currentPos - episodeSpawnPosition;
        delta.y = 0f;
        return delta.magnitude >= checkpointMinDistanceFromSpawn;
    }
}
