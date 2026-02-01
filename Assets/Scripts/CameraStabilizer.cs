using UnityEngine;

/// <summary>
/// 카메라 안정화 스크립트
/// 물리 시뮬레이션으로 인한 카메라 흔들림을 부드럽게 보정합니다.
/// URDF 조인트 계층에서 직접 물리 영향을 받는 대신,
/// 대상 위치/회전을 부드럽게 추적(Interpolation)하여 안정적인 영상을 제공합니다.
/// </summary>
public class CameraStabilizer : MonoBehaviour
{
    [Header("Target Settings")]
    [Tooltip("카메라가 따라갈 대상 Transform (예: camera_link 또는 base_link)")]
    public Transform targetTransform;

    [Tooltip("대상으로부터의 위치 오프셋 (로컬 좌표)")]
    public Vector3 positionOffset = Vector3.zero;

    [Tooltip("대상으로부터의 회전 오프셋 (오일러 각도)")]
    public Vector3 rotationOffset = Vector3.zero;

    [Header("Stabilization Settings")]
    [Tooltip("위치 안정화 강도 (0 = 즉시 이동, 낮을수록 부드러움)")]
    [Range(0.01f, 1f)]
    public float positionSmoothTime = 0.1f;

    [Tooltip("회전 안정화 강도 (0 = 즉시 회전, 낮을수록 부드러움)")]
    [Range(0.01f, 1f)]
    public float rotationSmoothTime = 0.1f;

    [Header("Advanced Stabilization")]
    [Tooltip("수직 흔들림(Y축) 추가 안정화")]
    public bool stabilizeVertical = true;

    [Tooltip("수직 안정화 강도 (낮을수록 더 안정)")]
    [Range(0.01f, 0.5f)]
    public float verticalSmoothTime = 0.2f;

    [Tooltip("롤(좌우 기울기) 안정화 - 차량 기울어짐 무시")]
    public bool stabilizeRoll = true;

    [Tooltip("피치(앞뒤 기울기) 안정화 - 가감속 시 끄덕임 감소")]
    public bool stabilizePitch = false;

    [Header("Damping")]
    [Tooltip("위치 변화 데드존 (이 이하의 미세 움직임 무시)")]
    public float positionDeadZone = 0.001f;

    [Tooltip("회전 변화 데드존 (이 이하의 미세 회전 무시)")]
    public float rotationDeadZone = 0.1f;

    // 내부 변수
    private Vector3 currentVelocity = Vector3.zero;
    private Vector3 smoothedPosition;
    private Quaternion smoothedRotation;
    private float smoothedY;
    private float yVelocity;
    private bool isInitialized = false;

    void Start()
    {
        if (targetTransform == null)
        {
            Debug.LogError("[CameraStabilizer] targetTransform이 할당되지 않았습니다!");
            enabled = false;
            return;
        }

        // 초기 위치/회전 설정
        InitializePosition();
    }

    void InitializePosition()
    {
        Vector3 targetPos = GetTargetPosition();
        Quaternion targetRot = GetTargetRotation();

        smoothedPosition = targetPos;
        smoothedRotation = targetRot;
        smoothedY = targetPos.y;

        transform.position = smoothedPosition;
        transform.rotation = smoothedRotation;

        isInitialized = true;
    }

    // LateUpdate를 사용하여 모든 물리 연산 후에 카메라 위치 업데이트
    void LateUpdate()
    {
        if (targetTransform == null || !isInitialized) return;

        UpdateStabilizedTransform();
    }

    void UpdateStabilizedTransform()
    {
        Vector3 targetPos = GetTargetPosition();
        Quaternion targetRot = GetTargetRotation();

        // ===== 위치 안정화 =====
        Vector3 newPosition;

        if (stabilizeVertical)
        {
            // 수평(XZ)과 수직(Y)을 별도로 처리하여 더 안정적인 수직 움직임
            Vector3 horizontalTarget = new Vector3(targetPos.x, 0, targetPos.z);
            Vector3 horizontalCurrent = new Vector3(smoothedPosition.x, 0, smoothedPosition.z);

            // 수평 위치 보간
            Vector3 newHorizontal = Vector3.SmoothDamp(
                horizontalCurrent,
                horizontalTarget,
                ref currentVelocity,
                positionSmoothTime
            );

            // 수직 위치 별도 보간 (더 부드럽게)
            smoothedY = Mathf.SmoothDamp(smoothedY, targetPos.y, ref yVelocity, verticalSmoothTime);

            newPosition = new Vector3(newHorizontal.x, smoothedY, newHorizontal.z);
        }
        else
        {
            // 일반적인 3D 위치 보간
            newPosition = Vector3.SmoothDamp(
                smoothedPosition,
                targetPos,
                ref currentVelocity,
                positionSmoothTime
            );
        }

        // ===== 전방 지연 방지 (카메라가 대상 뒤로 밀리지 않도록) =====
        // 대상의 전방 방향 기준으로 카메라가 뒤처지면 즉시 따라잡음
        Vector3 toCamera = newPosition - targetPos;
        Vector3 targetForward = targetTransform.forward;
        
        // 카메라가 대상의 뒤에 있는지 확인 (내적이 음수면 뒤에 있음)
        float forwardDot = Vector3.Dot(toCamera, targetForward);
        
        // 카메라가 대상보다 뒤에 있으면 (음수) 전방 성분만 즉시 따라잡음
        if (forwardDot < -0.01f)
        {
            // 카메라를 대상의 위치로 전방 성분만 보정
            Vector3 forwardCorrection = targetForward * forwardDot;
            newPosition -= forwardCorrection * 0.9f; // 90% 보정하여 자연스럽게
        }

        // 데드존 적용 (미세 움직임 필터링)
        if (Vector3.Distance(newPosition, smoothedPosition) > positionDeadZone)
        {
            smoothedPosition = newPosition;
        }

        // ===== 회전 안정화 =====
        Quaternion newRotation;

        if (stabilizeRoll || stabilizePitch)
        {
            // 롤/피치 안정화가 활성화된 경우
            Vector3 targetEuler = targetRot.eulerAngles;

            // 롤(Z축) 안정화: 0으로 고정
            if (stabilizeRoll)
            {
                targetEuler.z = 0f;
            }

            // 피치(X축) 안정화: 부드럽게 0으로 수렴
            if (stabilizePitch)
            {
                // 피치 각도를 -180~180 범위로 변환
                float pitch = targetEuler.x;
                if (pitch > 180f) pitch -= 360f;

                // 피치를 부드럽게 감쇠
                targetEuler.x = Mathf.Lerp(pitch, 0f, 0.5f);
            }

            Quaternion stabilizedTarget = Quaternion.Euler(targetEuler);
            newRotation = Quaternion.Slerp(
                smoothedRotation,
                stabilizedTarget,
                Time.deltaTime / rotationSmoothTime
            );
        }
        else
        {
            // 일반적인 회전 보간
            newRotation = Quaternion.Slerp(
                smoothedRotation,
                targetRot,
                Time.deltaTime / rotationSmoothTime
            );
        }

        // 회전 항상 부드럽게 적용 (점프 코드 제거)
        smoothedRotation = newRotation;

        // 최종 적용
        transform.position = smoothedPosition;
        transform.rotation = smoothedRotation;
    }

    /// <summary>
    /// 대상의 월드 위치 + 오프셋 계산
    /// </summary>
    Vector3 GetTargetPosition()
    {
        // 오프셋을 대상의 로컬 좌표계로 적용
        return targetTransform.position + targetTransform.TransformDirection(positionOffset);
    }

    /// <summary>
    /// 대상의 월드 회전 + 오프셋 계산
    /// </summary>
    Quaternion GetTargetRotation()
    {
        return targetTransform.rotation * Quaternion.Euler(rotationOffset);
    }

    /// <summary>
    /// 즉시 대상 위치로 스냅 (씬 전환 등에 사용)
    /// </summary>
    public void SnapToTarget()
    {
        if (targetTransform == null) return;

        InitializePosition();
    }

    /// <summary>
    /// 런타임에 대상 변경
    /// </summary>
    public void SetTarget(Transform newTarget)
    {
        targetTransform = newTarget;
        if (newTarget != null)
        {
            InitializePosition();
        }
    }

    // Inspector에서 변경 시 바로 적용 (에디터 전용)
    void OnValidate()
    {
        if (Application.isPlaying && isInitialized && targetTransform != null)
        {
            // 파라미터 변경 시 즉시 반영하지 않고 자연스럽게 전환
        }
    }
}
