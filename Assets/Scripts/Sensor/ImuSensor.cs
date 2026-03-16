using UnityEngine;

/// <summary>
/// IMU 센서 시뮬레이터.
/// 속도 버퍼 기반 가속도 추정, EMA 필터, 중력 보상을 수행한다.
/// ROS 발행은 ImuPublisher가 담당한다.
/// </summary>
public class ImuSensor : MonoBehaviour
{
    [Header("Vehicle Reference")]
    public ArticulationBody vehicleBody;

    [Header("Base Link Reference")]
    [Tooltip("base_link 게임오브젝트를 지정. 비워두면 이 오브젝트 자신을 사용.")]
    public GameObject baseLinkObject;

    [Header("Filter Settings")]
    [Tooltip("가속도 저역통과 필터 계수 (0.0~1.0). 낮을수록 더 부드러움")]
    [Range(0.05f, 1.0f)]
    public float accelFilterAlpha = 0.3f;

    [Tooltip("각속도 저역통과 필터 계수 (0.0~1.0). 낮을수록 더 부드러움")]
    [Range(0.05f, 1.0f)]
    public float gyroFilterAlpha = 0.3f;

    [Header("Acceleration Estimation")]
    [Tooltip("가속도 계산에 사용할 프레임 간격 (높을수록 부드럽지만 지연 증가)")]
    [Range(1, 8)]
    public int velocityBufferSize = 4;

    // 외부에서 읽기 위한 프로퍼티
    public Transform BaseLinkTransform => baseLinkTransform;
    public Vector3 FilteredAcceleration => filteredAcceleration;
    public Vector3 FilteredAngularVelocity => filteredAngularVelocity;

    private Transform baseLinkTransform;

    // 다중 프레임 속도 버퍼 (원형 버퍼)
    private Vector3[] velocityBuffer;
    private int bufferIndex;
    private int bufferCount;

    // 저역통과 필터 적용된 값
    private Vector3 filteredAcceleration;
    private Vector3 filteredAngularVelocity;
    private bool filterInitialized;

    void Start()
    {
        baseLinkTransform = baseLinkObject != null ? baseLinkObject.transform : transform;

        if (vehicleBody == null)
        {
            vehicleBody = baseLinkTransform.GetComponentInChildren<ArticulationBody>();
            if (vehicleBody == null)
                vehicleBody = baseLinkTransform.GetComponentInParent<ArticulationBody>();
            if (vehicleBody == null)
                Debug.LogError("[ImuSensor] ArticulationBody not found!");
        }

        filterInitialized = false;
        velocityBuffer = new Vector3[velocityBufferSize];
        bufferIndex = 0;
        bufferCount = 0;
    }

    void FixedUpdate()
    {
        if (vehicleBody == null) return;

        Vector3 currentVelocity = vehicleBody.velocity;

        // 가속도 계산: 다중 프레임 미분
        Vector3 rawAcceleration;

        if (bufferCount >= velocityBufferSize)
        {
            Vector3 oldVelocity = velocityBuffer[bufferIndex];
            float deltaTime = velocityBufferSize * Time.fixedDeltaTime;
            rawAcceleration = (currentVelocity - oldVelocity) / deltaTime;
        }
        else
        {
            if (bufferCount > 0)
            {
                int prevIndex = (bufferIndex - 1 + velocityBufferSize) % velocityBufferSize;
                rawAcceleration = (currentVelocity - velocityBuffer[prevIndex]) / Time.fixedDeltaTime;
            }
            else
            {
                rawAcceleration = Vector3.zero;
            }
        }

        velocityBuffer[bufferIndex] = currentVelocity;
        bufferIndex = (bufferIndex + 1) % velocityBufferSize;
        bufferCount++;

        // 중력 포함 (실제 IMU 비력 측정)
        rawAcceleration -= Physics.gravity;

        // 각속도: 물리 엔진 직접 제공
        Vector3 rawAngularVelocity = vehicleBody.angularVelocity;

        // 저역통과 필터 (EMA)
        if (!filterInitialized)
        {
            filteredAcceleration = rawAcceleration;
            filteredAngularVelocity = rawAngularVelocity;
            filterInitialized = true;
        }
        else
        {
            filteredAcceleration = Vector3.Lerp(filteredAcceleration, rawAcceleration, accelFilterAlpha);
            filteredAngularVelocity = Vector3.Lerp(filteredAngularVelocity, rawAngularVelocity, gyroFilterAlpha);
        }
    }
}
