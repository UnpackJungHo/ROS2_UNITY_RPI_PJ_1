using UnityEngine;
// Unity와 ROS2를 연결하기 위한 TCP 커넥터 라이브러리
using Unity.Robotics.ROSTCPConnector;
// ROS의 메세지 타입 센서, 표준, 내장 인터페이스를 사용하기 위한 네임스페이스
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

// 라이다 센서를 시뮬레이션하고 ROS 토픽으로 스캔 데이터를 발행하는 클래스
public class LidarPublisher : MonoBehaviour
{
    // ========== ROS 설정 ==========
    [Header("ROS Settings")]
    // ROS 토픽 이름 - 라이다 데이터가 발행될 토픽 경로
    public string topicName = "/scan";
    // ROS 프레임 ID - TF(Transform) 트리에서 이 센서의 좌표계 이름
    public string frameId = "lidar_link";
    // 초당 발행 횟수 (Hz) -> 1초에 10번 발행
    public float publishRate = 10f;

    // ========== 라이다 사양 설정 ==========
    [Header("Lidar Specifications")]
    [Tooltip("최소 감지 거리 (m)")]
    public float rangeMin = 0.1f;
    [Tooltip("최대 감지 거리 (m)")]
    public float rangeMax = 12f;
    [Tooltip("스캔 레이 개수 (360도 기준)")]
    public int numRays = 360;
    [Tooltip("스캔 시작 각도 (라디안, 0 = 전방)")]
    public float angleMin = -Mathf.PI;
    [Tooltip("스캔 종료 각도 (라디안)")]
    public float angleMax = Mathf.PI;

    // ========== 충돌 감지 레이어 설정 ==========
    [Header("Collision Detection Layer")]
    [Tooltip("라이다가 감지할 레이어")]
    // 라이다 레이가 충돌을 감지할 Unity 레이어 마스크 (~0은 모든 레이어를 의미)
    public LayerMask detectionLayer = ~0;

    // ========== 라이다 위치 참조 ==========
    [Header("Lidar Reference")]
    [Tooltip("라이다 센서 위치 Transform을 직접 할당하세요 (lidar_link)")]
    // 라이다 센서가 장착된 위치의 Transform (이 위치에서 레이캐스트 시작)
    public Transform lidarTransform;

    // ========== 디버그 설정 ==========
    [Header("Debug")]
    public bool showDebugRays = true;
    public Color hitColor = Color.red;
    public Color missColor = Color.green;

    // ========== Private 변수들 ==========
    // ROS와의 연결을 관리하는 객체
    private ROSConnection ros;

    // 발행 주기 (초 단위, publishRate의 역수)
    private float publishInterval;

    // 마지막으로 데이터를 발행한 시간
    private float lastPublishTime;

    // 각 레이의 거리 측정값을 저장하는 배열 (미터 단위)
    private float[] ranges;

    // 각 레이의 강도(intensity) 값을 저장하는 배열 (0~1)
    private float[] intensities;

    // 각 레이 사이의 각도 간격 (라디안 단위)
    private float angleIncrement;

    void Start()
    {
        // ROS 연결 인스턴스를 가져오거나 없으면 새로 생성 (싱글톤 패턴)
        ros = ROSConnection.GetOrCreateInstance();
        // 이 스크립트를 LaserScanMsg 타입의 퍼블리셔로 등록
        ros.RegisterPublisher<LaserScanMsg>(topicName);

        // lidarTransform이 Inspector에서 할당되지 않은 경우 체크
        if (lidarTransform == null)
        {
            Debug.LogError("[LidarPublisher] lidarTransform이 할당되지 않았습니다! Inspector에서 lidar_link를 할당하세요.");
            lidarTransform = transform;
        }

        // 라이다 초기화 함수 호출 (배열 생성 및 각도 계산)
        InitializeLidar();
        // 초기화 완료 로그 출력 (레이 개수, 거리 범위, 토픽 이름 정보 포함)
        //Debug.Log($"[LidarPublisher] Initialized - {numRays} rays, range: {rangeMin}-{rangeMax}m, topic: {topicName}");
    }

    // 라이다 센서의 파라미터를 초기화하는 함수
    void InitializeLidar()
    {
        // 발행 간격 계산 (초 단위, 예: publishRate=10이면 0.1초마다 발행)
        publishInterval = 1f / publishRate;
        // 마지막 발행 시간을 현재 시간으로 설정 (게임 시작 후 경과 시간)
        lastPublishTime = Time.time;

        // 거리 측정값 배열 생성 (레이 개수만큼)
        ranges = new float[numRays];
        // 강도값 배열 생성 (레이 개수만큼)
        intensities = new float[numRays];
        // 각 레이 사이의 각도 간격 계산 (전체 각도 범위를 레이 개수로 나눔)
        angleIncrement = (angleMax - angleMin) / numRays;
    }

    void Update()
    {
        // 마지막 발행 이후 충분한 시간이 지났는지 확인 (publishInterval 이상)
        if (Time.time - lastPublishTime >= publishInterval)
        {
            // 라이다 스캔 수행 (레이캐스트를 통해 거리 측정)
            PerformScan();
            // 측정된 데이터를 ROS 토픽으로 발행
            PublishScan();
            // 마지막 발행 시간을 현재 시간으로 업데이트
            lastPublishTime = Time.time;
        }
    }

    // 라이다 스캔을 수행하는 함수 (레이캐스트를 사용하여 360도 주변 물체까지의 거리 측정)
    void PerformScan()
    {
        // lidarTransform이 없으면 스캔 불가능, 함수 종료
        if (lidarTransform == null) return;

        // 레이캐스트의 시작점 = 라이다 센서의 월드 좌표 위치
        Vector3 origin = lidarTransform.position;

        // 모든 레이에 대해 반복 (0번부터 numRays-1번까지)
        for (int i = 0; i < numRays; i++)
        {
            // 현재 레이의 각도 계산 (라디안 단위, angleMin부터 시작해서 angleIncrement씩 증가)
            float angle = angleMin + (i * angleIncrement);

            // 로컬 좌표계에서의 방향 벡터를 월드 좌표계로 변환
            // Sin(angle)은 X축 성분, Cos(angle)은 Z축 성분 (Unity에서 Z가 전방)
            // Y=0이므로 수평면에서만 스캔 (2D 라이다 시뮬레이션)
            Vector3 direction = lidarTransform.TransformDirection(
                new Vector3(Mathf.Sin(angle), 0, Mathf.Cos(angle))
            );

            // 레이캐스트 충돌 정보를 저장할 변수
            RaycastHit hit;
            // Physics.Raycast: origin에서 direction 방향으로 최대 rangeMax 거리까지 레이를 쏴서 detectionLayer에 있는 물체와 충돌 검사
            if (Physics.Raycast(origin, direction, out hit, rangeMax, detectionLayer))
            {
                // 충돌 지점까지의 거리
                float distance = hit.distance;
                // 거리가 라이다의 유효 범위 내에 있는지 확인
                if (distance >= rangeMin && distance <= rangeMax)
                {
                    // 유효 범위 내: 실제 측정 거리 저장
                    ranges[i] = distance;
                    // 강도를 1.0으로 설정 (물체 감지됨)
                    intensities[i] = 1.0f;
                }
                else
                {
                    // 유효 범위 밖: 무한대로 설정 (측정 불가)
                    ranges[i] = float.PositiveInfinity;
                    // 강도를 0으로 설정 (물체 감지 안 됨)
                    intensities[i] = 0f;
                }

                // 디버그 레이 표시가 활성화되어 있으면
                if (showDebugRays)
                {
                    // Scene 뷰에 origin부터 충돌 지점까지 hitColor로 선을 그림 (publishInterval 시간 동안 표시)
                    Debug.DrawLine(origin, hit.point, hitColor, publishInterval);
                }
            }
            else // 레이가 아무것도 맞지 않은 경우
            {
                // 거리를 무한대로 설정 (물체 없음)
                ranges[i] = float.PositiveInfinity;
                // 강도를 0으로 설정
                intensities[i] = 0f;

                // 디버그 레이 표시가 활성화되어 있으면
                if (showDebugRays)
                {
                    // Scene 뷰에 origin부터 최대 거리까지 missColor로 레이를 그림
                    Debug.DrawRay(origin, direction * rangeMax, missColor, publishInterval);
                }
            }
        }
    }

    // 측정된 라이다 스캔 데이터를 ROS 토픽으로 발행하는 함수
    void PublishScan()
    {
        // 한 번의 전체 스캔에 걸린 시간 (초 단위)
        float scanTime = publishInterval;
        // 각 레이 측정 사이의 시간 간격 (전체 스캔 시간을 레이 개수로 나눔)
        float timeIncrement = scanTime / numRays;

        // ROS LaserScan 메시지 객체 생성 및 초기화
        LaserScanMsg scanMsg = new LaserScanMsg
        {
            // 메시지 헤더 (메타데이터 포함)
            header = new HeaderMsg
            {
                // 타임스탬프 (현재 시간)
                stamp = new TimeMsg
                {
                    // 초 단위 (정수 부분)
                    sec = (int)Time.time,
                    // 나노초 단위 (소수 부분을 나노초로 변환, 1e9 = 10억)
                    nanosec = (uint)((Time.time % 1) * 1e9)
                },
                // 이 데이터가 속한 좌표계 프레임 ID
                frame_id = frameId
            },
            // 스캔 시작 각도 (라디안)
            angle_min = angleMin,
            // 스캔 종료 각도 (라디안)
            angle_max = angleMax,
            // 각 레이 사이의 각도 간격 (라디안)
            angle_increment = angleIncrement,
            // 각 레이 측정 사이의 시간 간격 (초)
            time_increment = timeIncrement,
            // 전체 스캔에 걸린 시간 (초)
            scan_time = scanTime,
            // 최소 측정 가능 거리 (미터)
            range_min = rangeMin,
            // 최대 측정 가능 거리 (미터)
            range_max = rangeMax,
            // 각 레이의 거리 측정값 배열 (미터)
            ranges = ranges,
            // 각 레이의 강도값 배열 (0~1)
            intensities = intensities
        };

        // ROS 토픽으로 메시지 발행 (topicName으로 scanMsg 전송)
        ros.Publish(topicName, scanMsg);
    }
}
