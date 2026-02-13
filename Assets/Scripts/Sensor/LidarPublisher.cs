using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

public class LidarPublisher : MonoBehaviour
{
    // ========== ROS 설정 ==========
    [Header("ROS Settings")]
    public string topicName = "/velodyne_points";
    public string frameId = "lidar_link";
    public float publishRate = 10f;

    // ========== 라이다 사양 설정 ==========
    [Header("Lidar Specifications")]
    [Tooltip("최소 감지 거리 (m)")]
    public float rangeMin = 0.1f;
    [Tooltip("최대 감지 거리 (m)")]
    public float rangeMax = 3f; // 3D 라이다는 보통 더 긴 범위를 가짐

    [Tooltip("수평 스캔 레이 개수 (360도 기준)")]
    public int numHorizontalRays = 360;
    [Tooltip("수평 스캔 시작 각도 (라디안, -PI ~ PI)")]
    public float horizontalAngleMin = -Mathf.PI;
    [Tooltip("수평 스캔 종료 각도 (라디안)")]
    public float horizontalAngleMax = Mathf.PI;

    [Tooltip("수직 스캔 레이 개수 (채널 수)")]
    public int numVerticalRays = 16;
    [Tooltip("수직 스캔 시작 각도 (라디안, 아래쪽)")]
    public float verticalAngleMin = -15f * Mathf.Deg2Rad;
    [Tooltip("수직 스캔 종료 각도 (라디안, 위쪽)")]
    public float verticalAngleMax = 15f * Mathf.Deg2Rad;

    // ========== 충돌 감지 레이어 설정 ==========
    [Header("Collision Detection Layer")]
    [Tooltip("라이다가 감지할 레이어")]
    public LayerMask detectionLayer = ~0;

    // ========== 라이다 위치 참조 ==========
    [Header("Lidar Reference")]
    public Transform lidarTransform;

    // ========== 디버그 설정 ==========
    [Header("Debug")]
    public bool showDebugRays = false; // 3D는 레이가 많아서 기본적으로 끔
    public Color hitColor = Color.red;
    public Color missColor = Color.green;

    // ========== Private 변수들 ==========
    private ROSConnection ros;
    private float publishInterval;
    private float lastPublishTime;

    // 포인트 클라우드 데이터를 저장할 리스트 (x, y, z, intensity)
    private List<float> pointCloudData;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        // PointCloud2Msg로 변경
        ros.RegisterPublisher<PointCloud2Msg>(topicName);

        if (lidarTransform == null)
        {
            Debug.LogError("[LidarPublisher] lidarTransform이 할당되지 않았습니다! Inspector에서 lidar_link를 할당하세요.");
            lidarTransform = transform;
        }

        // Layer 9 (Zone) 제외
        // 기존 detectionLayer에서 9번 비트를 0으로 만듦
        // detectionLayer &= ~(1 << 9);

        InitializeLidar();
    }

    void InitializeLidar()
    {
        publishInterval = 1f / publishRate;
        lastPublishTime = Time.time;
        pointCloudData = new List<float>();
    }

    void Update()
    {
        if (Time.time - lastPublishTime >= publishInterval)
        {
            PerformScan();
            PublishScan();
            lastPublishTime = Time.time;
        }
    }

    void PerformScan()
    {
        if (lidarTransform == null) return;

        pointCloudData.Clear();
        Vector3 origin = lidarTransform.position;

        // 수직 각도 간격
        float verticalStep = (numVerticalRays > 1) ? (verticalAngleMax - verticalAngleMin) / (numVerticalRays - 1) : 0;
        // 수평 각도 간격
        float horizontalStep = (numHorizontalRays > 1) ? (horizontalAngleMax - horizontalAngleMin) / numHorizontalRays : 0;

        for (int v = 0; v < numVerticalRays; v++)
        {
            float verticalAngle = verticalAngleMin + (v * verticalStep);
            
            // 수직 각도에 따른 Y 성분과 수평 투영 길이 계산
            // Unity 좌표계: Y가 Up. 
            // verticalAngle이 0이면 수평. +면 위, -면 아래.
            float yPath = Mathf.Sin(verticalAngle);
            float hPath = Mathf.Cos(verticalAngle);

            for (int h = 0; h < numHorizontalRays; h++)
            {
                float horizontalAngle = horizontalAngleMin + (h * horizontalStep);

                // 방향 벡터 계산 (Unity 좌표계)
                // x = hPath * sin(horiz)
                // z = hPath * cos(horiz)
                // y = yPath
                Vector3 localDir = new Vector3(hPath * Mathf.Sin(horizontalAngle), yPath, hPath * Mathf.Cos(horizontalAngle));
                Vector3 direction = lidarTransform.TransformDirection(localDir);

                RaycastHit hit;
                if (Physics.Raycast(origin, direction, out hit, rangeMax, detectionLayer))
                {
                    float distance = hit.distance;
                    if (distance >= rangeMin && distance <= rangeMax)
                    {
                        // ROS 좌표계로 변환하여 저장
                        // Unity (x, y, z) -> ROS (z, x, y) usually, but standard simple conversion often does:
                        // Unity World -> ROS World via CoordinateSpace conversion if using ROS-TCP-Connector's helpers.
                        // However, PointCloud2 usually expects points in the frame_id (lidar_link) frame.
                        // So we should use the LOCAL coordinates of the hit point relative to the lidar_link.
                        
                        // Hit point in world space
                        Vector3 hitPoint = hit.point;
                        // Transform world point to local point relative to lidarTransform
                        Vector3 localHit = lidarTransform.InverseTransformPoint(hitPoint);

                        // Convert Unity Local (Left-handed: X Right, Y Up, Z Forward) 
                        // to ROS Local (Right-handed: X Forward, Y Left, Z Up)
                        // Unity X -> ROS -Y
                        // Unity Y -> ROS Z
                        // Unity Z -> ROS X
                        
                        // BUT, standard conventions vary. 
                        // Unity.Robotics.ROSTCPConnector provides GeometryUtils.
                        // If we assume standard: Unity(x,y,z) -> ROS(z, -x, y) aka FLU (Forward, Left, Up) vs RUF vs...
                        // Let's stick to standard ROS "lidar_link" convention: X forward, Y left, Z up.
                        // Unity "lidar_link": Z forward, X right, Y up.
                        // So: ROS_X = Unity_Z, ROS_Y = -Unity_X, ROS_Z = Unity_Y.
                        
                        pointCloudData.Add(localHit.z);  // x
                        pointCloudData.Add(-localHit.x); // y
                        pointCloudData.Add(localHit.y);  // z
                        pointCloudData.Add(1.0f);        // intensity

                        if (showDebugRays)
                            Debug.DrawLine(origin, hitPoint, hitColor, publishInterval);
                    }
                    else
                    {
                         if (showDebugRays)
                            Debug.DrawRay(origin, direction * rangeMax, missColor, publishInterval);
                    }
                }
                else
                {
                    if (showDebugRays)
                        Debug.DrawRay(origin, direction * rangeMax, missColor, publishInterval);
                }
            }
        }
    }

    void PublishScan()
    {
        // PointFields 설정
        PointFieldMsg[] fields = new PointFieldMsg[4];
        fields[0] = new PointFieldMsg("x", 0, PointFieldMsg.FLOAT32, 1);
        fields[1] = new PointFieldMsg("y", 4, PointFieldMsg.FLOAT32, 1);
        fields[2] = new PointFieldMsg("z", 8, PointFieldMsg.FLOAT32, 1);
        fields[3] = new PointFieldMsg("intensity", 12, PointFieldMsg.FLOAT32, 1);

        int pointStep = 16;
        int numPoints = pointCloudData.Count / 4;
        byte[] data = new byte[numPoints * pointStep];

        // 데이터 바이트 변환
        // Buffer.BlockCopy is faster for arrays, but we have a List<float>.
        // Using BitConverter loop for clarity and safety.
        // 데이터 바이트 변환 (Buffer.BlockCopy 사용으로 최적화)
        // float[]로 변환 후 복사
        float[] floatArray = pointCloudData.ToArray();
        Buffer.BlockCopy(floatArray, 0, data, 0, data.Length);

        PointCloud2Msg msg = new PointCloud2Msg
        {
            header = new HeaderMsg
            {
                stamp = new TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1) * 1e9)
                },
                frame_id = frameId
            },
            height = 1, // Unordered point cloud
            width = (uint)numPoints,
            fields = fields,
            is_bigendian = false, // C# uses Little Endian on most platforms
            point_step = (uint)pointStep,
            row_step = (uint)(numPoints * pointStep),
            data = data,
            is_dense = true // 유효한 포인트만 보냄
        };

        ros.Publish(topicName, msg);
    }
}
