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
    public float rangeMax = 10f;

    [Tooltip("수평 스캔 레이 개수 (360도 기준)")]
    public int numHorizontalRays = 360;
    [Tooltip("수평 스캔 시작 각도 (라디안, -PI ~ PI)")]
    public float horizontalAngleMin = -Mathf.PI;
    [Tooltip("수평 스캔 종료 각도 (라디안)")]
    public float horizontalAngleMax = Mathf.PI;

    [Tooltip("수직 스캔 레이 개수 (채널 수)")]
    public int numVerticalRays = 32;
    [Tooltip("수직 스캔 시작 각도 (라디안, 아래쪽)")]
    public float verticalAngleMin = -0.6f;
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
    public bool showDebugRays = false;
    public Color hitColor = Color.red;
    public Color missColor = Color.green;

    // ========== Private 변수들 ==========
    private ROSConnection ros;
    private float publishInterval;
    private float lastPublishTime;

    // PointCloud2 바이트 버퍼 (x,y,z,intensity,ring,time = 22 bytes per point)
    private const int POINT_STEP = 22;
    private byte[] pointBuffer;
    private int numPoints;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PointCloud2Msg>(topicName);

        if (lidarTransform == null)
        {
            Debug.LogError("[LidarPublisher] lidarTransform이 할당되지 않았습니다! Inspector에서 lidar_link를 할당하세요.");
            lidarTransform = transform;
        }

        InitializeLidar();
    }

    void InitializeLidar()
    {
        publishInterval = 1f / publishRate;
        lastPublishTime = Time.time;
        int maxPoints = numHorizontalRays * numVerticalRays;
        pointBuffer = new byte[maxPoints * POINT_STEP];
        numPoints = 0;
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

        numPoints = 0;
        Vector3 origin = lidarTransform.position;

        float verticalStep = (numVerticalRays > 1) ? (verticalAngleMax - verticalAngleMin) / (numVerticalRays - 1) : 0;
        float horizontalStep = (numHorizontalRays > 1) ? (horizontalAngleMax - horizontalAngleMin) / numHorizontalRays : 0;

        // 스캔 주기 (1회전 소요 시간)
        float scanPeriod = 1f / publishRate;

        // 실제 회전 LiDAR 동작 시뮬레이션: 수평 회전(외부) → 각 각도에서 수직 채널(내부) 동시 발사
        for (int h = 0; h < numHorizontalRays; h++)
        {
            float horizontalAngle = horizontalAngleMin + (h * horizontalStep);

            // 이 수평 각도에서의 스캔 시작 기준 상대 시간 (초)
            float pointTime = ((float)h / numHorizontalRays) * scanPeriod;

            for (int v = 0; v < numVerticalRays; v++)
            {
                float verticalAngle = verticalAngleMin + (v * verticalStep);

                float yPath = Mathf.Sin(verticalAngle);
                float hPath = Mathf.Cos(verticalAngle);

                Vector3 localDir = new Vector3(hPath * Mathf.Sin(horizontalAngle), yPath, hPath * Mathf.Cos(horizontalAngle));
                Vector3 direction = lidarTransform.TransformDirection(localDir);

                RaycastHit hit;
                if (Physics.Raycast(origin, direction, out hit, rangeMax, detectionLayer))
                {
                    float distance = hit.distance;
                    if (distance >= rangeMin && distance <= rangeMax)
                    {
                        Vector3 localHit = lidarTransform.InverseTransformPoint(hit.point);

                        // Unity Local → ROS Local 좌표 변환
                        // Unity (X Right, Y Up, Z Forward) → ROS (X Forward, Y Left, Z Up)
                        float rosX = localHit.z;
                        float rosY = -localHit.x;
                        float rosZ = localHit.y;

                        // 바이트 버퍼에 직접 쓰기 (point_step = 22 bytes)
                        int offset = numPoints * POINT_STEP;
                        WriteFloat(pointBuffer, offset, rosX);         // x: offset 0
                        WriteFloat(pointBuffer, offset + 4, rosY);     // y: offset 4
                        WriteFloat(pointBuffer, offset + 8, rosZ);     // z: offset 8
                        WriteFloat(pointBuffer, offset + 12, 1.0f);    // intensity: offset 12
                        WriteUInt16(pointBuffer, offset + 16, (ushort)v); // ring: offset 16
                        WriteFloat(pointBuffer, offset + 18, pointTime); // time: offset 18
                        numPoints++;

                        if (showDebugRays)
                            Debug.DrawLine(origin, hit.point, hitColor, publishInterval);
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

    // float → Little Endian 4 bytes
    static void WriteFloat(byte[] buf, int offset, float value)
    {
        byte[] bytes = System.BitConverter.GetBytes(value);
        buf[offset] = bytes[0];
        buf[offset + 1] = bytes[1];
        buf[offset + 2] = bytes[2];
        buf[offset + 3] = bytes[3];
    }

    // ushort → Little Endian 2 bytes
    static void WriteUInt16(byte[] buf, int offset, ushort value)
    {
        buf[offset] = (byte)(value);
        buf[offset + 1] = (byte)(value >> 8);
    }

    void PublishScan()
    {
        // PointCloud2 필드 정의 (Velodyne 호환: x, y, z, intensity, ring, time)
        PointFieldMsg[] fields = new PointFieldMsg[6];
        fields[0] = new PointFieldMsg("x", 0, PointFieldMsg.FLOAT32, 1);
        fields[1] = new PointFieldMsg("y", 4, PointFieldMsg.FLOAT32, 1);
        fields[2] = new PointFieldMsg("z", 8, PointFieldMsg.FLOAT32, 1);
        fields[3] = new PointFieldMsg("intensity", 12, PointFieldMsg.FLOAT32, 1);
        fields[4] = new PointFieldMsg("ring", 16, PointFieldMsg.UINT16, 1);
        fields[5] = new PointFieldMsg("time", 18, PointFieldMsg.FLOAT32, 1);

        int dataSize = numPoints * POINT_STEP;
        byte[] data = new byte[dataSize];
        Buffer.BlockCopy(pointBuffer, 0, data, 0, dataSize);

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
            height = 1,
            width = (uint)numPoints,
            fields = fields,
            is_bigendian = false,
            point_step = (uint)POINT_STEP,
            row_step = (uint)dataSize,
            data = data,
            is_dense = true
        };

        ros.Publish(topicName, msg);
    }
}
