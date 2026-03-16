using System;
using UnityEngine;

/// <summary>
/// 360x32 라이다 레이캐스트 시뮬레이터.
/// 스캔 결과를 바이트 버퍼(PointCloud2 포맷)로 관리하며,
/// ROS 발행은 LidarPublisher가 담당한다.
/// </summary>
public class LidarSensor : MonoBehaviour
{
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

    [Header("Collision Detection Layer")]
    [Tooltip("라이다가 감지할 레이어")]
    public LayerMask detectionLayer = ~0;

    [Header("Lidar Reference")]
    public Transform lidarTransform;

    [Header("Scan Rate")]
    public float scanRate = 10f;

    [Header("Performance (Multi-Vehicle)")]
    [Tooltip("다중 차량 시 스캔 시점을 분산")]
    public bool enableScanStagger = true;

    [Header("Debug")]
    public bool showDebugRays = false;
    public Color hitColor = Color.red;
    public Color missColor = Color.green;

    // PointCloud2 바이트 버퍼 (x,y,z,intensity,ring,time = 22 bytes per point)
    public const int POINT_STEP = 22;
    private byte[] pointBuffer;
    private int numPoints;
    private float scanInterval;
    private float lastScanTime;
    private readonly float[] floatWriteBuffer = new float[1];

    public int NumPoints => numPoints;
    public byte[] PointBuffer => pointBuffer;

    void Start()
    {
        if (lidarTransform == null)
        {
            Debug.LogError("[LidarSensor] lidarTransform이 할당되지 않았습니다! Inspector에서 lidar_link를 할당하세요.");
            lidarTransform = transform;
        }

        scanInterval = 1f / scanRate;
        lastScanTime = Time.time;

        if (enableScanStagger)
        {
            float stagger = Mathf.Abs(gameObject.GetInstanceID() % 97) / 97f * scanInterval;
            lastScanTime = Time.time + stagger - scanInterval;
        }

        int maxPoints = numHorizontalRays * numVerticalRays;
        pointBuffer = new byte[maxPoints * POINT_STEP];
        numPoints = 0;
    }

    void Update()
    {
        if (Time.time - lastScanTime >= scanInterval)
        {
            PerformScan();
            lastScanTime = Time.time;
        }
    }

    public void PerformScan()
    {
        if (lidarTransform == null) return;

        numPoints = 0;
        Vector3 origin = lidarTransform.position;

        float verticalStep = (numVerticalRays > 1) ? (verticalAngleMax - verticalAngleMin) / (numVerticalRays - 1) : 0;
        float horizontalStep = (numHorizontalRays > 1) ? (horizontalAngleMax - horizontalAngleMin) / numHorizontalRays : 0;

        float scanPeriod = 1f / scanRate;

        for (int h = 0; h < numHorizontalRays; h++)
        {
            float horizontalAngle = horizontalAngleMin + (h * horizontalStep);
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

                        // Unity Local -> ROS Local 좌표 변환
                        float rosX = localHit.z;
                        float rosY = -localHit.x;
                        float rosZ = localHit.y;

                        int offset = numPoints * POINT_STEP;
                        WriteFloat(pointBuffer, offset, rosX);
                        WriteFloat(pointBuffer, offset + 4, rosY);
                        WriteFloat(pointBuffer, offset + 8, rosZ);
                        WriteFloat(pointBuffer, offset + 12, 1.0f);
                        WriteUInt16(pointBuffer, offset + 16, (ushort)v);
                        WriteFloat(pointBuffer, offset + 18, pointTime);
                        numPoints++;

                        if (showDebugRays)
                            Debug.DrawLine(origin, hit.point, hitColor, scanInterval);
                    }
                    else
                    {
                        if (showDebugRays)
                            Debug.DrawRay(origin, direction * rangeMax, missColor, scanInterval);
                    }
                }
                else
                {
                    if (showDebugRays)
                        Debug.DrawRay(origin, direction * rangeMax, missColor, scanInterval);
                }
            }
        }
    }

    void WriteFloat(byte[] buf, int offset, float value)
    {
        floatWriteBuffer[0] = value;
        Buffer.BlockCopy(floatWriteBuffer, 0, buf, offset, 4);
    }

    static void WriteUInt16(byte[] buf, int offset, ushort value)
    {
        buf[offset] = (byte)(value);
        buf[offset + 1] = (byte)(value >> 8);
    }

    /// <summary>
    /// 현재 스캔 결과를 새 byte[]로 복사하여 반환한다.
    /// </summary>
    public byte[] CopyPointData()
    {
        int dataSize = numPoints * POINT_STEP;
        byte[] data = new byte[dataSize];
        Buffer.BlockCopy(pointBuffer, 0, data, 0, dataSize);
        return data;
    }
}
