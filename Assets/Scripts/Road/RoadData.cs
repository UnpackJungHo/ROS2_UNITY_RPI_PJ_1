using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 보상 구역 정보
/// </summary>
[Serializable]
public class RewardZoneInfo
{
    public string zoneName = "Zone";
    public float score = 0f;
    public float startOffset = 0f;
    public float endOffset = 1f;
}

/// <summary>
/// 도로 데이터를 저장하는 컴포넌트
/// </summary>
public class RoadData : MonoBehaviour
{
    [Header("Control Points")]
    public List<Vector3> controlPoints = new List<Vector3>();
    [Tooltip("true면 controlPoints가 Road 오브젝트 로컬 좌표 기준으로 저장됩니다. 구버전 데이터는 false(월드 좌표)일 수 있습니다.")]
    public bool controlPointsAreLocal = true;

    [Header("Road Settings")]
    public float roadWidth = 5f;
    public int curveResolution = 10;
    public Material roadMaterial;
    public bool isLooped = false;

    [Header("Curb Settings")]
    public bool hasCurbs = false;
    public float curbWidth = 0.5f;
    public float curbHeight = 0.2f;
    public Material curbMaterial;

    [Header("Reward Zones")]
    public List<RewardZoneInfo> rewardZones = new List<RewardZoneInfo>();
}
