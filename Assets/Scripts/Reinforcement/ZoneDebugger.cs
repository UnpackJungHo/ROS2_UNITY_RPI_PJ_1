using UnityEngine;
using TMPro;

public class ZoneDebugger : MonoBehaviour
{
    [SerializeField] private bool showDebugLog = true;
    [SerializeField] private TextMeshProUGUI zoneText;

    private string currentZoneName = "";

    private void Start()
    {
        if (zoneText == null)
        {
            Debug.LogWarning("[ZoneDebugger] zoneText가 할당되지 않았습니다.");
        }
    }

    private void Update()
    {
        if (zoneText != null)
        {
            zoneText.text = $"Current Zone: {currentZoneName}";
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        CheckZone(other);
    }

    private void OnTriggerStay(Collider other)
    {
        CheckZone(other);
    }

    private void OnTriggerExit(Collider other)
    {
        RewardZone zone = other.GetComponent<RewardZone>();
        if (zone != null && zone.zoneName == currentZoneName)
        {
            currentZoneName = "None";
            if (showDebugLog)
            {
                //Debug.Log($"[ZoneDebugger] Exited Zone: {zone.zoneName}");
            }
        }
    }

    private void CheckZone(Collider other)
    {
        RewardZone zone = other.GetComponent<RewardZone>();
        if (zone != null)
        {
            if (currentZoneName != zone.zoneName)
            {
                currentZoneName = zone.zoneName;
                if (showDebugLog)
                {
                    //Debug.Log($"[ZoneDebugger] Entered/In Zone: {currentZoneName} (Score: {zone.score})");
                }
            }
        }
    }
}
