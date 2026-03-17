using System.Collections;
using UnityEngine;

/// <summary>
/// Fixed joint에 붙은 JointControl 컴포넌트와 URDF Controller 컴포넌트를 자동 비활성화한다.
///
/// 문제: URDF 패키지의 JointControl이 차량당 22개(8대=176개) 각각 FixedUpdate()를 실행.
///       Fixed joint는 ArticulationBody.jointType == FixedJoint로 구동 target 설정이 불필요.
///       Controller 컴포넌트는 매 프레임 OnGUI()를 호출해 IMGUI GC alloc을 발생시킴.
///
/// 사용법: 차량 루트(RootAMR, base_footprint 등)에 이 컴포넌트를 추가.
/// </summary>
public class FixedJointControlDisabler : MonoBehaviour
{
    [Header("Debug (Read Only)")]
    [SerializeField] private int scannedJointControls;
    [SerializeField] private int disabledJointControls;
    [SerializeField] private int disabledControllers;

    // URDF 패키지는 Play 모드 진입 후 단계적으로 컴포넌트를 추가한다.
    // - JointControl: 첫 번째 Start() 프레임 이후 (yield return null 1회)
    // - Controller: 그 이후 추가 지연 필요 (1초 대기)
    IEnumerator Start()
    {
        // 1단계: 1프레임 대기 → JointControl 비활성화
        yield return null;

        int scanned = 0;
        int disabled = 0;

        foreach (var ab in GetComponentsInChildren<ArticulationBody>(includeInactive: true))
        {
            if (ab.jointType != ArticulationJointType.FixedJoint)
                continue;

            foreach (var mb in ab.GetComponents<MonoBehaviour>())
            {
                if (mb == null) continue;
                scanned++;
                if (mb.GetType().Name == "JointControl")
                {
                    mb.enabled = false;
                    disabled++;
                }
            }
        }

        scannedJointControls = scanned;
        disabledJointControls = disabled;
        Debug.Log($"[FixedJointControlDisabler] JointControl 비활성화: {disabled}/{scanned}");

        // 2단계: Controller가 등장할 때까지 대기 (URDF가 늦게 추가, 최대 120초)
        float waitStart = Time.time;
        yield return new WaitUntil(() =>
        {
            if (Time.time - waitStart > 120f) return true; // timeout
            foreach (var mb in GetComponentsInChildren<MonoBehaviour>(true))
            {
                if (mb != null && mb.GetType().Name == "Controller")
                    return true;
            }
            return false;
        });

        int controllersDisabled = 0;
        foreach (var mb in GetComponentsInChildren<MonoBehaviour>(includeInactive: true))
        {
            if (mb == null) continue;
            if (mb.GetType().Name == "Controller")
            {
                mb.enabled = false;
                controllersDisabled++;
            }
        }

        disabledControllers = controllersDisabled;
        Debug.Log($"[FixedJointControlDisabler] Controller 비활성화: {controllersDisabled}");
    }
}
