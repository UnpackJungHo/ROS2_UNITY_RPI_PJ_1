using UnityEngine;

public class WheelTest : MonoBehaviour
{
    public ArticulationBody frontLeftWheel;
    public ArticulationBody frontRightWheel;
    public ArticulationBody rearLeftWheel;
    public ArticulationBody rearRightWheel;

    public float targetVelocity = 300f; 

    void Update()
    {
        // 입력값 받기
        float vertical = Input.GetAxis("Vertical");
        float horizontal = Input.GetAxis("Horizontal");

        // 차동 구동(Differential Drive) 계산
        float leftSpeed = (vertical + horizontal) * targetVelocity; // 방향키에 따라 +/- 조정 필요
        float rightSpeed = (vertical - horizontal) * targetVelocity;

        SetWheelVelocity(frontLeftWheel, leftSpeed);
        SetWheelVelocity(rearLeftWheel, leftSpeed);
        SetWheelVelocity(frontRightWheel, rightSpeed);
        SetWheelVelocity(rearRightWheel, rightSpeed);
    }

    void SetWheelVelocity(ArticulationBody wheel, float velocity)
    {
        if (wheel == null)
        {
            Debug.LogError("Wheel is not assigned!");
            return;
        }

        ArticulationDrive drive = wheel.xDrive;
        
        // [중요] Velocity 구동을 위해서는 Stiffness(위치 복원력)가 0이어야 합니다.
        drive.stiffness = 0f; 
        
        // Damping은 목표 속도에 도달하기 위한 저항/힘의 역할을 합니다. 
        // 너무 낮으면 헛돌고, 너무 높으면 반응이 느려집니다. (URDF 마찰력 고려하여 조절)
        drive.damping = 1000f; 

        //drive.driveType = ArticulationDriveType.Velocity;
        drive.targetVelocity = velocity;
        
        // 최대 힘 제한 (URDF의 limit effort="10"은 좀 작을 수 있으니 필요시 넉넉하게)
        drive.forceLimit = 10000f; 

        wheel.xDrive = drive;
    }
}