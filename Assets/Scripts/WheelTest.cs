using UnityEngine;

public class WheelTest : MonoBehaviour
{
    [Header("Steering Links")]
    public ArticulationBody frontLeftSteering;
    public ArticulationBody frontRightSteering;

    [Header("Wheels")]
    public ArticulationBody frontLeftWheel;
    public ArticulationBody frontRightWheel;
    public ArticulationBody rearLeftWheel;
    public ArticulationBody rearRightWheel;

    [Header("Settings")]
    public float maxSteeringAngle = 30f;
    public float steeringSpeed = 100f;
    public float targetVelocity = 300f;

    [Header("Ackermann Geometry")]
    public float wheelBase = 0.6f;
    public float trackWidth = 0.65f;

    private float currentSteeringAngle = 0f;

    void Update()
    {
        float vertical = Input.GetAxis("Vertical");
        float horizontal = Input.GetAxis("Horizontal");

        UpdateSteering(horizontal);
        UpdateWheelDrive(vertical);
    }

    void UpdateSteering(float input)
    {
        float targetAngle = -input * maxSteeringAngle;
        currentSteeringAngle = Mathf.MoveTowards(currentSteeringAngle, targetAngle, steeringSpeed * Time.deltaTime);

        float leftAngle, rightAngle;
        CalculateAckermannAngles(currentSteeringAngle, out leftAngle, out rightAngle);

        SetSteeringAngle(frontLeftSteering, leftAngle);
        SetSteeringAngle(frontRightSteering, rightAngle);
    }

    void CalculateAckermannAngles(float steerAngle, out float leftAngle, out float rightAngle)
    {
        if (Mathf.Abs(steerAngle) < 0.01f)
        {
            leftAngle = 0f;
            rightAngle = 0f;
            return;
        }

        float steerRad = steerAngle * Mathf.Deg2Rad;
        float turnRadius = wheelBase / Mathf.Tan(Mathf.Abs(steerRad));

        float innerRadius = turnRadius - (trackWidth / 2f);
        float outerRadius = turnRadius + (trackWidth / 2f);
        float innerAngle = Mathf.Atan(wheelBase / innerRadius) * Mathf.Rad2Deg;
        float outerAngle = Mathf.Atan(wheelBase / outerRadius) * Mathf.Rad2Deg;

        if (steerAngle > 0)
        {
            // 왼쪽 회전: 왼쪽=내륜(큰 각도), 오른쪽=외륜(작은 각도)
            leftAngle = innerAngle;
            rightAngle = outerAngle;
        }
        else
        {
            // 오른쪽 회전: 오른쪽=내륜(큰 각도), 왼쪽=외륜(작은 각도)
            leftAngle = -outerAngle;
            rightAngle = -innerAngle;
        }
    }

    void SetSteeringAngle(ArticulationBody steering, float angle)
    {
        if (steering == null) return;

        ArticulationDrive drive = steering.xDrive;
        drive.stiffness = 10000f;
        drive.damping = 1000f;
        drive.forceLimit = 10000f;
        drive.target = angle;
        steering.xDrive = drive;
    }

    void UpdateWheelDrive(float input)
    {
        float speed = input * targetVelocity;

        SetWheelVelocity(rearLeftWheel, speed);
        SetWheelVelocity(rearRightWheel, speed);
        SetWheelVelocity(frontLeftWheel, speed);
        SetWheelVelocity(frontRightWheel, speed);
    }

    void SetWheelVelocity(ArticulationBody wheel, float velocity)
    {
        if (wheel == null) return;

        ArticulationDrive drive = wheel.xDrive;
        drive.stiffness = 0f;
        drive.damping = 1000f;
        drive.targetVelocity = velocity;
        drive.forceLimit = 10000f;
        wheel.xDrive = drive;
    }
}
