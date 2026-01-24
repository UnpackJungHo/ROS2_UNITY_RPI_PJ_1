using UnityEngine;

public class WheelTest : MonoBehaviour
{
    [Header("Steering Links (Auto-assigned)")]
    public ArticulationBody frontLeftSteering;
    public ArticulationBody frontRightSteering;

    [Header("Wheels (Auto-assigned)")]
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

    [Header("Auto-Find Settings")]
    public bool autoFindReferences = true;

    private float currentSteeringAngle = 0f;

    void Start()
    {
        if (autoFindReferences)
        {
            FindReferences();
        }
    }

    void FindReferences()
    {
        frontLeftSteering = FindArticulationBody("front_left_steering");
        frontRightSteering = FindArticulationBody("front_right_steering");
        frontLeftWheel = FindArticulationBody("front_left_wheel");
        frontRightWheel = FindArticulationBody("front_right_wheel");
        rearLeftWheel = FindArticulationBody("rear_left_wheel");
        rearRightWheel = FindArticulationBody("rear_right_wheel");
    }

    ArticulationBody FindArticulationBody(string name)
    {
        Transform found = FindChildRecursive(transform, name);
        if (found != null)
        {
            ArticulationBody ab = found.GetComponent<ArticulationBody>();
            if (ab != null)
            {
                Debug.Log($"[WheelTest] Found: {name}");
                return ab;
            }
        }
        Debug.LogWarning($"[WheelTest] Not found: {name}");
        return null;
    }

    Transform FindChildRecursive(Transform parent, string name)
    {
        foreach (Transform child in parent)
        {
            if (child.name == name)
                return child;

            Transform found = FindChildRecursive(child, name);
            if (found != null)
                return found;
        }
        return null;
    }

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
            leftAngle = innerAngle;
            rightAngle = outerAngle;
        }
        else
        {
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
