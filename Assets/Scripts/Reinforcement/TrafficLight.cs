using UnityEngine;
using System.Collections;

public class TrafficLight : MonoBehaviour
{
    [Header("Traffic Light Objects")]
    public GameObject redObject;
    public GameObject yellowObject;
    public GameObject greenObject;

    [Header("Durations")]
    public float redDuration = 10f;
    public float greenDuration = 10f;
    public float yellowDuration = 2f;

    private Renderer redRenderer;
    private Renderer yellowRenderer;
    private Renderer greenRenderer;
    
    // Use MaterialPropertyBlock for efficient and correct color updates on instances (even static ones)
    private MaterialPropertyBlock propBlock;
    private static readonly int ColorId = Shader.PropertyToID("_Color");

    private void Start()
    {
        // Cache Renderers instead of Material instances
        if (redObject != null) redRenderer = redObject.GetComponent<Renderer>();
        if (yellowObject != null) yellowRenderer = yellowObject.GetComponent<Renderer>();
        if (greenObject != null) greenRenderer = greenObject.GetComponent<Renderer>();

        // Initialize Property Block
        propBlock = new MaterialPropertyBlock();

        // Initialize all lights to black (off)
        SetColor(redRenderer, Color.black);
        SetColor(yellowRenderer, Color.black);
        SetColor(greenRenderer, Color.black);

        // Start the traffic light cycle
        StartCoroutine(TrafficCycle());
    }

    private IEnumerator TrafficCycle()
    {
        while (true)
        {
            // 1. Red Light ON
            SetColor(redRenderer, Color.red);
            yield return new WaitForSeconds(redDuration);
            SetColor(redRenderer, Color.black);

            // 2. Yellow Light ON (Transition: Red -> Green)
            SetColor(yellowRenderer, Color.yellow);
            yield return new WaitForSeconds(yellowDuration);
            SetColor(yellowRenderer, Color.black);

            // 3. Green Light ON
            SetColor(greenRenderer, Color.green);
            yield return new WaitForSeconds(greenDuration);
            SetColor(greenRenderer, Color.black);

            // 4. Yellow Light ON (Transition: Green -> Red)
            SetColor(yellowRenderer, Color.yellow);
            yield return new WaitForSeconds(yellowDuration);
            SetColor(yellowRenderer, Color.black);
        }
    }

    private void SetColor(Renderer rend, Color color)
    {
        if (rend != null)
        {
            // Get the current property block
            rend.GetPropertyBlock(propBlock);
            // Set the color property
            propBlock.SetColor(ColorId, color);
            // Apply the property block back to the renderer
            rend.SetPropertyBlock(propBlock);
        }
    }
}
