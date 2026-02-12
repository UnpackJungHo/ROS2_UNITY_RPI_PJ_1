using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class TrafficLight : MonoBehaviour
{
    public enum DatasetType
    {
        Train,
        Test
    }

    [Header("Dataset Settings")]
    public DatasetType datasetType = DatasetType.Train;

    [Header("Traffic Light Image")]
    public SpriteRenderer signalImage;

    [Header("Sprite Lists (Auto-Loaded)")]
    public List<Sprite> redSprites;
    public List<Sprite> yellowSprites;
    public List<Sprite> greenSprites;

    [Header("Auto-Fit Settings")]
    public Renderer fitTargetRenderer; // Assign 'layer 8' here
    public float zOffset = 3f; // Default to 3, can be negative if needed
    public Vector3 manualScaleAdjustment = Vector3.one; // Fine-tune if auto-fit is slightly off

    [Header("Durations")]
    public float redDuration = 10f;
    public float greenDuration = 10f;
    public float yellowDuration = 2f;

    [Header("Current State (Read Only)")]
    [SerializeField] private string _currentColor = "red";
    public string currentColor => _currentColor;

    private void Start()
    {
        // Auto-load sprites based on dataset type
        LoadSprites();

        // Start the traffic light cycle
        StartCoroutine(TrafficCycle());
    }

    [ContextMenu("Load Sprites")]
    public void LoadSprites()
    {
        string typeFolder = datasetType.ToString().ToLower(); // "train" or "test"
        string basePath = $"CrossWalkImage/{typeFolder}";

        // Load Red
        redSprites = new List<Sprite>(Resources.LoadAll<Sprite>($"{basePath}/Red"));
        
        // Load Yellow
        yellowSprites = new List<Sprite>(Resources.LoadAll<Sprite>($"{basePath}/Yellow"));
        
        // Load Green
        greenSprites = new List<Sprite>(Resources.LoadAll<Sprite>($"{basePath}/Green"));

        Debug.Log($"[TrafficLight] Loaded {datasetType} sprites: R={redSprites.Count}, Y={yellowSprites.Count}, G={greenSprites.Count}");
    }

    private IEnumerator TrafficCycle()
    {
        while (true)
        {
            // 1. Red Light ON
            _currentColor = "red";
            SetRandomSprite(redSprites);
            yield return new WaitForSeconds(redDuration);

            // 2. Yellow Light ON (Transition: Red -> Green)
            _currentColor = "yellow";
            SetRandomSprite(yellowSprites);
            yield return new WaitForSeconds(yellowDuration);

            // 3. Green Light ON
            _currentColor = "green";
            SetRandomSprite(greenSprites);
            yield return new WaitForSeconds(greenDuration);

            // 4. Yellow Light ON (Transition: Green -> Red)
            _currentColor = "yellow";
            SetRandomSprite(yellowSprites);
            yield return new WaitForSeconds(yellowDuration);
        }
    }

    private void SetRandomSprite(List<Sprite> sprites)
    {
        if (signalImage != null && sprites != null && sprites.Count > 0)
        {
            int randomIndex = Random.Range(0, sprites.Count);
            signalImage.sprite = sprites[randomIndex];
            
            // Auto-fit the new sprite to the target boundary
            if (fitTargetRenderer != null)
            {
                FitSpriteToBoundary();
            }
        }
    }

    private void FitSpriteToBoundary()
    {
        if (signalImage == null || fitTargetRenderer == null || signalImage.sprite == null) return;

        // Try to get mesh bounds for local size (works best for rotated objects)
        Vector3 targetLocalSize;
        MeshFilter targetMeshFilter = fitTargetRenderer.GetComponent<MeshFilter>();
        
        if (targetMeshFilter != null && targetMeshFilter.sharedMesh != null)
        {
            targetLocalSize = targetMeshFilter.sharedMesh.bounds.size;
        }
        else
        {
            // Fallback to renderer bounds in world space if no mesh (e.g. SkinnedMesh or other)
            // But we convert world bounds size to local space approx by dividing by lossyScale
            // This handles rotation poorly but is a fallback.
            Vector3 worldSize = fitTargetRenderer.bounds.size;
            Vector3 parentScale = fitTargetRenderer.transform.lossyScale;
            targetLocalSize = new Vector3(
                (parentScale.x != 0) ? worldSize.x / parentScale.x : worldSize.x,
                (parentScale.y != 0) ? worldSize.y / parentScale.y : worldSize.y,
                (parentScale.z != 0) ? worldSize.z / parentScale.z : worldSize.z
            );
        }

        Vector3 spriteLocalSize = signalImage.sprite.bounds.size;

        // Calculate scale factor: Target Local Size / Sprite Local Size
        // We assume the signalImage is a child, so it inherits parent scale. 
        // Thus we only need to match the LOCAL size of the sprite to the LOCAL size of the mesh.
        
        float xScale = (spriteLocalSize.x != 0) ? targetLocalSize.x / spriteLocalSize.x : 1f;
        float yScale = (spriteLocalSize.y != 0) ? targetLocalSize.y / spriteLocalSize.y : 1f;

        // Apply manual adjustment
        xScale *= manualScaleAdjustment.x;
        yScale *= manualScaleAdjustment.y;

        // Apply new scale (preserve z as 1 or match target if needed, usually 1 for 2D sprites)
        signalImage.transform.localScale = new Vector3(xScale, yScale, 1f);

        // Position Logic:
        // Place at the center of the target mesh (local space 0,0,0 usually, but bounds.center might be offset)
        // If fitTargetRenderer is the parent, we can just use localPosition.
        // If not parent, we must use world position. Let's assume parent/child relationship per user description.
        
        if (signalImage.transform.parent == fitTargetRenderer.transform)
        {
             Vector3 centerOffset = (targetMeshFilter != null) ? targetMeshFilter.sharedMesh.bounds.center : Vector3.zero;
             // Move forward by zOffset (Forward is usually +Z or -Z depending on model. We assume -Z is "Front" per user request or +Z)
             // User tried +3 and it worked somewhat? Or maybe -3? 
             // We'll use local Z offset directly.
             signalImage.transform.localPosition = centerOffset + new Vector3(0, 0, -zOffset); 
             // Note: If the mesh faces -Z, moving -Z moves it "out".
             
             // Reset local rotation to identity to match parent face (or keep user's if they rotated it manually?)
             // User said "Rotation should not change". So we don't touch localRotation.
        }
        else
        {
             // World space fallback
             signalImage.transform.position = fitTargetRenderer.bounds.center + fitTargetRenderer.transform.forward * -zOffset;
        }
    }
}
