using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

/// <summary>
/// Road Creator Editor Tool
/// Shift+클릭으로 포인트를 찍고, 포인트들을 연결하는 도로 메쉬를 생성합니다.
/// 곡선 구간은 Catmull-Rom Spline을 사용하여 부드럽게 처리합니다.
/// </summary>
public class RoadCreator : EditorWindow
{
    // 도로 설정
    private float roadWidth = 4f;
    private int curveResolution = 10; // 곡선 구간의 세분화 정도
    private Material roadMaterial;
    
    // 포인트 관리
    private List<Vector3> controlPoints = new List<Vector3>();
    private GameObject currentRoadObject;
    private bool isCreating = false;
    private bool isLooped = false; // 루프 닫기 여부
    
    // 포인트 선택 (삽입 기능용)
    private List<int> selectedPointIndices = new List<int>();
    private const float POINT_SELECT_RADIUS = 1.0f; // 포인트 선택 반경
    
    // UI 스크롤
    private Vector2 scrollPosition;
    
    [MenuItem("Tools/Road Creator")]
    public static void ShowWindow()
    {
        GetWindow<RoadCreator>("Road Creator");
    }
    
    private void OnEnable()
    {
        SceneView.duringSceneGui += OnSceneGUI;
        LoadDefaultMaterial();
    }
    
    private void OnDisable()
    {
        SceneView.duringSceneGui -= OnSceneGUI;
    }
    
    private void LoadDefaultMaterial()
    {
        // RoadMaterial 자동 로드
        string[] guids = AssetDatabase.FindAssets("RoadMaterial t:Material");
        if (guids.Length > 0)
        {
            string path = AssetDatabase.GUIDToAssetPath(guids[0]);
            roadMaterial = AssetDatabase.LoadAssetAtPath<Material>(path);
        }
    }
    
    private void OnGUI()
    {
        GUILayout.Label("🛣️ Road Creator", EditorStyles.boldLabel);
        EditorGUILayout.Space(10);
        
        // 도로 설정
        EditorGUILayout.LabelField("도로 설정", EditorStyles.boldLabel);
        roadWidth = EditorGUILayout.Slider("도로 너비", roadWidth, 1f, 20f);
        curveResolution = EditorGUILayout.IntSlider("곡선 해상도", curveResolution, 2, 30);
        roadMaterial = (Material)EditorGUILayout.ObjectField("도로 머테리얼", roadMaterial, typeof(Material), false);
        
        EditorGUILayout.Space(10);
        
        // 생성 모드 토글
        EditorGUI.BeginChangeCheck();
        isCreating = GUILayout.Toggle(isCreating, isCreating ? "🔴 생성 모드 활성화 (클릭하여 끄기)" : "⚪ 생성 모드 시작", "Button", GUILayout.Height(30));
        if (EditorGUI.EndChangeCheck())
        {
            SceneView.RepaintAll();
        }
        
        if (isCreating)
        {
            EditorGUILayout.HelpBox("Shift + 좌클릭: 포인트 추가/선택\nShift + 우클릭: 마지막 포인트 삭제\nI 키: 선택된 두 포인트 사이에 삽입\nESC: 선택 해제", MessageType.Info);
        }
        
        // 선택된 포인트 표시
        if (selectedPointIndices.Count > 0)
        {
            string selected = string.Join(", ", selectedPointIndices.ConvertAll(i => $"P{i}"));
            EditorGUILayout.HelpBox($"선택된 포인트: {selected}\n(I 키로 사이에 포인트 삽입)", MessageType.Warning);
        }
        
        EditorGUILayout.Space(10);
        
        // 포인트 목록
        EditorGUILayout.LabelField($"포인트 목록 ({controlPoints.Count}개)", EditorStyles.boldLabel);
        
        scrollPosition = EditorGUILayout.BeginScrollView(scrollPosition, GUILayout.Height(150));
        for (int i = 0; i < controlPoints.Count; i++)
        {
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField($"Point {i}", GUILayout.Width(60));
            controlPoints[i] = EditorGUILayout.Vector3Field("", controlPoints[i]);
            if (GUILayout.Button("X", GUILayout.Width(25)))
            {
                controlPoints.RemoveAt(i);
                UpdateRoadMesh();
                break;
            }
            EditorGUILayout.EndHorizontal();
        }
        EditorGUILayout.EndScrollView();
        
        EditorGUILayout.Space(5);
        
        // 기존 도로 불러오기 버튼
        EditorGUILayout.BeginHorizontal();
        if (GUILayout.Button("📂 기존 도로 불러오기", GUILayout.Height(25)))
        {
            LoadExistingRoad();
        }
        
        if (GUILayout.Button("💾 도로 저장", GUILayout.Height(25)))
        {
            SaveRoadData();
        }
        EditorGUILayout.EndHorizontal();
        
        EditorGUILayout.Space(10);
        
        // 버튼들
        EditorGUILayout.BeginHorizontal();
        
        GUI.enabled = controlPoints.Count >= 2;
        if (GUILayout.Button("도로 생성/업데이트", GUILayout.Height(35)))
        {
            CreateOrUpdateRoad();
        }
        GUI.enabled = true;
        
        GUI.backgroundColor = Color.red;
        if (GUILayout.Button("모두 초기화", GUILayout.Height(35), GUILayout.Width(100)))
        {
            ClearAll();
        }
        GUI.backgroundColor = Color.white;
        
        EditorGUILayout.EndHorizontal();
        
        EditorGUILayout.Space(5);
        
        EditorGUILayout.BeginHorizontal();
        if (GUILayout.Button("새 도로 시작", GUILayout.Height(25)))
        {
            StartNewRoad();
        }
        
        // 루프 닫기 버튼
        GUI.enabled = controlPoints.Count >= 3;
        GUI.backgroundColor = isLooped ? Color.green : Color.yellow;
        if (GUILayout.Button(isLooped ? "✓ 루프 완성됨" : "🔗 루프 닫기 (완성)", GUILayout.Height(25)))
        {
            isLooped = !isLooped;
            UpdateRoadMesh();
        }
        GUI.backgroundColor = Color.white;
        GUI.enabled = true;
        EditorGUILayout.EndHorizontal();
        
        if (isLooped)
        {
            EditorGUILayout.HelpBox("도로가 루프로 연결되었습니다. 처음과 끝이 자연스럽게 이어집니다.", MessageType.Info);
        }
    }
    
    private void OnSceneGUI(SceneView sceneView)
    {
        // 포인트 핸들 그리기
        Handles.color = Color.yellow;
        for (int i = 0; i < controlPoints.Count; i++)
        {
            // 드래그 가능한 핸들
            EditorGUI.BeginChangeCheck();
            Vector3 newPos = Handles.PositionHandle(controlPoints[i], Quaternion.identity);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(this, "Move Road Point");
                controlPoints[i] = newPos;
                UpdateRoadMesh();
            }
            
            // 포인트 번호 라벨
            Handles.Label(controlPoints[i] + Vector3.up * 0.5f, $"P{i}", EditorStyles.boldLabel);
            
            // 포인트 구체 (선택 상태에 따라 색상 변경)
            if (selectedPointIndices.Contains(i))
            {
                Handles.color = Color.magenta; // 선택된 포인트
                Handles.SphereHandleCap(0, controlPoints[i], Quaternion.identity, 0.5f, EventType.Repaint);
            }
            else
            {
                Handles.color = Color.cyan;
                Handles.SphereHandleCap(0, controlPoints[i], Quaternion.identity, 0.3f, EventType.Repaint);
            }
        }
        
        // 연결선 그리기
        if (controlPoints.Count >= 2)
        {
            Handles.color = Color.green;
            List<Vector3> splinePoints = GenerateSplinePoints();
            Handles.DrawPolyLine(splinePoints.ToArray());
        }
        
        // 생성 모드일 때 클릭 처리
        if (isCreating)
        {
            HandleInput(sceneView);
        }
    }
    
    private void HandleInput(SceneView sceneView)
    {
        Event e = Event.current;
        
        // ESC: 선택 해제
        if (e.type == EventType.KeyDown && e.keyCode == KeyCode.Escape)
        {
            selectedPointIndices.Clear();
            SceneView.RepaintAll();
            Repaint();
            e.Use();
            return;
        }
        
        // I 키: 선택된 두 포인트 사이에 포인트 삽입
        if (e.type == EventType.KeyDown && e.keyCode == KeyCode.I)
        {
            InsertPointBetweenSelected();
            e.Use();
            return;
        }
        
        // Shift + 좌클릭: 포인트 선택 또는 추가
        if (e.type == EventType.MouseDown && e.button == 0 && e.shift)
        {
            Ray ray = HandleUtility.GUIPointToWorldRay(e.mousePosition);
            
            // 먼저 기존 포인트 근처 클릭인지 확인
            int clickedPointIndex = GetPointIndexNearRay(ray);
            
            if (clickedPointIndex >= 0)
            {
                // 기존 포인트 선택/해제
                TogglePointSelection(clickedPointIndex);
                e.Use();
                return;
            }
            
            // 새 포인트 추가
            if (Physics.Raycast(ray, out RaycastHit hit))
            {
                AddPoint(hit.point);
            }
            else
            {
                Plane plane = new Plane(Vector3.up, Vector3.zero);
                if (plane.Raycast(ray, out float distance))
                {
                    Vector3 point = ray.GetPoint(distance);
                    AddPoint(point);
                }
            }
            
            e.Use();
        }
        
        // Shift + 우클릭: 마지막 포인트 삭제
        if (e.type == EventType.MouseDown && e.button == 1 && e.shift)
        {
            if (controlPoints.Count > 0)
            {
                Undo.RecordObject(this, "Remove Road Point");
                controlPoints.RemoveAt(controlPoints.Count - 1);
                selectedPointIndices.Clear();
                UpdateRoadMesh();
                e.Use();
            }
        }
        
        // 씨 뷰에 현재 모드 표시
        Handles.BeginGUI();
        string statusText = "🛣️ Road Creator: Shift+클릭=포인트 추가/선택, I=삽입, ESC=선택해제";
        if (selectedPointIndices.Count == 2)
        {
            statusText += $" | 선택: P{selectedPointIndices[0]}, P{selectedPointIndices[1]} (✨I키로 삽입✨)";
        }
        GUI.Label(new Rect(10, 10, 600, 20), statusText, EditorStyles.whiteLargeLabel);
        Handles.EndGUI();
    }
    
    /// <summary>
    /// 레이 근처의 포인트 인덱스 반환 (-1이면 없음)
    /// </summary>
    private int GetPointIndexNearRay(Ray ray)
    {
        float minDistance = float.MaxValue;
        int closestIndex = -1;
        
        for (int i = 0; i < controlPoints.Count; i++)
        {
            Vector3 point = controlPoints[i];
            float distance = Vector3.Cross(ray.direction, point - ray.origin).magnitude;
            
            if (distance < POINT_SELECT_RADIUS && distance < minDistance)
            {
                minDistance = distance;
                closestIndex = i;
            }
        }
        
        return closestIndex;
    }
    
    /// <summary>
    /// 포인트 선택 토글 (최대 2개까지)
    /// </summary>
    private void TogglePointSelection(int index)
    {
        if (selectedPointIndices.Contains(index))
        {
            selectedPointIndices.Remove(index);
        }
        else
        {
            if (selectedPointIndices.Count >= 2)
            {
                selectedPointIndices.RemoveAt(0); // 가장 먼저 선택된 것 제거
            }
            selectedPointIndices.Add(index);
        }
        
        SceneView.RepaintAll();
        Repaint();
    }
    
    /// <summary>
    /// 선택된 두 포인트 사이에 새 포인트 삽입
    /// </summary>
    private void InsertPointBetweenSelected()
    {
        if (selectedPointIndices.Count != 2)
        {
            Debug.LogWarning("Road Creator: 두 포인트를 선택해야 삽입할 수 있습니다.");
            return;
        }
        
        int idx1 = selectedPointIndices[0];
        int idx2 = selectedPointIndices[1];
        
        // 인접한 포인트인지 확인
        int minIdx = Mathf.Min(idx1, idx2);
        int maxIdx = Mathf.Max(idx1, idx2);
        
        // 루프일 때 첫번째와 마지막 포인트도 인접한 것으로 처리
        bool isAdjacent = (maxIdx - minIdx == 1) || 
                          (isLooped && minIdx == 0 && maxIdx == controlPoints.Count - 1);
        
        if (!isAdjacent)
        {
            Debug.LogWarning("Road Creator: 인접한 두 포인트를 선택해야 합니다.");
            return;
        }
        
        // 중간점 계산
        Vector3 midPoint = (controlPoints[idx1] + controlPoints[idx2]) / 2f;
        
        Undo.RecordObject(this, "Insert Road Point");
        
        // 루프이고 첫번째/마지막 사이일 때는 마지막에 추가
        if (isLooped && minIdx == 0 && maxIdx == controlPoints.Count - 1)
        {
            controlPoints.Add(midPoint);
        }
        else
        {
            controlPoints.Insert(maxIdx, midPoint);
        }
        
        selectedPointIndices.Clear();
        UpdateRoadMesh();
        SceneView.RepaintAll();
        Repaint();
        
        Debug.Log($"Road Creator: P{minIdx}와 P{maxIdx} 사이에 새 포인트 삽입됨");
    }
    
    private void AddPoint(Vector3 point)
    {
        Undo.RecordObject(this, "Add Road Point");
        controlPoints.Add(point);
        UpdateRoadMesh();
        Repaint();
    }
    
    private void CreateOrUpdateRoad()
    {
        if (controlPoints.Count < 2) return;
        
        if (currentRoadObject == null)
        {
            currentRoadObject = new GameObject("Road");
            currentRoadObject.AddComponent<MeshFilter>();
            currentRoadObject.AddComponent<MeshRenderer>();
            
            // RoadData 컴포넌트 추가
            var roadData = currentRoadObject.AddComponent<RoadData>();
            roadData.controlPoints = new List<Vector3>(controlPoints);
            roadData.roadWidth = roadWidth;
            roadData.curveResolution = curveResolution;
            roadData.isLooped = isLooped;
            
            Undo.RegisterCreatedObjectUndo(currentRoadObject, "Create Road");
        }
        else
        {
            // 기존 도로 업데이트
            var roadData = currentRoadObject.GetComponent<RoadData>();
            if (roadData != null)
            {
                roadData.controlPoints = new List<Vector3>(controlPoints);
                roadData.roadWidth = roadWidth;
                roadData.curveResolution = curveResolution;
                roadData.isLooped = isLooped;
            }
        }
        
        UpdateRoadMesh();
    }
    
    private void UpdateRoadMesh()
    {
        if (currentRoadObject == null || controlPoints.Count < 2) return;
        
        Mesh mesh = GenerateRoadMesh();
        
        MeshFilter mf = currentRoadObject.GetComponent<MeshFilter>();
        MeshRenderer mr = currentRoadObject.GetComponent<MeshRenderer>();
        
        if (mf.sharedMesh != null)
        {
            DestroyImmediate(mf.sharedMesh);
        }
        
        mf.sharedMesh = mesh;
        
        if (roadMaterial != null)
        {
            mr.sharedMaterial = roadMaterial;
        }
        
        // MeshCollider 자동 추가 (충돌 처리용)
        MeshCollider mc = currentRoadObject.GetComponent<MeshCollider>();
        if (mc == null)
        {
            mc = currentRoadObject.AddComponent<MeshCollider>();
        }
        mc.sharedMesh = mesh;
        
        SceneView.RepaintAll();
    }
    
    /// <summary>
    /// Catmull-Rom Spline을 사용하여 부드러운 경로 생성
    /// </summary>
    private List<Vector3> GenerateSplinePoints()
    {
        List<Vector3> result = new List<Vector3>();
        
        if (controlPoints.Count < 2) return result;
        
        // 컨트롤 포인트가 2개일 때는 직선
        if (controlPoints.Count == 2 && !isLooped)
        {
            int segments = curveResolution;
            for (int i = 0; i <= segments; i++)
            {
                float t = i / (float)segments;
                result.Add(Vector3.Lerp(controlPoints[0], controlPoints[1], t));
            }
            return result;
        }
        
        int pointCount = controlPoints.Count;
        int loopCount = isLooped ? pointCount : pointCount - 1;
        
        // Catmull-Rom Spline 생성
        for (int i = 0; i < loopCount; i++)
        {
            Vector3 p0, p1, p2, p3;
            
            if (isLooped)
            {
                // 루프일 때: 순환 인덱스 사용
                p0 = controlPoints[((i - 1) % pointCount + pointCount) % pointCount];
                p1 = controlPoints[i % pointCount];
                p2 = controlPoints[(i + 1) % pointCount];
                p3 = controlPoints[(i + 2) % pointCount];
            }
            else
            {
                // 열린 경로일 때: 끝점 클램핑
                p0 = controlPoints[Mathf.Max(i - 1, 0)];
                p1 = controlPoints[i];
                p2 = controlPoints[Mathf.Min(i + 1, pointCount - 1)];
                p3 = controlPoints[Mathf.Min(i + 2, pointCount - 1)];
            }
            
            for (int j = 0; j < curveResolution; j++)
            {
                float t = j / (float)curveResolution;
                result.Add(CatmullRom(p0, p1, p2, p3, t));
            }
        }
        
        // 마지막 포인트 추가 (루프가 아닐 때만)
        if (!isLooped)
        {
            result.Add(controlPoints[pointCount - 1]);
        }
        else
        {
            // 루프일 때: 시작점으로 돌아오기
            result.Add(result[0]);
        }
        
        return result;
    }
    
    /// <summary>
    /// Catmull-Rom Spline 보간
    /// </summary>
    private Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        float t2 = t * t;
        float t3 = t2 * t;
        
        return 0.5f * (
            (2f * p1) +
            (-p0 + p2) * t +
            (2f * p0 - 5f * p1 + 4f * p2 - p3) * t2 +
            (-p0 + 3f * p1 - 3f * p2 + p3) * t3
        );
    }
    
    /// <summary>
    /// 도로 메쉬 생성
    /// </summary>
    private Mesh GenerateRoadMesh()
    {
        List<Vector3> splinePoints = GenerateSplinePoints();
        
        if (splinePoints.Count < 2) return new Mesh();
        
        Mesh mesh = new Mesh();
        mesh.name = "RoadMesh";
        
        List<Vector3> vertices = new List<Vector3>();
        List<Vector2> uvs = new List<Vector2>();
        List<int> triangles = new List<int>();
        
        float totalLength = CalculateTotalLength(splinePoints);
        float currentLength = 0f;
        
        for (int i = 0; i < splinePoints.Count; i++)
        {
            Vector3 point = splinePoints[i];
            Vector3 forward;
            
            // 진행 방향 계산
            if (i < splinePoints.Count - 1)
            {
                forward = (splinePoints[i + 1] - point).normalized;
            }
            else
            {
                forward = (point - splinePoints[i - 1]).normalized;
            }
            
            // 오른쪽 방향 (Y축 기준)
            Vector3 right = Vector3.Cross(Vector3.up, forward).normalized;
            
            // 도로 양쪽 끝점
            Vector3 leftPoint = point - right * (roadWidth / 2f);
            Vector3 rightPoint = point + right * (roadWidth / 2f);
            
            vertices.Add(leftPoint);
            vertices.Add(rightPoint);
            
            // UV 좌표 (도로 길이에 따라 타일링)
            float v = currentLength / roadWidth; // 도로 너비 기준 타일링
            uvs.Add(new Vector2(0, v));
            uvs.Add(new Vector2(1, v));
            
            // 삼각형 생성 (두 번째 포인트부터)
            if (i > 0)
            {
                int baseIndex = (i - 1) * 2;
                
                // 마지막 포인트이고 루프일 때: 처음 버텍스와 연결
                int nextLeft, nextRight;
                if (i == splinePoints.Count - 1 && isLooped)
                {
                    nextLeft = 0;
                    nextRight = 1;
                }
                else
                {
                    nextLeft = baseIndex + 2;
                    nextRight = baseIndex + 3;
                }
                
                // 첫 번째 삼각형
                triangles.Add(baseIndex);
                triangles.Add(nextLeft);
                triangles.Add(baseIndex + 1);
                
                // 두 번째 삼각형
                triangles.Add(baseIndex + 1);
                triangles.Add(nextLeft);
                triangles.Add(nextRight);
            }
            
            // 길이 누적
            if (i > 0)
            {
                currentLength += Vector3.Distance(splinePoints[i - 1], splinePoints[i]);
            }
        }
        
        mesh.vertices = vertices.ToArray();
        mesh.uv = uvs.ToArray();
        mesh.triangles = triangles.ToArray();
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        
        return mesh;
    }
    
    private float CalculateTotalLength(List<Vector3> points)
    {
        float length = 0f;
        for (int i = 1; i < points.Count; i++)
        {
            length += Vector3.Distance(points[i - 1], points[i]);
        }
        return length;
    }
    
    private void ClearAll()
    {
        controlPoints.Clear();
        
        if (currentRoadObject != null)
        {
            Undo.DestroyObjectImmediate(currentRoadObject);
            currentRoadObject = null;
        }
        
        SceneView.RepaintAll();
    }
    
    private void StartNewRoad()
    {
        controlPoints.Clear();
        currentRoadObject = null;
        isLooped = false;
        selectedPointIndices.Clear();
        SceneView.RepaintAll();
    }
    
    /// <summary>
    /// 기존 도로 오브젝트 불러오기
    /// </summary>
    private void LoadExistingRoad()
    {
        // 현재 선택된 오브젝트에서 RoadData 찾기
        GameObject selected = Selection.activeGameObject;
        
        if (selected == null)
        {
            // Road 이름을 가진 오브젝트 찾기
            selected = GameObject.Find("Road");
        }
        
        if (selected == null)
        {
            EditorUtility.DisplayDialog("도로 불러오기", "Scene에서 Road 오브젝트를 선택하거나, 'Road' 이름의 오브젝트가 있어야 합니다.", "확인");
            return;
        }
        
        RoadData roadData = selected.GetComponent<RoadData>();
        
        if (roadData != null && roadData.controlPoints != null && roadData.controlPoints.Count > 0)
        {
            // RoadData에서 정보 로드
            controlPoints = new List<Vector3>(roadData.controlPoints);
            roadWidth = roadData.roadWidth;
            curveResolution = roadData.curveResolution;
            isLooped = roadData.isLooped;
            currentRoadObject = selected;
            selectedPointIndices.Clear();
            
            Debug.Log($"Road Creator: '{selected.name}'에서 {controlPoints.Count}개 포인트 로드됨");
        }
        else
        {
            // MeshFilter에서 버텍스 추출 시도
            MeshFilter mf = selected.GetComponent<MeshFilter>();
            if (mf != null && mf.sharedMesh != null)
            {
                currentRoadObject = selected;
                EditorUtility.DisplayDialog("도로 불러오기", 
                    "RoadData 컴포넌트가 없습니다. 이 도로는 Road Creator로 생성되지 않았을 수 있습니다.\n" +
                    "새 포인트를 추가하여 도로를 다시 만들 수 있습니다.", "확인");
            }
            else
            {
                EditorUtility.DisplayDialog("도로 불러오기", "선택된 오브젝트에 도로 데이터가 없습니다.", "확인");
            }
        }
        
        SceneView.RepaintAll();
        Repaint();
    }
    
    /// <summary>
    /// 도로 데이터 저장 (RoadData 컴포넌트 업데이트)
    /// </summary>
    private void SaveRoadData()
    {
        if (currentRoadObject == null)
        {
            CreateOrUpdateRoad();
            return;
        }
        
        RoadData roadData = currentRoadObject.GetComponent<RoadData>();
        if (roadData == null)
        {
            roadData = currentRoadObject.AddComponent<RoadData>();
        }
        
        Undo.RecordObject(roadData, "Save Road Data");
        roadData.controlPoints = new List<Vector3>(controlPoints);
        roadData.roadWidth = roadWidth;
        roadData.curveResolution = curveResolution;
        roadData.isLooped = isLooped;
        
        EditorUtility.SetDirty(roadData);
        
        Debug.Log($"Road Creator: 도로 데이터 저장됨 ({controlPoints.Count}개 포인트)");
    }
}

/// <summary>
/// 도로 데이터를 저장하는 컴포넌트
/// </summary>
public class RoadData : MonoBehaviour
{
    [Header("Control Points")]
    public List<Vector3> controlPoints = new List<Vector3>();
    
    [Header("Road Settings")]
    public float roadWidth = 4f;
    public int curveResolution = 10;
    public bool isLooped = false;
}
