using UnityEngine;
using UnityEditor;
using System.Collections.Generic;


/// <summary>
/// 도로 데이터를 저장하는 컴포넌트
/// </summary>
public class RoadData : MonoBehaviour
{
    [Header("Control Points")]
    public List<Vector3> controlPoints = new List<Vector3>();
    
    [Header("Road Settings")]
    public float roadWidth = 5f;
    public int curveResolution = 10;
    public Material roadMaterial;
    public bool isLooped = false;
    
    // 턱(Curb) 설정
    public bool hasCurbs = false;
    public float curbWidth = 0.5f;
    public float curbHeight = 0.2f;
    public Material curbMaterial;
}

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
    
    // 턱(Curb) 설정
    private bool hasCurbs = false;
    private float curbWidth = 0.5f;
    private float curbHeight = 0.2f;
    private Material curbMaterial;
    
    // 포인트 선택 (삽입 기능용)
    private List<int> selectedPointIndices = new List<int>();
    private const float POINT_SELECT_DISTANCE = 30f; // 포인트 선택 반경 (픽셀 단위)
    
    // UI 스크롤
    private Vector2 scrollPosition;
    
    // 도로 목록 관리
    private List<RoadData> sceneRoads = new List<RoadData>();
    private Vector2 roadListScrollPosition;
    

    
    // 최적화 변수
    private bool isDraggingPoint = false;
    
    [MenuItem("Tools/Road Creator")]
    public static void ShowWindow()
    {
        GetWindow<RoadCreator>("Road Creator");
    }
    
    private void OnEnable()
    {
        SceneView.duringSceneGui += OnSceneGUI;
        LoadDefaultMaterial();
        RefreshRoadList(); // 시작 시 목록 로드
    }
    
    private void OnDisable()
    {
        SceneView.duringSceneGui -= OnSceneGUI;
    }

    private void RefreshRoadList()
    {
        sceneRoads.Clear();
        // 씬 내의 모든 RoadData 컴포넌트 찾기
        RoadData[] foundRoads = FindObjectsOfType<RoadData>();
        sceneRoads.AddRange(foundRoads);
    }
    
    private void LoadRoad(RoadData roadData)
    {
        if (roadData == null) return;
        
        controlPoints = new List<Vector3>(roadData.controlPoints);
        roadWidth = roadData.roadWidth;
        curveResolution = roadData.curveResolution;
        isLooped = roadData.isLooped;
        
        // Curb Data Load
        hasCurbs = roadData.hasCurbs;
        curbWidth = roadData.curbWidth;
        curbHeight = roadData.curbHeight;
        curbMaterial = roadData.curbMaterial;
        
        currentRoadObject = roadData.gameObject;
        selectedPointIndices.Clear();
        
        Debug.Log($"Road Creator: '{roadData.name}' 로드 완료 ({controlPoints.Count} 포인트)");
        SceneView.RepaintAll();
        Repaint();
    }
    
    private void LoadDefaultMaterial()
    {
        // RoadMaterial 자동 로드
        string[] guids = AssetDatabase.FindAssets("one_lane_road_material t:Material");
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

        // --- 도로 목록 섹션 (수정된 요구사항 1, 2) ---
        EditorGUILayout.LabelField("📋 저장된 도로 목록", EditorStyles.boldLabel);
        if (GUILayout.Button("🔄 목록 새로고침"))
        {
            RefreshRoadList();
        }

        roadListScrollPosition = EditorGUILayout.BeginScrollView(roadListScrollPosition, GUILayout.Height(120));
        if (sceneRoads.Count == 0)
        {
            EditorGUILayout.LabelField("저장된 도로가 없습니다. (RoadData 컴포넌트 검색)", EditorStyles.miniLabel);
        }
        else
        {
            for (int i = 0; i < sceneRoads.Count; i++)
            {
                RoadData road = sceneRoads[i];
                if (road == null) continue;

                EditorGUILayout.BeginHorizontal();
                string roadName = road.name;
                if (road.gameObject == currentRoadObject) roadName += " (현재 편집 중)";
                
                if (GUILayout.Button(roadName, EditorStyles.miniButtonLeft))
                {
                    LoadRoad(road);
                }
                
                GUI.backgroundColor = Color.red;
                if (GUILayout.Button("X", EditorStyles.miniButtonRight, GUILayout.Width(25)))
                {
                    if (EditorUtility.DisplayDialog("도로 삭제", $"'{road.name}' 도로를 정말 삭제하시겠습니까?", "삭제", "취소"))
                    {
                        Undo.DestroyObjectImmediate(road.gameObject);
                        RefreshRoadList();
                        if (currentRoadObject == road.gameObject) ClearAll();
                    }
                }
                GUI.backgroundColor = Color.white;
                EditorGUILayout.EndHorizontal();
            }
        }
        EditorGUILayout.EndScrollView();
        
        if (GUILayout.Button("🚧 성능 최적화 (Collider 강제 업데이트)"))
        {
            UpdateRoadMesh(true);
        }
        
        EditorGUILayout.Space(10);

        // --- 기존 설정 UI ---
        
        // 도로 설정
        EditorGUILayout.LabelField("도로 설정", EditorStyles.boldLabel);
        roadWidth = EditorGUILayout.Slider("도로 너비", roadWidth, 1f, 20f);
        curveResolution = EditorGUILayout.IntSlider("곡선 해상도", curveResolution, 2, 30);
        roadMaterial = (Material)EditorGUILayout.ObjectField("도로 머테리얼", roadMaterial, typeof(Material), false);
        
        EditorGUILayout.Space(5);
        
        // 턱(Curbs) 설정 UI
        EditorGUILayout.LabelField("턱(Curb) 설정", EditorStyles.boldLabel);
        hasCurbs = EditorGUILayout.Toggle("턱 생성", hasCurbs);
        if (hasCurbs)
        {
            EditorGUI.indentLevel++;
            curbWidth = EditorGUILayout.Slider("턱 너비", curbWidth, 0.1f, 2.0f);
            curbHeight = EditorGUILayout.Slider("턱 높이", curbHeight, 0.05f, 1.0f);
            curbMaterial = (Material)EditorGUILayout.ObjectField("턱 머테리얼", curbMaterial, typeof(Material), false);
            EditorGUI.indentLevel--;
        }
        
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
            EditorGUILayout.HelpBox("Shift + 좌클릭: 포인트 추가/선택\nShift + 우클릭 (마지막 포인트): 삭제\nI 키: 선택된 두 포인트 사이에 삽입\nESC: 선택 해제", MessageType.Info);
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
                Undo.RecordObject(this, "Remove Point");
                controlPoints.RemoveAt(i);
                UpdateRoadMesh(true);
                break;
            }
            EditorGUILayout.EndHorizontal();
        }
        EditorGUILayout.EndScrollView();
        
        EditorGUILayout.Space(5);
        
        // 하단 버튼
        EditorGUILayout.BeginHorizontal();
        if (GUILayout.Button(" 도로 저장/업데이트", GUILayout.Height(30)))
        {
            SaveRoadData();
            RefreshRoadList(); // 저장 후 목록 갱신
        }
        
        GUI.backgroundColor = Color.red;
        if (GUILayout.Button("초기화", GUILayout.Height(30), GUILayout.Width(80)))
        {
            ClearAll();
        }
        GUI.backgroundColor = Color.white;
        EditorGUILayout.EndHorizontal();
        
        EditorGUILayout.Space(5);
        
        EditorGUILayout.BeginHorizontal();
        if (GUILayout.Button("새 도로 작업 시작", GUILayout.Height(25)))
        {
            StartNewRoad();
        }
        
        // 루프 닫기 버튼
        GUI.enabled = controlPoints.Count >= 3;
        GUI.backgroundColor = isLooped ? Color.green : Color.yellow;
        if (GUILayout.Button(isLooped ? "✓ 루프 완성됨" : "🔗 루프 닫기", GUILayout.Height(25)))
        {
            isLooped = !isLooped;
            UpdateRoadMesh(true);
        }
        GUI.backgroundColor = Color.white;
        GUI.enabled = true;
        EditorGUILayout.EndHorizontal();
        
        if (isLooped)
        {
            EditorGUILayout.HelpBox("도로가 루프로 연결되었습니다.", MessageType.Info);
        }
    }
    
    private void OnSceneGUI(SceneView sceneView)
    {
        Event e = Event.current; // Event 변수 정의 추가
        // 입력 처리 (우선 순위 높임)
        if (isCreating)
        {
            HandleInput(sceneView);
        }

        // 포인트 핸들 그리기
        for (int i = 0; i < controlPoints.Count; i++)
        {
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

            // 드래그 가능한 핸들 (입력 처리 후 그림)
            EditorGUI.BeginChangeCheck();
            Vector3 newPos = Handles.PositionHandle(controlPoints[i], Quaternion.identity);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(this, "Move Road Point");
                controlPoints[i] = newPos;
                isDraggingPoint = true;
                UpdateRoadMesh(); // 드래그 중에는 Mesh만 업데이트 (Collider X)
            }
            
            // 드래그 종료 감지 (Repaint 이벤트에서 확인)
            if (e.type == EventType.MouseUp && e.button == 0)
            {
                if (isDraggingPoint)
                {
                    isDraggingPoint = false;
                    UpdateRoadMesh(true); // 드래그 종료 시 Collider 포함 업데이트
                }
            }
        }
        
        // 연결선 그리기
        if (controlPoints.Count >= 2)
        {
            Handles.color = Color.green;
            List<Vector3> splinePoints = GenerateSplinePoints();
            Handles.DrawPolyLine(splinePoints.ToArray());
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
            
            // 먼저 기존 포인트 근처 클릭인지 확인 (수정된 로직)
            int clickedPointIndex = GetPointIndexNearRay(e.mousePosition);
            
            if (clickedPointIndex >= 0)
            {
                // 기존 포인트 선택/해제
                TogglePointSelection(clickedPointIndex);
                e.Use(); // 이벤트 소비하여 다른 핸들과 겹치지 않게 함
                return;
            }
            
            // 새 포인트 추가
            if (Physics.Raycast(ray, out RaycastHit hit))
            {
                // Y값은 항상 0으로 고정 (사용자 요구사항)
                Vector3 newPoint = hit.point;
                newPoint.y = 0f;
                AddPoint(newPoint);
            }
            else
            {
                Plane plane = new Plane(Vector3.up, Vector3.zero);
                if (plane.Raycast(ray, out float distance))
                {
                    Vector3 point = ray.GetPoint(distance);
                    point.y = 0f; // 명시적으로 0으로 설정
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
        
        if (selectedPointIndices.Count > 0)
        {
            string selected = string.Join(", ", selectedPointIndices.ConvertAll(i => $"P{i}"));
            EditorGUILayout.HelpBox($"선택된 포인트: {selected}\n(I: 삽입, Shift+Click: 추가/선택)", MessageType.Warning);
        }
        // 씨 뷰 UI
        Handles.BeginGUI();
        string statusText = "🛣️ Road Creator: Shift+Click=Point, I=Insert";
    }
    
    /// <summary>
    /// 마우스 위치 근처의 포인트 인덱스 반환 (화면 좌표 거리 기준)
    /// </summary>
    private int GetPointIndexNearRay(Vector2 mousePosition)
    {
        float minDistance = POINT_SELECT_DISTANCE;
        int closestIndex = -1;
        
        for (int i = 0; i < controlPoints.Count; i++)
        {
            // 월드 좌표를 화면(GUI) 좌표로 변환
            Vector2 guiPoint = HandleUtility.WorldToGUIPoint(controlPoints[i]);
            
            // 화면상 거리 계산
            float distance = Vector2.Distance(guiPoint, mousePosition);
            
            if (distance < minDistance)
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
        
        if (isLooped && minIdx == 0 && maxIdx == controlPoints.Count - 1)
        {
            controlPoints.Add(midPoint);
        }
        else
        {
            controlPoints.Insert(maxIdx, midPoint);
        }
        
        selectedPointIndices.Clear();
        UpdateRoadMesh(true);
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
                
                roadData.hasCurbs = hasCurbs;
                roadData.curbWidth = curbWidth;
                roadData.curbHeight = curbHeight;
                roadData.curbMaterial = curbMaterial;
            }
        }
        
        UpdateRoadMesh();
    }
    
    private void UpdateRoadMesh(bool updateCollider = true)
    {
        if (currentRoadObject == null || controlPoints.Count < 2) return;
        
        // 드래그 중일 때는 해상도 낮춤 (최적화)
        int logicCurveResolution = isDraggingPoint ? Mathf.Max(2, curveResolution / 4) : curveResolution;
        
        // GenerateSplinePoints 최적화 버전 호출이 필요하나, 
        // 기존 메서드 구조상 curveResolution 멤버를 임시로 바꿈
        int originalRes = curveResolution;
        curveResolution = logicCurveResolution;
        
        // 1. 도로 메쉬 업데이트
        Mesh mesh = GenerateRoadMesh();
        
        curveResolution = originalRes; // 복구

        MeshFilter mf = currentRoadObject.GetComponent<MeshFilter>();
        MeshRenderer mr = currentRoadObject.GetComponent<MeshRenderer>();
        
        if (mf.sharedMesh != null)
        {
            DestroyImmediate(mf.sharedMesh); // 메모리 누수 방지
        }
        
        mf.sharedMesh = mesh;
        
        if (roadMaterial != null)
        {
            mr.sharedMaterial = roadMaterial;
        }
        
        // MeshCollider 업데이트 (최적화: 드래그 중에는 스킵)
        if (updateCollider)
        {
            MeshCollider mc = currentRoadObject.GetComponent<MeshCollider>();
            if (mc == null)
            {
                mc = currentRoadObject.AddComponent<MeshCollider>();
            }
            mc.sharedMesh = mesh;
        }
        
        // 2. 턱(Curb) 메쉬 업데이트
        UpdateCurbMeshes(updateCollider, logicCurveResolution);
        
        SceneView.RepaintAll();
    }

    private void UpdateCurbMeshes(bool updateCollider, int tempResolution)
    {
        // 자식 오브젝트 찾기 또는 생성
        Transform leftCurbTr = currentRoadObject.transform.Find("CurbLeft");
        Transform rightCurbTr = currentRoadObject.transform.Find("CurbRight");
        
        if (!hasCurbs)
        {
            // 턱이 비활성화되었는데 오브젝트가 있으면 삭제
            if (leftCurbTr != null) Undo.DestroyObjectImmediate(leftCurbTr.gameObject);
            if (rightCurbTr != null) Undo.DestroyObjectImmediate(rightCurbTr.gameObject);
            return;
        }
        
        // Material 결정 (없으면 도로 재질 사용)
        Material mat = curbMaterial != null ? curbMaterial : (roadMaterial != null ? roadMaterial : null);
        
        // 임시 해상도 적용
        int originalRes = curveResolution;
        curveResolution = tempResolution;
        
        // 왼쪽 턱 업데이트
        UpdateSingleCurb(ref leftCurbTr, "CurbLeft", true, mat, updateCollider);
        
        // 오른쪽 턱 업데이트
        UpdateSingleCurb(ref rightCurbTr, "CurbRight", false, mat, updateCollider);
        
        curveResolution = originalRes; // 복구
    }

    private void UpdateSingleCurb(ref Transform curbTr, string name, bool isLeft, Material mat, bool updateCollider)
    {
        if (curbTr == null)
        {
            GameObject go = new GameObject(name);
            go.transform.SetParent(currentRoadObject.transform, false);
            go.AddComponent<MeshFilter>();
            go.AddComponent<MeshRenderer>();
            go.AddComponent<MeshCollider>();
            curbTr = go.transform;
            Undo.RegisterCreatedObjectUndo(go, "Create Curb");
        }
        
        MeshFilter mf = curbTr.GetComponent<MeshFilter>();
        MeshRenderer mr = curbTr.GetComponent<MeshRenderer>();
        MeshCollider mc = curbTr.GetComponent<MeshCollider>();
        
        mr.sharedMaterial = mat;
        
        Mesh mesh = GenerateCurbMesh(isLeft);
        
        if (mf.sharedMesh != null) DestroyImmediate(mf.sharedMesh);
        mf.sharedMesh = mesh;
        
        if (updateCollider)
        {
            if (mc == null) mc = curbTr.gameObject.AddComponent<MeshCollider>();
            mc.sharedMesh = mesh;
        }
    }
    
    private Mesh GenerateCurbMesh(bool isLeft)
    {
        List<Vector3> splinePoints = GenerateSplinePoints();
        if (splinePoints.Count < 2) return new Mesh();
        
        Mesh mesh = new Mesh();
        mesh.name = isLeft ? "CurbLeftMesh" : "CurbRightMesh";
        
        List<Vector3> vertices = new List<Vector3>();
        List<Vector2> uvs = new List<Vector2>();
        List<int> triangles = new List<int>();
        
        float currentLength = 0f;
        
        // 턱 오프셋 계산 (도로 중심에서 턱 중심까지가 아니라, 도로 끝에서 시작)
        // 왼쪽: -roadWidth/2 에서 시작해서 -direction * curbWidth 만큼 확장
        // 오른쪽: +roadWidth/2 에서 시작해서 +direction * curbWidth 만큼 확장
        
        for (int i = 0; i < splinePoints.Count; i++)
        {
            Vector3 point = splinePoints[i];
            

            
            Vector3 forward;
            
            // Tangents 계산 (도로 생성과 동일 로직)
            if (isLooped)
            {
                Vector3 prevPoint = (i == 0) ? splinePoints[splinePoints.Count - 2] : splinePoints[i - 1];
                Vector3 nextPoint = (i == splinePoints.Count - 1) ? splinePoints[1] : splinePoints[i + 1];
                forward = (nextPoint - prevPoint).normalized;
            }
            else
            {
                if (i == 0) forward = (splinePoints[i + 1] - point).normalized;
                else if (i == splinePoints.Count - 1) forward = (point - splinePoints[i - 1]).normalized;
                else forward = (splinePoints[i + 1] - splinePoints[i - 1]).normalized;
            }
            
            Vector3 right = Vector3.Cross(Vector3.up, forward).normalized;
            Vector3 up = Vector3.up;
            
            // 기준점: 도로의 가장자리
            Vector3 roadEdge;
            Vector3 outerEdge;
            
            if (isLeft)
            {
                // 왼쪽 가장자리 (-right)
                roadEdge = point - right * (roadWidth / 2f);
                outerEdge = roadEdge - right * curbWidth;
            }
            else
            {
                // 오른쪽 가장자리 (+right)
                roadEdge = point + right * (roadWidth / 2f);
                outerEdge = roadEdge + right * curbWidth;
            }
            
            // 4개의 버텍스 생성 (단면: 사각형)
            // 0: Inner Bottom (도로와 맞닿는 아래)
            // 1: Inner Top (도로와 맞닿는 위 - 턱 높이)
            // 2: Outer Top (바깥쪽 위)
            // 3: Outer Bottom (바깥쪽 아래)
            
            // 실제로는 바닥면(Bottom)은 필요 없을 수도 있지만 Collider를 위해 닫힌 메쉬 생성
            
            Vector3 vIB = roadEdge;
            Vector3 vIT = roadEdge + up * curbHeight;
            Vector3 vOT = outerEdge + up * curbHeight;
            Vector3 vOB = outerEdge;
            
            vertices.Add(vIB); // 0
            vertices.Add(vIT); // 1
            vertices.Add(vOT); // 2
            vertices.Add(vOB); // 3
            
            // UVs
            float v = currentLength / curbWidth; // 타일링
            uvs.Add(new Vector2(0, v));
            uvs.Add(new Vector2(0, v));
            uvs.Add(new Vector2(1, v));
            uvs.Add(new Vector2(1, v));
            
            // Triangles
            if (i > 0)
            {
                int b = (i - 1) * 4; // base index
                
                // 각 면(Face)에 대해 삼각형 2개씩 생성
                
                // Top Face (1 -> 1', 2' -> 2)
                // Left일 때는 Winding Reverse (1, 2, 6, 5) -> Normal Up
                // Right일 때는 Standard (1, 5, 6, 2) -> Normal Up
                if (isLeft) AddQuad(triangles, b+1, b+2, b+6, b+5);
                else AddQuad(triangles, b+1, b+5, b+6, b+2);
                
                // Outer Face (2 -> 2', 3' -> 3)
                // Left일 때는 Normal Left (2, 3, 7, 6)
                // Right일 때는 Normal Right (2, 6, 7, 3)
                if (isLeft) AddQuad(triangles, b+2, b+3, b+7, b+6);
                else AddQuad(triangles, b+2, b+6, b+7, b+3);
                
                // Inner Face (Optional - 도로에 가려지지만 생성)
                // Left: Normal Right (Towards Road) -> (1, 5, 4, 0)
                // Right: Normal Left (Towards Road) -> (0, 4, 5, 1)
                if (isLeft) AddQuad(triangles, b+1, b+5, b+4, b+0);
                else AddQuad(triangles, b+0, b+4, b+5, b+1);
                
                // Back/Bottom Face (생략 가능하나 닫힌 메쉬를 위해)
                // Bottom은 항상 Down.
                // Left: (0, 4, 7, 3) -> Fwd, Left -> Cross = Down
                // Right: (3, 7, 4, 0) -> Fwd, Right -> Cross = Down
                if (isLeft) AddQuad(triangles, b+0, b+4, b+7, b+3);
                else AddQuad(triangles, b+3, b+7, b+4, b+0);
            }
            
            // Start Cap (Loop가 아닐 때)
            if (i == 0 && !isLooped)
            {
                // 0, 1, 2, 3
                // Normal Back (Towards -Forward)
                // Left: (0, 1, 2, 3) -> Up, Left -> Back? NO.
                // 0->1(Up), 0->3(Left). Cross(Up, Left) = Fwd.
                // We want Back. So (0, 3, 2, 1) or (1, 2, 3, 0).
                // 0->3 (Left), 0->1 (Up). Cross(Left, Up) = Fwd.
                // Wait. 0->1 is Up. 1->2 is Left.
                // Draw 0,1,2,3 loop. 0->1->2->3. CCW. Normal Fwd.
                // We want Back. So 0->3->2->1.
                // Note: isLeft toggle might affect position but local indices 0,1,2,3 relative.
                // Left Curb: 0(IB), 1(IT), 2(OT), 3(OB).
                // 0 is Right of 3.
                // 0->1 Up. 1->2 Left.
                // Face pointing Back (Reverse spline direction).
                // 0->1->2->3 is FWD.
                // So we want 0->3->2->1 for Start Cap (Look at it from back).
                
                // Right Curb: 0(IB), 1(IT), 2(OT), 3(OB).
                // 0 is Left of 3.
                // 0->1 Up. 1->2 Right.
                // 0->1->2->3 is Back?
                // 0->1 (Up). 1->2 (Right). Cross(Up, Right) = Fwd? No.
                // (0,1,0) x (1,0,0) = (0,0,-1) = Back.
                // So for Right Curb, 0->1->2->3 points Back.
                // Since this is Start Cap, we want Normal pointing Back (away from road start).
                
                if (isLeft) AddQuad(triangles, 0, 3, 2, 1);
                else AddQuad(triangles, 0, 1, 2, 3);
            }
            
            // End Cap (Loop가 아닐 때)
            if (i == splinePoints.Count - 2 && !isLooped) // Last segment
            {
                // Last 4 vertices indices
                int b = (i) * 4;
                // indices: b, b+1, b+2, b+3
                
                // Normal Forward (Towards +Forward)
                // Left: 0->1->2->3 is Fwd.
                // Right: 0->3->2->1 is Fwd.
                
                if (isLeft) AddQuad(triangles, b+0, b+1, b+2, b+3);
                else AddQuad(triangles, b+0, b+3, b+2, b+1);
            }

            
            if (i < splinePoints.Count - 1)
            {
                currentLength += Vector3.Distance(splinePoints[i], splinePoints[i + 1]);
            }
        }
        
        mesh.vertices = vertices.ToArray();
        mesh.uv = uvs.ToArray();
        mesh.triangles = triangles.ToArray();
        mesh.RecalculateNormals();
        
        if (isLooped) WeldNormals(mesh, 4);
        
        mesh.RecalculateBounds();
        return mesh;
    }

    private void AddQuad(List<int> triangles, int v0, int v1, int v2, int v3)
    {
        triangles.Add(v0);
        triangles.Add(v1);
        triangles.Add(v2);
        
        triangles.Add(v0);
        triangles.Add(v2);
        triangles.Add(v3);
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
            
            // 진행 방향 계산 (Tangents)
            if (isLooped)
            {
                // 루프일 때: 시작과 끝이 부드럽게 이어지도록 처리
                Vector3 prevPoint, nextPoint;

                if (i == 0)
                {
                    // 시작점의 이전 점은 (끝점 바로 앞의 점)
                    // splinePoints의 마지막은 시작점과 같으므로 그 이전 점을 사용
                    prevPoint = splinePoints[splinePoints.Count - 2];
                    nextPoint = splinePoints[i + 1];
                }
                else if (i == splinePoints.Count - 1)
                {
                    // 끝점의 다음 점은 (시작점 바로 다음 점)
                    prevPoint = splinePoints[i - 1];
                    nextPoint = splinePoints[1];
                }
                else
                {
                    prevPoint = splinePoints[i - 1];
                    nextPoint = splinePoints[i + 1];
                }
                
                forward = (nextPoint - prevPoint).normalized;
            }
            else
            {
                // 열린 도로일 때: 기존 로직 + 양끝 처리
                if (i == 0)
                {
                    forward = (splinePoints[i + 1] - point).normalized;
                }
                else if (i == splinePoints.Count - 1)
                {
                    forward = (point - splinePoints[i - 1]).normalized;
                }
                else
                {
                    // 중간점은 앞뒤 평균 (Central Difference)
                    forward = (splinePoints[i + 1] - splinePoints[i - 1]).normalized;
                }
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
                
                // 루프여도 마지막 점을 복제하여 따로 생성하므로
                // 특별한 인덱스 연결 로직이 필요 없음 (자연스럽게 이어짐)
                int nextLeft = baseIndex + 2;
                int nextRight = baseIndex + 3;
                
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
            if (i < splinePoints.Count - 1)
            {
                currentLength += Vector3.Distance(splinePoints[i], splinePoints[i + 1]);
            }
        }
        
        mesh.vertices = vertices.ToArray();
        mesh.uv = uvs.ToArray();
        mesh.triangles = triangles.ToArray();
        
        mesh.RecalculateNormals();
        
        // 루프일 때 시작점과 끝점의 노말을 일치시킴 (심 제거)
        if (isLooped)
        {
            WeldNormals(mesh, 2);
        }
        
        mesh.RecalculateBounds();
        
        return mesh;
    }

    private void WeldNormals(Mesh mesh, int verticesPerSegment)
    {
        Vector3[] normals = mesh.normals;
        int vertexCount = normals.Length;
        
        // 시작점의 버텍스들과 끝점의 버텍스들은 같은 위치임
        // 각각의 평균 노말을 계산하여 적용
        
        // verticesPerSegment 만큼 반복 (Road=2, Curb=4)
        for (int i = 0; i < verticesPerSegment; i++)
        {
            // 시작점의 i번째 버텍스 인덱스: i
            // 끝점의 i번째 버텍스 인덱스: vertexCount - verticesPerSegment + i
            
            int idxStart = i;
            int idxEnd = vertexCount - verticesPerSegment + i;
            
            if (idxEnd >= vertexCount) continue; // 안전장치
            
            Vector3 avgNormal = (normals[idxStart] + normals[idxEnd]).normalized;
            
            normals[idxStart] = avgNormal;
            normals[idxEnd] = avgNormal;
        }
        
        mesh.normals = normals;
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
            
            // Curb 데이터 로드
            hasCurbs = roadData.hasCurbs;
            curbWidth = roadData.curbWidth;
            curbHeight = roadData.curbHeight;
            curbMaterial = roadData.curbMaterial;
            
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
        
        roadData.hasCurbs = hasCurbs;
        roadData.curbWidth = curbWidth;
        roadData.curbHeight = curbHeight;
        roadData.curbHeight = curbHeight;
        roadData.curbMaterial = curbMaterial;
        
        EditorUtility.SetDirty(roadData);
        
        // 데이터 저장 후 메쉬 업데이트하여 변경사항 즉시 반영 (Fix 1)
        UpdateRoadMesh(true);
        
        Debug.Log($"Road Creator: 도로 데이터 저장됨 ({controlPoints.Count}개 포인트)");
    }
}
