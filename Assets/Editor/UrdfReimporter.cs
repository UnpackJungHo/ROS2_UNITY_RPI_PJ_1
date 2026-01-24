using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Xml.Linq;
using UnityEngine;
using UnityEditor;
using Unity.EditorCoroutines.Editor;
using Unity.Robotics.UrdfImporter;
using Unity.Robotics.UrdfImporter.Editor;

public class UrdfReimporter : EditorWindow
{
    private static string urdfFilePath;
    private static string robotName;
    private static RobotBackupData backupData;
    private static ImportSettings importSettings;

    [MenuItem("Assets/Reimport URDF", true, 1)]
    public static bool ReimportUrdf_IsValid()
    {
        string assetPath = AssetDatabase.GetAssetPath(Selection.activeObject);
        return Path.GetExtension(assetPath)?.ToLower() == ".urdf";
    }

    [MenuItem("Assets/Reimport URDF", false, 1)]
    public static void ReimportUrdf()
    {
        string assetPath = AssetDatabase.GetAssetPath(Selection.activeObject);
        urdfFilePath = UrdfAssetPathHandler.GetFullAssetPath(assetPath);

        robotName = GetRobotNameFromUrdf(urdfFilePath);
        if (string.IsNullOrEmpty(robotName))
        {
            EditorUtility.DisplayDialog("Error", "Could not find robot name in URDF file.", "OK");
            return;
        }

        GameObject existingRobot = GameObject.Find(robotName);

        if (existingRobot != null)
        {
            backupData = BackupRobotData(existingRobot);
            Debug.Log($"[UrdfReimporter] Backed up data for: {robotName}");
            Debug.Log($"[UrdfReimporter] Root position: {backupData.rootPosition}");
            Debug.Log($"[UrdfReimporter] Custom components count: {backupData.customComponents.Count}");

            Undo.DestroyObjectImmediate(existingRobot);
            Debug.Log($"[UrdfReimporter] Deleted existing robot: {robotName}");
        }
        else
        {
            backupData = null;
            Debug.Log($"[UrdfReimporter] No existing robot found with name: {robotName}");
        }

        UrdfReimporter window = GetWindow<UrdfReimporter>("URDF Reimport");
        window.minSize = new Vector2(400, 150);
        window.Show();
    }

    private void OnGUI()
    {
        GUILayout.Space(10);
        GUILayout.Label("URDF Reimport Settings", EditorStyles.boldLabel);

        GUILayout.Space(10);
        GUILayout.Label($"Robot Name: {robotName}");
        GUILayout.Label($"URDF Path: {urdfFilePath}");

        if (backupData != null)
        {
            GUILayout.Space(5);
            GUILayout.Label($"Backup: {backupData.customComponents.Count} custom components saved");
        }

        GUILayout.Space(15);

        if (importSettings == null)
            importSettings = ImportSettings.DefaultSettings();

        importSettings.chosenAxis = (ImportSettings.axisType)EditorGUILayout.EnumPopup("Axis Type", importSettings.chosenAxis);
        importSettings.convexMethod = (ImportSettings.convexDecomposer)EditorGUILayout.EnumPopup("Mesh Decomposer", importSettings.convexMethod);

        GUILayout.Space(15);

        if (GUILayout.Button("Import URDF", GUILayout.Height(30)))
        {
            EditorCoroutineUtility.StartCoroutine(ImportAndRestore(), this);
        }
    }

    private IEnumerator ImportAndRestore()
    {
        importSettings.OverwriteExistingPrefabs = true;
        var createCoroutine = UrdfRobotExtensions.Create(urdfFilePath, importSettings, true);

        while (createCoroutine.MoveNext())
        {
            yield return null;
        }

        GameObject newRobot = createCoroutine.Current;

        if (newRobot != null && backupData != null)
        {
            yield return null;
            RestoreRobotData(newRobot, backupData);
            Debug.Log($"[UrdfReimporter] Restored custom data to: {robotName}");
        }

        Close();
    }

    private static string GetRobotNameFromUrdf(string filePath)
    {
        try
        {
            XDocument doc = XDocument.Load(filePath);
            XElement robot = doc.Element("robot");
            return robot?.Attribute("name")?.Value;
        }
        catch (Exception e)
        {
            Debug.LogError($"[UrdfReimporter] Error parsing URDF: {e.Message}");
            return null;
        }
    }

    private static RobotBackupData BackupRobotData(GameObject robot)
    {
        RobotBackupData data = new RobotBackupData();
        data.robotName = robot.name;
        data.rootPosition = robot.transform.position;
        data.rootRotation = robot.transform.rotation;
        data.customComponents = new List<ComponentBackupData>();

        BackupGameObjectComponents(robot, "", data.customComponents);

        return data;
    }

    private static void BackupGameObjectComponents(GameObject obj, string path, List<ComponentBackupData> backupList)
    {
        Component[] components = obj.GetComponents<Component>();

        foreach (Component comp in components)
        {
            if (comp == null) continue;

            Type compType = comp.GetType();

            if (IsCustomComponent(compType))
            {
                ComponentBackupData compData = new ComponentBackupData();
                compData.gameObjectPath = path;
                compData.componentTypeName = compType.AssemblyQualifiedName;
                compData.jsonData = JsonUtility.ToJson(comp);
                compData.enabled = (comp is Behaviour behaviour) ? behaviour.enabled : true;

                backupList.Add(compData);
                Debug.Log($"[UrdfReimporter] Backed up component: {compType.Name} on {path}");
            }
        }

        foreach (Transform child in obj.transform)
        {
            string childPath = string.IsNullOrEmpty(path) ? child.name : $"{path}/{child.name}";
            BackupGameObjectComponents(child.gameObject, childPath, backupList);
        }
    }

    private static bool IsCustomComponent(Type type)
    {
        if (type == null) return false;

        string[] builtInNamespaces = {
            "UnityEngine",
            "Unity.Robotics",
            "TMPro"
        };

        string typeNamespace = type.Namespace ?? "";

        foreach (string ns in builtInNamespaces)
        {
            if (typeNamespace.StartsWith(ns))
                return false;
        }

        if (type == typeof(Transform)) return false;
        if (type == typeof(MeshFilter)) return false;
        if (type == typeof(MeshRenderer)) return false;
        if (type == typeof(Collider)) return false;
        if (type.IsSubclassOf(typeof(Collider))) return false;
        if (type == typeof(Rigidbody)) return false;
        if (type == typeof(ArticulationBody)) return false;

        return true;
    }

    private static void RestoreRobotData(GameObject robot, RobotBackupData data)
    {
        robot.transform.position = data.rootPosition;
        robot.transform.rotation = data.rootRotation;

        foreach (ComponentBackupData compData in data.customComponents)
        {
            Transform target;
            if (string.IsNullOrEmpty(compData.gameObjectPath))
            {
                target = robot.transform;
            }
            else
            {
                target = robot.transform.Find(compData.gameObjectPath);
            }

            if (target == null)
            {
                Debug.LogWarning($"[UrdfReimporter] Could not find: {compData.gameObjectPath}");
                continue;
            }

            Type compType = Type.GetType(compData.componentTypeName);
            if (compType == null)
            {
                Debug.LogWarning($"[UrdfReimporter] Could not find type: {compData.componentTypeName}");
                continue;
            }

            Component existingComp = target.GetComponent(compType);
            if (existingComp == null)
            {
                existingComp = target.gameObject.AddComponent(compType);
            }

            if (existingComp != null)
            {
                JsonUtility.FromJsonOverwrite(compData.jsonData, existingComp);

                if (existingComp is Behaviour behaviour)
                {
                    behaviour.enabled = compData.enabled;
                }

                Debug.Log($"[UrdfReimporter] Restored: {compType.Name} on {compData.gameObjectPath}");
            }
        }
    }

    [Serializable]
    private class RobotBackupData
    {
        public string robotName;
        public Vector3 rootPosition;
        public Quaternion rootRotation;
        public List<ComponentBackupData> customComponents;
    }

    [Serializable]
    private class ComponentBackupData
    {
        public string gameObjectPath;
        public string componentTypeName;
        public string jsonData;
        public bool enabled;
    }
}
