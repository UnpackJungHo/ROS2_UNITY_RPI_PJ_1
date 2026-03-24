#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using Unity.MLAgents.Policies;
using Unity.Sentis;
using UnityEngine.SceneManagement;

/// <summary>
/// AutoCar_Root_Train의 모든 컴포넌트 참조를 RootAMR_Train 기준으로 재연결한다.
/// 메뉴: Tools/AutoCar/Setup References
/// 실행 후 씬을 저장할 것 (Ctrl+S).
/// </summary>
public class AutoCarSetup
{
    [MenuItem("Tools/AutoCar/Setup References")]
    public static void SetupReferences()
    {
        var log = new System.Text.StringBuilder("[AutoCarSetup] 참조 재연결 시작\n");
        int ok = 0, warn = 0;

        // ─── 루트 오브젝트 탐색 ───────────────────────────────────────
        var autoCarRoot = FindAutoCarRoot();
        if (autoCarRoot == null) { Debug.LogError("[AutoCarSetup] AutoCar_Root_Train not found"); return; }

        var autoCarT           = autoCarRoot.transform.Find("AutoCar");
        var autoCarControllerT = autoCarRoot.transform.Find("AutoCarController");
        var rlStartPoseT       = autoCarRoot.transform.Find("RL_StartPose");
        var amrViewT           = autoCarRoot.transform.Find("AMRVIew");

        if (autoCarT == null || autoCarControllerT == null)
        { Debug.LogError("[AutoCarSetup] AutoCar or AutoCarController not found"); return; }

        var autoCar           = autoCarT.gameObject;
        var autoCarController = autoCarControllerT.gameObject;

        // ─── 핵심 하위 오브젝트 탐색 ──────────────────────────────────
        var baseLinkGo     = autoCarT.Find("base_footprint/base_link")?.gameObject;
        var crosswalkGo    = autoCarControllerT.Find("Crosswalk")?.gameObject;
        var sensorsGo      = autoCarControllerT.Find("Sensors")?.gameObject;
        var rlGo           = autoCarControllerT.Find("RL")?.gameObject;
        var rewardGo       = rlGo?.transform.Find("Reward")?.gameObject;
        var rlEpisodeGo    = rlGo?.transform.Find("RL Episode")?.gameObject;
        var agentGo        = rlGo?.transform.Find("Agent")?.gameObject;
        var cameraLinkGo   = autoCarT.Find("base_footprint/base_link/camera_mount/camera_link")?.gameObject;
        var amrViewGo      = amrViewT?.gameObject;

        if (baseLinkGo == null) { Debug.LogError("[AutoCarSetup] base_link not found"); return; }

        // ─── 초음파 / 레이더 링크 탐색 ────────────────────────────────
        var baseLinkChildT = baseLinkGo.transform;
        var ultraFL = baseLinkChildT.Find("ultrasonic_fl_link")?.gameObject;
        var ultraFR = baseLinkChildT.Find("ultrasonic_fr_link")?.gameObject;
        var ultraFC = baseLinkChildT.Find("ultrasonic_fc_link")?.gameObject;
        var ultraRL = baseLinkChildT.Find("ultrasonic_rl_link")?.gameObject;
        var ultraRR = baseLinkChildT.Find("ultrasonic_rr_link")?.gameObject;
        var ultraRC = baseLinkChildT.Find("ultrasonic_rc_link")?.gameObject;
        var ultraSL = baseLinkChildT.Find("ultrasonic_sl_link")?.gameObject;
        var ultraSR = baseLinkChildT.Find("ultrasonic_sr_link")?.gameObject;
        var radarFrontGo = baseLinkChildT.Find("radar_front_link")?.gameObject;
        var radarRearGo  = baseLinkChildT.Find("radar_rear_link")?.gameObject;

        // ─── 컴포넌트 레퍼런스 수집 ───────────────────────────────────
        var baseLinkVMC    = baseLinkGo.GetComponent<VehicleMotionController>();
        var baseLinkAB     = baseLinkGo.GetComponent<ArticulationBody>();
        var regression     = autoCar.GetComponent<RegressionDrivingController>();
        var collector      = autoCar.GetComponent<DrivingDataCollectorV2>();
        var cmdSubscriber  = autoCar.GetComponent<VehicleCmdSubscriber>();
        var cmdController  = autoCar.GetComponent<VehicleCmdController>();
        var tlEngine       = crosswalkGo?.GetComponent<TrafficLightDecisionEngine>();
        var cwe            = sensorsGo?.GetComponent<CollisionWarningEngine>();
        var cwBridge       = sensorsGo?.GetComponent<CollisionWarningRosBridge>();
        var progressReward = rewardGo?.GetComponent<ProgressRewardProvider>();
        var obsPublisher   = rewardGo?.GetComponent<ReinforcementObservationPublisher>();
        var rlEval         = rlEpisodeGo?.GetComponent<RLEpisodeEvaluator>();
        var rlAgent        = agentGo?.GetComponent<AutoDriverRLAgent>();
        var behaviorParams = agentGo?.GetComponent<BehaviorParameters>();
        var camPub         = cameraLinkGo?.GetComponent<CameraPublisher>();
        var viewProvider   = amrViewGo?.GetComponent<VehicleViewProvider>();

        // 초음파 센서 컴포넌트
        var sFL = ultraFL?.GetComponent<SingleUltrasonicSensor>();
        var sFR = ultraFR?.GetComponent<SingleUltrasonicSensor>();
        var sFC = ultraFC?.GetComponent<SingleUltrasonicSensor>();
        var sRL = ultraRL?.GetComponent<SingleUltrasonicSensor>();
        var sRR = ultraRR?.GetComponent<SingleUltrasonicSensor>();
        var sRC = ultraRC?.GetComponent<SingleUltrasonicSensor>();
        var sSL = ultraSL?.GetComponent<SingleUltrasonicSensor>();
        var sSR = ultraSR?.GetComponent<SingleUltrasonicSensor>();
        var radarFrontSensor = radarFrontGo?.GetComponent<SingleRadarSensor>();
        var radarRearSensor  = radarRearGo?.GetComponent<SingleRadarSensor>();

        // ─── Sensor Publishers (Sensors 하위) ─────────────────────────
        var sensorsT         = sensorsGo?.transform;
        var imuSensor        = sensorsT?.Find("Imu")?.GetComponent<ImuSensor>();
        var odomPublisher    = sensorsT?.Find("Odom")?.GetComponent<OdometryPublisher>();
        var ultraPublisher   = sensorsT?.Find("Ultrasonic Sensor")?.GetComponent<UltrasonicSensorPublisher>();
        var radarPublisher   = sensorsT?.Find("Radar Sensor")?.GetComponent<RadarSensorPublisher>();

        // ─── 1. DrivingDataCollectorV2 ────────────────────────────────
        if (collector != null)
        {
            collector.cameraPublisher   = camPub;
            collector.wheelController   = baseLinkVMC;
            collector.aiController      = regression;
            collector.autoStartOnMove   = false;
            EditorUtility.SetDirty(collector);
            log.AppendLine("  ✅ DrivingDataCollectorV2"); ok++;
        }
        else { log.AppendLine("  ❌ DrivingDataCollectorV2 not found"); warn++; }

        // ─── 2. RegressionDrivingController ───────────────────────────
        if (regression != null)
        {
            regression.wheelController           = baseLinkVMC;
            regression.cameraPublisher           = camPub;
            regression.dataCollector             = collector;
            regression.collisionWarningEngine    = cwe;
            regression.trafficLightDecisionEngine = tlEngine;
            regression.inferenceInterval         = 0.2f;
            regression.autoResumeDelay           = 0.5f;
            regression.isAutonomousMode          = true;
            regression.maxSpeedLimit             = 2.0f;
            regression.useAcademyStepStagger     = true;
            regression.predictionOnlyMode        = true;

            // ModelAsset (Unity.Sentis) 로드
            var onnxPath = "Assets/Models/ONNX/driving_regression_3_best_steer_mae.onnx";
            var modelAsset = AssetDatabase.LoadAssetAtPath<ModelAsset>(onnxPath);
            if (modelAsset != null)
                regression.modelAsset = modelAsset;
            else
            { log.AppendLine($"  ⚠️ ModelAsset 파일 없음: {onnxPath}"); warn++; }

            EditorUtility.SetDirty(regression);
            log.AppendLine("  ✅ RegressionDrivingController"); ok++;
        }
        else { log.AppendLine("  ❌ RegressionDrivingController not found"); warn++; }

        // ─── 3. VehicleCmdSubscriber ──────────────────────────────────
        if (cmdSubscriber != null)
        {
            cmdSubscriber.controller = cmdController;
            EditorUtility.SetDirty(cmdSubscriber);
            log.AppendLine("  ✅ VehicleCmdSubscriber"); ok++;
        }
        else { log.AppendLine("  ❌ VehicleCmdSubscriber not found"); warn++; }

        // ─── 4. VehicleCmdController ──────────────────────────────────
        if (cmdController != null)
        {
            cmdController.wheelController            = baseLinkVMC;
            cmdController.regressionDrivingController = regression;
            EditorUtility.SetDirty(cmdController);
            log.AppendLine("  ✅ VehicleCmdController"); ok++;
        }
        else { log.AppendLine("  ❌ VehicleCmdController not found"); warn++; }

        // ─── 5. CollisionWarningEngine ────────────────────────────────
        if (cwe != null)
        {
            cwe.sensorFL     = sFL;
            cwe.sensorFR     = sFR;
            cwe.sensorFC     = sFC;
            cwe.sensorRL     = sRL;
            cwe.sensorRR     = sRR;
            cwe.sensorRC     = sRC;
            cwe.sensorSL     = sSL;
            cwe.sensorSR     = sSR;
            cwe.velocitySource = baseLinkAB;
            EditorUtility.SetDirty(cwe);
            string radarWarn = (radarFrontSensor == null || radarRearSensor == null)
                ? " ⚠️ SingleRadarSensor 없음 (AutoCar radar 링크 버그)" : "";
            log.AppendLine($"  ✅ CollisionWarningEngine{radarWarn}"); ok++;
        }
        else { log.AppendLine("  ❌ CollisionWarningEngine not found"); warn++; }

        // ─── 6. TrafficLightDecisionEngine ────────────────────────────
        if (tlEngine != null)
        {
            tlEngine.vehicleMotionController = baseLinkVMC;
            tlEngine.egoTransform            = baseLinkGo.transform;
            EditorUtility.SetDirty(tlEngine);
            log.AppendLine("  ✅ TrafficLightDecisionEngine"); ok++;
        }
        else { log.AppendLine("  ❌ TrafficLightDecisionEngine not found"); warn++; }

        // ─── 7. AutoDriverRLAgent ─────────────────────────────────────
        if (rlAgent != null)
        {
            rlAgent.vehicleMotionController    = baseLinkVMC;
            rlAgent.progressRewardProvider     = progressReward;
            rlAgent.episodeEvaluator           = rlEval;
            rlAgent.collisionWarningEngine     = cwe;
            rlAgent.trafficLightDecisionEngine = tlEngine;
            rlAgent.vehicleCmdSubscriber       = cmdSubscriber;
            rlAgent.sensorFL                   = sFL;
            rlAgent.sensorFR                   = sFR;
            rlAgent.sensorFC                   = sFC;
            rlAgent.sensorRL                   = sRL;
            rlAgent.sensorRR                   = sRR;
            rlAgent.sensorRC                   = sRC;
            rlAgent.sensorSL                   = sSL;
            rlAgent.sensorSR                   = sSR;
            rlAgent.radarFront                 = radarFrontSensor;
            rlAgent.radarRear                  = radarRearSensor;
            rlAgent.followTargetTransform      = baseLinkGo.transform;
            rlAgent.episodeStartTransform      = rlStartPoseT;
            EditorUtility.SetDirty(rlAgent);
            log.AppendLine("  ✅ AutoDriverRLAgent"); ok++;
        }
        else { log.AppendLine("  ❌ AutoDriverRLAgent not found"); warn++; }

        // ─── 8. ProgressRewardProvider ───────────────────────────────
        if (progressReward != null)
        {
            progressReward.triggerTarget              = baseLinkGo;
            progressReward.vehicleMotionController    = baseLinkVMC;
            progressReward.collisionWarningEngine     = cwe;
            progressReward.trafficLightDecisionEngine = tlEngine;
            progressReward.episodeEvaluator           = rlEval;
            EditorUtility.SetDirty(progressReward);
            log.AppendLine("  ✅ ProgressRewardProvider"); ok++;
        }
        else { log.AppendLine("  ❌ ProgressRewardProvider not found"); warn++; }

        // ─── 9. RLEpisodeEvaluator ───────────────────────────────────
        if (rlEval != null)
        {
            rlEval.progressRewardProvider      = progressReward;
            rlEval.collisionWarningEngine      = cwe;
            rlEval.vehicleMotionController     = baseLinkVMC;
            rlEval.trafficLightDecisionEngine  = tlEngine;
            rlEval.regressionDrivingController = regression;
            rlEval.autoDriverRLAgent           = rlAgent;
            EditorUtility.SetDirty(rlEval);
            log.AppendLine("  ✅ RLEpisodeEvaluator"); ok++;
        }
        else { log.AppendLine("  ❌ RLEpisodeEvaluator not found"); warn++; }

        // ─── 10. ReinforcementObservationPublisher ───────────────────
        if (obsPublisher != null)
        {
            obsPublisher.progressRewardProvider = progressReward;
            obsPublisher.autoDriverRLAgent      = rlAgent;
            EditorUtility.SetDirty(obsPublisher);
            log.AppendLine("  ✅ ReinforcementObservationPublisher"); ok++;
        }
        else { log.AppendLine("  ❌ ReinforcementObservationPublisher not found"); warn++; }

        // ─── 11. ImuSensor ───────────────────────────────────────────
        if (imuSensor != null)
        {
            imuSensor.vehicleBody    = baseLinkAB;
            imuSensor.baseLinkObject = baseLinkGo;
            EditorUtility.SetDirty(imuSensor);
            log.AppendLine("  ✅ ImuSensor"); ok++;
        }
        else { log.AppendLine("  ❌ ImuSensor not found"); warn++; }

        // ─── 12. OdometryPublisher ───────────────────────────────────
        if (odomPublisher != null)
        {
            odomPublisher.vehicleBody    = baseLinkAB;
            odomPublisher.baseLinkObject = baseLinkGo;
            EditorUtility.SetDirty(odomPublisher);
            log.AppendLine("  ✅ OdometryPublisher"); ok++;
        }
        else { log.AppendLine("  ❌ OdometryPublisher not found"); warn++; }

        // ─── 13. UltrasonicSensorPublisher ───────────────────────────
        if (ultraPublisher != null)
        {
            ultraPublisher.sensorFL = sFL;
            ultraPublisher.sensorFR = sFR;
            ultraPublisher.sensorFC = sFC;
            ultraPublisher.sensorRL = sRL;
            ultraPublisher.sensorRR = sRR;
            ultraPublisher.sensorRC = sRC;
            ultraPublisher.sensorSL = sSL;
            ultraPublisher.sensorSR = sSR;
            EditorUtility.SetDirty(ultraPublisher);
            log.AppendLine("  ✅ UltrasonicSensorPublisher"); ok++;
        }
        else { log.AppendLine("  ❌ UltrasonicSensorPublisher not found"); warn++; }

        // ─── 14. RadarSensorPublisher ─────────────────────────────────
        if (radarPublisher != null)
        {
            radarPublisher.sensorFront = radarFrontSensor;
            radarPublisher.sensorRear  = radarRearSensor;
            EditorUtility.SetDirty(radarPublisher);
            string radarWarn2 = (radarFrontSensor == null || radarRearSensor == null)
                ? " ⚠️ SingleRadarSensor 없음" : "";
            log.AppendLine($"  ✅ RadarSensorPublisher{radarWarn2}"); ok++;
        }
        else { log.AppendLine("  ❌ RadarSensorPublisher not found"); warn++; }

        // ─── 15. VehicleViewProvider ──────────────────────────────────
        if (viewProvider != null)
        {
            viewProvider.vehicleId    = "Vehicle0";
            viewProvider.cameraPublisher = camPub;
            viewProvider.followTarget = baseLinkGo.transform;
            EditorUtility.SetDirty(viewProvider);
            log.AppendLine("  ✅ VehicleViewProvider (vehicleId='Vehicle0')"); ok++;
        }
        else { log.AppendLine("  ❌ VehicleViewProvider not found"); warn++; }

        // ─── 16. AMRVIew Transform 위치 리셋 ─────────────────────────
        if (amrViewGo != null)
        {
            amrViewGo.transform.localPosition = Vector3.zero;
            amrViewGo.transform.localRotation = Quaternion.identity;
            EditorUtility.SetDirty(amrViewGo);
            log.AppendLine("  ✅ AMRVIew.Transform: localPosition=(0,0,0) 리셋"); ok++;
        }

        // ─── 17. TrainTestModeSwitcher 설정 + rosTopics 자동 구성 ─────
        var switcher = autoCarRoot.GetComponent<TrainTestModeSwitcher>();
        if (switcher != null)
        {
            switcher.selectedMode          = TrainTestModeSwitcher.RuntimeMode.Train;
            switcher.autoApplyOnStart      = true;
            switcher.autoApplyInEditor     = true;
            switcher.autoFindReferences    = false;  // 명시적 참조이므로 자동탐색 비활성화
            switcher.includeInactiveObjects = true;
            switcher.trainRenderRate       = 5f;
            switcher.testRenderRate        = 10f;
            switcher.trainFixedDeltaTime   = 0.04f;
            switcher.testFixedDeltaTime    = 0f;
            switcher.autoDriverRLAgent         = rlAgent;
            switcher.regressionDrivingController = regression;
            switcher.trafficLightDecisionEngine  = tlEngine;
            switcher.wheelController             = baseLinkVMC;
            switcher.behaviorParameters          = behaviorParams;
            // AutoCar_Root_Train 하위에서 모든 ROS 컴포넌트를 자동 수집
            switcher.PopulateRosTopics();
            EditorUtility.SetDirty(switcher);
            log.AppendLine($"  ✅ TrainTestModeSwitcher (rosTopics: {switcher.rosTopics?.Length ?? 0}개)"); ok++;
        }
        else { log.AppendLine("  ❌ TrainTestModeSwitcher not found (루트에 추가 필요)"); warn++; }

        // ─── 18. AutoCar 하위 레이어 → 11 (RLVehicle) ───────────────
        int rlVehicleLayer = LayerMask.NameToLayer("RLVehicle");
        if (rlVehicleLayer >= 0)
        {
            SetLayerRecursively(autoCar, rlVehicleLayer);
            EditorUtility.SetDirty(autoCar);
            log.AppendLine($"  ✅ AutoCar 하위 전체: layer → {rlVehicleLayer} (RLVehicle)"); ok++;
        }
        else
        {
            log.AppendLine("  ⚠️ Layer 'RLVehicle'가 존재하지 않음 - layer 수동 설정 필요"); warn++;
        }

        EditorUtility.SetDirty(autoCarRoot);

        log.AppendLine($"\n총 {ok}개 완료 / {warn}개 경고");
        Debug.Log(log.ToString());
        Debug.Log("[AutoCarSetup] 완료! Ctrl+S로 씬을 저장하세요.");
    }

    static void SetLayerRecursively(GameObject go, int layer)
    {
        go.layer = layer;
        foreach (Transform child in go.transform)
            SetLayerRecursively(child.gameObject, layer);
    }

    static GameObject FindAutoCarRoot()
    {
        var exact = GameObject.Find("AutoCar_Root_Train");
        if (exact != null)
            return exact;

        Scene activeScene = SceneManager.GetActiveScene();
        if (!activeScene.IsValid())
            return null;

        foreach (var root in activeScene.GetRootGameObjects())
        {
            if (root != null && root.name.StartsWith("AutoCar_Root_Train"))
                return root;
        }

        return null;
    }
}
#endif
