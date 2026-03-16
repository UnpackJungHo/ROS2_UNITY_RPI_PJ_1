### [Topic 리스트]

- **입력/상태**
    - ***/camera/image_raw***
        
        **타입**: sensor_msgs/Image
        
        **발행자**: Unity 카메라 발행기 (CameraPublisher.cs)
        
        **기본 토픽명**: /camera/image_raw
        
        **기본 주기**: 30 Hz (publishRate=30f)
        
        **역할**: 신호등 검출과 정지선 검출에 사용되는 전방 카메라 이미지 입력 토픽
        
        기본 해상도는 640x480, frame_id는 camera_link이며, Python의 traffic_light_detector.py와 stop_line_detector.py가 이 토픽을 직접 구독합니다.
        
    - ***/camera/policy/image_raw***
        
        **타입**: sensor_msgs/Image
        
        **발행자**: Unity 정책 전용 카메라 발행기 (PolicyCameraPublisher.cs)
        
        **기본 토픽명**: /camera/policy/image_raw
        
        **기본 주기**: 20 Hz (publishRate=20f)
        
        **역할**: 외부 정책 추론용으로 별도 리사이즈된 전방 이미지 입력 토픽
        
        기본 해상도는 200x66, frame_id는 camera_link이며, Python policy_cmd_publisher.py가 base regression 입력으로 직접 사용합니다.
        
    - ***/odom***
        
        **타입**: nav_msgs/Odometry
        
        **발행자**: Unity 오도메트리 발행기 (OdometryPublisher.cs)
        
        **기본 토픽명**: /odom
        
        **기본 주기**: 20 Hz (publishRate=20f)
        
        **역할**: 차량의 pose와 twist를 함께 제공하는 상태 피드백 토픽
        
        Python policy_cmd_publisher.py는 이 토픽의 twist를 읽어 현재 속도를 계산하며, base regression의 speed 입력, RL observation의 speed 항, TTC 및 신호 정지 판단에 사용합니다.
        
        /odom 메시지와 함께 /tf  에 odom -> base_link 변환을 발행
        
        유니티 전용 맵 누적 파이프라인은 /odom 메시지를 직접 사용하지 않고, /tf 체인을 이용해 lidar_link 점군을 odom 좌표계로 변환해 RViz2에 누적 맵을 표시
        
    - ***/tf***
        
        **타입**: tf2_msgs/TFMessage
        
        **발행자**: Unity 오도메트리 발행기 (OdometryPublisher.cs) + 정적 TF 발행기 (StaticTfPublisher.cs)
        
        **기본 토픽명**: /tf
        
        **기본 주기**: 동적 TF는 20 Hz, 정적 TF는 1 Hz 재발행
        
        **역할**: odom-base_link 및 base_link-센서 프레임 간 변환을 제공하는 좌표계 토픽
        
        ROS 시각화, frame_id 해석, SLAM·센서융합·좌표변환의 기반 인프라로 사용
        
    - ***/imu***
        
        **타입**: sensor_msgs/Imu
        
        **발행자**: Unity IMU 발행기 (ImuPublisher.cs)
        
        **기본 토픽명**: /imu
        
        **기본 주기**: FixedUpdate 기준 publishEveryNthTick=1이면 약 50 Hz
        
        **역할**: 자세, 각속도, 선형가속도를 제공하는 관성 센서 토픽
        
        frame_id는 imu_link이며, 상태추정, SLAM, 센서융합 확장을 위한 기반 토픽
        
    - ***/velodyne_points***
        
        **타입**: sensor_msgs/PointCloud2
        
        **발행자**: Unity LiDAR 발행기 (LidarPublisher.cs)
        
        **기본 토픽명**: /velodyne_points
        
        **기본 주기**: 10 Hz (publishRate=10f)
        
        **역할**: 3D LiDAR 포인트클라우드를 제공하는 상태 센서 토픽
        
        frame_id는 lidar_link이며, x/y/z/intensity/ring/time 필드를 포함한 PointCloud2 형식으로 발행됩니다. SLAM, 맵핑, 장애물 인지 확장에 사용됩니다.
        
    - ***/clock***
        
        **타입**: rosgraph_msgs/Clock
        
        **발행자**: Unity 시뮬레이션 시간 발행기 (ClockPublisher.cs)
        
        **기본 토픽명**: /clock
        
        **기본 주기**: 100 Hz (publishRate=100f)
        
        **역할**: ROS2 시뮬레이션 시간을 동기화하기 위한 상태 토픽
        
        use_sim_time=true 환경에서 필수이며, Unity Time.time을 ROS Clock 메시지로 변환해 발행합니다. ROS 도구와 다른 노드들의 시간 기준 동기화에 사용됩니다.
        
- **센서**
    - ***/ultrasonic***
        
        **타입**: std_msgs/Float32MultiArray
        
        **발행자**: Unity 초음파 집계 발행기 (UltrasonicSensorPublisher.cs)
        
        **기본 토픽명**: /ultrasonic
        
        **기본 주기**: 20 Hz (publishRate=20f)
        
        **역할**: 6개 초음파 센서 값을 한 번에 묶어 발행하는 집계 토픽
        
        FL, FR, FC, RL, RR, RC 개별 거리와 전방 최소 거리, 후방 최소 거리, closest sensor, closest confidence, 전체 최소 거리를 12개 float 배열로 담습니다.
        
        현재 Python 정책의 raw 모드는 이 집계 토픽보다 개별 /ultrasonic/fl~rc raw 토픽을 직접 사용하며, /ultrasonic은 상태 요약과 모니터링용에 가깝습니다.
        
    - ***/ultrasonic/fl, /ultrasonic/fr, /ultrasonic/fc, /ultrasonic/rl, /ultrasonic/rr, /ultrasonic/rc***
        
        **타입**: std_msgs/Float32MultiArray
        
        **발행자**: Unity 개별 초음파 센서 (SingleUltrasonicSensor.cs)
        
        **기본 토픽명**: /ultrasonic/fl, /ultrasonic/fr, /ultrasonic/fc, /ultrasonic/rl, /ultrasonic/rr, /ultrasonic/rc
        
        **기본 주기**: 센서 scan interval 기준이며, 일반적으로 UltrasonicSensorPublisher와 동기화되어 20 Hz로 동작
        
        **역할**: 위치별 개별 raw 초음파 센서 입력 토픽
        
        각 메시지는 [distance_m, confidence, angle_deg] 3개 값으로 구성되며, 감지가 없으면 distance는 -1로 발행됩니다.
        
        Python 정책은 --collision-mode raw 또는 hybrid에서 이 토픽들을 직접 구독해 최소 거리, TTC, warning level, closest sensor 위치를 계산합니다.
        
    - ***/radar***
        
        **타입**: std_msgs/Float32MultiArray
        
        **발행자**: Unity 레이더 집계 발행기 (RadarSensorPublisher.cs)
        
        **기본 토픽명**: /radar
        
        **기본 주기**: 20 Hz (publishRate=20f)
        
        **역할**: 전방/후방 레이더의 대표 타깃을 한 번에 묶어 발행하는 집계 토픽
        
        전방 거리/각도/상대속도, 후방 거리/각도/상대속도, 가장 가까운 거리, closest sensor 위치를 8개 float 배열로 담습니다.
        
        현재 Python 정책은 /radar 집계 토픽보다 /radar/front, /radar/rear raw 토픽을 직접 사용하는 구조입니다.
        
    - ***/radar/front, /radar/rear***
        
        **타입**: std_msgs/Float32MultiArray
        
        **발행자**: Unity 개별 레이더 센서 (SingleRadarSensor.cs)
        
        **기본 토픽명**: /radar/front, /radar/rear
        
        **기본 주기**: 센서 scan interval 기준이며, 일반적으로 RadarSensorPublisher와 동기화되어 20 Hz로 동작
        
        **역할**: 전방/후방 레이더의 대표 raw 타깃 토픽
        
        각 메시지는 [distance, angle, radial_velocity, rcs, confidence, isGhost, isClutter] 7개 값으로 구성됩니다.
        
        Python 정책은 --collision-mode raw 또는 hybrid에서 이 토픽들을 직접 구독하며, 현재는 주로 front_dist, rear_dist 중심의 충돌 상태 계산에 사용합니다.
        
    - ***/radar/targets***
        
        **타입**: std_msgs/Float32MultiArray
        
        **발행자**: Unity 레이더 집계 발행기 (RadarSensorPublisher.cs)
        
        **기본 토픽명**: /radar/targets
        
        **기본 주기**: 20 Hz (publishRate=20f)
        
        **역할**: 전방/후방 레이더가 감지한 모든 타깃을 packed array로 발행하는 멀티 타깃 상세 토픽
        
        타깃 하나당 [sensor_id, distance, angle, radial_velocity, rcs, confidence, is_ghost, is_clutter] 8개 필드가 반복되는 구조입니다.
        
        현재 Python 정책은 직접 구독하지 않으며, 다중 객체 추적, 고스트/클러터 분석, 시각화·디버깅, 향후 확장용 상세 토픽에 가깝습니다.
        
- **안전/인지**
    - ***/collision_warning***
        
        **타입**: std_msgs/Float32MultiArray
        
        **발행자**: Unity 충돌 경고 발행기 (CollisionWarningRosBridge.cs)
        
        **기본 토픽명**: /collision_warning
        
        **기본 주기**: 20 Hz (publishRate=20f)
        
        **역할**: 초음파와 레이더를 융합해 현재 충돌 위험 상태를 요약해 발행하는 종합 안전 인지 토픽
        
        min_distance, TTC, warning_level, ego_speed, closing_speed, 초음파 6채널, 레이더 전/후 거리, 감지 소스, closest sensor, confidence를 하나의 17개 float 배열로 담습니다.
        
        Python 정책 노드는 collision_mode=topic 또는 hybrid fallback일 때 이 토픽을 구독해 충돌 위험도와 RL observation 일부를 구성합니다.
        
    - ***/obstacle_distance***
        
        **타입**: std_msgs/Float32
        
        **발행자**: Unity 충돌 경고 발행기 (CollisionWarningRosBridge.cs)
        
        **기본 토픽명**: /obstacle_distance
        
        **기본 주기**: 20 Hz (publishRate=20f)
        
        **역할**: 현재 가장 가까운 장애물 거리만 단일 scalar 값으로 내보내는 경량 안전 인지 토픽
        
        값은 currentMinDistance의 단순 요약본이며, 감지가 없으면 -1로 발행됩니다. 현재 코드상 뚜렷한 직접 구독처보다는 경량 모니터링·디버깅 용도로 제공됩니다.
        
    - ***/traffic_light/state***
        
        **타입**: std_msgs/String
        
        **발행자**: Python 신호등 인지 노드 (traffic_light_detector.py)
        
        **기본 토픽명**: /traffic_light/state
        
        **기본 주기**: simulation에서는 입력 /camera/image_raw 수신 주기 기준, real 모드에서는 약 10 Hz 처리 주기
        
        **역할**: 디바운싱된 최종 신호등 상태를 문자열로 발행하는 안정화 상태 토픽
        
        값은 red / yellow / green / none이며, Python 정책과 Unity TrafficLightStateSubscriber가 모두 구독해 현재 신호 상태를 저장하고 교차로 판단에 사용합니다.
        
    - ***/traffic_light/perception***
        
        **타입**: std_msgs/Float32MultiArray
        
        **발행자**: Python 신호등 인지 노드 (traffic_light_detector.py)
        
        **기본 토픽명**: /traffic_light/perception
        
        **기본 주기**: simulation에서는 입력 /camera/image_raw 수신 주기 기준, real 모드에서는 약 10 Hz 처리 주기
        
        **역할**: 신호등 인지의 신뢰도와 위치 근거를 수치로 전달하는 perception 토픽
        
        stable_state_id, stable_state_ratio, raw_state_id, raw_confidence, bbox_center_x_norm, bbox_center_y_norm, bbox_area_norm의 7개 값으로 구성됩니다.
        
        Python 정책은 이 토픽에서 stable_state_ratio, bbox_center_x, bbox_area를 사용해 해당 신호가 내 진행 방향에서 의미 있는지 gate 판단합니다.
        
    - ***/traffic_light/debug***
        
        **타입**: sensor_msgs/Image
        
        **발행자**: Python 신호등 인지 노드 (traffic_light_detector.py)
        
        **기본 토픽명**: /traffic_light/debug
        
        **기본 주기**: simulation에서는 입력 /camera/image_raw 수신 주기 기준, real 모드에서는 약 10 Hz 처리 주기
        
        **역할**: 신호등 검출 결과를 오버레이한 디버그 이미지 토픽
        
        YOLO 검출 박스, 상태, confidence 등이 그려진 시각화 결과로, 모델 튜닝과 오검출 분석에 사용됩니다.
        
    - ***/stop_line/state***
        
        **타입**: std_msgs/String
        
        **발행자**: Python 정지선 인지 노드 (stop_line_detector.py)
        
        **기본 토픽명**: /stop_line/state
        
        **기본 주기**: 입력 /camera/image_raw 수신 주기와 동일
        
        **역할**: 정지선의 안정화된 탐지 여부를 문자열로 발행하는 상태 토픽
        
        값은 detected / none이며, Unity StopLineStateSubscriber는 state와 perception을 모두 구독하지만 현재 Python 외부 정책 노드는 stop_line/state를 직접 사용하지 않습니다.
        
    - ***/stop_line/perception***
        
        **타입**: std_msgs/Float32MultiArray
        
        **발행자**: Python 정지선 인지 노드 (stop_line_detector.py)
        
        **기본 토픽명**: /stop_line/perception
        
        **기본 주기**: 입력 /camera/image_raw 수신 주기와 동일
        
        **역할**: 정지선 검출 결과의 신뢰도와 선 기하 정보를 수치로 전달하는 perception 토픽
        
        raw_detected, stable_detected, confidence, x1_norm, y1_norm, x2_norm, y2_norm, y_center_norm, distance_from_bottom_norm, stripe_count의 10개 값으로 구성됩니다.
        
        Python 정책은 stable_detected, confidence, distance_from_bottom_norm을 주로 사용해 정지선 거리와 신호 정지 판단을 계산합니다.
        
    - ***/stop_line/debug***
        
        **타입**: sensor_msgs/Image
        
        **발행자**: Python 정지선 인지 노드 (stop_line_detector.py)
        
        **기본 토픽명**: /stop_line/debug
        
        **기본 주기**: 입력 /camera/image_raw 수신 주기와 동일
        
        **역할**: ROI, 검출 stripe, fitted line을 오버레이한 정지선 디버그 이미지 토픽
        
        threshold, ROI, BEV 관련 파라미터를 조정하거나 정지선 오검출 원인을 분석할 때 사용됩니다.
        
- **RL 관측 브리지**
    
    **강화학습 학습 시 쓰던 관측값 일부를 Unity가 ROS 토픽으로 다시 꺼내 주는 것**
    
    - ***/rl/lateral_error***
        
        **타입**: std_msgs/Float32
        
        **발행자**: Unity ReinforcementObservationPublisher (ReinforcementObservationPublisher.cs)
        
        **기본 토픽명**: /rl/lateral_error
        
        **기본 주기**: 20 Hz
        
        **역할**: 경로 중심선에서 차량이 얼마나 옆으로 벗어났는지의 절대값을 외부 RL 정책에 전달하는 RL 관측 브리지 토픽
        
        항상 0 이상이고, 단위는 사실상 m
        
    - ***/rl/signed_lateral_error***
        
        **타입**: std_msgs/Float32
        
        **발행자**: Unity ReinforcementObservationPublisher (ReinforcementObservationPublisher.cs)
        
        **기본 토픽명**: /rl/signed_lateral_error
        
        **기본 주기**: 20 Hz
        
        **역할**: 경로 중심선 대비 횡방향 오차의 부호 포함 값을 외부 RL 정책에 전달하는 RL 관측 브리지 토픽
        
        같은 횡방향 오차이지만 좌우 부호를 포함하며, 코드 주석 기준으로 양수=오른쪽, 음수=왼쪽
        
    - ***/rl/heading_error_deg***
        
        **타입**: std_msgs/Float32
        
        **발행자**: Unity ReinforcementObservationPublisher (ReinforcementObservationPublisher.cs)
        
        **기본 토픽명**: /rl/heading_error_deg
        
        **기본 주기**: 20 Hz
        
        **역할**: 차량이 현재 경로 접선 방향과 얼마나 각도 차이가 나는지 외부 RL 정책에 전달하는 RL 관측 브리지 토픽
        
        단위는 deg이며, 차가 경로를 향해 정렬돼 있는지 또는 비스듬히 틀어져 있는지를 나타냄
        
    - ***/rl/progress_ratio***
        
        **타입**: std_msgs/Float32
        
        **발행자**: Unity ReinforcementObservationPublisher (ReinforcementObservationPublisher.cs)
        
        **기본 토픽명**: /rl/progress_ratio
        
        **기본 주기**: 20 Hz
        
        **역할**: 경로 전체 길이 대비 현재 얼마나 진행했는지의 비율을 외부 RL 정책에 전달하는 RL 관측 브리지 토픽
        
        사실상 currentPathS / totalPathLength 계산 결과이며, 현재 경로의 어느 구간에 있는지를 나타냄
        
    
    → 이 데이터들이 RL ONNX 모델에 넣어서 delta_steer, delta_accel 을 얻어 base 결과에 더하여 최종 출력을 함
    
- **제어/UI**
    - ***/vehicle/cmd***
        
        **타입**: geometry_msgs/Twist
        
        **발행자**: Python 외부 정책 노드 (policy_cmd_publisher.py)
        
        **기본 토픽명**: /vehicle/cmd
        
        **기본 주기:** 20 Hz (--control-hz=20.0) (policy_cmd_publisher.py)
        
        **역할**: base imitation + residual RL + safety override 결과를 Unity 차량 제어 명령으로 내보내는 최종 출력 토픽
        
        **/vehicle/cmd가 만들어지기 위한 핵심 선행 조건**
        
        - **/camera/policy/image_raw**
            
            base regression ONNX의 이미지 입력. 기본 200x66 RGB 전방 이미지
            
        - **/odom**
            
            속도 입력. base regression과 RL observation 구성에 모두 사용
            
        - **base ONNX와 RL ONNX 파일**
            
            둘 다 존재해야 노드가 실행
            
    - ***/policy/collision_warning***
        
        **타입**: std_msgs/Float32MultiArray
        
        **발행자**:  Python 외부 정책 노드 (policy_cmd_publisher.py)
        
        **기본 토픽명**: /policy/collision_warning
        
        **기본 주기**: /vehicle/cmd와 동일한 control loop 주기, 기본 20 Hz
        
        **역할**: Python 정책이 계산한 현재 충돌 위험 상태를 Unity UI에 전달하는 외부 경고토픽
        
        제어용 토픽이 아니라 UI/모니터링용 요약 토픽