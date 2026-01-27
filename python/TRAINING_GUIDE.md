# 자율주행 모방학습 가이드

## train_classification.py 사용법 가이드

 python train_classification.py --test = test 파라미터 적용

  1. 순서대로 학습하는가?                                                                                                                      
                                                                                                                                               
  아닙니다. 순서를 의도적으로 섞어서 학습합니다.                                                                                               
                                                                                                                                               
  코드에서 두 단계로 섞임:                                                                                                                     
                                                                                                                                               
  - 세션 단위: np.random.permutation으로 세션 순서를 랜덤 셔플 후 train/val 분할 (train_classification.py:285)                                 
  - 프레임 단위: WeightedRandomSampler가 매 epoch마다 프레임을 랜덤 추출 (train_classification.py:304)                                         
                                                                                                                                               
  순서대로 학습하면 모델이 "앞 프레임 다음에는 이 행동"이라는 시간 패턴을 외워버립니다. 셔플하면 각 프레임을 독립적으로 보고 "이 장면에서는    
  어떤 키를 눌러야 하나"를 학습하게 됩니다.                                                                                                    
                                                                                                                                               
  timestamp는 데이터 기록용일 뿐, 학습에 사용되지 않습니다.                                                                                    
                                                                                                                                               
  ---                                                                                                                                          
  2. CSV의 6개 컬럼 중 실제 학습에 사용되는 것                                                                                                 
  ┌─────────────────┬───────────┬───────────────────────────────────────┐                                                                      
  │      컬럼       │ 사용 여부 │                 역할                      │                                                                        
  ├─────────────────┼───────────┼───────────────────────────────────────┤                                                                      
  │ front_image     │ O         │ 모델 입력 1 (전방 카메라 이미지 경로)        │                                                                      
  ├─────────────────┼───────────┼───────────────────────────────────────┤                                                                      
  │ top_image       │ O         │ 모델 입력 2 (탑뷰 이미지 경로)             │                                                                      
  ├─────────────────┼───────────┼───────────────────────────────────────┤                                                                      
  │ key_action      │ O         │ 정답 레이블 (0~6)                       │                                                                      
  ├─────────────────┼───────────┼───────────────────────────────────────┤                                                                      
  │ key_action_name │ X         │ 사람이 읽기 위한 문자열, 무시됨             │                                                                      
  ├─────────────────┼───────────┼───────────────────────────────────────┤                                                                      
  │ speed           │ X         │ 로드만 하고 학습에 미사용                  │                                                                      
  ├─────────────────┼───────────┼───────────────────────────────────────┤                                                                      
  │ timestamp       │ X         │ 로드조차 안 함                           │                                                                      
  └─────────────────┴───────────┴───────────────────────────────────────┘                                                                      
  DrivingClassificationDataset.__getitem__ (train_classification.py:167~199)에서 실제로 반환하는 값:                                           
                                                                                                                                               
  return front_image, top_image, torch.tensor(label, dtype=torch.long)                                                                         
                                                                                                                                               
  3개만 사용합니다. 모델이 보는 것은 오직:                                                                                                     
  - 입력: 전방 이미지 + 탑뷰 이미지                                                                                                            
  - 정답: key_action (0~6 중 하나)                                                                                                             
                                                                                                                                               
  speed, timestamp, key_action_name은 학습에 관여하지 않습니다.                                                                                
                                                                                                                                               
  ---                                                                                                                                          
  3. 이미지에서 어떤 정보를 얻는가                                                                                                             
                                                                                                                                               
  모델이 "명시적으로 특정 정보를 추출하라"고 프로그래밍된 것은 아닙니다. ResNet18이 스스로 유용한 특징을 찾아냅니다.                           
                                                                                                                                               
  학습 과정:                                                                                                                                   
                                                                                                                                               
  전방 이미지 → ResNet18 → 512차원 특징벡터                                                                                                    
  탑뷰 이미지 → ResNet18 → 512차원 특징벡터                                                                                                    
                                ↓                                                                                                              
                      concat → 1024차원                                                                                                        
                                ↓                                                                                                              
                      classifier → 7개 클래스 확률                                                                                             
                                ↓                                                                                                              
                      정답(key_action)과 비교 → 오차(Loss) 계산                                                                                
                                ↓                                                                                                              
                      오차를 줄이는 방향으로 가중치 업데이트                                                                                   
                                                                                                                                               
  ResNet18은 ImageNet(동물, 사물 등 1000종 분류)으로 사전학습되어 있어서, 이미 "가장자리, 질감, 형태, 색상" 같은 기본적인 시각 특징을 추출할 수
   있습니다. 이 위에서 추가 학습하면:                                                                                                          
                                                                                                                                               
  - 전방 이미지에서: 도로 경계선, 커브 방향, 도로 폭, 차선 위치 등                                                                             
  - 탑뷰 이미지에서: 차량의 도로 내 위치, 주변 도로 형태, 진행 방향 등                                                                         
                                                                                                                                               
  이런 특징들을 알아서 조합해서 "이 장면 = FORWARD_RIGHT를 눌러야 할 상황"이라고 판단하게 됩니다.                                              
                                                                                                                                               
  다만 차선을 인식해서 중앙을 유지하라는 명시적 로직은 없습니다. "이렇게 생긴 장면에서 운전자가 W+D를 눌렀다"는 패턴을 통계적으로 학습할       
  뿐입니다.                                                                             

### 3.2 파라미터별 효과

| 파라미터 | 값 범위 | 효과 |
|----------|---------|------|
| `freeze_backbone` | True/False | True: 학습 파라미터 대폭 감소 (22M→1M), 과적합 방지 |
| `dropout` | 0.3~0.7 | 높을수록 과적합 방지, 너무 높으면 언더피팅 |
| `augment_strength` | light/medium/strong | 강할수록 데이터 다양성 증가 |
| `learning_rate` | 1e-5~5e-4 | 높으면 빠른 학습, 불안정 / 낮으면 안정, 느림 |
| `use_weighted_sampling` | True/False | True: 희귀 데이터(커브) 더 자주 샘플링 |
| `early_stopping_patience` | 10~20 | 낮으면 조기 종료, 높으면 오래 학습 |

  ┌─────────────────────────┬──────────────────┐                                                           
  │        파라미터         │       설명          │                                                           
  ├─────────────────────────┼──────────────────┤                                                           
  │ val_ratio               │ 검증 세션 비율      │                                                           
  ├─────────────────────────┼──────────────────┤                                                           
  │ epochs                  │ 최대 학습 횟수      │                                                           
  ├─────────────────────────┼──────────────────┤                                                           
  │ batch_size              │ 배치 크기          │                                                           
  ├─────────────────────────┼──────────────────┤                                                           
  │ learning_rate           │ 학습률            │                                                           
  ├─────────────────────────┼──────────────────┤                                                           
  │ weight_decay            │ L2 정규화         │                                                           
  ├─────────────────────────┼──────────────────┤                                                           
  │ dropout                 │ 드롭아웃 비율       │                                                           
  ├─────────────────────────┼──────────────────┤                                                           
  │ freeze_backbone         │ 백본 동결 여부      │                                                           
  ├─────────────────────────┼──────────────────┤                                                           
  │ use_weighted_sampling   │ 불균형 보정        │                                                           
  ├─────────────────────────┼──────────────────┤                                                           
  │ augment_strength        │ 데이터 증강 강도    │                                                           
  ├─────────────────────────┼──────────────────┤                                                           
  │ early_stopping_patience │ 조기 종료 인내      │                                                           
  └─────────────────────────┴──────────────────┘    

Rule-Based Replay Controller (DART 데이터 증강) 구현 계획                                                                                     
                                                                                                                                               
 개요                                                                                                                                          
                                                                                                                                               
 기존 세션의 액션 시퀀스를 변형(perturbation)하여 규칙 기반으로 재주행, 새로운 학습 데이터를 생성하는 시스템.                                  
                                                                                                                                               
 ---                                                                                                                                           
 파일 변경 목록                                                                                                                                
                                                                                                                                               
 1. 새로 생성: Assets/Scripts/Learning/RuleBasedReplayController.cs                                                                            
                                                                                                                                               
 Inspector 필드:                                                                                                                               
 - WheelTest wheelController - 차량 참조                                                                                                       
 - int sessionIndex - 세션 선택 (0-based)                                                                                                      
 - string selectedSessionName (읽기전용) - 선택된 세션명                                                                                       
 - int totalSessions (읽기전용) - 사용 가능한 세션 수                                                                                          
 - KeyCode replayToggleKey = K - 리플레이 토글 키                                                                                              
 - float replayInterval = 0.05f - 재생 간격 (원본과 동일 20FPS)                                                                                
 - float forwardThrottle = 0.8f - 전진 throttle                                                                                                
 - float steeringStrength = 1.0f - 조향 강도                                                                                                   
 - float perturbationProbability = 0.12f - 변형 확률 (12%)                                                                                     
 - int maxConsecutivePerturbed = 3 - 최대 연속 변형 프레임                                                                                     
                                                                                                                                               
 핵심 메서드:                                                                                                                                  
 - RefreshSessionList() - TrainingDataV2/session_* 스캔, 정렬                                                                                  
 - LoadSession(path) - CSV 파싱 (key_action 컬럼만 사용)                                                                                       
 - ApplyPerturbation() - DART 알고리즘으로 액션 변형                                                                                           
 - GetNeighborAction(action) - 이웃 액션 반환                                                                                                  
 - StartReplay() / StopReplay() - K키로 토글                                                                                                   
 - AdvanceFrame() - 프레임 순차 진행                                                                                                           
 - ClassToControl(action) - 액션 → steering/throttle 변환                                                                                      
 - OnGUI() - Y=350 위치에 리플레이 상태 표시                                                                                                   
                                                                                                                                               
 변형(Perturbation) 규칙:                                                                                                                      
 FORWARD       → {FORWARD_LEFT, FORWARD_RIGHT} 중 랜덤                                                                                         
 FORWARD_LEFT  → {FORWARD, LEFT} 중 랜덤                                                                                                       
 FORWARD_RIGHT → {FORWARD, RIGHT} 중 랜덤                                                                                                      
 LEFT          → FORWARD_LEFT                                                                                                                  
 RIGHT         → FORWARD_RIGHT                                                                                                                 
 BACKWARD      → 변형 없음                                                                                                                     
 NONE          → 변형 없음                                                                                                                     
 - 각 프레임 12% 확률로 변형                                                                                                                   
 - 3프레임 이상 연속 변형 금지                                                                                                                 
 - 로드 시점에 한번 적용 (실행 중 변경 없음)                                                                                                   
                                                                                                                                               
 공개 프로퍼티 (DrivingDataCollectorV2 연동용):                                                                                                
 - bool IsReplaying - 리플레이 중 여부                                                                                                         
 - KeyAction CurrentAction - 현재 실행 중인 액션                                                                                               
                                                                                                                                               
 동작 흐름:                                                                                                                                    
 1. Start() → 세션 목록 스캔                                                                                                                   
 2. K키 → LoadSession → ApplyPerturbation → 재생 시작                                                                                          
 3. replayInterval마다 AdvanceFrame → ClassToControl → ApplyControl                                                                            
 4. 마지막 프레임 도달 시 자동 정지                                                                                                            
                                                                                                                                               
 안전장치:                                                                                                                                     
 - 다른 컨트롤러가 externalControlEnabled 사용 중이면 시작 거부                                                                                
 - CSV 파싱 실패 시 에러 로그 후 리플레이 미시작                                                                                               
 - 리플레이 종료 시 steering=0, throttle=0, externalControlEnabled=false                                                                       
                                                                                                                                               
 ---                                                                                                                                           
 2. 수정: Assets/Scripts/Learning/DrivingDataCollectorV2.cs                                                                                    
                                                                                                                                               
 3곳만 수정 (기존 로직 변경 없음):                                                                                                             
                                                                                                                                               
 변경 1 - 필드 추가 (line 53 뒤):                                                                                                              
 [Header("External Action Source (Optional)")]                                                                                                 
 public RuleBasedReplayController replayController;                                                                                            
                                                                                                                                               
 변경 2 - Update() line 188:                                                                                                                   
 // 기존: currentAction = GetCurrentKeyAction();                                                                                               
 currentAction = (replayController != null && replayController.IsReplaying)                                                                    
     ? replayController.CurrentAction                                                                                                          
     : GetCurrentKeyAction();                                                                                                                  
                                                                                                                                               
 변경 3 - CaptureFrame() line 272:                                                                                                             
 // 기존: KeyAction action = GetCurrentKeyAction();                                                                                            
 KeyAction action = (replayController != null && replayController.IsReplaying)                                                                 
     ? replayController.CurrentAction                                                                                                          
     : GetCurrentKeyAction();                                                                                                                  
                                                                                                                                               
 ---                                                                                                                                           
 사용 워크플로우                                                                                                                               
                                                                                                                                               
 1. Inspector에서 sessionIndex 설정 (0~N)                                                                                                      
 2. 차량을 트랙 시작점에 배치                                                                                                                  
 3. R키 → 녹화 시작 (DrivingDataCollectorV2)                                                                                                   
 4. K키 → 변형된 액션으로 자동 주행 시작                                                                                                       
 5. 주행 종료 시 자동 정지 (또는 K키로 수동 정지)                                                                                              
 6. R키 → 녹화 중지, T키 → 저장                                                                                                                
 7. 결과 확인 후 데이터 채택/삭제 판단                                                                                                         
                                                                                                                                               
 ---                                                                                                                                           
 검증 방법                                                                                                                                     
                                                                                                                                               
 1. Unity에서 RuleBasedReplayController 컴포넌트 추가                                                                                          
 2. sessionIndex=0 설정, K키로 리플레이 시작                                                                                                   
 3. 차량이 기존 세션과 유사하지만 약간 다르게 주행하는지 확인                                                                                  
 4. GUI에서 [P] 표시로 변형된 프레임 확인                                                                                                      
 5. R→K→(주행완료)→R→T로 저장, CSV에 액션이 올바르게 기록되었는지 확인               