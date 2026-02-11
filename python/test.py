from ultralytics import YOLO                                                        

model = YOLO('runs/detect/traffic_light_yolov8n/weights/best.pt')                                                                                                                             
results = model.predict(source=0, show=True, conf=0.5, stream=True)         
# conf 0.1 ~ 0.9 -> 높을수록 정확도 상승, 오탐 감소                                                                                                                   
for r in results:                                                                                                                                                                             
    for box in r.boxes:                                                                                                                                                                       
        cls = int(box.cls[0])
        conf = float(box.conf[0])
        name = r.names[cls]
        print(f'{name} {conf:.2f}')

        