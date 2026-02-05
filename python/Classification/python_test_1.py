# PyTorch 모델(.pth) 로드 및 추론 테스트.

import torch
from train_classification import SpeedAwareSingleViewNet, CLASS_NAMES

# 모델 로드
checkpoint = torch.load('checkpoints/driving_classifier.pth', map_location='cpu')
model = SpeedAwareSingleViewNet(backbone='resnet18', pretrained=False, dropout=0)
model.load_state_dict(checkpoint['model_state_dict'])
model.eval()

# 랜덤 테스트
import numpy as np
predictions = []
for i in range(10):
    front = torch.randn(1, 3, 66, 200)
    speed = torch.tensor([[0.6]])  # 3m/s / 5.0
    
    with torch.no_grad():
        logits = model(front, speed)
        pred = torch.argmax(logits, dim=1).item()
        probs = torch.softmax(logits, dim=1)[0]
        predictions.append(CLASS_NAMES[pred])
        print(f'Test {i+1}: {CLASS_NAMES[pred]:15s} | probs: {probs.numpy().round(3)}')

print(f'\nPredictions: {predictions}')