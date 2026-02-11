#!/usr/bin/env python3
"""
RPi5 NCNN vs PT 실시간 비교 진단
카메라를 신호등에 비추면서 실행하세요. 5초 동안 매 프레임 비교합니다.

실행: cd ~/ros2_unity_yolo/src/ros_yolo && python3 diag_ncnn.py
"""

from ultralytics import YOLO
import cv2
import time

# 모델 로드
print("Loading NCNN model...")
model_ncnn = YOLO('best_ncnn_model', task='detect')

print("Loading PT model...")
model_pt = YOLO('best.pt')

# 웹캠
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Webcam FAILED")
    exit(1)

print("\n=== 5초 동안 비교 시작 - 카메라를 신호등에 비춰주세요 ===\n")
print(f"{'Frame':>5} | {'PT result':>20} {'PT ms':>8} | {'NCNN result':>20} {'NCNN ms':>8}")
print("-" * 80)

start = time.time()
frame_count = 0

while time.time() - start < 5.0:
    ret, frame = cap.read()
    if not ret:
        continue

    frame_count += 1

    # PT 추론
    t0 = time.time()
    r_pt = model_pt.predict(frame, conf=0.1, imgsz=640, verbose=False)
    t_pt = (time.time() - t0) * 1000

    # NCNN 추론 (같은 프레임)
    t0 = time.time()
    r_ncnn = model_ncnn.predict(frame, conf=0.1, imgsz=640, verbose=False)
    t_ncnn = (time.time() - t0) * 1000

    # PT 결과
    pt_str = "none"
    if r_pt and len(r_pt[0].boxes) > 0:
        best = r_pt[0].boxes[0]
        name = r_pt[0].names[int(best.cls[0])]
        conf = float(best.conf[0])
        pt_str = f"{name} {conf:.2f}"

    # NCNN 결과
    ncnn_str = "none"
    if r_ncnn and len(r_ncnn[0].boxes) > 0:
        best = r_ncnn[0].boxes[0]
        name = r_ncnn[0].names[int(best.cls[0])]
        conf = float(best.conf[0])
        ncnn_str = f"{name} {conf:.2f}"

    print(f"{frame_count:>5} | {pt_str:>20} {t_pt:>7.0f}ms | {ncnn_str:>20} {t_ncnn:>7.0f}ms")

cap.release()

print(f"\n=== {frame_count} frames in 5 seconds ===")
