import cv2
import numpy as np
import os
import argparse
import pandas as pd
from tqdm import tqdm

def process_image(image_path):
    """
    Apply lane detection logic to generate a mask.
    Matches the logic in lane_detector.py
    """
    if not os.path.exists(image_path):
        print(f"Warning: Image not found {image_path}")
        return None

    # Load image
    img = cv2.imread(image_path)
    if img is None:
        return None
        
    height, width = img.shape[:2]
    
    # 1. ROI Crop (Top 60% removal - matching lane_detector.py)
    # Note: Training data usually keeps full size, but for UNet we might want to train on ROI or full.
    # If we train on full, the model learns to ignore the sky.
    # However, to match the "Teacher", let's process the full image but zero out the top.session_20260203_012834
    
    roi_top = int(height * 0.6)
    
    # Convert to Gray
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # ============================================================
    # Gaussian Blur (가우시안 블러)
    # ============================================================
    # 파라미터: (커널 크기, 커널 크기), 시그마
    # 
    # 커널 크기 (5, 5):
    #   - 작을수록 (3,3): 노이즈 제거 약함, 디테일 유지, 에지가 날카로움
    #   - 클수록 (7,7), (9,9): 노이즈 제거 강함, 에지가 부드러워짐 (뭉개짐)
    #   - 홀수만 사용 가능 (3, 5, 7, 9...)
    #
    # 시그마 (0):
    #   - 0이면 커널 크기에서 자동 계산
    #   - 값이 클수록 더 많이 블러됨
    # ============================================================
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # ============================================================
    # Canny Edge Detection (캐니 에지 검출)
    # ============================================================
    # 파라미터: (낮은 임계값, 높은 임계값)
    #
    # 낮은 임계값 (50):
    #   - 작을수록 (30): 약한 에지도 검출 → 노이즈 증가, 더 많은 선 검출
    #   - 클수록 (70, 100): 강한 에지만 검출 → 깔끔하지만 일부 에지 누락
    #
    # 높은 임계값 (150):
    #   - 작을수록 (100): 더 많은 에지 검출
    #   - 클수록 (200, 250): 매우 강한 에지만 검출, 선이 끊어질 수 있음
    #
    # 권장 비율: 높은 임계값 = 낮은 임계값 × 2~3
    # 예시:
    #   - (30, 90): 많은 에지 검출 (노이즈 포함 가능)
    #   - (50, 150): 균형잡힌 기본값 (현재 사용)
    #   - (100, 200): 강한 에지만 검출 (깔끔하지만 누락 가능)
    # ============================================================
    edges = cv2.Canny(blur, 50, 150)
    
    # Manual ROI Masking (Zero out top part)
    mask = np.zeros_like(edges)
    mask[roi_top:, :] = 255
    masked_edges = cv2.bitwise_and(edges, mask)
    
    # ============================================================
    # Dilation (팽창)
    # ============================================================
    # 파라미터: 커널 크기, iterations
    #   - 커널이 클수록, iterations가 많을수록 선이 두꺼워짐
    #   - 학습 시 더 굵은 선이 타겟으로 사용되어 학습 신호 증가
    # ============================================================
    kernel = np.ones((3,3), np.uint8)
    dilated = cv2.dilate(masked_edges, kernel, iterations=1)
    
    # Optional: Fill gaps if needed, but dilation is usually enough for "semantic segmentation" targets
    
    return dilated

def main():
    parser = argparse.ArgumentParser(description="Generate Segmentation Masks from Driving Log")
    parser.add_argument("--session", type=str, required=True, help="Path to session folder (e.g. TrainingData/session_...)")
    args = parser.parse_args()
    
    session_dir = args.session
    csv_path = os.path.join(session_dir, "driving_log.csv")
    
    if not os.path.exists(csv_path):
        print(f"Error: driving_log.csv not found in {session_dir}")
        return

    print(f"Propcessing session: {session_dir}")
    
    # Create masks directory
    masks_dir = os.path.join(session_dir, "generated_masks")
    os.makedirs(masks_dir, exist_ok=True)
    
    # Read CSV
    df = pd.read_csv(csv_path)
    
    # Add mask column if not exists
    new_rows = []
    
    print("Generating masks...")
    for index, row in tqdm(df.iterrows(), total=len(df)):
        # Handle different path formats (absolute vs relative)
        # Assuming relative path from session dir or absolute
        img_rel_path = row['front_image']
        
        # Check if path is absolute
        if os.path.isabs(img_rel_path):
            img_path = img_rel_path
        else:
            # Usually stored as 'front/frame_...jpg'
            img_path = os.path.join(session_dir, img_rel_path)
            
        # Generate Mask
        mask = process_image(img_path)
        
        if mask is not None:
            # Save Mask
            mask_filename = f"mask_{index:06d}.png"
            mask_path = os.path.join(masks_dir, mask_filename)
            cv2.imwrite(mask_path, mask)
            
            # Store relative path for CSV
            new_rows.append(f"generated_masks/{mask_filename}")
        else:
            new_rows.append("")

    # Update Dataframe
    df['mask_image'] = new_rows
    
    # Save new CSV
    new_csv_path = os.path.join(session_dir, "driving_log_masks.csv")
    df.to_csv(new_csv_path, index=False)
    
    print(f"Done! Saved updated log to {new_csv_path}")
    print(f"Masks saved to {masks_dir}")

if __name__ == "__main__":
    main()
