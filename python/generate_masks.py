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
    
    # 1. ROI Crop (Top 35% removal - matching lane_detector.py)
    # Note: Training data usually keeps full size, but for UNet we might want to train on ROI or full.
    # If we train on full, the model learns to ignore the sky.
    # However, to match the "Teacher", let's process the full image but zero out the top.session_20260203_012834
    
    roi_top = int(height * 0.35)
    
    # Convert to Gray
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Gaussian Blur
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Canny Edge Detection (Parameters from lane_detector.py defaults: 50, 150)
    edges = cv2.Canny(blur, 50, 150)
    
    # Manual ROI Masking (Zero out top part)
    mask = np.zeros_like(edges)
    mask[roi_top:, :] = 255
    masked_edges = cv2.bitwise_and(edges, mask)
    
    # Dilation (Thicken lines for better training signal)
    # A 3x3 kernel with 1 iteration makes lines slightly thicker
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
