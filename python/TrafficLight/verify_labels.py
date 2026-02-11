#!/usr/bin/env python3
"""
수집된 데이터셋의 YOLO 라벨을 이미지 위에 시각화하여 검증

실행: python python/TrafficLight/verify_labels.py
      python python/TrafficLight/verify_labels.py --dataset python/datasets/directlyDataset
"""
import cv2
import argparse
from pathlib import Path

CLASS_NAMES = {0: 'Green', 1: 'Red', 2: 'Yellow'}
CLASS_COLORS = {0: (0, 255, 0), 1: (0, 0, 255), 2: (0, 255, 255)}


def draw_yolo_labels(img, label_path):
    """YOLO 라벨을 이미지 위에 그리기"""
    h, w = img.shape[:2]
    count = 0

    if not label_path.exists():
        return img, 0

    for line in label_path.read_text().strip().split('\n'):
        if not line.strip():
            continue
        parts = line.strip().split()
        cls_id = int(parts[0])
        cx, cy, bw, bh = float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])

        # YOLO 정규화 → 픽셀
        x1 = int((cx - bw / 2) * w)
        y1 = int((cy - bh / 2) * h)
        x2 = int((cx + bw / 2) * w)
        y2 = int((cy + bh / 2) * h)

        color = CLASS_COLORS.get(cls_id, (128, 128, 128))
        name = CLASS_NAMES.get(cls_id, f'cls{cls_id}')

        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        cv2.putText(img, f'{name}', (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        count += 1

    return img, count


def main():
    parser = argparse.ArgumentParser(description='YOLO 라벨 검증 시각화')
    parser.add_argument('--dataset', type=str,
                        default='python/datasets/directlyDataset',
                        help='데이터셋 경로')
    args = parser.parse_args()

    dataset_path = Path(args.dataset)
    img_dir = dataset_path / 'images'
    lbl_dir = dataset_path / 'labels'

    if not img_dir.exists():
        print(f'이미지 폴더 없음: {img_dir}')
        return

    # 라벨이 있는 이미지만 필터링
    images = sorted(img_dir.glob('*.jpg'))
    labeled_images = []
    for img_path in images:
        lbl_path = lbl_dir / (img_path.stem + '.txt')
        if lbl_path.exists() and lbl_path.stat().st_size > 0:
            labeled_images.append(img_path)

    if not labeled_images:
        print('라벨이 있는 이미지가 없습니다.')
        return

    print(f'전체 이미지: {len(images)}장  |  라벨 있는 이미지: {len(labeled_images)}장')
    print('조작: [Space] 다음  |  [B] 이전  |  [Q] 종료')

    idx = 0
    while True:
        img_path = labeled_images[idx]
        lbl_path = lbl_dir / (img_path.stem + '.txt')

        img = cv2.imread(str(img_path))
        if img is None:
            idx = (idx + 1) % len(labeled_images)
            continue

        vis, count = draw_yolo_labels(img.copy(), lbl_path)

        # 정보 표시
        cv2.putText(vis, f'[{idx+1}/{len(labeled_images)}] {img_path.name}  Labels: {count}',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow('Label Verification', vis)
        key = cv2.waitKey(0) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('b'):
            idx = (idx - 1) % len(labeled_images)
        else:
            idx = (idx + 1) % len(labeled_images)

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
