# train_imitation.py
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
import pandas as pd
import numpy as np
from PIL import Image
import os
from pathlib import Path
from tqdm import tqdm
import albumentations as A
from albumentations.pytorch import ToTensorV2

from models.driving_model import DrivingNet, DualViewDrivingNet


class DualViewDataset(Dataset):
    """Unity에서 수집한 듀얼 뷰 주행 데이터셋 (Front + Top View)"""

    def __init__(
        self,
        data_dir: str,
        augment: bool = True
    ):
        self.data_dir = Path(data_dir)
        self.df = pd.read_csv(self.data_dir / "driving_log.csv")
        self.augment = augment

        # 기본 transform (정규화)
        self.base_transform = A.Compose([
            A.Normalize(mean=[0.485, 0.456, 0.406],
                       std=[0.229, 0.224, 0.225]),
            ToTensorV2()
        ])

        # 데이터 증강 transform (밝기/대비/노이즈 - 양쪽 이미지에 동일 적용)
        if augment:
            self.aug_transform = A.Compose([
                A.RandomBrightnessContrast(p=0.3),
                A.MotionBlur(blur_limit=3, p=0.1),
            ])
        else:
            self.aug_transform = None

    def __len__(self):
        return len(self.df)

    def __getitem__(self, idx):
        row = self.df.iloc[idx]

        # Front 이미지 로드
        front_path = self.data_dir / row['front_image']
        front_image = np.array(Image.open(front_path).convert('RGB'))

        # Top 이미지 로드
        top_path = self.data_dir / row['top_image']
        top_image = np.array(Image.open(top_path).convert('RGB'))

        # 타겟
        steering = float(row['steering'])
        throttle = float(row['throttle'])

        # 좌우 반전 augmentation (50% 확률) - 양쪽 이미지와 steering 동시 반전
        if self.augment and np.random.random() < 0.5:
            front_image = np.fliplr(front_image).copy()
            top_image = np.fliplr(top_image).copy()
            steering = -steering

        # 기타 augmentation 적용 (동일한 시드로 양쪽에 적용)
        if self.aug_transform is not None:
            # 동일한 증강을 양쪽 이미지에 적용하기 위해 replay 사용
            augmented_front = self.aug_transform(image=front_image)
            front_image = augmented_front['image']

            augmented_top = self.aug_transform(image=top_image)
            top_image = augmented_top['image']

        # 정규화 및 텐서 변환
        front_transformed = self.base_transform(image=front_image)
        front_image = front_transformed['image']

        top_transformed = self.base_transform(image=top_image)
        top_image = top_transformed['image']

        return front_image, top_image, torch.tensor([steering, throttle], dtype=torch.float32)


def train_dual_view(
    data_dirs: list,
    model_save_path: str = "checkpoints/driving_dual_bc.pth",
    epochs: int = 100,
    batch_size: int = 32,
    learning_rate: float = 1e-4,
    device: str = "cuda"
):
    """
    듀얼 뷰 모방학습 (Behavioral Cloning) - Front + Top View
    """

    # 데이터셋 로드
    datasets = [DualViewDataset(d, augment=True) for d in data_dirs]
    combined_dataset = torch.utils.data.ConcatDataset(datasets)

    # Train/Val 분할
    train_size = int(0.9 * len(combined_dataset))
    val_size = len(combined_dataset) - train_size
    train_dataset, val_dataset = torch.utils.data.random_split(
        combined_dataset, [train_size, val_size]
    )

    train_loader = DataLoader(
        train_dataset,
        batch_size=batch_size,
        shuffle=True,
        num_workers=4,
        pin_memory=True
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=4,
        pin_memory=True
    )

    print(f"Train: {len(train_dataset)}, Val: {len(val_dataset)}")
    print(f"Using Dual View Model (Front + Top)")

    # 듀얼 뷰 모델 초기화
    model = DualViewDrivingNet(
        backbone="resnet18",
        pretrained=True,
        dropout=0.5,
        freeze_backbone=False  # 파인튜닝
    ).to(device)

    # 파라미터 수 출력
    total_params = sum(p.numel() for p in model.parameters())
    trainable_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"Model parameters: {total_params:,} (trainable: {trainable_params:,})")

    # 손실 함수 & 옵티마이저
    criterion = nn.MSELoss()
    optimizer = optim.AdamW(model.parameters(), lr=learning_rate, weight_decay=1e-4)
    scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)

    # 학습 루프
    best_val_loss = float('inf')

    for epoch in range(epochs):
        # Training
        model.train()
        train_loss = 0.0

        pbar = tqdm(train_loader, desc=f"Epoch {epoch+1}/{epochs}")
        for front_imgs, top_imgs, targets in pbar:
            front_imgs = front_imgs.to(device)
            top_imgs = top_imgs.to(device)
            targets = targets.to(device)

            optimizer.zero_grad()

            steering, throttle = model(front_imgs, top_imgs)
            predictions = torch.stack([steering, throttle], dim=1)

            loss = criterion(predictions, targets)
            loss.backward()

            # Gradient clipping
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)

            optimizer.step()

            train_loss += loss.item()
            pbar.set_postfix({'loss': loss.item()})

        train_loss /= len(train_loader)

        # Validation
        model.eval()
        val_loss = 0.0

        with torch.no_grad():
            for front_imgs, top_imgs, targets in val_loader:
                front_imgs = front_imgs.to(device)
                top_imgs = top_imgs.to(device)
                targets = targets.to(device)

                steering, throttle = model(front_imgs, top_imgs)
                predictions = torch.stack([steering, throttle], dim=1)

                loss = criterion(predictions, targets)
                val_loss += loss.item()

        val_loss /= len(val_loader)
        scheduler.step()

        print(f"Epoch {epoch+1}: Train Loss = {train_loss:.4f}, Val Loss = {val_loss:.4f}")

        # Best 모델 저장
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save({
                'epoch': epoch,
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'val_loss': val_loss,
            }, model_save_path)
            print(f"  → Best model saved!")

    return model


if __name__ == "__main__":
    import glob

    # 모든 세션 폴더 자동 검색
    base_path = Path(__file__).parent.parent / "TrainingData"
    data_dirs = sorted(glob.glob(str(base_path / "session_*")))

    print(f"Found {len(data_dirs)} training sessions:")
    for d in data_dirs:
        print(f"  - {Path(d).name}")

    if len(data_dirs) == 0:
        print("Error: No training data found!")
        exit(1)

    # checkpoints 폴더 생성
    checkpoint_dir = Path(__file__).parent / "checkpoints"
    checkpoint_dir.mkdir(exist_ok=True)

    # 듀얼 뷰 (Front + Top) 모델 학습
    model = train_dual_view(
        data_dirs=data_dirs,
        model_save_path=str(checkpoint_dir / "driving_dual_bc.pth"),
        epochs=100,
        batch_size=16,  # 듀얼 뷰는 메모리 사용량이 높아 배치 크기 감소
        learning_rate=1e-4
    )
