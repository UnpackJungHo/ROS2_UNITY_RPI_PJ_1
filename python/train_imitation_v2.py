# train_imitation_v2.py
"""
개선된 모방학습 (Behavioral Cloning) 스크립트

개선 사항:
1. 세션별 Train/Val 분할 (데이터 누출 방지)
2. 클래스 가중치로 데이터 불균형 해결
3. Early Stopping 추가
4. Loss 그래프 저장
5. 더 강력한 데이터 증강
6. 학습률 스케줄러 개선 (ReduceLROnPlateau)
7. 백본 동결 옵션 (적은 데이터 대응)
"""

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader, WeightedRandomSampler
import pandas as pd
import numpy as np
from PIL import Image
import os
from pathlib import Path
from tqdm import tqdm
import albumentations as A
from albumentations.pytorch import ToTensorV2
import matplotlib.pyplot as plt
import json
from datetime import datetime
import warnings
warnings.filterwarnings('ignore')

from models.driving_model import DualViewDrivingNet


class DualViewDatasetV2(Dataset):
    """
    개선된 듀얼 뷰 주행 데이터셋
    - 더 강력한 데이터 증강
    - 샘플 가중치 계산 지원
    """

    def __init__(
        self,
        data_dirs: list,
        augment: bool = True,
        augment_strength: str = "medium"  # "light", "medium", "strong"
    ):
        self.augment = augment
        self.data = []

        # 모든 세션에서 데이터 로드
        for data_dir in data_dirs:
            data_dir = Path(data_dir)
            df = pd.read_csv(data_dir / "driving_log.csv")

            for idx, row in df.iterrows():
                self.data.append({
                    'front_path': str(data_dir / row['front_image']),
                    'top_path': str(data_dir / row['top_image']),
                    'steering': float(row['steering']),
                    'throttle': float(row['throttle']),
                    'speed': float(row['speed'])
                })

        # 기본 transform (정규화)
        self.base_transform = A.Compose([
            A.Normalize(mean=[0.485, 0.456, 0.406],
                       std=[0.229, 0.224, 0.225]),
            ToTensorV2()
        ])

        # 데이터 증강 설정 (강도별)
        if augment:
            if augment_strength == "light":
                self.aug_transform = A.Compose([
                    A.RandomBrightnessContrast(brightness_limit=0.1, contrast_limit=0.1, p=0.3),
                ])
            elif augment_strength == "medium":
                self.aug_transform = A.Compose([
                    A.RandomBrightnessContrast(brightness_limit=0.2, contrast_limit=0.2, p=0.5),
                    A.GaussNoise(var_limit=(5.0, 20.0), p=0.3),
                    A.MotionBlur(blur_limit=3, p=0.2),
                    A.HueSaturationValue(hue_shift_limit=10, sat_shift_limit=20, val_shift_limit=10, p=0.3),
                ])
            else:  # strong
                self.aug_transform = A.Compose([
                    A.RandomBrightnessContrast(brightness_limit=0.3, contrast_limit=0.3, p=0.6),
                    A.GaussNoise(var_limit=(10.0, 30.0), p=0.4),
                    A.MotionBlur(blur_limit=5, p=0.3),
                    A.HueSaturationValue(hue_shift_limit=15, sat_shift_limit=30, val_shift_limit=15, p=0.4),
                    A.RandomShadow(shadow_roi=(0, 0.5, 1, 1), p=0.2),
                    A.CoarseDropout(max_holes=5, max_height=8, max_width=8, p=0.2),
                ])
        else:
            self.aug_transform = None

        # 샘플 가중치 계산 (데이터 불균형 해결용)
        self._compute_sample_weights()

    def _compute_sample_weights(self):
        """steering 값 기반으로 샘플 가중치 계산 (D자 트랙 최적화)"""
        steerings = np.array([d['steering'] for d in self.data])

        # steering을 구간으로 분류 (7개 구간 - D자 트랙 맞춤)
        # 급좌 | 중좌 | 완만좌 | 직진 | 완만우 | 중우 | 급우
        bins = [-1.0, -0.35, -0.15, -0.03, 0.03, 0.15, 0.35, 1.0]
        bin_names = ['급좌', '중좌', '완만좌', '직진', '완만우', '중우', '급우']

        bin_indices = np.digitize(steerings, bins) - 1
        bin_indices = np.clip(bin_indices, 0, len(bins) - 2)

        # 각 구간별 빈도수 계산
        bin_counts = np.bincount(bin_indices, minlength=len(bins)-1)

        # 가중치 = 1 / 빈도수 (희귀할수록 높은 가중치)
        bin_weights = 1.0 / (bin_counts + 1)
        bin_weights = bin_weights / bin_weights.sum() * len(bins)  # 정규화

        # 각 샘플에 가중치 할당
        self.sample_weights = bin_weights[bin_indices]

        print(f"[Dataset] Steering 분포 (D자 트랙 기준):")
        for i, name in enumerate(bin_names):
            ratio = bin_counts[i] / len(steerings) * 100 if len(steerings) > 0 else 0
            print(f"  {name:6s}: {bin_counts[i]:5d} ({ratio:5.1f}%) - 가중치 {bin_weights[i]:.2f}")

    def get_sample_weights(self):
        """WeightedRandomSampler용 가중치 반환"""
        return self.sample_weights

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        item = self.data[idx]

        # 이미지 로드
        front_image = np.array(Image.open(item['front_path']).convert('RGB'))
        top_image = np.array(Image.open(item['top_path']).convert('RGB'))

        steering = item['steering']
        throttle = item['throttle']

        # 좌우 반전 augmentation (50% 확률)
        # steering != 0일 때 더 높은 확률로 반전 (데이터 불균형 완화)
        flip_prob = 0.5 if abs(steering) < 0.05 else 0.6

        if self.augment and np.random.random() < flip_prob:
            front_image = np.fliplr(front_image).copy()
            top_image = np.fliplr(top_image).copy()
            steering = -steering

        # 기타 augmentation 적용
        if self.aug_transform is not None:
            augmented = self.aug_transform(image=front_image)
            front_image = augmented['image']

            augmented = self.aug_transform(image=top_image)
            top_image = augmented['image']

        # 정규화 및 텐서 변환
        front_image = self.base_transform(image=front_image)['image']
        top_image = self.base_transform(image=top_image)['image']

        return front_image, top_image, torch.tensor([steering, throttle], dtype=torch.float32)


class EarlyStopping:
    """Early Stopping 클래스"""

    def __init__(self, patience: int = 20, min_delta: float = 0.0001, mode: str = 'min'):
        self.patience = patience
        self.min_delta = min_delta
        self.mode = mode
        self.counter = 0
        self.best_value = None
        self.early_stop = False

    def __call__(self, value):
        if self.best_value is None:
            self.best_value = value
            return False

        if self.mode == 'min':
            improved = value < self.best_value - self.min_delta
        else:
            improved = value > self.best_value + self.min_delta

        if improved:
            self.best_value = value
            self.counter = 0
        else:
            self.counter += 1
            if self.counter >= self.patience:
                self.early_stop = True

        return self.early_stop


def save_loss_graph(train_losses, val_losses, save_path, title="Training Progress"):
    """학습 과정 그래프 저장"""
    plt.figure(figsize=(12, 5))

    # Loss 그래프
    plt.subplot(1, 2, 1)
    epochs = range(1, len(train_losses) + 1)
    plt.plot(epochs, train_losses, 'b-', label='Train Loss', linewidth=2)
    plt.plot(epochs, val_losses, 'r-', label='Val Loss', linewidth=2)
    plt.xlabel('Epoch', fontsize=12)
    plt.ylabel('Loss (MSE)', fontsize=12)
    plt.title('Training & Validation Loss', fontsize=14)
    plt.legend(fontsize=11)
    plt.grid(True, alpha=0.3)

    # 갭 그래프 (과적합 지표)
    plt.subplot(1, 2, 2)
    gaps = [v - t for t, v in zip(train_losses, val_losses)]
    plt.plot(epochs, gaps, 'g-', linewidth=2)
    plt.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    plt.xlabel('Epoch', fontsize=12)
    plt.ylabel('Val - Train Loss', fontsize=12)
    plt.title('Generalization Gap (Overfitting Indicator)', fontsize=14)
    plt.grid(True, alpha=0.3)

    # 색상으로 과적합 정도 표시
    plt.fill_between(epochs, 0, gaps, where=[g > 0 for g in gaps],
                     alpha=0.3, color='red', label='Overfitting')
    plt.fill_between(epochs, 0, gaps, where=[g <= 0 for g in gaps],
                     alpha=0.3, color='green', label='Good')
    plt.legend(fontsize=10)

    plt.suptitle(title, fontsize=16, fontweight='bold')
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"[Graph] Saved to: {save_path}")


def train_dual_view_v2(
    data_dirs: list,
    val_ratio: float = 0.2,
    model_save_path: str = "checkpoints/driving_dual_bc_v2.pth",
    epochs: int = 100,
    batch_size: int = 32,
    learning_rate: float = 1e-4,
    weight_decay: float = 1e-4,
    dropout: float = 0.5,
    freeze_backbone: bool = False,
    use_weighted_sampling: bool = True,
    augment_strength: str = "medium",
    early_stopping_patience: int = 15,
    device: str = "cuda",
    graph_save_dir: str = None
):
    """
    개선된 듀얼 뷰 모방학습

    Args:
        data_dirs: 학습 데이터 디렉토리 리스트
        val_ratio: 검증 세션 비율 (세션 단위 분할)
        model_save_path: 모델 저장 경로
        epochs: 최대 에포크
        batch_size: 배치 크기
        learning_rate: 학습률
        weight_decay: L2 정규화 계수
        dropout: 드롭아웃 비율
        freeze_backbone: 백본 동결 여부 (데이터 적을 때 True 권장)
        use_weighted_sampling: 가중치 샘플링 사용 여부
        augment_strength: 데이터 증강 강도 ("light", "medium", "strong")
        early_stopping_patience: Early stopping patience
        device: 학습 장치
        graph_save_dir: 그래프 저장 디렉토리
    """

    print("=" * 60)
    print("Improved Dual View Behavioral Cloning Training")
    print("=" * 60)

    # 세션 단위로 Train/Val 분할 (데이터 누출 방지!)
    np.random.seed(42)
    n_sessions = len(data_dirs)
    n_val = max(1, int(n_sessions * val_ratio))

    indices = np.random.permutation(n_sessions)
    val_indices = indices[:n_val]
    train_indices = indices[n_val:]

    train_dirs = [data_dirs[i] for i in train_indices]
    val_dirs = [data_dirs[i] for i in val_indices]

    print(f"\n[Data Split] 세션 기반 분할 (데이터 누출 방지)")
    print(f"  Train sessions ({len(train_dirs)}): {[Path(d).name for d in train_dirs]}")
    print(f"  Val sessions ({len(val_dirs)}): {[Path(d).name for d in val_dirs]}")

    # 데이터셋 생성
    train_dataset = DualViewDatasetV2(train_dirs, augment=True, augment_strength=augment_strength)
    val_dataset = DualViewDatasetV2(val_dirs, augment=False)  # 검증은 증강 없음

    print(f"\n[Dataset] Train: {len(train_dataset)}, Val: {len(val_dataset)}")

    # 가중치 샘플링 (데이터 불균형 해결)
    if use_weighted_sampling:
        sample_weights = train_dataset.get_sample_weights()
        sampler = WeightedRandomSampler(
            weights=sample_weights,
            num_samples=len(train_dataset),
            replacement=True
        )
        train_loader = DataLoader(
            train_dataset,
            batch_size=batch_size,
            sampler=sampler,
            num_workers=4,
            pin_memory=True
        )
        print("[Sampler] Weighted Random Sampling 활성화")
    else:
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

    # 모델 초기화
    model = DualViewDrivingNet(
        backbone="resnet18",
        pretrained=True,
        dropout=dropout,
        freeze_backbone=freeze_backbone
    ).to(device)

    # 파라미터 수 출력
    total_params = sum(p.numel() for p in model.parameters())
    trainable_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"\n[Model] Total params: {total_params:,}")
    print(f"[Model] Trainable params: {trainable_params:,}")
    print(f"[Model] Backbone frozen: {freeze_backbone}")
    print(f"[Model] Dropout: {dropout}")

    # 손실 함수 - Steering에 더 높은 가중치
    steering_weight = 2.0  # steering 오차에 더 큰 패널티
    throttle_weight = 1.0

    def weighted_mse_loss(pred, target):
        steering_loss = ((pred[:, 0] - target[:, 0]) ** 2).mean() * steering_weight
        throttle_loss = ((pred[:, 1] - target[:, 1]) ** 2).mean() * throttle_weight
        return steering_loss + throttle_loss

    # 옵티마이저 & 스케줄러
    optimizer = optim.AdamW(
        filter(lambda p: p.requires_grad, model.parameters()),
        lr=learning_rate,
        weight_decay=weight_decay
    )

    # ReduceLROnPlateau: val_loss가 개선 안되면 학습률 감소
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(
        optimizer,
        mode='min',
        factor=0.5,
        patience=5,
        min_lr=1e-6
    )

    # Early Stopping
    early_stopping = EarlyStopping(patience=early_stopping_patience, min_delta=0.0001)

    # 학습 기록
    train_losses = []
    val_losses = []
    best_val_loss = float('inf')
    best_epoch = 0

    print(f"\n[Training] Starting... (Early stopping patience: {early_stopping_patience})")
    print("-" * 60)

    for epoch in range(epochs):
        # Training
        model.train()
        train_loss = 0.0
        train_steering_loss = 0.0
        train_throttle_loss = 0.0

        pbar = tqdm(train_loader, desc=f"Epoch {epoch+1}/{epochs}")
        for front_imgs, top_imgs, targets in pbar:
            front_imgs = front_imgs.to(device)
            top_imgs = top_imgs.to(device)
            targets = targets.to(device)

            optimizer.zero_grad()

            steering, throttle = model(front_imgs, top_imgs)
            predictions = torch.stack([steering, throttle], dim=1)

            loss = weighted_mse_loss(predictions, targets)
            loss.backward()

            # Gradient clipping
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)

            optimizer.step()

            train_loss += loss.item()

            # 개별 손실 추적
            with torch.no_grad():
                train_steering_loss += ((predictions[:, 0] - targets[:, 0]) ** 2).mean().item()
                train_throttle_loss += ((predictions[:, 1] - targets[:, 1]) ** 2).mean().item()

            pbar.set_postfix({'loss': f'{loss.item():.4f}'})

        train_loss /= len(train_loader)
        train_steering_loss /= len(train_loader)
        train_throttle_loss /= len(train_loader)

        # Validation
        model.eval()
        val_loss = 0.0
        val_steering_loss = 0.0
        val_throttle_loss = 0.0

        with torch.no_grad():
            for front_imgs, top_imgs, targets in val_loader:
                front_imgs = front_imgs.to(device)
                top_imgs = top_imgs.to(device)
                targets = targets.to(device)

                steering, throttle = model(front_imgs, top_imgs)
                predictions = torch.stack([steering, throttle], dim=1)

                loss = weighted_mse_loss(predictions, targets)
                val_loss += loss.item()

                val_steering_loss += ((predictions[:, 0] - targets[:, 0]) ** 2).mean().item()
                val_throttle_loss += ((predictions[:, 1] - targets[:, 1]) ** 2).mean().item()

        val_loss /= len(val_loader)
        val_steering_loss /= len(val_loader)
        val_throttle_loss /= len(val_loader)

        # 스케줄러 업데이트
        scheduler.step(val_loss)
        current_lr = optimizer.param_groups[0]['lr']

        # 기록
        train_losses.append(train_loss)
        val_losses.append(val_loss)

        # 로그 출력
        gap = val_loss - train_loss
        gap_indicator = "↑ OVERFITTING" if gap > 0.01 else "✓ OK"

        print(f"Epoch {epoch+1:3d}: "
              f"Train={train_loss:.4f} (S:{train_steering_loss:.4f}, T:{train_throttle_loss:.4f}) | "
              f"Val={val_loss:.4f} (S:{val_steering_loss:.4f}, T:{val_throttle_loss:.4f}) | "
              f"Gap={gap:+.4f} {gap_indicator} | "
              f"LR={current_lr:.2e}")

        # Best 모델 저장
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            best_epoch = epoch + 1

            torch.save({
                'epoch': epoch,
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'train_loss': train_loss,
                'val_loss': val_loss,
                'config': {
                    'dropout': dropout,
                    'freeze_backbone': freeze_backbone,
                    'augment_strength': augment_strength,
                    'use_weighted_sampling': use_weighted_sampling,
                }
            }, model_save_path)
            print(f"  → Best model saved! (val_loss: {val_loss:.4f})")

        # Early Stopping 체크
        if early_stopping(val_loss):
            print(f"\n[Early Stopping] No improvement for {early_stopping_patience} epochs. Stopping.")
            break

    print("=" * 60)
    print(f"Training Complete!")
    print(f"  Best epoch: {best_epoch}")
    print(f"  Best val_loss: {best_val_loss:.4f}")
    print(f"  Model saved: {model_save_path}")
    print("=" * 60)

    # 그래프 저장
    if graph_save_dir:
        os.makedirs(graph_save_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        graph_path = os.path.join(graph_save_dir, f"loss_graph_{timestamp}.png")
        save_loss_graph(
            train_losses, val_losses, graph_path,
            title=f"Training Progress (Best: Epoch {best_epoch}, Val Loss: {best_val_loss:.4f})"
        )

        # 학습 결과 JSON 저장
        result_path = os.path.join(graph_save_dir, f"training_result_{timestamp}.json")
        with open(result_path, 'w') as f:
            json.dump({
                'best_epoch': best_epoch,
                'best_val_loss': float(best_val_loss),
                'final_train_loss': float(train_losses[-1]),
                'final_val_loss': float(val_losses[-1]),
                'total_epochs': len(train_losses),
                'config': {
                    'batch_size': batch_size,
                    'learning_rate': learning_rate,
                    'dropout': dropout,
                    'freeze_backbone': freeze_backbone,
                    'augment_strength': augment_strength,
                    'use_weighted_sampling': use_weighted_sampling,
                    'early_stopping_patience': early_stopping_patience,
                },
                'train_sessions': [Path(d).name for d in train_dirs],
                'val_sessions': [Path(d).name for d in val_dirs],
                'train_losses': [float(l) for l in train_losses],
                'val_losses': [float(l) for l in val_losses],
            }, f, indent=2)
        print(f"[Result] Saved to: {result_path}")

    return model


if __name__ == "__main__":
    import glob

    # 데이터 경로
    base_path = Path(__file__).parent.parent / "TrainingData"
    data_dirs = sorted(glob.glob(str(base_path / "session_*")))

    print(f"Found {len(data_dirs)} training sessions:")
    for d in data_dirs:
        print(f"  - {Path(d).name}")

    if len(data_dirs) < 2:
        print("Error: Need at least 2 sessions for train/val split!")
        exit(1)

    # 디렉토리 생성
    checkpoint_dir = Path(__file__).parent / "checkpoints"
    checkpoint_dir.mkdir(exist_ok=True)

    graph_dir = base_path / "loss_graph"
    graph_dir.mkdir(exist_ok=True)

    # ============================================================
    # 학습 설정 가이드
    # ============================================================
    #
    # [데이터가 적을 때 권장 설정] (현재 상황: ~5,000개)
    #   - freeze_backbone=True: 백본 동결로 학습 파라미터 감소
    #   - dropout=0.5~0.7: 과적합 방지
    #   - augment_strength="strong": 강한 데이터 증강
    #   - use_weighted_sampling=True: 불균형 데이터 보정
    #   - batch_size=16~32: 작은 배치
    #   - learning_rate=1e-4~5e-4: 동결 시 더 높은 학습률 가능
    #
    # [데이터가 충분할 때] (10,000개 이상)
    #   - freeze_backbone=False: 전체 파인튜닝
    #   - dropout=0.3~0.5
    #   - augment_strength="medium"
    #   - learning_rate=1e-4
    #
    # ============================================================

    # ============================================================
    # 데이터 양에 따른 권장 설정
    # ============================================================
    #
    # [적은 데이터: ~10,000개]
    #   freeze_backbone=True, dropout=0.6, augment="strong", lr=5e-4, batch=16
    #
    # [중간 데이터: 10,000~30,000개]
    #   freeze_backbone=False, dropout=0.4, augment="medium", lr=1e-4, batch=32
    #
    # [충분한 데이터: 50,000개 이상] ← 현재 설정
    #   freeze_backbone=False, dropout=0.3, augment="light", lr=1e-4, batch=64
    #
    # ============================================================

    model = train_dual_view_v2(
        data_dirs=data_dirs,
        val_ratio=0.2,
        model_save_path=str(checkpoint_dir / "driving_dual_bc_v2.pth"),
        epochs=100,
        batch_size=32,
        learning_rate=1e-4,  # 낮은 학습률
        weight_decay=1e-4,  # 약한 정규화
        dropout=0.4,  # 중간 드롭아웃
        freeze_backbone=False,  # 백본 파인튜닝
        use_weighted_sampling=True,
        augment_strength="medium",  # 중간 증강
        early_stopping_patience=10,  # 빠른 조기 종료
        device="cuda" if torch.cuda.is_available() else "cpu",
        graph_save_dir=str(graph_dir)
    )
