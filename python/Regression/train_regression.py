# train_regression.py
"""
Speed-Aware 회귀(Regression) 기반 자율주행 학습 스크립트 (Single View)

구조:
  입력: front_image (3, 66, 200) + speed (1,)
  출력: steering [-1, 1], throttle [0, 1]

회귀 타겟 선정 근거:
  - steering_input: 연속값 [-1, 1], std=0.45, 분산 충분 → tanh
  - throttle_input: 연속값 [0, 1], 대부분 1.0이지만 감속 구간 존재 → sigmoid
  - brake_input / slip_ratio: 전부 0.0 → 학습 불가, 제외
  - steering_angle: steering_input과 상관계수 0.976 → 중복, 제외
"""

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import torchvision.models as models
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


class SpeedAwareRegressionNet(nn.Module):
    """
    Speed-Aware 단일 뷰 회귀 모델
    Front View + Speed → steering [-1,1], throttle [0,1]
    """

    def __init__(self, backbone="resnet18", pretrained=True, dropout=0.5):
        super().__init__()

        if backbone == "resnet18":
            self.front_encoder = models.resnet18(
                weights=models.ResNet18_Weights.IMAGENET1K_V1 if pretrained else None
            )
            feature_dim = 512
            self.front_encoder.fc = nn.Identity()
        elif backbone == "mobilenet_v3_small":
            self.front_encoder = models.mobilenet_v3_small(
                weights=models.MobileNet_V3_Small_Weights.IMAGENET1K_V1 if pretrained else None
            )
            feature_dim = 576
            self.front_encoder.classifier = nn.Identity()
        else:
            raise ValueError(f"Unknown backbone: {backbone}")

        self.backbone = backbone
        self.feature_dim = feature_dim

        # 속도 임베딩 (1차원 → 64차원)
        self.speed_embedding = nn.Sequential(
            nn.Linear(1, 32),
            nn.ReLU(),
            nn.Linear(32, 64),
            nn.ReLU()
        )

        # 회귀 헤드 (front_features + speed_embedding → steering, throttle)
        combined_dim = feature_dim + 64
        self.regressor = nn.Sequential(
            nn.Linear(combined_dim, 256),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(128, 2)  # [steering_raw, throttle_raw]
        )

    def forward(self, front_img, speed):
        """
        Args:
            front_img: (B, 3, H, W)
            speed: (B, 1)
        Returns:
            output: (B, 2) → [:, 0]=steering [-1,1], [:, 1]=throttle [0,1]
        """
        front_features = self.front_encoder(front_img)

        if self.backbone == "mobilenet_v3_small":
            if len(front_features.shape) == 4:
                front_features = front_features.mean([2, 3])

        speed_features = self.speed_embedding(speed)
        combined = torch.cat([front_features, speed_features], dim=1)
        raw = self.regressor(combined)

        steering = torch.tanh(raw[:, 0:1])         # [-1, 1]
        throttle = torch.sigmoid(raw[:, 1:2])       # [0, 1]

        return torch.cat([steering, throttle], dim=1)


class RegressionDrivingDataset(Dataset):
    """
    회귀 기반 주행 데이터셋 (Single View)

    타겟: steering_input [-1, 1], throttle_input [0, 1]
    """

    def __init__(self, data_dirs: list, augment: bool = True,
                 speed_normalize: float = 5.0, image_type: str = "front_1"):
        self.augment = augment
        self.speed_normalize = speed_normalize
        self.image_type = image_type
        self.data = []
        self.intervention_count = 0

        print(f"[Dataset] Image Source: {image_type}")

        for data_dir in data_dirs:
            data_dir = Path(data_dir)
            csv_path = data_dir / "driving_log.csv"

            if not csv_path.exists():
                print(f"[Warning] {csv_path} not found, skipping...")
                continue

            df = pd.read_csv(csv_path)

            required_cols = {'front_image', 'steering_input', 'throttle_input', 'speed'}
            if not required_cols.issubset(df.columns):
                print(f"[Warning] {csv_path} missing columns: {required_cols - set(df.columns)}, skipping...")
                continue

            for _, row in df.iterrows():
                original_path = row['front_image']

                if image_type == "front_1":
                    final_path = original_path
                elif image_type == "front_2":
                    final_path = original_path.replace("front_1", "front_2")
                elif image_type == "front_3":
                    final_path = original_path.replace("front_1", "front_3").replace(".jpg", ".png")
                elif image_type == "front_4":
                    final_path = original_path.replace("front_1", "front_4").replace(".jpg", ".png")
                else:
                    final_path = original_path

                self.data.append({
                    'front_path': str(data_dir / final_path),
                    'steering': float(row['steering_input']),
                    'throttle': float(row['throttle_input']),
                    'speed': float(row['speed']),
                    'intervention': float(row['intervention']) if 'intervention' in df.columns else 0.0
                })

        if len(self.data) == 0:
            raise ValueError("No valid data found!")

        self.intervention_count = int(sum(1 for d in self.data if d['intervention'] > 0.5))

        # 정규화 transform (ImageNet)
        self.normalize = A.Compose([
            A.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            ToTensorV2()
        ])

        if augment:
            self.aug_transform = A.Compose([
                A.RandomBrightnessContrast(brightness_limit=0.2, contrast_limit=0.2, p=0.5),
                A.GaussNoise(var_limit=(5.0, 20.0), p=0.3),
                A.MotionBlur(blur_limit=3, p=0.2),
            ])
        else:
            self.aug_transform = None

        # 타겟 통계 출력
        steerings = [d['steering'] for d in self.data]
        throttles = [d['throttle'] for d in self.data]
        speeds = [d['speed'] for d in self.data]
        intervention_ratio = (self.intervention_count / len(self.data)) * 100.0
        print(f"[Dataset] Loaded {len(self.data)} samples")
        print(f"[Dataset] steering: min={min(steerings):.3f}, max={max(steerings):.3f}, "
              f"mean={np.mean(steerings):.3f}, std={np.std(steerings):.3f}")
        print(f"[Dataset] throttle: min={min(throttles):.3f}, max={max(throttles):.3f}, "
              f"mean={np.mean(throttles):.3f}, std={np.std(throttles):.3f}")
        print(f"[Dataset] speed:    min={min(speeds):.3f}, max={max(speeds):.3f}, "
              f"mean={np.mean(speeds):.3f}")
        print(f"[Dataset] intervention: {self.intervention_count}/{len(self.data)} ({intervention_ratio:.2f}%)")

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        item = self.data[idx]

        try:
            pil_image = Image.open(item['front_path']).convert('RGB')
            front_image = np.array(pil_image)
        except Exception as e:
            print(f"Error loading image: {item['front_path']} - {e}")
            front_image = np.zeros((66, 200, 3), dtype=np.uint8)

        steering = item['steering']
        throttle = item['throttle']
        speed = item['speed'] / self.speed_normalize
        intervention = item['intervention']

        # 좌우 반전 augmentation (50% 확률)
        if self.augment and np.random.random() < 0.5:
            front_image = np.fliplr(front_image).copy()
            steering = -steering  # 좌우 반전 시 조향 부호 반전

        if self.aug_transform is not None:
            front_image = self.aug_transform(image=front_image)['image']

        front_image = self.normalize(image=front_image)['image']

        target = torch.tensor([steering, throttle], dtype=torch.float32)
        speed_tensor = torch.tensor([speed], dtype=torch.float32)
        intervention_tensor = torch.tensor(intervention, dtype=torch.float32)

        return front_image, speed_tensor, target, intervention_tensor


class EarlyStopping:
    def __init__(self, patience=10, min_delta=0.0001):
        self.patience = patience
        self.min_delta = min_delta
        self.counter = 0
        self.best_value = None
        self.early_stop = False

    def __call__(self, value):
        if self.best_value is None:
            self.best_value = value
            return False

        if value < self.best_value - self.min_delta:
            self.best_value = value
            self.counter = 0
        else:
            self.counter += 1
            if self.counter >= self.patience:
                self.early_stop = True

        return self.early_stop


def save_training_graph(train_losses, val_losses, train_steer_maes, val_steer_maes,
                        save_path, title=""):
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    epochs = range(1, len(train_losses) + 1)

    axes[0].plot(epochs, train_losses, 'b-', label='Train Loss', linewidth=2)
    axes[0].plot(epochs, val_losses, 'r-', label='Val Loss', linewidth=2)
    axes[0].set_xlabel('Epoch')
    axes[0].set_ylabel('Loss (Weighted MSE)')
    axes[0].set_title('Training & Validation Loss')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(epochs, train_steer_maes, 'b-', label='Train Steering MAE', linewidth=2)
    axes[1].plot(epochs, val_steer_maes, 'r-', label='Val Steering MAE', linewidth=2)
    axes[1].set_xlabel('Epoch')
    axes[1].set_ylabel('Steering MAE')
    axes[1].set_title('Steering Mean Absolute Error')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.suptitle(title, fontsize=14, fontweight='bold')
    plt.tight_layout()
    try:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[Graph] Saved to: {save_path}")
    except Exception as e:
        print(f"[Graph] Error saving graph: {e}")
    finally:
        plt.close()


def train_regression(
    data_dirs: list,
    val_ratio: float = 0.2,
    model_save_path: str = "checkpoints/driving_regression.pth",
    epochs: int = 50,
    batch_size: int = 32,
    learning_rate: float = 3e-4,
    weight_decay: float = 1e-4,
    dropout: float = 0.5,
    backbone: str = "resnet18",
    early_stopping_patience: int = 10,
    speed_normalize: float = 5.0,
    freeze_backbone: bool = False,
    steering_loss_weight: float = 5.0,
    intervention_loss_weight: float = 2.0,
    use_intervention_weighting: bool = True,
    device: str = "cuda",
    graph_save_dir: str = None,
    image_type: str = "front_1"
):
    """
    Speed-Aware 회귀 학습 (Single View)

    Args:
        steering_loss_weight: steering loss 가중치.
            throttle은 대부분 1.0이라 쉽게 수렴하므로
            steering에 더 높은 가중치를 부여 (기본 5.0)
    """
    print("=" * 60)
    print("Speed-Aware Single View Regression Training")
    print(f"Image Source: {image_type}")
    print(f"Steering loss weight: {steering_loss_weight}")
    print(f"Intervention weighting: {'ON' if use_intervention_weighting else 'OFF'} "
          f"(intervention loss weight={intervention_loss_weight:.2f})")
    print("=" * 60)

    # 세션 단위 Train/Val 분할
    np.random.seed(42)
    n_sessions = len(data_dirs)
    n_val = max(1, int(n_sessions * val_ratio))

    indices = np.random.permutation(n_sessions)
    val_indices = indices[:n_val]
    train_indices = indices[n_val:]

    train_dirs = [data_dirs[i] for i in train_indices]
    val_dirs = [data_dirs[i] for i in val_indices]

    print(f"\n[Data Split] Train: {len(train_dirs)} sessions, Val: {len(val_dirs)} sessions")

    train_dataset = RegressionDrivingDataset(
        train_dirs, augment=True, speed_normalize=speed_normalize, image_type=image_type
    )
    val_dataset = RegressionDrivingDataset(
        val_dirs, augment=False, speed_normalize=speed_normalize, image_type=image_type
    )

    print(f"\n[Dataset] Train: {len(train_dataset)}, Val: {len(val_dataset)}")
    train_intervention_ratio = (train_dataset.intervention_count / len(train_dataset)) * 100.0
    val_intervention_ratio = (val_dataset.intervention_count / len(val_dataset)) * 100.0
    print(f"[Dataset] Train intervention ratio: {train_intervention_ratio:.2f}%")
    print(f"[Dataset] Val intervention ratio:   {val_intervention_ratio:.2f}%")

    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True,
                              num_workers=4, pin_memory=True)
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False,
                            num_workers=4, pin_memory=True)

    # 모델
    model = SpeedAwareRegressionNet(backbone=backbone, pretrained=True, dropout=dropout).to(device)

    if freeze_backbone:
        for param in model.front_encoder.parameters():
            param.requires_grad = False
        print(f"\n[Model] Backbone FROZEN")

    total_params = sum(p.numel() for p in model.parameters())
    trainable_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"\n[Model] Backbone: {backbone}")
    print(f"[Model] Total params: {total_params:,}")
    print(f"[Model] Trainable params: {trainable_params:,}")

    # 손실 함수: steering 가중 + (선택) intervention 샘플 가중
    intervention_loss_weight = max(1.0, float(intervention_loss_weight))

    optimizer = optim.AdamW(model.parameters(), lr=learning_rate, weight_decay=weight_decay)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.5,
                                                      patience=5, min_lr=1e-6)
    early_stopping = EarlyStopping(patience=early_stopping_patience)

    train_losses, val_losses = [], []
    train_steer_maes, val_steer_maes = [], []
    best_val_loss = float('inf')
    best_epoch = 0

    print(f"\n[Training] Starting...")
    print("-" * 60)

    for epoch in range(epochs):
        # === Training ===
        model.train()
        train_loss_num = 0.0
        train_loss_den = 0.0
        train_steer_ae_sum = 0.0
        train_total = 0

        pbar = tqdm(train_loader, desc=f"Epoch {epoch+1}/{epochs}")
        for front_imgs, speeds, targets, interventions in pbar:
            front_imgs = front_imgs.to(device)
            speeds = speeds.to(device)
            targets = targets.to(device)
            interventions = interventions.to(device)

            optimizer.zero_grad()
            preds = model(front_imgs, speeds)

            steer_sq = (preds[:, 0] - targets[:, 0]).pow(2)
            throttle_sq = (preds[:, 1] - targets[:, 1]).pow(2)
            per_sample_loss = steering_loss_weight * steer_sq + throttle_sq

            if use_intervention_weighting:
                sample_weights = 1.0 + (intervention_loss_weight - 1.0) * interventions
            else:
                sample_weights = torch.ones_like(interventions)

            weighted_loss_sum = (per_sample_loss * sample_weights).sum()
            weight_sum = sample_weights.sum().clamp_min(1e-6)
            loss = weighted_loss_sum / weight_sum

            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()

            batch_size_actual = targets.size(0)
            train_loss_num += weighted_loss_sum.item()
            train_loss_den += weight_sum.item()
            train_steer_ae_sum += (preds[:, 0] - targets[:, 0]).abs().sum().item()
            train_total += batch_size_actual

            pbar.set_postfix({
                'loss': f'{loss.item():.4f}',
                'steer_mae': f'{(preds[:, 0] - targets[:, 0]).abs().mean().item():.4f}'
            })

        train_loss = train_loss_num / max(train_loss_den, 1e-6)
        train_steer_mae = train_steer_ae_sum / train_total

        # === Validation ===
        model.eval()
        val_loss_num = 0.0
        val_loss_den = 0.0
        val_steer_ae_sum = 0.0
        val_total = 0

        with torch.no_grad():
            for front_imgs, speeds, targets, interventions in val_loader:
                front_imgs = front_imgs.to(device)
                speeds = speeds.to(device)
                targets = targets.to(device)
                interventions = interventions.to(device)

                preds = model(front_imgs, speeds)

                steer_sq = (preds[:, 0] - targets[:, 0]).pow(2)
                throttle_sq = (preds[:, 1] - targets[:, 1]).pow(2)
                per_sample_loss = steering_loss_weight * steer_sq + throttle_sq

                if use_intervention_weighting:
                    sample_weights = 1.0 + (intervention_loss_weight - 1.0) * interventions
                else:
                    sample_weights = torch.ones_like(interventions)

                weighted_loss_sum = (per_sample_loss * sample_weights).sum()
                weight_sum = sample_weights.sum().clamp_min(1e-6)

                batch_size_actual = targets.size(0)
                val_loss_num += weighted_loss_sum.item()
                val_loss_den += weight_sum.item()
                val_steer_ae_sum += (preds[:, 0] - targets[:, 0]).abs().sum().item()
                val_total += batch_size_actual

        val_loss = val_loss_num / max(val_loss_den, 1e-6)
        val_steer_mae = val_steer_ae_sum / val_total

        scheduler.step(val_loss)
        current_lr = optimizer.param_groups[0]['lr']

        train_losses.append(train_loss)
        val_losses.append(val_loss)
        train_steer_maes.append(train_steer_mae)
        val_steer_maes.append(val_steer_mae)

        print(f"Epoch {epoch+1:3d}: Train Loss={train_loss:.4f}, Steer MAE={train_steer_mae:.4f} | "
              f"Val Loss={val_loss:.4f}, Steer MAE={val_steer_mae:.4f} | LR={current_lr:.2e}")

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            best_val_steer_mae = val_steer_mae
            best_epoch = epoch + 1

            torch.save({
                'epoch': epoch,
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'val_loss': val_loss,
                'val_steer_mae': val_steer_mae,
                'backbone': backbone,
                'speed_normalize': speed_normalize,
                'steering_loss_weight': steering_loss_weight,
                'intervention_loss_weight': intervention_loss_weight,
                'use_intervention_weighting': use_intervention_weighting,
                'image_type': image_type,
                'model_version': 'speed_aware_regression_v1',
            }, model_save_path)
            print(f"  -> Best model saved! (val_loss: {val_loss:.4f}, steer_mae: {val_steer_mae:.4f})")

        if early_stopping(val_loss):
            print(f"\n[Early Stopping] No improvement for {early_stopping_patience} epochs.")
            break

    print("=" * 60)
    print(f"Training Complete!")
    print(f"  Best epoch: {best_epoch}")
    print(f"  Best val_loss: {best_val_loss:.4f}")
    print(f"  Best val_steer_mae: {best_val_steer_mae:.4f}")
    print(f"  Model saved: {model_save_path}")
    print("=" * 60)

    if graph_save_dir:
        os.makedirs(graph_save_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        graph_path = os.path.join(graph_save_dir, f"regression_graph_{image_type}_{timestamp}.png")
        save_training_graph(
            train_losses, val_losses, train_steer_maes, val_steer_maes, graph_path,
            title=f"Regression ({image_type}) (Best: Epoch {best_epoch}, Val Loss: {best_val_loss:.4f}, Steer MAE: {best_val_steer_mae:.4f})"
        )

        result_path = os.path.join(graph_save_dir, f"regression_result_{timestamp}.json")
        with open(result_path, 'w') as f:
            json.dump({
                'best_epoch': best_epoch,
                'best_val_loss': float(best_val_loss),
                'best_val_steer_mae': float(best_val_steer_mae),
                'total_epochs': len(train_losses),
                'backbone': backbone,
                'speed_normalize': speed_normalize,
                'steering_loss_weight': steering_loss_weight,
                'intervention_loss_weight': intervention_loss_weight,
                'use_intervention_weighting': use_intervention_weighting,
                'image_type': image_type,
                'train_losses': [float(l) for l in train_losses],
                'val_losses': [float(l) for l in val_losses],
                'train_steer_maes': [float(m) for m in train_steer_maes],
                'val_steer_maes': [float(m) for m in val_steer_maes],
            }, f, indent=2)

    return model


if __name__ == "__main__":
    import glob
    import argparse

    parser = argparse.ArgumentParser(description="Speed-Aware Regression Training")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch_size", type=int, default=64)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--weight_decay", type=float, default=1e-5)
    parser.add_argument("--dropout", type=float, default=0.3)
    parser.add_argument("--val_ratio", type=float, default=0.3)
    parser.add_argument("--early_stopping", type=int, default=15)
    parser.add_argument("--backbone", type=str, default="resnet18",
                        choices=["resnet18", "mobilenet_v3_small"])
    parser.add_argument("--speed_norm", type=float, default=3.0)
    parser.add_argument("--steering_weight", type=float, default=5.0,
                        help="steering loss 가중치 (throttle 대비)")
    parser.add_argument("--intervention_weight", type=float, default=2.0,
                        help="intervention=1 샘플에 적용할 추가 손실 가중치 (최소 1.0)")
    parser.add_argument("--no_intervention_weighting", action="store_true",
                        help="intervention 샘플 가중치 비활성화")
    parser.add_argument("--freeze_backbone", action="store_true")
    parser.add_argument("--image_type", type=str, default="front_1",
                        choices=["front_1", "front_2", "front_3", "front_4"])
    parser.add_argument("--test", action="store_true", help="테스트 모드")
    args = parser.parse_args()

    if args.test:
        args.epochs = 20
        args.batch_size = 16
        args.val_ratio = 0.3
        args.early_stopping = 15
        print("[TEST MODE]")

    # Prefer project-root TrainingData, fallback to legacy python/TrainingData.
    project_root = Path(__file__).resolve().parents[2]
    candidate_paths = [
        project_root / "TrainingData",
        Path(__file__).resolve().parents[1] / "TrainingData",
    ]
    base_path = next((p for p in candidate_paths if p.exists()), None)

    if base_path is None:
        print(f"Error: TrainingData does not exist in any of: {candidate_paths}")
        exit(1)

    data_dirs = sorted(glob.glob(str(base_path / "session_*")))

    if len(data_dirs) == 0:
        print("Error: No sessions found!")
        exit(1)

    print(f"Found {len(data_dirs)} sessions")

    checkpoint_dir = Path(__file__).parent / "checkpoints"
    checkpoint_dir.mkdir(exist_ok=True)

    graph_dir = base_path / "loss_graph"
    graph_dir.mkdir(exist_ok=True)

    model = train_regression(
        data_dirs=data_dirs,
        val_ratio=args.val_ratio,
        model_save_path=str(checkpoint_dir / "driving_regression.pth"),
        epochs=args.epochs,
        batch_size=args.batch_size,
        learning_rate=args.lr,
        weight_decay=args.weight_decay,
        dropout=args.dropout,
        backbone=args.backbone,
        early_stopping_patience=args.early_stopping,
        speed_normalize=args.speed_norm,
        freeze_backbone=args.freeze_backbone,
        steering_loss_weight=args.steering_weight,
        intervention_loss_weight=args.intervention_weight,
        use_intervention_weighting=not args.no_intervention_weighting,
        device="cuda" if torch.cuda.is_available() else "cpu",
        graph_save_dir=str(graph_dir),
        image_type=args.image_type
    )
