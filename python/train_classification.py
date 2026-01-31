# train_classification.py
"""
Speed-Aware DAgger 분류 기반 자율주행 학습 스크립트

개선점:
- 속도 정보를 모델 입력에 추가
- 같은 이미지라도 속도에 따라 다른 액션 예측 가능
- DAgger 워크플로우 지원

키 입력 분류:
  0: FORWARD (W)
  1: FORWARD_LEFT (W+A)
  2: FORWARD_RIGHT (W+D)
  3: LEFT (A)
  4: RIGHT (D)
  5: BACKWARD (S)
  6: NONE (관성 주행)
"""

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader, WeightedRandomSampler
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


# 클래스 정의
CLASS_NAMES = ['FORWARD', 'FORWARD_LEFT', 'FORWARD_RIGHT', 'LEFT', 'RIGHT', 'BACKWARD', 'NONE']
NUM_CLASSES = 7


class SpeedAwareDualViewNet(nn.Module):
    """
    Speed-Aware 듀얼 뷰 분류 모델
    Front View + Top View + Speed → 7개 클래스 분류

    속도 정보를 함께 학습하여 같은 이미지라도
    속도에 따라 다른 액션을 예측할 수 있습니다.
    """

    def __init__(self, backbone="resnet18", pretrained=True, dropout=0.5):
        super().__init__()

        # Front View 인코더
        if backbone == "resnet18":
            self.front_encoder = models.resnet18(
                weights=models.ResNet18_Weights.IMAGENET1K_V1 if pretrained else None
            )
            self.top_encoder = models.resnet18(
                weights=models.ResNet18_Weights.IMAGENET1K_V1 if pretrained else None
            )
            feature_dim = 512
        elif backbone == "mobilenet_v3_small":
            # 경량 모델 (라즈베리파이용)
            self.front_encoder = models.mobilenet_v3_small(
                weights=models.MobileNet_V3_Small_Weights.IMAGENET1K_V1 if pretrained else None
            )
            self.top_encoder = models.mobilenet_v3_small(
                weights=models.MobileNet_V3_Small_Weights.IMAGENET1K_V1 if pretrained else None
            )
            feature_dim = 576  # mobilenet_v3_small의 마지막 레이어 출력
            self.front_encoder.classifier = nn.Identity()
            self.top_encoder.classifier = nn.Identity()
        else:
            raise ValueError(f"Unknown backbone: {backbone}")

        # ResNet의 FC 레이어 제거
        if backbone == "resnet18":
            self.front_encoder.fc = nn.Identity()
            self.top_encoder.fc = nn.Identity()

        self.backbone = backbone
        self.feature_dim = feature_dim

        # 속도 임베딩 (1차원 → 64차원)
        # 속도를 고차원 특징으로 변환하여 이미지 특징과 결합
        self.speed_embedding = nn.Sequential(
            nn.Linear(1, 32),
            nn.ReLU(),
            nn.Linear(32, 64),
            nn.ReLU()
        )

        # 분류 헤드 (front_features + top_features + speed_embedding)
        combined_dim = feature_dim * 2 + 64
        self.classifier = nn.Sequential(
            nn.Linear(combined_dim, 512),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(512, 128),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(128, NUM_CLASSES)
        )

    def forward(self, front_img, top_img, speed):
        """
        Args:
            front_img: (B, 3, H, W) 전방 카메라 이미지
            top_img: (B, 3, H, W) 탑뷰 이미지
            speed: (B, 1) 현재 속도 (정규화됨)
        Returns:
            logits: (B, 7) 클래스별 로짓
        """
        front_features = self.front_encoder(front_img)
        top_features = self.top_encoder(top_img)

        # MobileNet의 경우 pooling 적용
        if self.backbone == "mobilenet_v3_small":
            if len(front_features.shape) == 4:
                front_features = front_features.mean([2, 3])
                top_features = top_features.mean([2, 3])

        speed_features = self.speed_embedding(speed)

        combined = torch.cat([front_features, top_features, speed_features], dim=1)
        logits = self.classifier(combined)

        return logits


class SpeedAwareClassificationDataset(Dataset):
    """
    Speed-Aware 분류 기반 주행 데이터셋

    특징:
    - 속도 정보를 정규화하여 모델 입력에 포함
    - 좌우 반전 augmentation 시 레이블도 교환
    - 클래스 불균형 해결을 위한 가중치 계산
    """

    def __init__(self, data_dirs: list, augment: bool = True, speed_normalize: float = 5.0):
        """
        Args:
            data_dirs: 데이터 디렉토리 리스트
            augment: 데이터 증강 여부
            speed_normalize: 속도 정규화 기준값 (m/s), 이 값으로 나누어 0~1 범위로 정규화
        """
        self.augment = augment
        self.speed_normalize = speed_normalize
        self.data = []

        for data_dir in data_dirs:
            data_dir = Path(data_dir)
            csv_path = data_dir / "driving_log.csv"

            if not csv_path.exists():
                print(f"[Warning] {csv_path} not found, skipping...")
                continue

            df = pd.read_csv(csv_path)

            # V2 형식 확인 (key_action 컬럼 필수)
            if 'key_action' not in df.columns:
                print(f"[Warning] {csv_path} is not V2 format (no key_action), skipping...")
                continue

            for idx, row in df.iterrows():
                self.data.append({
                    'front_path': str(data_dir / row['front_image']),
                    'top_path': str(data_dir / row['top_image']),
                    'key_action': int(row['key_action']),
                    'speed': float(row['speed'])
                })

        if len(self.data) == 0:
            raise ValueError("No valid V2 data found! Make sure CSV has 'key_action' column.")

        # 정규화 transform
        self.normalize = A.Compose([
            A.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            ToTensorV2()
        ])

        # 데이터 증강
        if augment:
            self.aug_transform = A.Compose([
                A.RandomBrightnessContrast(brightness_limit=0.2, contrast_limit=0.2, p=0.5),
                A.GaussNoise(var_limit=(5.0, 20.0), p=0.3),
                A.MotionBlur(blur_limit=3, p=0.2),
            ])
        else:
            self.aug_transform = None

        # 클래스 가중치 계산
        self._compute_class_weights()

        # 속도 통계 출력
        speeds = [d['speed'] for d in self.data]
        print(f"[Dataset] 속도 통계: min={min(speeds):.2f}, max={max(speeds):.2f}, avg={np.mean(speeds):.2f} m/s")
        print(f"[Dataset] 속도 정규화 기준: {speed_normalize} m/s")

    def _compute_class_weights(self):
        """클래스 불균형 해결을 위한 가중치 계산"""
        labels = np.array([d['key_action'] for d in self.data])
        class_counts = np.bincount(labels, minlength=NUM_CLASSES)

        # 가중치 = 1 / count (희귀 클래스에 높은 가중치)
        class_weights = 1.0 / np.sqrt(class_counts + 1)  # sqrt로 완화
        class_weights = class_weights / class_weights.sum() * NUM_CLASSES

        self.class_weights = torch.FloatTensor(class_weights)

        # 샘플별 가중치 (WeightedRandomSampler용)
        self.sample_weights = class_weights[labels]

        print(f"[Dataset] 클래스 분포:")
        for i, name in enumerate(CLASS_NAMES):
            print(f"  {name:14s}: {class_counts[i]:5d} ({class_counts[i]/len(labels)*100:5.1f}%) - weight {class_weights[i]:.2f}")

    def get_sample_weights(self):
        return self.sample_weights

    def get_class_weights(self):
        return self.class_weights

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        item = self.data[idx]

        front_image = np.array(Image.open(item['front_path']).convert('RGB'))
        top_image = np.array(Image.open(item['top_path']).convert('RGB'))

        label = item['key_action']
        speed = item['speed'] / self.speed_normalize  # 정규화된 속도

        # 좌우 반전 augmentation (50% 확률)
        if self.augment and np.random.random() < 0.5:
            front_image = np.fliplr(front_image).copy()
            top_image = np.fliplr(top_image).copy()

            # 레이블 교환: LEFT <-> RIGHT, FORWARD_LEFT <-> FORWARD_RIGHT
            if label == 1:  # FORWARD_LEFT -> FORWARD_RIGHT
                label = 2
            elif label == 2:  # FORWARD_RIGHT -> FORWARD_LEFT
                label = 1
            elif label == 3:  # LEFT -> RIGHT
                label = 4
            elif label == 4:  # RIGHT -> LEFT
                label = 3

        # 기타 augmentation
        if self.aug_transform is not None:
            front_image = self.aug_transform(image=front_image)['image']
            top_image = self.aug_transform(image=top_image)['image']

        # 정규화
        front_image = self.normalize(image=front_image)['image']
        top_image = self.normalize(image=top_image)['image']

        return (
            front_image,
            top_image,
            torch.tensor([speed], dtype=torch.float32),
            torch.tensor(label, dtype=torch.long)
        )


class EarlyStopping:
    """Early Stopping 클래스"""
    def __init__(self, patience=10, min_delta=0.001):
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


def save_training_graph(train_losses, val_losses, train_accs, val_accs, save_path, title=""):
    """학습 그래프 저장"""
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    epochs = range(1, len(train_losses) + 1)

    # Loss 그래프
    axes[0].plot(epochs, train_losses, 'b-', label='Train Loss', linewidth=2)
    axes[0].plot(epochs, val_losses, 'r-', label='Val Loss', linewidth=2)
    axes[0].set_xlabel('Epoch')
    axes[0].set_ylabel('Loss (CrossEntropy)')
    axes[0].set_title('Training & Validation Loss')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # Accuracy 그래프
    axes[1].plot(epochs, train_accs, 'b-', label='Train Acc', linewidth=2)
    axes[1].plot(epochs, val_accs, 'r-', label='Val Acc', linewidth=2)
    axes[1].set_xlabel('Epoch')
    axes[1].set_ylabel('Accuracy (%)')
    axes[1].set_title('Training & Validation Accuracy')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.suptitle(title, fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"[Graph] Saved to: {save_path}")


def train_speed_aware(
    data_dirs: list,
    val_ratio: float = 0.2,
    model_save_path: str = "checkpoints/driving_classifier.pth",
    epochs: int = 50,
    batch_size: int = 32,
    learning_rate: float = 1e-4,
    weight_decay: float = 1e-4,
    dropout: float = 0.5,
    backbone: str = "resnet18",
    use_weighted_sampling: bool = True,
    use_class_weights: bool = True,
    early_stopping_patience: int = 10,
    speed_normalize: float = 5.0,
    freeze_backbone: bool = False,
    device: str = "cuda",
    graph_save_dir: str = None
):
    """
    Speed-Aware DAgger 분류 학습

    Args:
        data_dirs: 학습 데이터 디렉토리 리스트
        val_ratio: 검증 세션 비율
        speed_normalize: 속도 정규화 기준값 (m/s)
        backbone: 백본 모델 ("resnet18" 또는 "mobilenet_v3_small")
    """
    print("=" * 60)
    print("Speed-Aware DAgger Classification Training")
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

    print(f"\n[Data Split]")
    print(f"  Train: {len(train_dirs)} sessions")
    print(f"  Val: {len(val_dirs)} sessions")

    # 데이터셋 생성
    train_dataset = SpeedAwareClassificationDataset(train_dirs, augment=True, speed_normalize=speed_normalize)
    val_dataset = SpeedAwareClassificationDataset(val_dirs, augment=False, speed_normalize=speed_normalize)

    print(f"\n[Dataset] Train: {len(train_dataset)}, Val: {len(val_dataset)}")

    # DataLoader
    if use_weighted_sampling:
        sampler = WeightedRandomSampler(
            weights=train_dataset.get_sample_weights(),
            num_samples=len(train_dataset),
            replacement=True
        )
        train_loader = DataLoader(train_dataset, batch_size=batch_size, sampler=sampler, num_workers=4, pin_memory=True)
    else:
        train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=4, pin_memory=True)

    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False, num_workers=4, pin_memory=True)

    # 모델
    model = SpeedAwareDualViewNet(backbone=backbone, pretrained=True, dropout=dropout).to(device)

    # Backbone Freeze (과적합 방지)
    if freeze_backbone:
        for param in model.front_encoder.parameters():
            param.requires_grad = False
        for param in model.top_encoder.parameters():
            param.requires_grad = False
        print(f"\n[Model] Backbone FROZEN - only classifier will be trained")

    total_params = sum(p.numel() for p in model.parameters())
    trainable_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"\n[Model] Backbone: {backbone}")
    print(f"[Model] Total params: {total_params:,}")
    print(f"[Model] Trainable params: {trainable_params:,}")
    print(f"[Model] Speed normalization: {speed_normalize} m/s")

    # 손실 함수 (클래스 가중치 적용)
    if use_class_weights:
        class_weights = train_dataset.get_class_weights().to(device)
        criterion = nn.CrossEntropyLoss(weight=class_weights)
        print(f"[Loss] CrossEntropyLoss with class weights")
    else:
        criterion = nn.CrossEntropyLoss()

    # 옵티마이저 & 스케줄러
    optimizer = optim.AdamW(model.parameters(), lr=learning_rate, weight_decay=weight_decay)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=5, min_lr=1e-6)

    # Early Stopping
    early_stopping = EarlyStopping(patience=early_stopping_patience)

    # 학습 기록
    train_losses, val_losses = [], []
    train_accs, val_accs = [], []
    best_val_loss = float('inf')
    best_epoch = 0

    print(f"\n[Training] Starting...")
    print("-" * 60)

    for epoch in range(epochs):
        # Training
        model.train()
        train_loss = 0.0
        train_correct = 0
        train_total = 0

        pbar = tqdm(train_loader, desc=f"Epoch {epoch+1}/{epochs}")
        for front_imgs, top_imgs, speeds, labels in pbar:
            front_imgs = front_imgs.to(device)
            top_imgs = top_imgs.to(device)
            speeds = speeds.to(device)
            labels = labels.to(device)

            optimizer.zero_grad()
            logits = model(front_imgs, top_imgs, speeds)
            loss = criterion(logits, labels)
            loss.backward()

            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()

            train_loss += loss.item()
            _, predicted = torch.max(logits, 1)
            train_correct += (predicted == labels).sum().item()
            train_total += labels.size(0)

            pbar.set_postfix({'loss': f'{loss.item():.4f}', 'acc': f'{train_correct/train_total*100:.1f}%'})

        train_loss /= len(train_loader)
        train_acc = train_correct / train_total * 100

        # Validation
        model.eval()
        val_loss = 0.0
        val_correct = 0
        val_total = 0

        with torch.no_grad():
            for front_imgs, top_imgs, speeds, labels in val_loader:
                front_imgs = front_imgs.to(device)
                top_imgs = top_imgs.to(device)
                speeds = speeds.to(device)
                labels = labels.to(device)

                logits = model(front_imgs, top_imgs, speeds)
                loss = criterion(logits, labels)

                val_loss += loss.item()
                _, predicted = torch.max(logits, 1)
                val_correct += (predicted == labels).sum().item()
                val_total += labels.size(0)

        val_loss /= len(val_loader)
        val_acc = val_correct / val_total * 100

        # 스케줄러
        scheduler.step(val_loss)
        current_lr = optimizer.param_groups[0]['lr']

        # 기록
        train_losses.append(train_loss)
        val_losses.append(val_loss)
        train_accs.append(train_acc)
        val_accs.append(val_acc)

        print(f"Epoch {epoch+1:3d}: Train Loss={train_loss:.4f}, Acc={train_acc:.1f}% | "
              f"Val Loss={val_loss:.4f}, Acc={val_acc:.1f}% | LR={current_lr:.2e}")

        # Best 모델 저장 (val_loss 기준)
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            best_epoch = epoch + 1

            torch.save({
                'epoch': epoch,
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'val_loss': val_loss,
                'val_acc': val_acc,
                'class_names': CLASS_NAMES,
                'num_classes': NUM_CLASSES,
                'backbone': backbone,
                'speed_normalize': speed_normalize,
                'model_version': 'speed_aware_v2',
            }, model_save_path)
            print(f"  → Best model saved! (val_loss: {val_loss:.4f})")

        # Early Stopping
        if early_stopping(val_loss):
            print(f"\n[Early Stopping] No improvement for {early_stopping_patience} epochs.")
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
        graph_path = os.path.join(graph_save_dir, f"speed_aware_graph_{timestamp}.png")
        save_training_graph(
            train_losses, val_losses, train_accs, val_accs, graph_path,
            title=f"Speed-Aware DAgger (Best: Epoch {best_epoch}, Val Loss: {best_val_loss:.4f})"
        )

        # 결과 JSON 저장
        result_path = os.path.join(graph_save_dir, f"speed_aware_result_{timestamp}.json")
        with open(result_path, 'w') as f:
            json.dump({
                'best_epoch': best_epoch,
                'best_val_loss': float(best_val_loss),
                'total_epochs': len(train_losses),
                'class_names': CLASS_NAMES,
                'backbone': backbone,
                'speed_normalize': speed_normalize,
                'train_losses': [float(l) for l in train_losses],
                'val_losses': [float(l) for l in val_losses],
                'train_accs': [float(a) for a in train_accs],
                'val_accs': [float(a) for a in val_accs],
            }, f, indent=2)

    return model


if __name__ == "__main__":
    import glob
    import argparse

    parser = argparse.ArgumentParser(description="Speed-Aware DAgger Classification Training")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch_size", type=int, default=64)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--weight_decay", type=float, default=1e-5)
    parser.add_argument("--dropout", type=float, default=0.3)
    parser.add_argument("--val_ratio", type=float, default=0.3)
    parser.add_argument("--early_stopping", type=int, default=15)
    parser.add_argument("--backbone", type=str, default="resnet18",
                        choices=["resnet18", "mobilenet_v3_small"])
    parser.add_argument("--speed_norm", type=float, default=5.0,
                        help="속도 정규화 기준값 (m/s)")
    parser.add_argument("--freeze_backbone", action="store_true",
                        help="Backbone(ResNet/MobileNet) 고정, classifier만 학습")
    parser.add_argument("--test", action="store_true", help="테스트 모드")
    args = parser.parse_args()

    if args.test:
        args.epochs = 20
        args.batch_size = 16
        args.val_ratio = 0.3
        args.early_stopping = 15
        print("[TEST MODE] 소규모 데이터 테스트용 파라미터 적용")

    # V2 데이터 경로
    base_path = Path(__file__).parent.parent / "TrainingData"

    if not base_path.exists():
        print(f"Error: {base_path} does not exist!")
        print("먼저 Unity에서 DrivingDataCollector로 데이터를 수집하세요.")
        exit(1)

    data_dirs = sorted(glob.glob(str(base_path / "session_*")))

    if len(data_dirs) == 0:
        print("Error: No sessions found!")
        exit(1)

    print(f"Found {len(data_dirs)} sessions")

    # 디렉토리 생성
    checkpoint_dir = Path(__file__).parent / "checkpoints"
    checkpoint_dir.mkdir(exist_ok=True)

    graph_dir = base_path / "loss_graph"
    graph_dir.mkdir(exist_ok=True)

    # 학습
    model = train_speed_aware(
        data_dirs=data_dirs,
        val_ratio=args.val_ratio,
        model_save_path=str(checkpoint_dir / "driving_classifier.pth"),
        epochs=args.epochs,
        batch_size=args.batch_size,
        learning_rate=args.lr,
        weight_decay=args.weight_decay,
        dropout=args.dropout,
        backbone=args.backbone,
        use_weighted_sampling=False,
        use_class_weights=False,
        early_stopping_patience=args.early_stopping,
        speed_normalize=args.speed_norm,
        freeze_backbone=args.freeze_backbone,
        device="cuda" if torch.cuda.is_available() else "cpu",
        graph_save_dir=str(graph_dir)
    )
