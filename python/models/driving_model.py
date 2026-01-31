# models/driving_model.py
import torch
import torch.nn as nn
import torchvision.models as models
from typing import Tuple, Optional

class DrivingNet(nn.Module):
    """
    자율주행용 딥러닝 모델
    - 백본: ResNet18 (ImageNet pretrained)
    - 헤드: 조향/가속 예측
    
    RTX 4060 Ti (8GB)에서 학습 가능하도록 최적화
    """
    
    def __init__(
        self,
        backbone: str = "resnet18",
        pretrained: bool = True,
        dropout: float = 0.5,
        freeze_backbone: bool = False
    ):
        super().__init__()
        
        # 백본 선택
        if backbone == "resnet18":
            self.backbone = models.resnet18(
                weights=models.ResNet18_Weights.IMAGENET1K_V1 if pretrained else None
            )
            feature_dim = 512
        elif backbone == "resnet34":
            self.backbone = models.resnet34(
                weights=models.ResNet34_Weights.IMAGENET1K_V1 if pretrained else None
            )
            feature_dim = 512
        elif backbone == "efficientnet_b0":
            self.backbone = models.efficientnet_b0(
                weights=models.EfficientNet_B0_Weights.IMAGENET1K_V1 if pretrained else None
            )
            feature_dim = 1280
        elif backbone == "mobilenet_v3_small":
            self.backbone = models.mobilenet_v3_small(
                weights=models.MobileNet_V3_Small_Weights.IMAGENET1K_V1 if pretrained else None
            )
            feature_dim = 576
        else:
            raise ValueError(f"Unknown backbone: {backbone}")
        
        # 백본의 FC 레이어 제거
        if hasattr(self.backbone, 'fc'):
            self.backbone.fc = nn.Identity()
        elif hasattr(self.backbone, 'classifier'):
            self.backbone.classifier = nn.Identity()
        
        # 백본 동결 (옵션)
        if freeze_backbone:
            for param in self.backbone.parameters():
                param.requires_grad = False
        
        # 자율주행 헤드
        self.driving_head = nn.Sequential(
            nn.Linear(feature_dim, 256),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(256, 64),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(64, 2)  # [steering, throttle]
        )
        
        # 출력 활성화 (steering: tanh, throttle: tanh)
        self.steering_activation = nn.Tanh()
        self.throttle_activation = nn.Tanh()
        
    def forward(self, x: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Args:
            x: 입력 이미지 [B, 3, H, W]
        Returns:
            steering: 조향값 [-1, 1]
            throttle: 가속값 [-1, 1]
        """
        # 백본 특징 추출
        features = self.backbone(x)
        
        # 자율주행 예측
        output = self.driving_head(features)
        
        steering = self.steering_activation(output[:, 0])
        throttle = self.throttle_activation(output[:, 1])
        
        return steering, throttle
    
    def predict(self, x: torch.Tensor) -> torch.Tensor:
        """추론용 (ONNX 변환 호환)"""
        steering, throttle = self.forward(x)
        return torch.stack([steering, throttle], dim=1)

class DrivingNetWithAux(DrivingNet):
    """
    보조 출력 추가 버전 (차선 감지 등)
    멀티태스크 학습으로 성능 향상
    """
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        # 보조 헤드: 도로 상태 분류
        self.aux_head = nn.Sequential(
            nn.Linear(512, 64),
            nn.ReLU(),
            nn.Linear(64, 3)  # [on_road, left_lane, right_lane]
        )
    
    def forward(self, x):
        features = self.backbone(x)
        
        # 메인 출력
        output = self.driving_head(features)
        steering = self.steering_activation(output[:, 0])
        throttle = self.throttle_activation(output[:, 1])
        
        # 보조 출력
        aux_output = self.aux_head(features)
        
        return steering, throttle, aux_output

class DualViewDrivingNet(nn.Module):
    """
    듀얼 뷰 자율주행 모델 (Front View + Top View)
    - Front View: 전방 카메라 (차선, 장애물 인식)
    - Top View: 상단 카메라 (공간 인식, 위치 파악)

    두 인코더의 특징을 결합하여 더 정확한 주행 예측
    """

    def __init__(
        self,
        backbone: str = "resnet18",
        pretrained: bool = True,
        dropout: float = 0.5,
        freeze_backbone: bool = False
    ):
        super().__init__()

        # Feature dimension based on backbone
        if backbone == "resnet18":
            feature_dim = 512
            weights = models.ResNet18_Weights.IMAGENET1K_V1 if pretrained else None
            self.front_encoder = models.resnet18(weights=weights)
            self.top_encoder = models.resnet18(weights=weights)
        elif backbone == "resnet34":
            feature_dim = 512
            weights = models.ResNet34_Weights.IMAGENET1K_V1 if pretrained else None
            self.front_encoder = models.resnet34(weights=weights)
            self.top_encoder = models.resnet34(weights=weights)
        else:
            raise ValueError(f"Unknown backbone: {backbone}")

        # FC 레이어 제거
        self.front_encoder.fc = nn.Identity()
        self.top_encoder.fc = nn.Identity()

        # 백본 동결 (옵션)
        if freeze_backbone:
            for param in self.front_encoder.parameters():
                param.requires_grad = False
            for param in self.top_encoder.parameters():
                param.requires_grad = False

        # 특징 융합 및 주행 헤드 (front + top = 2 * feature_dim)
        self.fusion_head = nn.Sequential(
            nn.Linear(feature_dim * 2, 512),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(512, 128),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(128, 2)  # [steering, throttle]
        )

        self.steering_activation = nn.Tanh()
        self.throttle_activation = nn.Tanh()

    def forward(
        self,
        front_image: torch.Tensor,
        top_image: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Args:
            front_image: Front view 이미지 [B, 3, H, W]
            top_image: Top view 이미지 [B, 3, H, W]
        Returns:
            steering: 조향값 [-1, 1]
            throttle: 가속값 [-1, 1]
        """
        # 각 뷰에서 특징 추출
        front_features = self.front_encoder(front_image)
        top_features = self.top_encoder(top_image)

        # 특징 결합
        combined = torch.cat([front_features, top_features], dim=1)

        # 주행 예측
        output = self.fusion_head(combined)

        steering = self.steering_activation(output[:, 0])
        throttle = self.throttle_activation(output[:, 1])

        return steering, throttle

    def predict(self, front_image: torch.Tensor, top_image: torch.Tensor) -> torch.Tensor:
        """추론용"""
        steering, throttle = self.forward(front_image, top_image)
        return torch.stack([steering, throttle], dim=1)


# 모델 테스트
if __name__ == "__main__":
    print("=== Single View Model ===")
    model = DrivingNet(backbone="resnet18", pretrained=True)
    total_params = sum(p.numel() for p in model.parameters())
    trainable_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"Total parameters: {total_params:,}")
    print(f"Trainable parameters: {trainable_params:,}")

    dummy_input = torch.randn(1, 3, 66, 200)
    steering, throttle = model(dummy_input)
    print(f"Steering: {steering.item():.4f}, Throttle: {throttle.item():.4f}")

    print("\n=== Dual View Model ===")
    dual_model = DualViewDrivingNet(backbone="resnet18", pretrained=True)
    total_params = sum(p.numel() for p in dual_model.parameters())
    trainable_params = sum(p.numel() for p in dual_model.parameters() if p.requires_grad)
    print(f"Total parameters: {total_params:,}")
    print(f"Trainable parameters: {trainable_params:,}")

    front_input = torch.randn(1, 3, 66, 200)
    top_input = torch.randn(1, 3, 128, 128)
    steering, throttle = dual_model(front_input, top_input)
    print(f"Steering: {steering.item():.4f}, Throttle: {throttle.item():.4f}")
