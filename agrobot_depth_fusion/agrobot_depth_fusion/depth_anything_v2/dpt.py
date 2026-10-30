import os
import urllib.request
import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from .dinov2 import DINOv2


def _make_scratch(in_shape, out_shape, groups=1, expand=False):
    scratch = nn.Module()
    out_shape1 = out_shape
    out_shape2 = out_shape
    out_shape3 = out_shape
    out_shape4 = out_shape
    if expand:
        out_shape1 = out_shape
        out_shape2 = out_shape * 2
        out_shape3 = out_shape * 4
        out_shape4 = out_shape * 8

    scratch.layer1_rn = nn.Conv2d(in_shape[0], out_shape1, kernel_size=3, stride=1, padding=1, bias=False, groups=groups)
    scratch.layer2_rn = nn.Conv2d(in_shape[1], out_shape2, kernel_size=3, stride=1, padding=1, bias=False, groups=groups)
    scratch.layer3_rn = nn.Conv2d(in_shape[2], out_shape3, kernel_size=3, stride=1, padding=1, bias=False, groups=groups)
    scratch.layer4_rn = nn.Conv2d(in_shape[3], out_shape4, kernel_size=3, stride=1, padding=1, bias=False, groups=groups)
    return scratch


class ResidualConvUnit(nn.Module):
    def __init__(self, features, activation=nn.ReLU(True)):
        super().__init__()
        self.conv1 = nn.Conv2d(features, features, kernel_size=3, stride=1, padding=1, bias=True)
        self.conv2 = nn.Conv2d(features, features, kernel_size=3, stride=1, padding=1, bias=True)
        self.act = activation

    def forward(self, x):
        out = self.act(x)
        out = self.conv1(out)
        out = self.act(out)
        out = self.conv2(out)
        return out + x


class FeatureFusionBlock(nn.Module):
    def __init__(self, features, activation=nn.ReLU(True), deconv=False, expand=False, align_corners=True):
        super().__init__()
        self.deconv = deconv
        self.align_corners = align_corners

        self.resConfUnit1 = ResidualConvUnit(features, activation)
        self.resConfUnit2 = ResidualConvUnit(features, activation)

    def forward(self, *xs):
        output = xs[0]
        if len(xs) == 2:
            res = self.resConfUnit1(xs[1])
            output = output + res
        output = self.resConfUnit2(output)
        output = F.interpolate(output, scale_factor=2, mode="bilinear", align_corners=self.align_corners)
        return output


class DPTHead(nn.Module):
    def __init__(self, in_channels, features=64, use_bn=False, out_channels=[256, 512, 1024, 1024]):
        super().__init__()
        self.use_bn = use_bn

        self.projects = nn.ModuleList([
            nn.Conv2d(in_channels=in_channels, out_channels=out_channel, kernel_size=1, stride=1, padding=0)
            for out_channel in out_channels
        ])

        self.resize_layers = nn.ModuleList([
            nn.ConvTranspose2d(out_channels[0], out_channels[0], kernel_size=4, stride=4, padding=0),
            nn.ConvTranspose2d(out_channels[1], out_channels[1], kernel_size=2, stride=2, padding=0),
            nn.Identity(),
            nn.Conv2d(out_channels[3], out_channels[3], kernel_size=3, stride=2, padding=1),
        ])

        self.scratch = _make_scratch(out_channels, features, groups=1, expand=False)

        self.scratch.refinenet1 = FeatureFusionBlock(features)
        self.scratch.refinenet2 = FeatureFusionBlock(features)
        self.scratch.refinenet3 = FeatureFusionBlock(features)
        self.scratch.refinenet4 = FeatureFusionBlock(features)

        self.scratch.output_conv1 = nn.Conv2d(features, features // 2, kernel_size=3, stride=1, padding=1)
        self.scratch.output_conv2 = nn.Sequential(
            nn.Conv2d(features // 2, 32, kernel_size=3, stride=1, padding=1),
            nn.ReLU(True),
            nn.Conv2d(32, 1, kernel_size=1, stride=1, padding=0),
            nn.ReLU(True),
        )

    def forward(self, out_features, patch_h, patch_w):
        out = []
        for i, x in enumerate(out_features):
            x = self.projects[i](x)
            x = self.resize_layers[i](x)
            out.append(x)

        layer_1_rn = self.scratch.layer1_rn(out[0])
        layer_2_rn = self.scratch.layer2_rn(out[1])
        layer_3_rn = self.scratch.layer3_rn(out[2])
        layer_4_rn = self.scratch.layer4_rn(out[3])

        path_4 = self.scratch.refinenet4(layer_4_rn)
        path_3 = self.scratch.refinenet3(path_4, layer_3_rn)
        path_2 = self.scratch.refinenet2(path_3, layer_2_rn)
        path_1 = self.scratch.refinenet1(path_2, layer_1_rn)

        out = self.scratch.output_conv1(path_1)
        out = F.interpolate(out, (int(patch_h * 14), int(patch_w * 14)), mode="bilinear", align_corners=True)
        out = self.scratch.output_conv2(out)
        return out


class DepthAnythingV2(nn.Module):
    def __init__(self, encoder='vits', features=64, out_channels=[48, 96, 192, 384], max_depth=20.0):
        super().__init__()
        self.encoder = encoder
        self.max_depth = max_depth

        embed_dim = 384 if encoder == 'vits' else (768 if encoder == 'vitb' else 1024)
        depth = 12 if encoder in ['vits', 'vitb'] else 24
        num_heads = 6 if encoder == 'vits' else (12 if encoder == 'vitb' else 16)

        self.pretrained = DINOv2(
            patch_size=14,
            embed_dim=embed_dim,
            depth=depth,
            num_heads=num_heads,
            mlp_ratio=4.0
        )

        self.depth_head = DPTHead(
            in_channels=embed_dim,
            features=features,
            out_channels=out_channels
        )

    def forward(self, x):
        h, w = x.shape[-2:]
        patch_h, patch_w = h // 14, w // 14

        features = self.pretrained.get_intermediate_layers(x, n=[2, 5, 8, 11])
        depth = self.depth_head(features, patch_h, patch_w)
        depth = F.interpolate(depth, size=(h, w), mode="bilinear", align_corners=True)
        return depth.squeeze(1)

    @torch.no_grad()
    def infer_image(self, raw_image: np.ndarray, input_size: int = 518) -> np.ndarray:
        """
        Runs monocular relative depth estimation on a single image.
        Args:
            raw_image: np.ndarray of shape (H, W, 3) in RGB or BGR format (uint8)
            input_size: target inference resolution (multiple of 14)
        Returns:
            depth_map: np.ndarray of shape (H, W) float32 relative depth map
        """
        orig_h, orig_w = raw_image.shape[:2]

        # Resize keeping aspect ratio to multiple of 14
        scale = input_size / max(orig_h, orig_w)
        new_h, new_w = int(round(orig_h * scale / 14)) * 14, int(round(orig_w * scale / 14)) * 14
        new_h = max(14, new_h)
        new_w = max(14, new_w)

        resized = cv2.resize(raw_image, (new_w, new_h), interpolation=cv2.INTER_CUBIC)
        image = resized.astype(np.float32) / 255.0

        # Normalize with ImageNet stats
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        image = (image - mean) / std

        # Transpose to (1, 3, H, W)
        tensor = torch.from_numpy(image).permute(2, 0, 1).unsqueeze(0)

        # Move to model's device and dtype
        device = next(self.parameters()).device
        dtype = next(self.parameters()).dtype
        tensor = tensor.to(device=device, dtype=dtype)

        depth = self.forward(tensor)

        # Resize back to original image size
        depth = F.interpolate(depth.unsqueeze(1), size=(orig_h, orig_w), mode="bilinear", align_corners=True)
        depth = depth.squeeze().cpu().float().numpy()

        return depth


def load_depth_anything_v2(checkpoint_path: str = None, encoder: str = 'vits', device: str = 'cuda', fp16: bool = True) -> DepthAnythingV2:
    """
    Factory function to instantiate Depth Anything V2 Small, load weights, and set FP16/CUDA mode.
    """
    configs = {
        'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
        'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
        'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
    }
    cfg = configs.get(encoder, configs['vits'])
    model = DepthAnythingV2(**cfg)

    # Determine checkpoint location
    default_cache_dir = os.path.expanduser('~/.cache/depth_anything_v2')
    os.makedirs(default_cache_dir, exist_ok=True)
    default_ckpt = os.path.join(default_cache_dir, f'depth_anything_v2_{encoder}.pth')

    target_ckpt = checkpoint_path if (checkpoint_path and os.path.exists(checkpoint_path)) else default_ckpt

    if not os.path.exists(target_ckpt):
        url = f"https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_{encoder}.pth"
        try:
            print(f"[DepthAnythingV2] Checkpoint not found at {target_ckpt}. Downloading from Hugging Face: {url}")
            urllib.request.urlretrieve(url, target_ckpt)
            print(f"[DepthAnythingV2] Download completed: {target_ckpt}")
        except Exception as e:
            print(f"[DepthAnythingV2] Warning: Could not download weights ({e}). Model will run uninitialized or using fallback.")

    if os.path.exists(target_ckpt):
        try:
            state_dict = torch.load(target_ckpt, map_location='cpu')
            model.load_state_dict(state_dict, strict=False)
            print(f"[DepthAnythingV2] Successfully loaded checkpoint: {target_ckpt}")
        except Exception as e:
            print(f"[DepthAnythingV2] Error loading checkpoint {target_ckpt}: {e}")

    model_device = torch.device(device if (torch.cuda.is_available() and 'cuda' in device) else 'cpu')
    model.to(model_device)
    model.eval()

    if model_device.type == 'cuda' and fp16:
        model.half()
        print(f"[DepthAnythingV2] Model running on {model_device} in FP16 mode.")
    else:
        print(f"[DepthAnythingV2] Model running on {model_device} in FP32 mode.")

    return model
