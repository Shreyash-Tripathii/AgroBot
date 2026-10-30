import math
import torch
import torch.nn as nn
import torch.nn.functional as F


class PatchEmbed(nn.Module):
    def __init__(self, patch_size=14, in_chans=3, embed_dim=384):
        super().__init__()
        self.patch_size = patch_size
        self.proj = nn.Conv2d(in_chans, embed_dim, kernel_size=patch_size, stride=patch_size)

    def forward(self, x):
        return self.proj(x)


class Attention(nn.Module):
    def __init__(self, dim, num_heads=6, qkv_bias=True):
        super().__init__()
        self.num_heads = num_heads
        self.head_dim = dim // num_heads
        self.scale = self.head_dim ** -0.5

        self.qkv = nn.Linear(dim, dim * 3, bias=qkv_bias)
        self.proj = nn.Linear(dim, dim)

    def forward(self, x):
        B, N, C = x.shape
        qkv = self.qkv(x).reshape(B, N, 3, self.num_heads, self.head_dim).permute(2, 0, 3, 1, 4)
        q, k, v = qkv[0], qkv[1], qkv[2]

        attn = (q @ k.transpose(-2, -1)) * self.scale
        attn = attn.softmax(dim=-1)

        x = (attn @ v).transpose(1, 2).reshape(B, N, C)
        x = self.proj(x)
        return x


class Mlp(nn.Module):
    def __init__(self, in_features, hidden_features=None, out_features=None, act_layer=nn.GELU):
        super().__init__()
        out_features = out_features or in_features
        hidden_features = hidden_features or in_features
        self.fc1 = nn.Linear(in_features, hidden_features)
        self.act = act_layer()
        self.fc2 = nn.Linear(hidden_features, out_features)

    def forward(self, x):
        x = self.fc1(x)
        x = self.act(x)
        x = self.fc2(x)
        return x


class Block(nn.Module):
    def __init__(self, dim, num_heads, mlp_ratio=4.0, qkv_bias=True, init_values=None):
        super().__init__()
        self.norm1 = nn.LayerNorm(dim, eps=1e-6)
        self.attn = Attention(dim, num_heads=num_heads, qkv_bias=qkv_bias)
        self.ls1 = nn.Parameter(init_values * torch.ones(dim)) if init_values is not None else nn.Identity()

        self.norm2 = nn.LayerNorm(dim, eps=1e-6)
        self.mlp = Mlp(in_features=dim, hidden_features=int(dim * mlp_ratio))
        self.ls2 = nn.Parameter(init_values * torch.ones(dim)) if init_values is not None else nn.Identity()

    def forward(self, x):
        if isinstance(self.ls1, nn.Identity):
            x = x + self.attn(self.norm1(x))
            x = x + self.mlp(self.norm2(x))
        else:
            x = x + self.ls1 * self.attn(self.norm1(x))
            x = x + self.ls2 * self.mlp(self.norm2(x))
        return x


class DINOv2(nn.Module):
    def __init__(self, patch_size=14, embed_dim=384, depth=12, num_heads=6, mlp_ratio=4.0, init_values=1e-5):
        super().__init__()
        self.patch_size = patch_size
        self.embed_dim = embed_dim
        self.patch_embed = PatchEmbed(patch_size, 3, embed_dim)

        self.cls_token = nn.Parameter(torch.zeros(1, 1, embed_dim))
        self.pos_embed = nn.Parameter(torch.zeros(1, 10000, embed_dim))

        self.blocks = nn.ModuleList([
            Block(embed_dim, num_heads, mlp_ratio=mlp_ratio, qkv_bias=True, init_values=init_values)
            for _ in range(depth)
        ])
        self.norm = nn.LayerNorm(embed_dim, eps=1e-6)

    def interpolate_pos_encoding(self, x, w, h):
        npatch = x.shape[1] - 1
        N = self.pos_embed.shape[1] - 1
        if npatch == N and w == h:
            return self.pos_embed

        class_pos_embed = self.pos_embed[:, 0]
        patch_pos_embed = self.pos_embed[:, 1:]
        dim = x.shape[-1]
        w0 = w // self.patch_size
        h0 = h // self.patch_size

        M = int(math.sqrt(N))
        patch_pos_embed = patch_pos_embed[:, :M * M].reshape(1, M, M, dim).permute(0, 3, 1, 2)
        patch_pos_embed = F.interpolate(patch_pos_embed, size=(h0, w0), mode='bicubic', align_corners=False)
        patch_pos_embed = patch_pos_embed.permute(0, 2, 3, 1).view(1, -1, dim)
        return torch.cat((class_pos_embed.unsqueeze(0), patch_pos_embed), dim=1)

    def get_intermediate_layers(self, x, n=[2, 5, 8, 11]):
        B, C, H, W = x.shape
        x = self.patch_embed(x)
        w0, h0 = W // self.patch_size, H // self.patch_size
        x = x.flatten(2).transpose(1, 2)

        cls_tokens = self.cls_token.expand(B, -1, -1)
        x = torch.cat((cls_tokens, x), dim=1)
        x = x + self.interpolate_pos_encoding(x, W, H)

        outputs = []
        for i, blk in enumerate(self.blocks):
            x = blk(x)
            if i in n:
                # Remove cls token and reshape to (B, C, H_patch, W_patch)
                feat = x[:, 1:].permute(0, 2, 1).reshape(B, self.embed_dim, h0, w0).contiguous()
                outputs.append(feat)

        return outputs
