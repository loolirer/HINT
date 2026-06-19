"""Export a Depth Anything 3 model to ONNX for OpenVINO inference.

The exported graph takes a single image of shape (1, 3, H, W) and returns the
depth map. H and W are fixed at export time and must be divisible by 14.

We call the inner network directly (api_model.model) to bypass DepthAnything3's
internal autocast, which forces fp16/bf16 -- bf16 is not supported by ONNX. The
result is a clean fp32 graph; let OpenVINO pick fp16 on the iGPU at runtime.

Example:
    python3 scripts/export_onnx.py \
        --model-dir ~/turtlebot3_ws/src/turtlebot3/depth_anything/models/da3-small \
        --height 224 --width 294 \
        --output da3-small.onnx
"""

import argparse
import os

import torch
import torch.nn as nn

from depth_anything_3.api import DepthAnything3


def _onnx_cartesian_prod(*tensors):
    """ONNX-exportable torch.cartesian_prod (aten::cartesian_prod is unsupported).

    Matches torch.cartesian_prod ordering: first tensor varies slowest.
    """
    grids = torch.meshgrid(*tensors, indexing="ij")
    return torch.stack([g.reshape(-1) for g in grids], dim=-1)


class DepthOnnxWrapper(nn.Module):
    """Wraps the DA3 network to take (B, 3, H, W) and return only depth."""

    def __init__(self, api_model: DepthAnything3) -> None:
        super().__init__()
        # Inner network: calling it directly skips the api-level autocast.
        self.net = api_model.model

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        x = image.unsqueeze(1)  # (B, 3, H, W) -> (B, 1, 3, H, W) single view
        out = self.net(x, None, None, [], False, False, "saddle_balanced")
        return out["depth"]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--model-dir", required=True, help="Path to the local DA3 model directory."
    )
    ap.add_argument(
        "--height", type=int, default=224, help="Input height (divisible by 14)."
    )
    ap.add_argument(
        "--width", type=int, default=294, help="Input width (divisible by 14)."
    )
    ap.add_argument("--output", default="da3-small.onnx", help="Output .onnx path.")
    ap.add_argument("--opset", type=int, default=20, help="ONNX opset version.")
    args = ap.parse_args()

    if args.height % 14 or args.width % 14:
        raise SystemExit("--height and --width must both be divisible by 14.")

    model_dir = os.path.expanduser(os.path.expandvars(args.model_dir))
    print(f"Loading model from: {model_dir}")
    model = DepthAnything3.from_pretrained(model_dir, local_files_only=True)
    model.eval()

    # Replace unsupported ops with ONNX-exportable equivalents.
    torch.cartesian_prod = _onnx_cartesian_prod

    wrapper = DepthOnnxWrapper(model).eval()
    dummy = torch.zeros(1, 3, args.height, args.width, dtype=torch.float32)

    print(
        f"Exporting ONNX ({args.height}x{args.width}, opset {args.opset}) -> {args.output}"
    )
    with torch.no_grad():
        torch.onnx.export(
            wrapper,
            dummy,
            args.output,
            export_params=True,
            opset_version=args.opset,
            do_constant_folding=True,
            input_names=["image"],
            output_names=["depth"],
            training=torch.onnx.TrainingMode.EVAL,
            # Use the legacy TorchScript exporter: the dynamo path fails on DA3's
            # data-dependent code (int(positions.max()) in the RoPE layer). The
            # tracer bakes those values as constants for the fixed input size.
            dynamo=False,
        )
    print("Done.")


if __name__ == "__main__":
    main()
