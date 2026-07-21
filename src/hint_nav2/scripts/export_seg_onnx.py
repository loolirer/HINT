"""Export a Hugging Face semantic-segmentation model to ONNX for OpenVINO inference.

Takes a downloaded HF model directory (`config.json` + `model.safetensors`) and emits a
graph that accepts a single image of shape (1, 3, H, W) and returns per-class logits.
H and W are fixed at export time — `ground_segmenter` resizes the frame to whatever the
graph declares, so the export size *is* the inference size.

Most encoder-decoder semantic-segmentation architectures work (SegFormer, DPT, BEiT,
UPerNet, DeepLabV3). **Mask2Former / MaskFormer / OneFormer do not** — they emit
query-based `masks_queries_logits` + `class_queries_logits` that need a post-processing
step to become a class map, which this exporter (and the node) don't do.

The logits come out at the model's native stride (SegFormer: H/4 x W/4). That's left
as-is rather than upsampled in-graph — `ground_segmenter` nearest-resizes the mask to
the camera frame anyway, and argmaxing at stride is cheaper than argmaxing at full res.
Pass --upsample if you want the graph to do it instead.

The export is fp32; let OpenVINO pick fp16 on the iGPU at runtime (same as the DA3 path).

Example:
    python3 scripts/export_seg_onnx.py \\
        --model-dir ~/turtlebot3_ws/src/hint_nav2/models/segformer-b0-ade \\
        --height 384 --width 512 \\
        --output ~/turtlebot3_ws/src/hint_nav2/models/ground-seg.onnx
"""

import argparse
import os
import re

import torch
import torch.nn as nn
import torch.nn.functional as F

from transformers import AutoConfig, AutoModelForSemanticSegmentation

# Label words that read as traversable ground, used to suggest ground_class_ids. Matched
# on word boundaries, not as substrings: "land" must not fire on "kitchen island".
_GROUND_WORDS = (
    "floor", "flooring", "road", "earth", "ground", "rug", "carpet", "path",
    "pathway", "pavement", "sidewalk", "runway", "dirt", "sand", "grass", "land",
)
_GROUND_RE = re.compile(
    r"\b(" + "|".join(_GROUND_WORDS) + r")\b", flags=re.IGNORECASE
)


class SegOnnxWrapper(nn.Module):
    """Wraps an HF segmentation model to take (B, 3, H, W) and return only logits."""

    def __init__(self, model: nn.Module, size=None) -> None:
        super().__init__()
        self.model = model
        self.size = size  # (H, W) to upsample logits to, or None for native stride

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        logits = self.model(pixel_values=image).logits  # (B, C, h, w)
        if self.size is not None:
            logits = F.interpolate(
                logits, size=self.size, mode="bilinear", align_corners=False
            )
        return logits


def _report_labels(config) -> None:
    """Print the model's label map + the ground_class_ids it implies."""
    id2label = getattr(config, "id2label", None) or {}
    if not id2label:
        print(
            "\n! This model's config carries no id2label map. You'll have to source the\n"
            "  label ordering from the model card to set ground_class_ids."
        )
        return

    hits = sorted(
        int(i) for i, name in id2label.items() if _GROUND_RE.search(str(name))
    )
    print(f"\nLabel map: {len(id2label)} classes")
    if hits:
        print("Ground-looking classes:")
        for i in hits:
            print(f"  {i:3d}  {id2label[i] if i in id2label else id2label[str(i)]}")
        print(
            "\nCandidate ids (a name match — NOT a traversability judgement; review\n"
            "them). Most are outdoor scenery an indoor robot never meets, and only\n"
            "you know whether e.g. grass or sand carries your robot:\n"
            f"  -p ground_class_ids:=\"{hits}\"\n"
            "Indoors, 'floor' alone is usually the honest answer."
        )
    else:
        print(
            "! No label looked like ground. Inspect the full map and pick manually:\n"
            "  python3 -c \"from transformers import AutoConfig; "
            "print(AutoConfig.from_pretrained('<dir>').id2label)\""
        )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--model-dir", required=True, help="Path to the local HF model directory."
    )
    ap.add_argument("--height", type=int, default=384, help="Input height.")
    ap.add_argument("--width", type=int, default=512, help="Input width.")
    ap.add_argument("--output", default="ground-seg.onnx", help="Output .onnx path.")
    ap.add_argument("--opset", type=int, default=17, help="ONNX opset version.")
    ap.add_argument(
        "--upsample",
        action="store_true",
        help="Upsample logits to the input size inside the graph (default: native stride).",
    )
    args = ap.parse_args()

    if args.height % 32 or args.width % 32:
        print(
            f"! {args.height}x{args.width} is not divisible by 32. Hierarchical encoders "
            "(SegFormer et al.) downsample by up to 32x; a non-multiple can shift the "
            "output shape or fail outright. Continuing anyway."
        )

    model_dir = os.path.expanduser(os.path.expandvars(args.model_dir))
    print(f"Loading model from: {model_dir}")
    config = AutoConfig.from_pretrained(model_dir, local_files_only=True)
    model = AutoModelForSemanticSegmentation.from_pretrained(
        model_dir, local_files_only=True
    )
    model.eval()

    size = (args.height, args.width) if args.upsample else None
    wrapper = SegOnnxWrapper(model, size).eval()
    dummy = torch.zeros(1, 3, args.height, args.width, dtype=torch.float32)

    with torch.no_grad():
        out = wrapper(dummy)
        print(f"Traced output shape: {tuple(out.shape)}  (1, classes, h, w)")

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
            output_names=["logits"],
            training=torch.onnx.TrainingMode.EVAL,
            # Legacy TorchScript exporter, matching export_onnx.py: the dynamo path is
            # still flaky across HF vision architectures, and the input size is fixed
            # here anyway, so tracing loses nothing.
            dynamo=False,
        )
    print("Done.")
    _report_labels(config)


if __name__ == "__main__":
    main()
