#!/usr/bin/env python3
"""Xuất checkpoint YOLO (.pt) → ONNX theo layout DeepStream của project.

Hỗ trợ 2 loại checkpoint:
  * ultralytics (YOLOv8/YOLO11) — nạp bằng package `ultralytics`
  * yolov5 (repo ultralytics/yolov5) — cần đường dẫn repo (`--yolov5-repo`)

Output ONNX: 1 tensor `output` shape [batch, num_boxes, 6]
    [x1, y1, x2, y2, score, class_id]   (toạ độ pixel theo input mạng)

Layout này khớp parser `NvDsInferParseYolo`
(resources/ds/nvdsinfer_custom_impl_Yolo) — NMS để nvinfer lo (cluster-mode=2).

Ví dụ:
    python3 scripts/export_onnx.py \
        --weights resources/weights/vehicle_n_best.pt \
        --imgsz 640 --max-batch 8
"""

from __future__ import annotations

import argparse
import io
import os
import pickle
import sys
import zipfile

import torch
import torch.nn as nn


# --------------------------------------------------------------------------- #
# Nhận dạng loại checkpoint
# --------------------------------------------------------------------------- #

def detect_kind(weights: str) -> str:
    """Trả về 'ultralytics' hoặc 'yolov5' bằng cách soi module trong pickle."""

    class _Probe(pickle.Unpickler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.modules: set[str] = set()

        def find_class(self, module, name):
            self.modules.add(module)
            return type(name, (object,), {"__init__": lambda self, *a, **k: None})

        def persistent_load(self, pid):
            return None

    with zipfile.ZipFile(weights) as archive:
        entry = next(n for n in archive.namelist() if n.endswith("data.pkl"))
        probe = _Probe(io.BytesIO(archive.read(entry)))
        try:
            probe.load()
        except Exception:  # noqa: BLE001 - chỉ cần tập module đã thu được
            pass

    if any(m.startswith("ultralytics") for m in probe.modules):
        return "ultralytics"
    if any(m == "models.yolo" or m.startswith("models.") for m in probe.modules):
        return "yolov5"
    raise SystemExit(f"Không nhận dạng được loại checkpoint: {weights}")


# --------------------------------------------------------------------------- #
# Đầu ra chuẩn DeepStream
# --------------------------------------------------------------------------- #

class UltralyticsHead(nn.Module):
    """[b, 4+nc, n] (xywh pixel) → [b, n, 6] (xyxy + score + class)."""

    def forward(self, x):
        if isinstance(x, (list, tuple)):
            x = x[0]
        x = x.transpose(1, 2)
        xy, wh = x[:, :, :2], x[:, :, 2:4]
        boxes = torch.cat([xy - wh / 2, xy + wh / 2], dim=-1)
        scores, labels = torch.max(x[:, :, 4:], dim=-1, keepdim=True)
        return torch.cat([boxes, scores, labels.to(boxes.dtype)], dim=-1)


class YoloV5Head(nn.Module):
    """[b, n, 5+nc] (xywh pixel + objectness) → [b, n, 6]."""

    def forward(self, x):
        if isinstance(x, (list, tuple)):
            x = x[0]
        xy, wh = x[:, :, :2], x[:, :, 2:4]
        boxes = torch.cat([xy - wh / 2, xy + wh / 2], dim=-1)
        objectness = x[:, :, 4:5]
        scores, labels = torch.max(x[:, :, 5:], dim=-1, keepdim=True)
        scores = scores * objectness
        return torch.cat([boxes, scores, labels.to(boxes.dtype)], dim=-1)


# --------------------------------------------------------------------------- #
# Nạp model
# --------------------------------------------------------------------------- #

def load_ultralytics(weights: str):
    from ultralytics import YOLO

    model = YOLO(weights)
    net = model.model.float().eval()
    net = net.fuse()
    for param in net.parameters():
        param.requires_grad = False
    names = [str(net.names[i]) for i in sorted(net.names)]
    return nn.Sequential(net, UltralyticsHead()).eval(), names


def load_yolov5(weights: str, repo: str):
    if not repo or not os.path.isdir(os.path.join(repo, "models")):
        raise SystemExit(
            "Checkpoint yolov5 cần repo ultralytics/yolov5.\n"
            "  --yolov5-repo <path> hoặc env YOLOV5_REPO=<path>\n"
            "  git clone https://github.com/ultralytics/yolov5 scripts/vendor/yolov5"
        )
    sys.path.insert(0, repo)
    # torch >= 2.6 mặc định weights_only=True, checkpoint yolov5 cần False.
    original_load = torch.load
    torch.load = lambda *a, **k: original_load(*a, **{**k, "weights_only": False})
    try:
        from models.experimental import attempt_load
    finally:
        pass

    net = attempt_load(weights, device=torch.device("cpu"))
    net.eval()
    from models.yolo import Detect

    for module in net.modules():
        if isinstance(module, Detect):
            module.inplace = False
            module.dynamic = False
            module.export = False  # cần nhánh inference (đã decode ra pixel)
    for param in net.parameters():
        param.requires_grad = False
    names = [str(net.names[i]) for i in sorted(net.names)]
    torch.load = original_load
    return nn.Sequential(net, YoloV5Head()).eval(), names


# --------------------------------------------------------------------------- #

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", required=True, help="File .pt đầu vào")
    parser.add_argument("--output", help="File .onnx đầu ra (mặc định: cùng tên .onnx)")
    parser.add_argument("--labels", help="File labels đầu ra (mặc định: <output>.labels.txt)")
    parser.add_argument("--imgsz", type=int, default=640, help="Kích thước input mạng")
    parser.add_argument("--max-batch", type=int, default=1, help="Batch tối đa (dynamic axis)")
    parser.add_argument("--opset", type=int, default=17)
    parser.add_argument("--yolov5-repo", default=os.environ.get("YOLOV5_REPO", ""))
    parser.add_argument("--kind", choices=["auto", "ultralytics", "yolov5"], default="auto")
    args = parser.parse_args()

    weights = os.path.abspath(args.weights)
    if not os.path.isfile(weights):
        raise SystemExit(f"Không thấy file weights: {weights}")
    output = os.path.abspath(args.output or os.path.splitext(weights)[0] + ".onnx")
    labels_path = os.path.abspath(args.labels or os.path.splitext(output)[0] + ".labels.txt")

    kind = args.kind if args.kind != "auto" else detect_kind(weights)
    print(f"[export] {os.path.basename(weights)} — loại: {kind}, imgsz={args.imgsz}")

    if kind == "ultralytics":
        model, names = load_ultralytics(weights)
    else:
        model, names = load_yolov5(weights, args.yolov5_repo)

    print(f"[export] {len(names)} class: {names if len(names) <= 12 else names[:12] + ['…']}")

    dummy = torch.zeros(1, 3, args.imgsz, args.imgsz)
    with torch.no_grad():
        sample = model(dummy)
    print(f"[export] output shape: {tuple(sample.shape)} (kỳ vọng [b, n, 6])")
    if sample.shape[-1] != 6:
        raise SystemExit("Layout output không đúng — parser cần 6 cột")

    dynamic_axes = {"input": {0: "batch"}, "output": {0: "batch"}}
    export_kwargs = dict(
        input_names=["input"],
        output_names=["output"],
        dynamic_axes=dynamic_axes if args.max_batch > 1 else None,
        opset_version=args.opset,
        do_constant_folding=True,
    )
    try:
        # torch >= 2.9 mặc định dùng exporter dynamo (cần onnxscript); bản
        # TorchScript vẫn đủ cho graph tĩnh này và ít phụ thuộc hơn.
        torch.onnx.export(model, dummy, output, dynamo=False, **export_kwargs)
    except TypeError:
        torch.onnx.export(model, dummy, output, **export_kwargs)

    try:
        import onnx
        import onnxslim

        onnx.save(onnxslim.slim(onnx.load(output)), output)
        print("[export] đã tối giản graph bằng onnxslim")
    except Exception as exc:  # noqa: BLE001 - slim là tuỳ chọn
        print(f"[export] bỏ qua onnxslim: {exc}")

    with open(labels_path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(names) + "\n")

    print(f"[export] ONNX   → {output}")
    print(f"[export] labels → {labels_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
