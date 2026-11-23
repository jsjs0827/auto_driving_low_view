#!/usr/bin/env python3
"""Batch-convert one perception dataset into diagnostic PNG images.

Usage:
  python3 visualize_saved_data.py /path/perception_output [-test]

-tolerant 模式：目录名、图像格式（jpg/png/bin）与地图格式（bin/png）都尽量兼容；
  单个时间戳出错只记录并跳过，不中断整体流程；异常汇总写入 broken.txt。
"""
from __future__ import annotations

import argparse
import json
import struct
import traceback
import zlib
from pathlib import Path
from typing import Iterable, Optional

import cv2
import numpy as np

DEFAULT_DATASET = "~/perception_output"

RAW_MAGIC = 0x52415731
CLOUD_MASK_MAGIC = 0x44595249
MAP_MAGIC = 0x50414D52
DEFAULT_KEEP_CLASSES = {"person", "cat", "dog", "charger", "toy", "hedgehog"}
# Detection class ID -> AiFusion::map_label grid type.
DETECTION_MAP_LABEL = {
    8: 4, 9: 5, 10: 6, 18: 7, 16: 8, 11: 9,
    0: 10, 1: 11, 2: 12, 3: 13, 4: 14, 5: 15, 6: 16,
    7: 17, 12: 18, 13: 19, 14: 20, 15: 21, 17: 22,
    19: 23, 20: 24, 21: 25, 22: 26, 23: 27, 24: 28,
}

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bin", ".bgr"}

# 目录候选按优先级排列：先新命名，再回退历史命名。
RGB_DIRS = ("rgb", "rgb_raw")
MAP_ROOT_DIRS = ("map", "map_raw")
AI_DET_DIRS = ("ai_det", "ai_overlay")
AI_MASK_DIRS = ("ai_mask", "ai_overlay")
AI_OVERLAY_DIRS = ("ai_overlay", "ai_det")
CLOUD_OVERLAY_DIRS = ("ai_cloud_overlay_raw", "ai_cloud_overlay")


class BrokenCollector:
    """收集异常文件，进程结束统一写 broken.txt。"""

    def __init__(self) -> None:
        self.records: list[str] = []

    def add(self, name: str, reason: str) -> None:
        self.records.append(f"{name}\t{reason}")

    def __len__(self) -> int:
        return len(self.records)

    def write(self, out_dir: Path) -> Optional[Path]:
        if not self.records:
            return None
        out_dir.mkdir(parents=True, exist_ok=True)
        target = out_dir / "broken.txt"
        with target.open("w", encoding="utf-8") as f:
            f.write(f"broken_files={len(self.records)}\n")
            for record in self.records:
                f.write(record + "\n")
        return target


def first_existing(root: Path, names: Iterable[str]) -> Optional[Path]:
    for name in names:
        path = root / name
        if path.is_dir():
            return path
    return None


def resolve_file(root: Path, dir_names: Iterable[str], stem: str,
                 suffixes: Iterable[str] = IMAGE_SUFFIXES) -> Optional[Path]:
    """在候选目录里按时间戳找文件，兼容不同后缀命名。"""
    for name in dir_names:
        folder = root / name
        if not folder.is_dir():
            continue
        for suffix in suffixes:
            path = folder / f"{stem}{suffix}"
            if path.is_file():
                return path
    return None


def read_raw(path: Path) -> tuple[int, int, int, str, bytes]:
    with path.open("rb") as f:
        header = f.read(32)
        if len(header) != 32:
            raise ValueError(f"raw header too short: {path.name}")
        magic, version, width, height, channels, fmt_len, data_size = struct.unpack(
            "<6IQ", header
        )
        if magic != RAW_MAGIC or version not in {1, 2, 3}:
            raise ValueError(f"unsupported raw header: {path.name}")
        fmt_raw = f.read(fmt_len)
        if len(fmt_raw) != fmt_len:
            raise ValueError(f"truncated raw format string: {path.name}")
        fmt = fmt_raw.decode("utf-8", errors="replace")
        data = f.read(data_size)
        if len(data) != data_size:
            raise ValueError(f"truncated raw payload: {path.name}")
        if version == 2:
            if not fmt.endswith("_zlib"):
                raise ValueError(f"unsupported compressed format: {path.name}")
            try:
                data = zlib.decompress(data)
            except zlib.error as exc:
                raise ValueError(f"invalid zlib payload: {path.name}") from exc
            fmt = fmt[:-5]
        # RLE mask 解压后仍是行程记录，长度不等于 width*height；
        # JPEG/PNG 是编码后字节，长度同样不等于 width*height*channels。
        # 这两类只校验实际读取到的字节数是否完整，不做像素长度换算。
        if not fmt.startswith("mask_rle_") and fmt.upper() not in {"JPEG", "JPG", "PNG"}:
            if width > 0 and height > 0 and len(data) != width * height * channels:
                raise ValueError(
                    f"payload size mismatch: {path.name} "
                    f"got={len(data)} expect={width * height * channels}"
                )
        return width, height, channels, fmt, data


def decode_image_payload(width: int, height: int, channels: int, fmt: str,
                         data: bytes) -> np.ndarray:
    """把 BIN 内图像负载解成 RGB ndarray，兼容 JPEG 与裸 RGB。"""
    fmt_upper = fmt.upper()
    if fmt_upper in {"JPEG", "JPG"} or data[:2] == b"\xff\xd8":
        decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if decoded is None:
            raise ValueError("invalid JPEG payload")
        return cv2.cvtColor(decoded, cv2.COLOR_BGR2RGB)
    if fmt_upper in {"PNG"} or data[:8] == b"\x89PNG\r\n\x1a\n":
        decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if decoded is None:
            raise ValueError("invalid PNG payload")
        return cv2.cvtColor(decoded, cv2.COLOR_BGR2RGB)
    if fmt_upper in {"BGR24"}:
        channels = 3
    elif fmt_upper in {"RGB24", "RGB8", "RGB8_OVERLAY"}:
        channels = 3
    elif fmt_upper in {"GRAY8", "MASK_U8"}:
        channels = 1
    elif width <= 0 or height <= 0:
        raise ValueError(f"cannot infer shape for format {fmt}")
    expected = width * height * channels
    if len(data) != expected:
        raise ValueError(f"image payload mismatch: got={len(data)} expect={expected}")
    array = np.frombuffer(data, np.uint8)
    if channels == 1:
        return array.reshape(height, width)
    return array.reshape(height, width, channels).copy()


def read_map_raw(path: Path) -> tuple[int, int, float, float, float, float, float, list[tuple[float, float, float, int]]]:
    with path.open("rb") as f:
        header = f.read(44)
        if len(header) != 44:
            raise ValueError(f"map header too short: {path.name}")
        magic, version, _reserved, rows, cols, resolution, width, height, _ts, count = \
            struct.unpack("<IHHIIfffQQ", header)
        camera_x = camera_y = 0.0
        if version == 4:
            extrinsics = f.read(8)
            if len(extrinsics) != 8:
                raise ValueError(f"map extrinsics missing: {path.name}")
            camera_x, camera_y = struct.unpack("<ff", extrinsics)
        if magic != MAP_MAGIC or count != rows * cols:
            raise ValueError(f"unsupported map file: {path.name}")
        if version == 1:
            payload = f.read(count * 16)
            if len(payload) != count * 16:
                raise ValueError(f"map payload mismatch: {path.name}")
            points = list(struct.iter_unpack("<fffi", payload))
        elif version in {2, 3, 4}:
            size_raw = f.read(8)
            if len(size_raw) != 8:
                raise ValueError(f"map payload header missing: {path.name}")
            payload_size = struct.unpack("<Q", size_raw)[0]
            payload = f.read(payload_size)
            if len(payload) != payload_size:
                raise ValueError(f"truncated map payload: {path.name}")
            try:
                payload = zlib.decompress(payload)
            except zlib.error as exc:
                raise ValueError(f"invalid map zlib payload: {path.name}") from exc
            expect = count * 16 if version == 2 else count
            if len(payload) != expect:
                raise ValueError(f"map payload mismatch: {path.name}")
            if version == 2:
                points = list(struct.iter_unpack("<fffi", payload))
            else:
                points = [(0.0, 0.0, 0.0, t) for t, in struct.iter_unpack("<b", payload)]
        else:
            raise ValueError(f"unsupported map version {version}: {path.name}")
        return rows, cols, resolution, width, height, camera_x, camera_y, points


def load_existing_map_image(path: Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"failed to read map image: {path.name}")
    return image


def draw_maps(root: Path, timestamp: str, out_dir: Path,
              map_types: tuple[str, ...] = ("pcl", "ai", "fusion")) -> None:
    """优先渲染 BIN 地图；目录里只有 PNG 时直接复用，不做栅格重建。"""
    for map_type in map_types:
        target = out_dir / "map" / map_type / f"{timestamp}.png"
        bin_path = resolve_file(root, [f"{d}/{map_type}" for d in MAP_ROOT_DIRS],
                                timestamp, suffixes=(".bin",))
        if bin_path is not None:
            rows, cols, resolution, _w, _h, camera_x, camera_y, points = read_map_raw(bin_path)
            colors = MAP_COLORS
            image = np.full((rows, cols, 3), 255, dtype=np.uint8)
            for i, point in enumerate(points):
                row, col = divmod(i, cols)
                if 0 <= row < rows and 0 <= col < cols:
                    image[rows - row - 1, col] = colors.get(point[3], (255, 255, 255))
            axis_length = 0.1
            def base_to_pixel(base_x: float, base_y: float) -> tuple[int, int]:
                return (int((-base_y) / resolution + cols / 2),
                        int((-base_x) / resolution + rows))
            origin = base_to_pixel(0.0, 0.0)
            x_end = base_to_pixel(axis_length, 0.0)
            y_end = base_to_pixel(0.0, axis_length)
            camera_origin = base_to_pixel(camera_x, camera_y)
            camera_x_end = base_to_pixel(camera_x + axis_length, camera_y)
            camera_y_end = base_to_pixel(camera_x, camera_y + axis_length)
            cv2.arrowedLine(image, origin, x_end, (0, 0, 255), 1)
            cv2.arrowedLine(image, origin, y_end, (0, 255, 0), 1)
            cv2.circle(image, origin, 1, (255, 0, 0), -1)
            cv2.arrowedLine(image, camera_origin, camera_x_end, (0, 0, 255), 1)
            cv2.arrowedLine(image, camera_origin, camera_y_end, (0, 255, 0), 1)
            cv2.circle(image, camera_origin, 1, (255, 0, 0), -1)
            image = upscale_grid_image(image)
            target.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(target), image, [cv2.IMWRITE_PNG_COMPRESSION, 9])
            continue
        existing = resolve_file(root, [f"{d}/{map_type}" for d in MAP_ROOT_DIRS],
                                timestamp, suffixes=(".png", ".jpg"))
        if existing is not None:
            target.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(target), load_existing_map_image(existing),
                        [cv2.IMWRITE_PNG_COMPRESSION, 9])


MAP_COLORS = {
    -1: (255, 0, 0), 0: (0, 255, 255), 1: (0, 0, 0),
    2: (0, 0, 255), 3: (0, 255, 0), 4: (255, 0, 255),
    5: (255, 255, 0), 6: (255, 255, 255), 8: (0, 140, 255),
}


def upscale_grid_image(image: np.ndarray, scale: int = 8) -> np.ndarray:
    """栅格图放大并画网格，保持与旧在线存档一致的观感。"""
    rows, cols = image.shape[:2]
    image = cv2.resize(image, (cols * scale, rows * scale),
                       interpolation=cv2.INTER_NEAREST)
    grid_color = (230, 230, 230)
    step = scale * 2
    for x in range(0, image.shape[1], step):
        cv2.line(image, (x, 0), (x, image.shape[0] - 1), grid_color, 1)
    for y in range(0, image.shape[0], step):
        cv2.line(image, (0, y), (image.shape[1] - 1, y), grid_color, 1)
    return image


def read_cloud_classes(path: Path) -> tuple[int, int, int, bytes]:
    with path.open("rb") as f:
        header = f.read(36)
        if len(header) != 36:
            raise ValueError(f"cloud overlay header too short: {path.name}")
        magic, version, fmt, width, height, stride, _timestamp, size = struct.unpack(
            "<IHHIIIQQ", header
        )
        if magic != CLOUD_MASK_MAGIC or version != 1 or fmt not in {6, 7}:
            raise ValueError(f"unsupported cloud overlay: {path.name}")
        data = f.read(size)
        if len(data) != size:
            raise ValueError(f"cloud overlay truncated: {path.name}")
        return fmt, width, height, data


def read_disp_uv(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with path.open("rb") as f:
        header = f.read(8)
        if len(header) != 8:
            raise ValueError(f"disp header too short: {path.name}")
        point_count = struct.unpack("<Q", header)[0]
        if point_count == 0:
            raise ValueError("disp has zero points")
        if path.stat().st_size < 8 + point_count * 12:
            raise ValueError(f"disp truncated: {path.name}")
        u = np.frombuffer(f.read(point_count * 4), dtype="<u4").copy()
        v = np.frombuffer(f.read(point_count * 4), dtype="<u4").copy()
        if u.size != point_count or v.size != point_count:
            raise ValueError(f"disp uv payload mismatch: {path.name}")
        return u, v


def mask_payload_to_pixels(fmt: str, data: bytes, width: int, height: int) -> np.ndarray:
    if fmt in {"mask_u8", "mask"}:
        return np.frombuffer(data, dtype=np.uint8)
    if fmt.startswith("mask_rle_"):
        raw = np.frombuffer(data, dtype=np.uint8)
        if raw.size % 3 != 0:
            raise ValueError("invalid RLE mask payload length")
        values = raw[0::3].astype(np.uint32)
        counts = raw[1::3].astype(np.uint32) | (raw[2::3].astype(np.uint32) << 8)
        return np.repeat(values, counts).astype(np.uint8)
    raise ValueError(f"unsupported segmentation format: {fmt}")


def load_keep_classes(root: Path) -> set[str]:
    candidates = [root / "config" / "start_config.json", root / "start_config.json"]
    for path in candidates:
        if not path.exists():
            continue
        try:
            doc = json.loads(path.read_text())
            value = doc.get("dnn", {}).get("detect", {}).get("keep_classes", "")
            if isinstance(value, str) and value.strip():
                return {item.strip() for item in value.split(",") if item.strip()}
        except (OSError, json.JSONDecodeError, AttributeError, TypeError):
            continue
    return DEFAULT_KEEP_CLASSES


def selected_detections(meta: dict, keep_classes: set[str]) -> list[dict]:
    detections = meta.get("detections")
    if not isinstance(detections, list):
        return []
    result = []
    for det in detections:
        if not isinstance(det, dict):
            continue
        try:
            float(det["x1"]), float(det["y1"]), float(det["x2"]), float(det["y2"])
        except (KeyError, TypeError, ValueError):
            continue
        if (det.get("class_name") or "") in keep_classes:
            result.append(det)
    return result


def load_rgb(root: Path, timestamp: str) -> Optional[np.ndarray]:
    path = resolve_file(root, [f"data/{d}" for d in RGB_DIRS], timestamp)
    if path is None:
        return None
    suffix = path.suffix.lower()
    if suffix in {".jpg", ".jpeg", ".png"}:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"failed to read {path.name}")
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    width, height, channels, fmt, data = read_raw(path)
    if fmt.upper() in {"JPEG", "JPG", "PNG"}:
        return decode_image_payload(width, height, channels, fmt, data)
    image = decode_image_payload(width, height, channels, fmt, data)
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return cv2.cvtColor(image[:, :, :3], cv2.COLOR_RGB2BGR)


def load_rendered_overlay(root: Path, timestamp: str) -> Optional[np.ndarray]:
    """板端已渲染 overlay 快照，兼容 bin 内 JPEG 与裸 RGB。"""
    path = resolve_file(root, [f"data/{d}" for d in AI_OVERLAY_DIRS],
                        timestamp, suffixes=(".bin",))
    if path is None:
        return None
    width, height, channels, fmt, data = read_raw(path)
    image = decode_image_payload(width, height, channels, fmt, data)
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return cv2.cvtColor(image[:, :, :3], cv2.COLOR_RGB2BGR)


def load_ai_meta(root: Path, timestamp: str) -> tuple[Path, dict]:
    path = resolve_file(root, [f"data/{d}" for d in AI_DET_DIRS], timestamp,
                        suffixes=(".json",))
    if path is None:
        raise FileNotFoundError(f"missing AI meta json for {timestamp}")
    try:
        meta = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid AI meta json: {path.name}") from exc
    if not isinstance(meta, dict):
        raise ValueError(f"unexpected AI meta json: {path.name}")
    return path, meta


def draw_ai_overlay(root: Path, timestamp: str, out: Path) -> None:
    rendered = load_rendered_overlay(root, timestamp)
    if rendered is not None:
        out.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out), cv2.cvtColor(rendered, cv2.COLOR_RGB2BGR),
                    [cv2.IMWRITE_JPEG_QUALITY, 95])
        return
    rgb = load_rgb(root, timestamp)
    if rgb is None:
        raise FileNotFoundError(f"RGB is required for AI overlay: {timestamp}")
    meta_path, meta = load_ai_meta(root, timestamp)
    image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    width = int(meta.get("width") or 0)
    height = int(meta.get("height") or 0)
    mask_file = meta.get("seg_mask_file")
    if not mask_file:
        mask = np.asarray(meta.get("seg_mask", []), dtype=np.uint8)
    else:
        mask_path = resolve_file(root, [f"data/{d}" for d in AI_MASK_DIRS],
                                 mask_file, suffixes=("", ".bin"))
        if mask_path is None:
            mask_path = meta_path.parent / mask_file
        if not mask_path.exists():
            mask = np.empty(0, dtype=np.uint8)
        else:
            if mask_path.suffix.lower() in {".jpg", ".png"}:
                mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
            elif mask_path.suffix.lower() == ".bin":
                m_w, m_h, m_c, fmt, data = read_raw(mask_path)
                if m_c != 1:
                    raise ValueError(f"expected 1-channel mask: {mask_path.name}")
                mask = mask_payload_to_pixels(fmt, data, m_w, m_h)
            else:
                mask = np.fromfile(str(mask_path), dtype=np.uint8)
            if width and height and (mask.size != width * height):
                raise ValueError(f"mask size does not match JSON: {mask_path.name}")
    if mask.size == width * height and width > 0 and height > 0:
        mask = mask.reshape(height, width)
        # Segmentation IDs follow dnn.seg.class_names: 1=grass, 2=non-grass.
        colors = np.array([[0, 0, 0], [0, 180, 0], [0, 0, 255], [255, 0, 0]], np.uint8)
        colored = colors[np.minimum(mask, len(colors) - 1)]
        valid = mask != 0
        image[valid] = (0.55 * image[valid] + 0.45 * colored[valid]).astype(np.uint8)
    sx = image.shape[1] / max(width, 1)
    sy = image.shape[0] / max(height, 1)
    for det in selected_detections(meta, load_keep_classes(root)):
        p1 = (round(float(det["x1"]) * sx), round(float(det["y1"]) * sy))
        p2 = (round(float(det["x2"]) * sx), round(float(det["y2"]) * sy))
        cv2.rectangle(image, p1, p2, (0, 0, 255), 2)
        class_name = det.get("class_name") or f'id={det.get("class_id", -1)}'
        text = f'{class_name} {float(det.get("confidence", 0)):.2f}'
        cv2.putText(image, text, (p1[0], max(15, p1[1] - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), image)


def draw_cloud_mask(root: Path, timestamp: str, out: Path, point_radius: int) -> None:
    rgb = load_rgb(root, timestamp)
    if rgb is None:
        raise FileNotFoundError(f"RGB is required for cloud overlay: {timestamp}")
    mask_path = resolve_file(root, [f"data/{d}" for d in CLOUD_OVERLAY_DIRS],
                             timestamp, suffixes=(".bin",))
    if mask_path is None:
        raise FileNotFoundError(f"missing cloud overlay for {timestamp}")
    fmt, width, height, payload = read_cloud_classes(mask_path)
    classes = np.frombuffer(payload, np.uint8)
    if fmt == 6:
        if classes.size != width * height:
            raise ValueError(f"dense cloud mask size mismatch: {mask_path.name}")
        mask = classes.reshape(height, width)
    else:
        disp_path = resolve_file(root, ["data/disp"], timestamp, suffixes=(".bin",))
        if disp_path is None:
            raise FileNotFoundError(f"disp is required for cloud overlay: {timestamp}")
        u, v = read_disp_uv(disp_path)
        if classes.size != u.size:
            raise ValueError(
                f"cloud class count {classes.size} != disp point count {u.size}"
            )
        mask = np.zeros((height, width), dtype=np.uint8)
        valid = (u < width) & (v < height) & (classes != 0)
        # Detection obstacles (1) take priority if multiple points share a pixel.
        mask[v[valid & (classes == 2)], u[valid & (classes == 2)]] = 2
        mask[v[valid & (classes == 1)], u[valid & (classes == 1)]] = 1
        try:
            _meta_path, meta = load_ai_meta(root, timestamp)
        except (FileNotFoundError, ValueError):
            meta = {}
        det_u, det_v = u[valid], v[valid]
        for det in selected_detections(meta, load_keep_classes(root)):
            x1, y1 = float(det["x1"]), float(det["y1"])
            x2, y2 = float(det["x2"]), float(det["y2"])
            inside = ((det_u >= x1) & (det_u <= x2) &
                      (det_v >= y1) & (det_v <= y2))
            label = DETECTION_MAP_LABEL.get(int(det.get("class_id", -1)), 2)
            mask[det_v[inside], det_u[inside]] = label
    image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if (width, height) != (image.shape[1], image.shape[0]):
        mask = cv2.resize(mask, (image.shape[1], image.shape[0]),
                          interpolation=cv2.INTER_NEAREST)
    if point_radius > 0:
        kernel_size = point_radius * 2 + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        original = mask.copy()
        expanded = np.zeros_like(mask)
        for label in np.unique(original):
            if label == 0:
                continue
            pixels = cv2.dilate((original == label).astype(np.uint8), kernel)
            expanded[pixels != 0] = label
        # Detection labels (4+) override segmentation labels (1/2).
        for label in sorted(x for x in np.unique(original) if x >= 4):
            pixels = cv2.dilate((original == label).astype(np.uint8), kernel)
            expanded[pixels != 0] = label
        mask = expanded
    colors = np.zeros((29, 3), np.uint8)
    colors[1] = (0, 0, 255)
    colors[2] = (255, 0, 0)
    palette = [(255, 0, 255), (0, 165, 255), (255, 0, 0),
               (0, 255, 255), (255, 128, 0), (128, 0, 255)]
    for label in range(4, 29):
        colors[label] = palette[(label - 4) % len(palette)]
    colored = colors[np.minimum(mask, len(colors) - 1)]
    valid = mask != 0
    image[valid] = colored[valid]
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), image)


def render_one(root: Path, timestamp: str, out_dir: Path,
               test_mode: bool = False) -> list[tuple[str, str]]:
    """渲染单个时间戳，返回 [(相对路径, 异常原因)]。"""
    broken: list[tuple[str, str]] = []

    def step(name: str, func) -> None:
        try:
            func()
        except FileNotFoundError:
            # 数据缺失属于正常情况（例如未开对应开关），不记为异常。
            pass
        except Exception as exc:  # noqa: BLE001 单帧异常不影响整体流程
            broken.append((f"{name}/{timestamp}", f"{type(exc).__name__}: {exc}"))

    step("rgb", lambda: _render_rgb(root, timestamp, out_dir))
    if test_mode:
        step("map", lambda: draw_maps(root, timestamp, out_dir, map_types=("fusion",)))
        return broken
    step("ai_overlay", lambda: draw_ai_overlay(
        root, timestamp, out_dir / "ai_overlay" / f"{timestamp}.jpg"))
    step("ai_cloud_overlay", lambda: draw_cloud_mask(
        root, timestamp, out_dir / "ai_cloud_overlay" / f"{timestamp}.jpg",
        point_radius=2))
    step("map", lambda: draw_maps(root, timestamp, out_dir))
    return broken


def _render_rgb(root: Path, timestamp: str, out_dir: Path) -> None:
    rgb = load_rgb(root, timestamp)
    if rgb is None:
        raise FileNotFoundError(f"missing RGB for {timestamp}")
    rgb_path = out_dir / "rgb" / f"{timestamp}.jpg"
    rgb_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(rgb_path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                [cv2.IMWRITE_JPEG_QUALITY, 95])


def timestamp_of(path: Path) -> str:
    """从文件名提取时间戳，保留毫秒中的点。

    时间戳形如 `2026-10-20 01:36:07.201`，本身含点，
    不能用 split(".")[0]，否则毫秒被截断成伪造的时间戳。
    """
    stem = path.stem  # 仅去掉最后一层后缀
    # 兼容历史命名 `<ts>.mask.bin`。
    if stem.lower().endswith(".mask"):
        stem = stem[: -len(".mask")]
    return stem


def collect_timestamps(root: Path) -> list[str]:
    """取所有数据目录时间戳的并集。

    不能只以 RGB 目录为准：手动删过 RGB 后，仅剩地图的时间戳也要渲染，
    否则 map 数量会少于原始数据数量。
    """
    stems: set[str] = set()
    for name in RGB_DIRS:
        folder = root / "data" / name
        if not folder.is_dir():
            continue
        for path in folder.iterdir():
            if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES:
                stems.add(timestamp_of(path))
    for name in CLOUD_OVERLAY_DIRS + AI_DET_DIRS + AI_MASK_DIRS + ("disp",):
        folder = root / "data" / name
        if not folder.is_dir():
            continue
        for path in folder.iterdir():
            if path.is_file():
                stems.add(timestamp_of(path))
    for map_root in MAP_ROOT_DIRS:
        folder = root / map_root
        if not folder.is_dir():
            continue
        for map_dir in folder.iterdir():
            if not map_dir.is_dir():
                continue
            for path in map_dir.iterdir():
                if path.is_file() and path.suffix.lower() in {".bin", ".png", ".jpg"}:
                    stems.add(timestamp_of(path))
    return sorted(stem for stem in stems if stem)


def count_stems(folder: Path, suffixes: Optional[set[str]] = None) -> int:
    if not folder.is_dir():
        return 0
    stems = set()
    for path in folder.iterdir():
        if not path.is_file():
            continue
        if suffixes is not None and path.suffix.lower() not in suffixes:
            continue
        stems.add(timestamp_of(path))
    return len(stems)


def describe_sources(root: Path) -> str:
    """打印各数据源时间戳数量，便于核对进度总数。"""
    parts = []
    for name in RGB_DIRS:
        count = count_stems(root / "data" / name, IMAGE_SUFFIXES)
        if count:
            parts.append(f"rgb[{name}]={count}")
    for name in CLOUD_OVERLAY_DIRS + AI_DET_DIRS + ("disp",):
        count = count_stems(root / "data" / name)
        if count:
            parts.append(f"{name}={count}")
    for map_root in MAP_ROOT_DIRS:
        folder = root / map_root
        if not folder.is_dir():
            continue
        for map_dir in folder.iterdir():
            if map_dir.is_dir():
                count = count_stems(map_dir, {".bin", ".png", ".jpg"})
                if count:
                    parts.append(f"{map_root}/{map_dir.name}={count}")
    return ", ".join(parts) if parts else "none"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Batch-convert one perception dataset into visualized images"
    )
    parser.add_argument(
        "dataset",
        nargs="?",
        default=DEFAULT_DATASET,
        type=Path,
        help="perception_output dataset path (default: ~/perception_output)"
    )
    parser.add_argument(
        "-test",
        action="store_true",
        help="quick preview: visualize only RGB and fusion map"
    )

    args = parser.parse_args()
    root = args.dataset.expanduser()
    if not root.is_dir():
        parser.error(f"dataset not found: {root}")
    out_dir = root / "visualized"

    timestamps = collect_timestamps(root)
    if not timestamps:
        parser.error(f"no timestamped files found under {root}")

    print(f"unique timestamps: {len(timestamps)} ({describe_sources(root)})")

    collector = BrokenCollector()
    for index, timestamp in enumerate(timestamps, start=1):
        print(f"[{index}/{len(timestamps)}] rendering {timestamp}")
        for name, reason in render_one(root, timestamp, out_dir, test_mode=args.test):
            collector.add(name, reason)
            print(f"  skip {name}: {reason}")

    target = collector.write(root)
    if target is not None:
        print(f"broken files: {len(collector)} -> {target}")
    else:
        print("broken files: 0")
    rendered = {
        "rgb": count_stems(out_dir / "rgb"),
    }
    for name in ("pcl", "ai", "fusion"):
        rendered[name] = count_stems(out_dir / "map" / name)
    rendered["ai_overlay"] = count_stems(out_dir / "ai_overlay")
    rendered["ai_cloud_overlay"] = count_stems(out_dir / "ai_cloud_overlay")
    print("visualized: " + ", ".join(f"{k}={v}" for k, v in rendered.items() if v))


if __name__ == "__main__":
    main()
