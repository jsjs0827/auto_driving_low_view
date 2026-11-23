
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
从 [GM] 单行稀疏栅格日志还原类别地图 PNG。

支持日志格式：
[2000-01-01 09:00:20.365][info][common.hpp:110]
[GM] f=232 r=100 c=80 s=50 k=3 d=7944,1,-50;1,2,24,15,31;2,3,10
f:帧 ID。
r、c:行数、列数。
s:分辨率，单位 mm。
k:总栅格数。
d:非未知栅格列表。
每个分号分隔一条栅格记录，格式为 索引差,类别,平均高度,最低高度,最高高度。
实际日志必须是一整行。

输出：
    2000-01-01 09:00:20.365.png

功能：
1. 只恢复 type，不生成高度图或 CSV。
2. 未记录的栅格默认为未知类别 0。
3. type=255 恢复为坡道 -1。
4. 默认每个栅格放大 8 倍。
5. 每隔 2 个栅格绘制网格线。
6. 绘制车体中心坐标系和相机坐标系。
7. 优先读取日志中的 x_cali_m、y_cali_m；
   如果日志没有这两个字段，可以通过命令行指定。
"""
import shutil
import argparse
import re
from pathlib import Path

import cv2
import numpy as np


INPUT_DIR = "~/ai_node"
OUTPUT_DIR = INPUT_DIR + "/fusion_map"

# 日志行开头的时间戳
TIMESTAMP_RE = re.compile(
    r"^\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\]"
)

# [GM] 主记录
GM_RE = re.compile(
    r"\[GM\]\s+"
    r"f=(\d+)\s+"
    r"r=(\d+)\s+"
    r"c=(\d+)\s+"
    r"s=(\d+)\s+"
    r"k=(\d+)\s+"
    r"(?:x_cali_m=(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s+)?"
    r"(?:y_cali_m=(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s+)?"
    r"d=(.*)"
)

def find_log_files(log_dir):
    """查找 ai_node.log、ai_node.1.log、ai_node.2.log 等文件。"""
    log_dir = Path(log_dir).expanduser()

    if not log_dir.is_dir():
        raise NotADirectoryError(f"日志目录不存在: {log_dir}")

    pattern = re.compile(r"^ai_node(?:\.\d+)?\.log$")

    files = [
        p for p in log_dir.iterdir()
        if p.is_file() and pattern.fullmatch(p.name)
    ]

    # 按日志轮转编号排序：
    # ai_node.log -> ai_node.1.log -> ai_node.2.log ...
    def sort_key(path):
        match = re.fullmatch(r"ai_node(?:\.(\d+))?\.log", path.name)
        if match.group(1) is None:
            return -1
        return int(match.group(1))

    files.sort(key=sort_key)
    return files

def decode_type(data_text, rows, cols, saved_count):
    """解析稀疏文本，仅恢复 type 数组。"""
    total = rows * cols

    # 其余未记录位置都是未知栅格
    type_flat = np.zeros(total, dtype=np.uint8)

    if saved_count == 0:
        if data_text.strip():
            raise ValueError("k=0，但 d 字段不为空")
        return type_flat

    if not data_text.strip():
        raise ValueError("k 大于 0，但 d 字段为空")

    records = data_text.strip().split(";")

    if len(records) != saved_count:
        raise ValueError(
            f"声明 k={saved_count}，实际记录数={len(records)}"
        )

    prev_idx = 0

    for record_no, record in enumerate(records):
        parts = record.split(",")

        # 允许：
        # delta,type,height
        # delta,type,avg,min,max
        if len(parts) not in (3, 5):
            raise ValueError(
                f"第 {record_no + 1} 条记录格式错误: {record}"
            )

        try:
            values = [int(v) for v in parts]
        except ValueError as exc:
            raise ValueError(
                f"第 {record_no + 1} 条记录包含非法整数: {record}"
            ) from exc

        delta, typ = values[0], values[1]

        if delta < 0:
            raise ValueError(f"索引差为负数: {record}")

        # 第一条记录保存绝对索引，之后保存索引差
        idx = delta if record_no == 0 else prev_idx + delta

        if idx >= total:
            raise ValueError(
                f"栅格索引越界: idx={idx}, total={total}"
            )

        if record_no > 0 and delta == 0:
            raise ValueError(f"索引重复或未递增: {record}")

        if typ == 0:
            raise ValueError(
                f"稀疏日志不应包含 type=0: idx={idx}"
            )

        if not 0 <= typ <= 255:
            raise ValueError(f"type 超出 uint8 范围: {typ}")

        type_flat[idx] = typ
        prev_idx = idx

    return type_flat


def to_image_orientation(type_grid):
    """
    坐标约定：
      原点：地图底边中心
      x 正方向：朝前、图像向上
      y 正方向：向左、图像向左

    当前假定原始数组的行列方向与此前脚本一致，
    因此对行和列均进行翻转。

    如果实际地图索引方向不同，只需要调整这里。
    """
    return np.flip(type_grid, axis=0).copy()
    


def base_to_pixel(x_m, y_m, rows, cols, resolution_m, scale):
    """
    将车体坐标转换为放大后的图像像素坐标。

    x_m：向前为正，单位 m
    y_m：向左为正，单位 m
    """
    # 图像左侧为 y 正方向，因此 y 增大时像素 x 减小
    px = cols * scale / 2.0 - y_m / resolution_m * scale

    # 图像上方为 x 正方向，因此 x 增大时像素 y 减小
    py = rows * scale - 1 - x_m / resolution_m * scale

    return int(round(px)), int(round(py))


def draw_arrow(image, start, end, color, thickness=2):
    """绘制坐标轴箭头。"""
    cv2.arrowedLine(
        image,
        start,
        end,
        color,
        thickness,
        line_type=cv2.LINE_AA,
        tipLength=0.25,
    )


def render_type_map(
    type_flat,
    rows,
    cols,
    resolution_mm,
    scale=8,
    grid_interval=2,
    camera_x_m=0.0,
    camera_y_m=0.0,
    draw_camera=True,
):
    """仅生成类别地图 PNG。"""
    resolution_m = resolution_mm / 1000.0

    if resolution_m <= 0:
        raise ValueError("地图分辨率必须大于 0")

    # 先恢复原始二维数组
    type_grid = type_flat.reshape(rows, cols)

    # 转换为图像方向
    grid = to_image_orientation(type_grid).astype(np.int32)

    # OpenCV 颜色使用 BGR
    colors = {
        -1: (255, 0, 0),       # 坡道：蓝色
         0: (0, 255, 255),     # 未知：黄色
         1: (0, 0, 0),         # 地面/可通行：黑色
         2: (0, 0, 255),       # 障碍物：红色
         3: (0, 255, 0),       # 草：绿色
         4: (255, 0, 255),     # 人：品红
         5: (255, 255, 0),     # 猫：青色
         6: (255, 255, 255),   # 狗：白色
         7: (128, 128, 128),   # 充电站：灰色
         8: (0, 165, 255),     # 玩具：橙色
    }

    # 先创建每个栅格 1 像素的原图
    image = np.zeros((rows, cols, 3), dtype=np.uint8)
    image[:] = colors[0]

    # uint8 的 255 表示原始类别 -1（坡道）
    grid[grid == 255] = -1

    for label, color in colors.items():
        image[grid == label] = color

    # 每个栅格放大 scale 倍
    image = cv2.resize(
        image,
        (cols * scale, rows * scale),
        interpolation=cv2.INTER_NEAREST,
    )

    # 网格线间隔为 grid_interval 个原始栅格
    grid_color = (230, 230, 230)
    line_width = 1
    step_px = scale * grid_interval

    for x in range(0, cols * scale, step_px):
        cv2.line(
            image,
            (x, 0),
            (x, rows * scale - 1),
            grid_color,
            line_width,
        )

    for y in range(0, rows * scale, step_px):
        cv2.line(
            image,
            (0, y),
            (cols * scale - 1, y),
            grid_color,
            line_width,
        )

    # 坐标轴长度 0.1 m
    axis_length_m = 0.1

    # 车体中心坐标系原点：地图底边中心
    base_origin = base_to_pixel(
        0.0, 0.0, rows, cols, resolution_m, scale
    )

    x_end = base_to_pixel(
        axis_length_m, 0.0,
        rows, cols, resolution_m, scale
    )
    y_end = base_to_pixel(
        0.0, axis_length_m,
        rows, cols, resolution_m, scale
    )

    # 车体坐标系：X 红色，Y 绿色，原点蓝色
    draw_arrow(image, base_origin, x_end, (0, 0, 255))
    draw_arrow(image, base_origin, y_end, (0, 255, 0))

    cv2.circle(image, base_origin, 3, (255, 0, 0), -1)

    # 相机坐标系：位置取自日志或命令行参数
    if draw_camera:
        camera_origin = base_to_pixel(
            camera_x_m,
            camera_y_m,
            rows,
            cols,
            resolution_m,
            scale,
        )

        camera_x_end = base_to_pixel(
            camera_x_m + axis_length_m,
            camera_y_m,
            rows,
            cols,
            resolution_m,
            scale,
        )

        camera_y_end = base_to_pixel(
            camera_x_m,
            camera_y_m + axis_length_m,
            rows,
            cols,
            resolution_m,
            scale,
        )

        # 相机坐标轴：X 橙红色，Y 青绿色
        draw_arrow(
            image, camera_origin, camera_x_end, (0, 128, 255)
        )
        draw_arrow(
            image, camera_origin, camera_y_end, (255, 255, 0)
        )

        cv2.circle(image, camera_origin, 3, (255, 0, 0), -1)

    return image


def main():
    parser = argparse.ArgumentParser(
        description="解析目录下所有 ai_node 日志并还原类别地图 PNG"
    )

    parser.add_argument(
        "log_dir",
        nargs="?",
        default=INPUT_DIR,
        help="日志目录",
    )
    parser.add_argument(
        "-o", "--output",
        default=OUTPUT_DIR,
        help="PNG 输出目录",
    )
    parser.add_argument(
        "--scale",
        type=int,
        default=8,
        help="每个栅格放大倍数，默认 8",
    )
    parser.add_argument(
        "--grid-interval",
        type=int,
        default=2,
        help="网格线间隔的原始栅格数，默认 2",
    )
    parser.add_argument(
        "--camera-x-m",
        type=float,
        default=0.45,
        help="相机原点相对车体原点的 x 坐标，单位 m",
    )
    parser.add_argument(
        "--camera-y-m",
        type=float,
        default=0.0,
        help="相机原点相对车体原点的 y 坐标，单位 m",
    )
    parser.add_argument(
        "--no-camera",
        action="store_true",
        help="不绘制相机坐标系",
    )

    args = parser.parse_args()

    if args.scale < 1:
        parser.error("--scale 必须大于等于 1")

    if args.grid_interval < 1:
        parser.error("--grid-interval 必须大于等于 1")

    try:
        log_files = find_log_files(args.log_dir)
    except Exception as exc:
        parser.error(str(exc))

    if not log_files:
        parser.error(
            f"目录下没有找到 ai_node.log 或 ai_node.N.log: "
            f"{Path(args.log_dir).expanduser()}"
        )

    
    output_dir = Path(args.output).expanduser()
    # 删除旧输出目录及其中所有内容
    if output_dir.exists():
        shutil.rmtree(output_dir)
        print(f"已删除旧输出目录: {output_dir}")

    # 重新创建输出目录
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"已创建输出目录: {output_dir}")

    print(f"找到 {len(log_files)} 个日志文件：")
    for path in log_files:
        print(f"  {path}")

    total_lines = 0
    matched = 0
    success = 0
    failed = 0

    for log_path in log_files:
        print(f"\n正在解析: {log_path}")

        with log_path.open(
            "r", encoding="utf-8", errors="replace"
        ) as log_file:

            for line_no, line in enumerate(log_file, 1):
                total_lines += 1

                gm_match = GM_RE.search(line)
                if not gm_match:
                    continue

                matched += 1

                try:
                    timestamp_match = TIMESTAMP_RE.search(line)

                    if not timestamp_match:
                        raise ValueError("日志行开头没有合法时间戳")

                    timestamp = timestamp_match.group(1)

                    (
                        frame_id,
                        rows,
                        cols,
                        resolution_mm,
                        saved_count,
                        log_camera_x,
                        log_camera_y,
                        data_text,
                    ) = gm_match.groups()

                    frame_id = int(frame_id)
                    rows = int(rows)
                    cols = int(cols)
                    resolution_mm = int(resolution_mm)
                    saved_count = int(saved_count)

                    if rows <= 0 or cols <= 0:
                        raise ValueError("rows 和 cols 必须大于 0")

                    total = rows * cols

                    if total > 100_000_000:
                        raise ValueError(
                            f"地图尺寸异常：{rows}x{cols}"
                        )

                    if not 0 <= saved_count <= total:
                        raise ValueError(
                            f"k={saved_count} 不在合法范围内"
                        )

                    type_flat = decode_type(
                        data_text, rows, cols, saved_count
                    )

                    # 命令行参数优先；否则读取日志字段。
                    camera_x = (
                        args.camera_x_m
                        if args.camera_x_m is not None
                        else (
                            float(log_camera_x)
                            if log_camera_x is not None
                            else 0.0
                        )
                    )

                    camera_y = (
                        args.camera_y_m
                        if args.camera_y_m is not None
                        else (
                            float(log_camera_y)
                            if log_camera_y is not None
                            else 0.0
                        )
                    )

                    image = render_type_map(
                        type_flat=type_flat,
                        rows=rows,
                        cols=cols,
                        resolution_mm=resolution_mm,
                        scale=args.scale,
                        grid_interval=args.grid_interval,
                        camera_x_m=camera_x,
                        camera_y_m=camera_y,
                        draw_camera=not args.no_camera,
                    )

                    output_path = output_dir / f"{timestamp}.png"

                    if not cv2.imwrite(str(output_path), image):
                        raise RuntimeError(
                            f"PNG 保存失败: {output_path}"
                        )

                    success += 1

                except Exception as exc:
                    failed += 1
                    print(
                        f"[ERROR] 文件={log_path.name} "
                        f"行={line_no}: {exc}"
                    )

    print("\n========== 解析完成 ==========")
    print(f"扫描日志文件数: {len(log_files)}")
    print(f"总日志行数: {total_lines}")
    print(f"匹配地图日志: {matched}")
    print(f"成功生成 PNG: {success}")
    print(f"失败: {failed}")
    print(f"输出目录: {output_dir.resolve()}")


if __name__ == "__main__":
    main()
