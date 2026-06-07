"""Viewer-style visualizer for OpenNI ASCII PLY point clouds.

The bundled GeneratePointCloud.exe stops after 50 frames by design. This script
can optionally run and restart that sample while continuously loading the newest
PLY file into a viewer window.
"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from run_openni_sample import BIN_DIR


POINT_CLOUD_DIR = BIN_DIR / "PointCloud"
GENERATOR_EXE = BIN_DIR / "GeneratePointCloud.exe"


@dataclass
class CloudFrame:
    path: Path
    points: np.ndarray
    colors: np.ndarray
    mtime: float
    raw_count: int
    color_mode: str


def find_ply_files() -> list[Path]:
    search_dirs = [POINT_CLOUD_DIR, BIN_DIR]
    files: list[Path] = []
    for directory in search_dirs:
        if directory.exists():
            files.extend(path for path in directory.rglob("*.ply") if path.is_file())
    return files


def find_latest_ply() -> Path | None:
    candidates = []
    for path in find_ply_files():
        try:
            if path.stat().st_size > 0:
                candidates.append(path)
        except OSError:
            continue
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


def read_ascii_ply(path: Path) -> tuple[np.ndarray, np.ndarray | None]:
    with path.open("r", encoding="utf-8", errors="ignore") as file:
        if file.readline().strip() != "ply":
            raise ValueError(f"{path} is not a PLY file")

        vertex_count: int | None = None
        properties: list[str] = []
        in_vertex_element = False

        for line in file:
            stripped = line.strip()
            if stripped.startswith("element vertex"):
                vertex_count = int(stripped.split()[-1])
                in_vertex_element = True
                continue
            if stripped.startswith("element ") and not stripped.startswith("element vertex"):
                in_vertex_element = False
            if in_vertex_element and stripped.startswith("property"):
                properties.append(stripped.split()[-1])
                continue
            if stripped == "end_header":
                break

        if vertex_count is None:
            raise ValueError("PLY header has no element vertex")
        if vertex_count == 0:
            raise ValueError("PLY has no valid point")

        rows: list[list[float]] = []
        for _ in range(vertex_count):
            line = file.readline()
            if not line:
                break
            values = line.split()
            if len(values) < 3:
                continue
            rows.append([float(value) for value in values[: len(properties)]])

    data = np.asarray(rows, dtype=np.float32)
    if data.shape[0] == 0:
        raise ValueError("PLY has no parseable point")

    property_index = {name: idx for idx, name in enumerate(properties)}
    for required in ("x", "y", "z"):
        if required not in property_index:
            raise ValueError(f"PLY has no {required} property")

    points = data[:, [property_index["x"], property_index["y"], property_index["z"]]]

    color_names = ("red", "green", "blue")
    if all(name in property_index for name in color_names):
        colors = data[:, [property_index[name] for name in color_names]]
        colors = np.clip(colors, 0, 255).astype(np.uint8)
    else:
        colors = None

    return points, colors


def depth_colors(points: np.ndarray) -> np.ndarray:
    z = points[:, 2]
    z_min, z_max = np.percentile(z, [2, 98])
    t = np.clip((z - z_min) / max(z_max - z_min, 1.0), 0.0, 1.0)

    # Viewer-like depth pseudo color: near blue/green, far yellow/red.
    red = np.clip(255 * (1.65 * t - 0.15), 0, 255)
    green = np.clip(255 * (1.0 - np.abs(t - 0.5) * 1.6), 0, 255)
    blue = np.clip(255 * (1.25 - 1.7 * t), 0, 255)
    return np.stack([red, green, blue], axis=1).astype(np.uint8)


def downsample_indices(count: int, max_points: int) -> np.ndarray:
    if count <= max_points:
        return np.arange(count, dtype=np.int64)
    return np.linspace(0, count - 1, max_points, dtype=np.int64)


def load_cloud(path: Path, max_points: int) -> CloudFrame:
    points, file_colors = read_ascii_ply(path)
    raw_count = points.shape[0]
    indices = downsample_indices(raw_count, max_points)
    points = points[indices]

    if file_colors is None:
        colors = depth_colors(points)
        color_mode = "depth"
    else:
        file_colors = file_colors[indices]
        sample = file_colors[: min(len(file_colors), 1000)]
        if np.unique(sample, axis=0).shape[0] <= 1:
            colors = depth_colors(points)
            color_mode = "depth"
        else:
            colors = file_colors
            color_mode = "ply"

    return CloudFrame(
        path=path,
        points=points,
        colors=colors,
        mtime=path.stat().st_mtime,
        raw_count=raw_count,
        color_mode=color_mode,
    )


def rotation_matrix(yaw: float, pitch: float) -> np.ndarray:
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    yaw_mat = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float32)
    pitch_mat = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]], dtype=np.float32)
    return pitch_mat @ yaw_mat


def project_points(points: np.ndarray, view_mode: str, yaw: float, pitch: float) -> np.ndarray:
    centered = points - np.median(points, axis=0)
    if view_mode == "front":
        return centered
    return centered @ rotation_matrix(yaw, pitch).T


def render_points(
    screen,
    frame: CloudFrame | None,
    view_mode: str,
    yaw: float,
    pitch: float,
    zoom: float,
    point_size: int,
) -> None:
    width, height = screen.get_size()
    screen.fill((7, 9, 12))
    if frame is None:
        return

    projected = project_points(frame.points, view_mode, yaw, pitch)
    x_extent = np.percentile(np.abs(projected[:, 0]), 98)
    y_extent = np.percentile(np.abs(projected[:, 1]), 98)
    extent = max(x_extent, y_extent, 1.0)
    scale = min(width, height) * 0.44 * zoom / extent

    xs = (projected[:, 0] * scale + width / 2).astype(np.int32)
    ys = (-projected[:, 1] * scale + height / 2).astype(np.int32)

    mask = (xs >= 0) & (xs < width) & (ys >= 0) & (ys < height)
    if not np.any(mask):
        return

    xs = xs[mask]
    ys = ys[mask]
    colors = frame.colors[mask]

    # Draw far points first so nearer points win visually.
    order = np.argsort(projected[:, 2][mask])
    xs = xs[order]
    ys = ys[order]
    colors = colors[order]

    import pygame

    pixels = pygame.surfarray.pixels3d(screen)
    radius = max(point_size, 1)
    for dx in range(radius):
        for dy in range(radius):
            px = np.clip(xs + dx, 0, width - 1)
            py = np.clip(ys + dy, 0, height - 1)
            pixels[px, py] = colors
    del pixels


def start_generator(show_output: bool) -> subprocess.Popen:
    if not GENERATOR_EXE.exists():
        raise FileNotFoundError(GENERATOR_EXE)

    env = os.environ.copy()
    env["PATH"] = str(BIN_DIR) + os.pathsep + env.get("PATH", "")
    output = None if show_output else subprocess.DEVNULL
    return subprocess.Popen(
        [str(GENERATOR_EXE)],
        cwd=str(BIN_DIR),
        env=env,
        stdout=output,
        stderr=output,
    )


def stop_generator(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        process.kill()


def draw_overlay(screen, font, frame: CloudFrame | None, status: str, view_mode: str, point_size: int) -> None:
    import pygame

    if frame is None:
        lines = [
            "Waiting for point cloud...",
            status,
            "Close Orbbec Viewer before live capture.",
        ]
    else:
        age = time.time() - frame.mtime
        lines = [
            f"PLY: {frame.path.name}  age: {age:.1f}s",
            f"points: {len(frame.points)} / raw: {frame.raw_count}  color: {frame.color_mode}  view: {view_mode}  size: {point_size}",
            status,
            "drag: rotate | wheel: zoom | V: front/orbit | +/-: point size | R: reset | S: save | Esc: quit",
        ]

    y = 12
    for line in lines:
        shadow = font.render(line, True, (0, 0, 0))
        text = font.render(line, True, (235, 240, 248))
        screen.blit(shadow, (15, y + 1))
        screen.blit(text, (14, y))
        y += 24

    if frame is not None:
        width, height = screen.get_size()
        for i in range(120):
            t = i / 119
            color = depth_colors(np.array([[0.0, 0.0, t * 1000.0]], dtype=np.float32))[0]
            pygame.draw.line(screen, tuple(int(v) for v in color), (width - 34, height - 24 - i), (width - 18, height - 24 - i))


def run_viewer(args: argparse.Namespace) -> None:
    import pygame

    pygame.init()
    screen = pygame.display.set_mode((1120, 760), pygame.RESIZABLE)
    pygame.display.set_caption("Gemini Point Cloud Viewer")
    font = pygame.font.SysFont("Consolas", 18)
    clock = pygame.time.Clock()

    generator: subprocess.Popen | None = None
    frame: CloudFrame | None = None
    last_seen_path: Path | None = None
    last_seen_mtime = 0.0
    last_reload_attempt = 0.0
    last_generator_start = 0.0
    status = "Static mode"

    if args.live:
        generator = start_generator(args.show_generator_output)
        last_generator_start = time.time()
        status = "Live mode: GeneratePointCloud.exe is running"
    elif args.ply:
        path = Path(args.ply)
        frame = load_cloud(path, args.max_points)
        last_seen_path = path
        last_seen_mtime = frame.mtime
        status = "Static PLY loaded"
    elif args.watch:
        status = "Watch mode: waiting for newest PLY"
    else:
        path = find_latest_ply()
        if path is None:
            raise FileNotFoundError("No .ply found. Run pointcloud first, or use --live.")
        frame = load_cloud(path, args.max_points)
        last_seen_path = path
        last_seen_mtime = frame.mtime
        status = "Latest PLY loaded"

    yaw = 0.0
    pitch = -0.25
    zoom = 1.0
    point_size = args.point_size
    view_mode = args.view
    dragging = False
    last_mouse = (0, 0)
    running = True

    try:
        while running:
            now = time.time()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_r:
                        yaw, pitch, zoom = 0.0, -0.25, 1.0
                    elif event.key == pygame.K_v:
                        view_mode = "orbit" if view_mode == "front" else "front"
                    elif event.key in (pygame.K_EQUALS, pygame.K_PLUS):
                        point_size = min(point_size + 1, 10)
                    elif event.key == pygame.K_MINUS:
                        point_size = max(point_size - 1, 1)
                    elif event.key == pygame.K_s:
                        output = Path.cwd() / "pointcloud_viewer.png"
                        pygame.image.save(screen, str(output))
                        status = f"Saved screenshot: {output}"
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        dragging = True
                        last_mouse = event.pos
                    elif event.button == 4:
                        zoom *= 1.12
                    elif event.button == 5:
                        zoom /= 1.12
                elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    dragging = False
                elif event.type == pygame.MOUSEMOTION and dragging:
                    x, y = event.pos
                    lx, ly = last_mouse
                    yaw += (x - lx) * 0.008
                    pitch += (y - ly) * 0.008
                    pitch = max(min(pitch, 1.45), -1.45)
                    last_mouse = event.pos

            if args.live and generator is not None and generator.poll() is not None:
                status = "GeneratePointCloud.exe reached 50 frames; restarting"
                if now - last_generator_start >= args.restart_delay:
                    generator = start_generator(args.show_generator_output)
                    last_generator_start = now
                    status = "Live mode: GeneratePointCloud.exe restarted"

            should_reload = args.live or args.watch
            if should_reload and now - last_reload_attempt >= args.reload_interval:
                last_reload_attempt = now
                latest = find_latest_ply()
                if latest is not None:
                    try:
                        latest_mtime = latest.stat().st_mtime
                        is_new = latest != last_seen_path or latest_mtime > last_seen_mtime
                        if is_new:
                            frame = load_cloud(latest, args.max_points)
                            last_seen_path = latest
                            last_seen_mtime = frame.mtime
                            if args.live:
                                status = "Live mode: newest PLY loaded"
                            else:
                                status = "Watch mode: newest PLY loaded"
                    except (OSError, ValueError) as exc:
                        status = f"Waiting for complete PLY: {exc}"

            render_points(screen, frame, view_mode, yaw, pitch, zoom, point_size)
            draw_overlay(screen, font, frame, status, view_mode, point_size)
            pygame.display.flip()
            clock.tick(args.fps)
    finally:
        stop_generator(generator)
        pygame.quit()


def main() -> None:
    parser = argparse.ArgumentParser(description="Viewer-style Gemini/OpenNI PLY point cloud visualizer")
    parser.add_argument("ply", nargs="?", help="PLY file path. If omitted, the latest PLY is used.")
    parser.add_argument("--live", action="store_true", help="run and restart GeneratePointCloud.exe while viewing")
    parser.add_argument("--watch", action="store_true", help="watch the OpenNI PointCloud folder for newest PLY")
    parser.add_argument("--show-generator-output", action="store_true", help="show GeneratePointCloud.exe console output")
    parser.add_argument("--max-points", type=int, default=90_000, help="maximum rendered points")
    parser.add_argument("--point-size", type=int, default=2, help="initial rendered point size")
    parser.add_argument("--reload-interval", type=float, default=0.35, help="seconds between newest PLY checks")
    parser.add_argument("--restart-delay", type=float, default=0.5, help="seconds before restarting the generator")
    parser.add_argument("--fps", type=int, default=30, help="viewer refresh rate")
    parser.add_argument("--view", choices=["front", "orbit"], default="front", help="initial view mode")
    args = parser.parse_args()

    if args.live and args.ply:
        parser.error("Do not pass a PLY path with --live")

    run_viewer(args)


if __name__ == "__main__":
    main()

