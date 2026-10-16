#!/usr/bin/env python3
"""Visualize pairs of traces from corner_parameter_test.py as side-by-side GIFs."""

import csv
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont, ImageOps


ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "corner_test_results"
TRACK = ROOT / "track_data/global_waypoints.json"
MAP_IMAGE = ROOT / "track_data/test1.png"
START, FINISH = 365, 400

# Each comparison: (output name, title, subtitle, [(trace, label, colour), (trace, label, colour)])
COMPARISONS = [
    ("corner_comparison_lookahead",
     "High-speed corner: same speed, different speed lookahead",
     "Target speed x0.8  |  curvature_factor 0.22  |  simulated time step = 0.02 s  |  GIF played in slow motion",
     [(RESULTS / "trace_speed_0.25_curvature_0.22.csv", "(5) Current  |  speed_lookahead 0.25 s", "#e05a46"),
      (RESULTS / "trace_speed_0.2_curvature_0.22.csv", "(2) Candidate  |  speed_lookahead 0.20 s", "#1479c9")]),
    ("corner_comparison_speed",
     "High-speed corner: same settings, different entry speed",
     "speed_lookahead 0.25 s, curvature_factor 0.22 (current)  |  "
     "simulated time step = 0.02 s  |  GIF played in slow motion",
     [(RESULTS / "trace_speed_0.25_curvature_0.22.csv", "Target speed x0.8", "#1479c9"),
      (RESULTS / "speed_1_0/trace_speed_0.25_curvature_0.22.csv", "Target speed x1.0 (original)", "#e05a46")]),
]


def font(size, bold=False):
    names = (["C:/Windows/Fonts/arialbd.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"]
             if bold else ["C:/Windows/Fonts/arial.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"])
    for name in names:
        if Path(name).exists():
            return ImageFont.truetype(name, size)
    return ImageFont.load_default()


def read_trace(path):
    with path.open(newline="", encoding="utf-8") as file:
        return [{key: float(value) for key, value in row.items()} for row in csv.DictReader(file)]


def make_gif(track, name, title, subtitle, runs):
    traces = [read_trace(path) for path, _, _ in runs]
    names = [label for _, label, _ in runs]
    panel_colors = [colour for _, _, colour in runs]
    # The simulation stops a run when the car crosses the boundary, so a short trace means it left the track.
    completed = [trace[-1]["waypoint_index"] >= FINISH - 1 for trace in traces]

    points = track[START - 5:FINISH + 8]
    all_xy = [(p["x_m"], p["y_m"]) for p in points]
    xmin, xmax = min(p[0] for p in all_xy), max(p[0] for p in all_xy)
    ymin, ymax = min(p[1] for p in all_xy), max(p[1] for p in all_xy)
    cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
    scale = min(430 / (xmax - xmin), 300 / (ymax - ymin))
    map_image = Image.open(MAP_IMAGE).convert("L")
    colored_map = ImageOps.colorize(map_image, black="#24384b", white="#ffffff", mid="#dce5ed")
    origin_x, origin_y, resolution = -1.293542, -1.389102, 0.05
    world_left, world_right = cx - 280/scale, cx + 280/scale
    world_bottom, world_top = cy - 210/scale, cy + 210/scale
    map_crop = colored_map.crop(((world_left-origin_x)/resolution,
                                 map_image.height-1-(world_top-origin_y)/resolution,
                                 (world_right-origin_x)/resolution,
                                 map_image.height-1-(world_bottom-origin_y)/resolution))
    map_crop = map_crop.resize((560, 420), Image.BILINEAR)

    def project(x, y, panel):
        return (int(panel * 620 + 310 + (x - cx) * scale),
                int(370 - (y - cy) * scale))

    title_font, label_font, small_font = font(28, True), font(19, True), font(16)
    frames = []
    max_count = max(len(t) for t in traces)
    for frame_index in list(range(0, max_count, 2)) + [max_count - 1] * 8:
        image = Image.new("RGB", (1240, 710), "#f7f9fc")
        draw = ImageDraw.Draw(image)
        draw.text((32, 22), title, font=title_font, fill="#142438")
        draw.text((32, 62), subtitle, font=small_font, fill="#526477")
        for panel in (0, 1):
            x0 = panel * 620
            draw.rounded_rectangle((x0 + 20, 95, x0 + 600, 680), radius=16, fill="white", outline="#d8e2eb", width=2)
            draw.text((x0 + 43, 113), names[panel], font=label_font, fill=panel_colors[panel])
            image.paste(map_crop, (x0 + 30, 160))
            draw.line([project(p["x_m"], p["y_m"], panel) for p in points], fill="#8bacc2", width=2)
            start_point = project(track[START]["x_m"], track[START]["y_m"], panel)
            finish_point = project(track[FINISH]["x_m"], track[FINISH]["y_m"], panel)
            draw.ellipse((start_point[0]-6, start_point[1]-6, start_point[0]+6, start_point[1]+6), fill="#3c5569")
            draw.ellipse((finish_point[0]-8, finish_point[1]-8, finish_point[0]+8, finish_point[1]+8), fill="#1aaf7a")
            trace = traces[panel]
            index = min(frame_index, len(trace) - 1)
            path = [project(row["x_m"], row["y_m"], panel) for row in trace[:index + 1]]
            if len(path) > 1:
                draw.line(path, fill=panel_colors[panel], width=5, joint="curve")
            row = trace[index]
            previous = trace[max(0, index - 2)]
            angle = math.atan2(row["y_m"] - previous["y_m"], row["x_m"] - previous["x_m"])
            if index == 0:
                angle = float(track[START]["psi_rad"])
            fx, fy = math.cos(angle), math.sin(angle)
            nx, ny = -fy, fx
            car = [(row["x_m"] + 0.22*fx, row["y_m"] + 0.22*fy),
                   (row["x_m"] - 0.15*fx + 0.11*nx, row["y_m"] - 0.15*fy + 0.11*ny),
                   (row["x_m"] - 0.15*fx - 0.11*nx, row["y_m"] - 0.15*fy - 0.11*ny)]
            draw.polygon([project(*p, panel) for p in car], fill=panel_colors[panel], outline="#142438", width=2)
            if index == len(trace) - 1:
                if completed[panel]:
                    draw.text((x0 + 380, 599), "Corner completed", font=label_font, fill="#1aaf7a")
                else:
                    cx_px, cy_px = project(row["x_m"], row["y_m"], panel)
                    draw.line((cx_px-14, cy_px-14, cx_px+14, cy_px+14), fill="#c0262d", width=5)
                    draw.line((cx_px-14, cy_px+14, cx_px+14, cy_px-14), fill="#c0262d", width=5)
                    draw.text((x0 + 380, 599), "Left the track", font=label_font, fill="#c0262d")
            draw.text((x0 + 43, 599), f"Simulated time: {row['time_s']:.2f} s", font=label_font, fill="#142438")
            draw.text((x0 + 43, 633), f"Speed: {row['speed_mps']:.2f} m/s    L1: {row['l1_m']:.2f} m",
                      font=small_font, fill="#526477")
        frames.append(image)
    output = RESULTS / f"{name}.gif"
    frames[0].save(output, save_all=True, append_images=frames[1:], duration=[80]*(len(frames)-8)+[180]*8,
                   loop=0, optimize=True)
    frames[-1].save(RESULTS / f"{name}_preview.png")
    print(f"GIF saved to: {output}")
    for label, trace, done in zip(names, traces, completed):
        print(f"  {label}: {'completed' if done else 'left the track'} at {trace[-1]['time_s']:.2f} s")


def main():
    missing = [path for _, _, _, runs in COMPARISONS for path, _, _ in runs if not path.exists()]
    if missing:
        raise SystemExit("Run `python corner_parameter_test.py --speed-scale 0.8` and "
                         "`--speed-scale 1.0 --output corner_test_results/speed_1_0` before making the GIFs.")
    data = json.loads(TRACK.read_text(encoding="utf-8"))
    track = data["global_traj_wpnts_sp"]["wpnts"]
    for name, title, subtitle, runs in COMPARISONS:
        make_gif(track, name, title, subtitle, runs)
    print("These are simulated segment times, not real-car or ROS/Gym measurements.")


if __name__ == "__main__":
    main()
