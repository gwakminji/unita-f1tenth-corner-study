#!/usr/bin/env python3
"""Run a repeatable, lightweight corner-parameter study using repository waypoints.

This is a proxy experiment, not the ROS controller or F1TENTH Gym dynamics.
It copies the repository's L1 distance and speed lookahead rules and advances a
kinematic bicycle. It is useful for screening parameters before full-stack tests.
"""

import argparse
import csv
import json
import math
import re
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent
# Copies of racing_stack/stack_master/config/controller.yaml and maps/test1/.
CONFIG = ROOT / "track_data/controller.yaml"
WAYPOINTS = ROOT / "track_data/global_waypoints.json"


def config_number(name):
    match = re.search(rf"^\s*{re.escape(name)}:\s*([-+\d.eE]+)", CONFIG.read_text(encoding="utf-8"), re.M)
    if not match:
        raise ValueError(f"Missing {name} in {CONFIG}")
    return float(match.group(1))


def load_track(path):
    data = json.loads(path.read_text(encoding="utf-8"))
    points = data["global_traj_wpnts_sp"]["wpnts"]
    return {key: np.array([float(p[key]) for p in points]) for key in
            ("x_m", "y_m", "psi_rad", "kappa_radpm", "vx_mps", "d_left", "d_right")}


def nearest(track, x, y, lo=0, hi=None):
    hi = len(track["x_m"]) if hi is None else hi
    dx = track["x_m"][lo:hi] - x
    dy = track["y_m"][lo:hi] - y
    return lo + int(np.argmin(dx * dx + dy * dy))


def run_case(track, start, finish, speed_lookahead, curvature_factor, cfg, speed_scale):
    x = float(track["x_m"][start])
    y = float(track["y_m"][start])
    yaw = float(track["psi_rad"][start])
    speed = max(0.5, float(track["vx_mps"][start] * speed_scale))
    steer = 0.0
    dt = 0.02
    elapsed = 0.0
    max_error = 0.0
    min_clearance = float("inf")
    min_l1 = float("inf")
    max_l1 = 0.0
    entry_speed = None
    status = "timeout"
    trace = []
    # The segment starts before the chosen corner; nearest points may briefly
    # move backward because the vehicle is between waypoints.
    cursor = start
    for _ in range(int(25.0 / dt)):
        i = nearest(track, x, y, max(start, cursor - 8), min(len(track["x_m"]), finish + 15))
        cursor = max(cursor, i)
        if i >= finish:
            status = "completed"
            break
        if i >= start + max(1, (finish - start) // 4) and entry_speed is None:
            entry_speed = speed

        nx = -math.sin(float(track["psi_rad"][i]))
        ny = math.cos(float(track["psi_rad"][i]))
        lateral = (x - track["x_m"][i]) * nx + (y - track["y_m"][i]) * ny
        clearance = (track["d_left"][i] - lateral if lateral >= 0 else
                     track["d_right"][i] + lateral) - 0.17  # approximate half-width
        max_error = max(max_error, abs(lateral))
        min_clearance = min(min_clearance, clearance)
        if clearance < 0:
            status = "boundary_crossed"
            break

        # Repository calc_speed_command(): sample waypoint speed at a predicted
        # position, then use a 2-waypoint offset.
        px = x + speed * math.cos(yaw) * speed_lookahead
        py = y + speed * math.sin(yaw) * speed_lookahead
        speed_idx = min(nearest(track, px, py), len(track["x_m"]) - 3) + 2
        desired_speed = max(0.0, float(track["vx_mps"][speed_idx] * speed_scale))

        # Repository calc_future_L1_point(): speed-scaled distance with a
        # curvature term, bounded by t_clip_min and t_clip_max.
        curvature = float(np.mean(np.abs(track["kappa_radpm"][max(0, i-2):i+3])))
        l1 = cfg["m_l1"] * speed - curvature_factor * curvature * speed * speed + cfg["q_l1"]
        l1 = float(np.clip(l1, cfg["t_clip_min"], cfg["t_clip_max"]))
        min_l1 = min(min_l1, l1)
        max_l1 = max(max_l1, l1)
        target = i
        while target < len(track["x_m"]) - 1:
            if math.hypot(track["x_m"][target] - x, track["y_m"][target] - y) >= l1:
                break
            target += 1
        alpha = math.atan2(track["y_m"][target] - y, track["x_m"][target] - x) - yaw
        requested_steer = math.atan2(2 * 0.33 * math.sin(alpha), l1)
        requested_steer = float(np.clip(requested_steer, -0.4, 0.4))
        steer += float(np.clip(requested_steer - steer, -3.2 * dt, 3.2 * dt))
        speed += float(np.clip(desired_speed - speed, -6.0 * dt, 4.0 * dt))
        yaw += speed * math.tan(steer) / 0.33 * dt
        x += speed * math.cos(yaw) * dt
        y += speed * math.sin(yaw) * dt
        elapsed += dt
        trace.append((round(elapsed, 3), i, round(x, 4), round(y, 4),
                      round(speed, 4), round(l1, 4), round(lateral, 4)))
    return {
        "speed_lookahead_s": speed_lookahead,
        "curvature_factor": curvature_factor,
        "status": status,
        "segment_time_s": round(elapsed, 3),
        "entry_speed_mps": round(entry_speed, 3) if entry_speed is not None else "",
        "max_lateral_error_m": round(max_error, 3),
        "min_boundary_clearance_m": round(min_clearance, 3),
        "min_l1_m": round(min_l1, 3),
        "max_l1_m": round(max_l1, 3),
    }, trace


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--waypoints", type=Path, default=WAYPOINTS)
    parser.add_argument("--start", type=int, default=365, help="first waypoint of test segment")
    parser.add_argument("--finish", type=int, default=400, help="last waypoint of test segment")
    parser.add_argument("--speed-scale", type=float, default=0.8)
    parser.add_argument("--minimum-clearance", type=float, default=0.05,
                        help="minimum allowed modelled boundary clearance in metres")
    parser.add_argument("--output", type=Path, default=ROOT / "corner_test_results")
    args = parser.parse_args()
    track = load_track(args.waypoints)
    if not 0 <= args.start < args.finish < len(track["x_m"]):
        parser.error("Require 0 <= start < finish < number of waypoints")
    if args.speed_scale <= 0:
        parser.error("--speed-scale must be positive")
    if args.minimum_clearance < 0:
        parser.error("--minimum-clearance must be non-negative")
    cfg = {name: config_number(name) for name in ("m_l1", "q_l1", "t_clip_min", "t_clip_max",
                                                   "speed_lookahead", "curvature_factor")}
    lookaheads = sorted(set([round(cfg["speed_lookahead"] * f, 4) for f in (0.8, 1.0, 1.2)]))
    curvature_factors = sorted(set([round(cfg["curvature_factor"] * f, 4) for f in (0.8, 1.0, 1.2)]))
    args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    for lookahead in lookaheads:
        for factor in curvature_factors:
            row, trace = run_case(track, args.start, args.finish, lookahead, factor, cfg, args.speed_scale)
            rows.append(row)
            trace_path = args.output / f"trace_speed_{lookahead}_curvature_{factor}.csv"
            with trace_path.open("w", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow(("time_s", "waypoint_index", "x_m", "y_m", "speed_mps", "l1_m", "lateral_error_m"))
                writer.writerows(trace)
    with (args.output / "results.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    valid = [r for r in rows if r["status"] == "completed" and
             r["min_boundary_clearance_m"] >= args.minimum_clearance]
    winner = None
    if valid:
        fastest = min(r["segment_time_s"] for r in valid)
        near_fastest = [r for r in valid if r["segment_time_s"] <= fastest + 0.01]
        best_clearance = max(r["min_boundary_clearance_m"] for r in near_fastest)
        near_equal = [r for r in near_fastest if r["min_boundary_clearance_m"] >= best_clearance - 0.01]
        winner = min(near_equal, key=lambda r: abs(r["curvature_factor"] - cfg["curvature_factor"]))
    lines = ["# 고속 코너 파라미터 자동 비교 결과", "",
             f"- 경로: `{args.waypoints}`", f"- 시험 구간: 웨이포인트 {args.start}–{args.finish}",
             f"- 속도 배율: {args.speed_scale}",
             f"- 통과 기준: 완주 및 경계 여유 {args.minimum_clearance} m 이상",
             "- 방법: 저장소의 목표 속도 참조와 L1 거리 식을 이용한 간이 운동학 시뮬레이션", "",
             "| 속도 전방 참조 (s) | 곡률 계수 | 결과 | 통과 시간 (s) | 최대 횡오차 (m) | 최소 경계 여유 (m) |",
             "|---:|---:|---|---:|---:|---:|"]
    for row in rows:
        lines.append(f"| {row['speed_lookahead_s']} | {row['curvature_factor']} | {row['status']} | "
                     f"{row['segment_time_s']} | {row['max_lateral_error_m']} | {row['min_boundary_clearance_m']} |")
    lines += ["", (f"**간이 모델에서 기준을 통과한 가장 빠른 설정:** `speed_lookahead={winner['speed_lookahead_s']}`, "
                    f"`curvature_factor={winner['curvature_factor']}` ({winner['segment_time_s']}초)."
                    if winner else "**기준 통과 설정 없음:** 시험 구간 또는 속도 배율을 조정해야 한다."), "",
              "통과 시간 차이가 0.01초 이내이고 경계 여유 차이가 0.01m 이내면 동률로 보고, "
              "원래 곡률 계수에 가까운 값을 표시했다. 작은 차이의 우열은 확정할 수 없다.", "",
              "이 결과는 파라미터 후보를 추리는 용도다. 실제 ROS 컨트롤러와 F1TENTH Gym의 차량 동역학, "
              "노면 접지, 지연, 장애물 회피를 재현하지 않으므로 대회 성능을 확정하지 않는다.", ""]
    (args.output / "report.md").write_text("\n".join(lines), encoding="utf-8")
    print("\n".join(lines))
    print(f"\nSaved: {args.output / 'report.md'} and {args.output / 'results.csv'}")


if __name__ == "__main__":
    main()
