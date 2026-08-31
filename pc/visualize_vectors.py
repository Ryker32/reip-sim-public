#!/usr/bin/env python3
"""
Post-processing overlay for REIP hardware trials.

Takes a clean recorded video (overhead.mp4) and the synchronized state log
(robot_states.jsonl) produced by aruco_position_server.py, and renders a
publication-quality annotated video showing:

  - Arena boundary, interior wall, and grid overlay
  - Exploration coverage (green=visited, dark=unexplored, gray=wall)
  - Per-robot trust halos (green/yellow/orange/red)
  - Heading arrow (white)
  - Navigation target arrow (cyan)
  - Predicted vs commanded divergence (green vs red arrows when >30deg)
  - Leader crown marker
  - Fault injection event banners (from trial_meta.json)
  - Stats panel: time, coverage, per-robot trust

Optionally merges per-robot JSONL logs (from run_trial.py) for richer data
(fault classification, impeachment timing, etc.).

Usage:
  # Minimal -- just the server's trial directory:
  python visualize_vectors.py --trial-dir trials/20250303_143052

  # With robot-side logs and fault metadata:
  python visualize_vectors.py --trial-dir trials/20250303_143052 \
      --robot-logs trials/reip_bad_leader_t1_20250303_143052 \
      --meta trials/reip_bad_leader_t1_20250303_143052/trial_meta.json

  # Custom output and time window:
  python visualize_vectors.py --trial-dir trials/20250303_143052 \
      --output showcase.mp4 --start 5 --duration 60
"""

import argparse
import cv2
import json
import math
import numpy as np
import os
from glob import glob
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Set
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from arena_coverage import ARENA  # noqa: E402  single source of arena geometry

# ============== Colors (BGR) ==============
C_ARENA       = (0, 255, 0)       # green  -- arena boundary
C_WALL        = (0, 255, 0)       # green  -- interior wall
C_MARGIN      = (0, 200, 255)     # yellow -- wall margin
C_GRID        = (100, 100, 100)   # gray   -- grid lines
C_VISITED     = (0, 140, 0)       # dark green -- visited cell fill
C_UNEXPLORED  = (40, 20, 20)      # dark red   -- unexplored cell fill
C_WALL_CELL   = (60, 60, 60)      # dark gray  -- wall cell fill
C_HEADING     = (255, 255, 255)   # white  -- heading arrow
C_NAV_TARGET  = (255, 200, 0)     # cyan   -- navigation target
C_PREDICTED   = (0, 255, 100)     # green  -- predicted target
C_COMMANDED   = (0, 0, 255)       # red    -- commanded target
C_TRUST_OK    = (0, 255, 0)       # green  -- trust healthy
C_TRUST_WARN  = (0, 180, 255)     # orange -- suspicion rising
C_TRUST_BAD   = (0, 0, 255)       # red    -- trust lost
C_LEADER      = (0, 255, 255)     # yellow -- leader crown
C_FAULT_BG    = (0, 0, 180)       # dark red -- fault banner background
C_WHITE       = (255, 255, 255)
C_TEXT_DIM    = (180, 180, 180)

TRUST_WARN_THRESH = 0.7
TRUST_BAD_THRESH  = 0.4
SUSPICION_THRESH  = 0.4

# Robot color palette (per robot ID, consistent with the physical robots)
ROBOT_COLORS = {
    1: (0, 165, 255),   # orange
    2: (255, 0, 0),     # blue
    3: (0, 255, 0),     # green
    4: (0, 0, 0),       # black
    5: (255, 255, 255), # white
}


# ============== Calibration ==============
@dataclass
class Calibration:
    arena_w: float = 2000
    arena_h: float = 1500
    cell_size: int = 125
    cam_h: float = 2190
    corner_h: float = 152
    robot_tag_h: float = 80
    camera_center_x: float = 1000
    camera_center_y: float = -40
    floor_scale: float = 1.0
    robot_scale: float = 1.0
    homography: Optional[np.ndarray] = None
    inv_homography: Optional[np.ndarray] = None
    wall_x_left: int = 1000
    wall_x_right: int = 1036
    wall_y_end: int = 1200

    def arena_to_pixel(self, x_mm: float, y_mm: float) -> Tuple[int, int]:
        """Convert floor-level arena coords to pixel coords."""
        if self.inv_homography is not None:
            wx = self.camera_center_x + (x_mm - self.camera_center_x) / self.floor_scale
            wy = self.camera_center_y + (y_mm - self.camera_center_y) / self.floor_scale
            pt = np.array([[[wx, wy]]], dtype=np.float32)
            px_pt = cv2.perspectiveTransform(pt, self.inv_homography)
            return int(px_pt[0][0][0]), int(px_pt[0][0][1])
        return int(x_mm), int(y_mm)

    def arena_to_pixel_robot(self, x_mm: float, y_mm: float) -> Tuple[int, int]:
        """Convert robot-level arena coords to pixel coords.
        Uses ROBOT_SCALE so the cell grid aligns with detected robot markers."""
        if self.inv_homography is not None:
            wx = self.camera_center_x + (x_mm - self.camera_center_x) / self.robot_scale
            wy = self.camera_center_y + (y_mm - self.camera_center_y) / self.robot_scale
            pt = np.array([[[wx, wy]]], dtype=np.float32)
            px_pt = cv2.perspectiveTransform(pt, self.inv_homography)
            return int(px_pt[0][0][0]), int(px_pt[0][0][1])
        return int(x_mm), int(y_mm)

    def is_wall_cell(self, cx: int, cy: int) -> bool:
        """Delegate to the canonical arena geometry.

        This used to be a third, independent implementation using outer margin
        0, divider margin 20 and body radius 77 -- none of which match the
        geometry the robots and the coverage metric actually use (110 / 64 /
        100).  The overlay therefore drew a different reachable set than the
        one every reported number was computed against.
        """
        return ARENA.is_wall_cell(cx, cy)


def load_calibration(header: dict) -> Calibration:
    cal = Calibration(
        arena_w=header.get('arena_width_mm', 2000),
        arena_h=header.get('arena_height_mm', 1500),
        cell_size=header.get('cell_size', 125),
        cam_h=header.get('cam_h', 2190),
        corner_h=header.get('corner_h', 152),
        robot_tag_h=header.get('robot_tag_h', 80),
        camera_center_x=header.get('camera_center_x', 1000),
        camera_center_y=header.get('camera_center_y', -40),
        floor_scale=header.get('floor_scale', 1.075),
        robot_scale=header.get('robot_scale', 1.035),
        wall_x_left=header.get('interior_wall_x_left', 1000),
        wall_x_right=header.get('interior_wall_x_right', 1036),
        wall_y_end=header.get('interior_wall_y_end', 1200),
    )
    if header.get('homography') is not None:
        cal.homography = np.array(header['homography'], dtype=np.float64)
    if header.get('inv_homography') is not None:
        cal.inv_homography = np.array(header['inv_homography'], dtype=np.float64)
    return cal


# ============== Cell polygon cache ==============
def build_cell_cache(cal: Calibration):
    """Pre-compute cell polygons in pixel space + wall status.
    Uses robot-level projection so cells align with detected robot markers."""
    cells_x = int(cal.arena_w / cal.cell_size)
    cells_y = int(cal.arena_h / cal.cell_size)
    polys = []
    num_explorable = 0
    for cy in range(cells_y):
        for cx in range(cells_x):
            x0, y0 = cx * cal.cell_size, cy * cal.cell_size
            corners = [(x0, y0), (x0 + cal.cell_size, y0),
                       (x0 + cal.cell_size, y0 + cal.cell_size), (x0, y0 + cal.cell_size)]
            pts = np.array([cal.arena_to_pixel_robot(ax, ay) for ax, ay in corners],
                           dtype=np.int32)
            is_wall = cal.is_wall_cell(cx, cy)
            polys.append(((cx, cy), pts, is_wall))
            if not is_wall:
                num_explorable += 1
    return polys, num_explorable


# ============== Drawing functions ==============
def draw_exploration(frame, cell_polys, visited: Set[Tuple[int, int]], num_explorable: int):
    """Semi-transparent exploration coverage overlay."""
    overlay = frame.copy()
    for (cx, cy), pts, is_wall in cell_polys:
        if is_wall:
            cv2.fillConvexPoly(overlay, pts, C_WALL_CELL)
        elif (cx, cy) in visited:
            cv2.fillConvexPoly(overlay, pts, C_VISITED)
        else:
            cv2.fillConvexPoly(overlay, pts, C_UNEXPLORED)
    cv2.addWeighted(overlay, 0.3, frame, 0.7, 0, frame)

    pct = len(visited) / max(1, num_explorable) * 100
    h, w = frame.shape[:2]
    text = f"Coverage: {len(visited)} cells ({pct:.0f}%)"
    sz = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
    cv2.putText(frame, text, ((w - sz[0]) // 2, 85),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)


def draw_arena(frame, cal: Calibration):
    """Arena boundary, interior wall, margin, and grid -- semi-transparent."""
    WALL_MARGIN = 115
    DIVIDER_MARGIN = 125
    overlay = frame.copy()

    arena_pts = [(0, 0), (cal.arena_w, 0),
                 (cal.arena_w, cal.arena_h), (0, cal.arena_h)]
    pts = [cal.arena_to_pixel(x, y) for x, y in arena_pts]
    for i in range(4):
        cv2.line(overlay, pts[i], pts[(i + 1) % 4], C_ARENA, 2)

    cv2.line(overlay, cal.arena_to_pixel(cal.wall_x_left, 0),
             cal.arena_to_pixel(cal.wall_x_left, cal.wall_y_end), C_WALL, 2)
    cv2.line(overlay, cal.arena_to_pixel(cal.wall_x_right, 0),
             cal.arena_to_pixel(cal.wall_x_right, cal.wall_y_end), C_WALL, 2)

    margin_pts = [(WALL_MARGIN, WALL_MARGIN),
                  (cal.arena_w - WALL_MARGIN, WALL_MARGIN),
                  (cal.arena_w - WALL_MARGIN, cal.arena_h - WALL_MARGIN),
                  (WALL_MARGIN, cal.arena_h - WALL_MARGIN)]
    mpts = [cal.arena_to_pixel(x, y) for x, y in margin_pts]
    for i in range(4):
        cv2.line(overlay, mpts[i], mpts[(i + 1) % 4], C_MARGIN, 1)

    cv2.line(overlay, cal.arena_to_pixel(cal.wall_x_left - DIVIDER_MARGIN, 0),
             cal.arena_to_pixel(cal.wall_x_left - DIVIDER_MARGIN, cal.wall_y_end), C_MARGIN, 1)
    cv2.line(overlay, cal.arena_to_pixel(cal.wall_x_right + DIVIDER_MARGIN, 0),
             cal.arena_to_pixel(cal.wall_x_right + DIVIDER_MARGIN, cal.wall_y_end), C_MARGIN, 1)

    cells_x = int(cal.arena_w / cal.cell_size)
    cells_y = int(cal.arena_h / cal.cell_size)
    for cx in range(1, cells_x):
        cv2.line(overlay, cal.arena_to_pixel(cx * cal.cell_size, 0),
                 cal.arena_to_pixel(cx * cal.cell_size, cal.arena_h), C_GRID, 1)
    for cy in range(1, cells_y):
        cv2.line(overlay, cal.arena_to_pixel(0, cy * cal.cell_size),
                 cal.arena_to_pixel(cal.arena_w, cy * cal.cell_size), C_GRID, 1)

    cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)


def draw_robots(frame, cal: Calibration, poses: dict, robot_states: dict):
    """Draw each robot: trust halo, heading, nav vector, predicted/commanded."""
    for rid_str, pose in poses.items():
        rid = int(rid_str)
        st = robot_states.get(rid_str, robot_states.get(str(rid), {}))
        x_mm = pose.get('x', 0)
        y_mm = pose.get('y', 0)
        theta = pose.get('theta', 0)

        px, py = cal.arena_to_pixel(x_mm, y_mm)

        is_leader = st.get('state') == 'leader'
        trust = st.get('trust_in_leader')
        suspicion = st.get('suspicion', 0)

        # Trust halo
        if trust is not None and trust < TRUST_BAD_THRESH:
            ring_color = C_TRUST_BAD
        elif suspicion and suspicion > SUSPICION_THRESH:
            ring_color = C_TRUST_WARN
        else:
            ring_color = C_TRUST_OK

        ring_thick = 3 if is_leader else 2
        cv2.circle(frame, (px, py), 22, ring_color, ring_thick)
        if is_leader:
            cv2.circle(frame, (px, py), 28, C_LEADER, 1)

        # Robot body dot (use per-robot color)
        body_color = ROBOT_COLORS.get(rid, (200, 200, 200))
        cv2.circle(frame, (px, py), 10, body_color, -1)
        cv2.circle(frame, (px, py), 10, C_WHITE, 1)

        # Heading arrow
        arrow_len = 40
        ax = int(px + arrow_len * math.cos(-theta))
        ay = int(py + arrow_len * math.sin(-theta))
        cv2.arrowedLine(frame, (px, py), (ax, ay), C_HEADING, 2)

        # Navigation target (cyan)
        nav_tgt = st.get('navigation_target')
        if nav_tgt and cal.inv_homography is not None:
            tpx, tpy = cal.arena_to_pixel(nav_tgt[0], nav_tgt[1])
            dx, dy = tpx - px, tpy - py
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > 5:
                scale = min(80 / dist, 1.0)
                cv2.arrowedLine(frame, (px, py),
                                (int(px + dx * scale), int(py + dy * scale)),
                                C_NAV_TARGET, 2, tipLength=0.25)

        # Predicted vs commanded divergence
        cmd_tgt = st.get('commanded_target')
        pred_tgt = st.get('predicted_target')
        if cmd_tgt and pred_tgt and cal.inv_homography is not None:
            cdx = cmd_tgt[0] - x_mm
            cdy = cmd_tgt[1] - y_mm
            pdx = pred_tgt[0] - x_mm
            pdy = pred_tgt[1] - y_mm
            cmd_len = math.sqrt(cdx * cdx + cdy * cdy)
            pred_len = math.sqrt(pdx * pdx + pdy * pdy)
            if cmd_len > 1 and pred_len > 1:
                dot = (cdx * pdx + cdy * pdy) / (cmd_len * pred_len)
                dot = max(-1.0, min(1.0, dot))
                angle_diff = math.degrees(math.acos(dot))
                if angle_diff > 30:
                    cpx, cpy = cal.arena_to_pixel(cmd_tgt[0], cmd_tgt[1])
                    ddx, ddy = cpx - px, cpy - py
                    cd = math.sqrt(ddx * ddx + ddy * ddy)
                    if cd > 5:
                        sc = min(90 / cd, 1.0)
                        cv2.arrowedLine(frame, (px, py),
                                        (int(px + ddx * sc), int(py + ddy * sc)),
                                        C_COMMANDED, 3, tipLength=0.3)
                    ppx, ppy = cal.arena_to_pixel(pred_tgt[0], pred_tgt[1])
                    ddx2, ddy2 = ppx - px, ppy - py
                    pd = math.sqrt(ddx2 * ddx2 + ddy2 * ddy2)
                    if pd > 5:
                        sc2 = min(90 / pd, 1.0)
                        cv2.arrowedLine(frame, (px, py),
                                        (int(px + ddx2 * sc2), int(py + ddy2 * sc2)),
                                        C_PREDICTED, 3, tipLength=0.3)

        # Label
        label = f"{'L' if is_leader else 'R'}{rid}"
        cv2.putText(frame, label, (px - 15, py - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, ring_color, 2)
        trust_str = f"T:{trust:.2f}" if trust is not None else ""
        cv2.putText(frame, f"({x_mm:.0f},{y_mm:.0f}) {trust_str}",
                    (px - 40, py + 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, C_WHITE, 1)


def draw_legend(frame):
    """Static legend in top-left."""
    x, y = 15, 25
    sp = 22

    cv2.putText(frame, "REIP Post-Processed", (x, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_WHITE, 2)
    y += sp + 5

    items = [
        (lambda f, x, y: cv2.arrowedLine(f, (x, y), (x + 35, y), C_HEADING, 2), "Heading"),
        (lambda f, x, y: cv2.arrowedLine(f, (x, y), (x + 35, y), C_NAV_TARGET, 2), "Nav target"),
        (lambda f, x, y: cv2.arrowedLine(f, (x, y), (x + 35, y), C_PREDICTED, 3), "Predicted (local)"),
        (lambda f, x, y: cv2.arrowedLine(f, (x, y), (x + 35, y), C_COMMANDED, 3), "Commanded (leader)"),
        (lambda f, x, y: cv2.circle(f, (x + 17, y), 10, C_LEADER, 2), "Leader"),
        (lambda f, x, y: cv2.circle(f, (x + 17, y), 10, C_TRUST_WARN, 2), "Suspicious"),
        (lambda f, x, y: cv2.circle(f, (x + 17, y), 10, C_TRUST_BAD, 2), "Trust lost"),
    ]
    for draw_fn, label in items:
        draw_fn(frame, x, y)
        cv2.putText(frame, label, (x + 45, y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, C_TEXT_DIM, 1)
        y += sp


def draw_stats(frame, robot_states: dict, coverage_pct: float, elapsed: float,
               fault_events: list):
    """Right-side stats panel."""
    h, w = frame.shape[:2]
    x = w - 230
    y = 25
    sp = 20

    cv2.putText(frame, f"Time: {elapsed:.1f}s", (x, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_WHITE, 2)
    y += sp
    cv2.putText(frame, f"Coverage: {coverage_pct:.1f}%", (x, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_WHITE, 2)

    y += sp + 5
    cv2.putText(frame, "Robot Trust:", (x, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, C_TEXT_DIM, 1)
    for rid_str in sorted(robot_states.keys(), key=lambda k: int(k)):
        st = robot_states[rid_str]
        y += sp
        trust = st.get('trust_in_leader')
        trust_val = f"{trust:.2f}" if trust is not None else "?"
        is_leader = st.get('state') == 'leader'
        label = f"R{rid_str}: {trust_val}" + (" (L)" if is_leader else "")
        fault = st.get('fault')
        if fault and fault != 'none':
            label += f" [{fault}]"
        color = C_WHITE
        if trust is not None:
            if trust < TRUST_BAD_THRESH:
                color = C_TRUST_BAD
            elif trust < TRUST_WARN_THRESH:
                color = C_TRUST_WARN
        cv2.putText(frame, label, (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)

    # Fault event banners
    for evt in fault_events:
        y += sp + 5
        cv2.putText(frame, evt, (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, C_FAULT_BG, 1)


def draw_fault_banner(frame, text: str, alpha: float):
    """Full-width banner across the middle for fault injection events."""
    if alpha <= 0:
        return
    h, w = frame.shape[:2]
    banner_h = 50
    y_top = h // 2 - banner_h // 2
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, y_top), (w, y_top + banner_h), C_FAULT_BG, -1)
    cv2.addWeighted(overlay, min(alpha, 0.7), frame, 1 - min(alpha, 0.7), 0, frame)
    sz = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)[0]
    cv2.putText(frame, text, ((w - sz[0]) // 2, y_top + banner_h // 2 + 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, C_WHITE, 2)


# ============== Robot-side log loader ==============
def load_robot_logs(log_dir: str) -> Dict[int, List[dict]]:
    """Load per-robot JSONL files. Handles both robot_*.jsonl and r{N}_robot_*.jsonl."""
    logs: Dict[int, List[dict]] = {}

    patterns = ["robot_*.jsonl", "r*_robot_*.jsonl", "raft_*.jsonl", "r*_raft_*.jsonl"]
    files = []
    for pat in patterns:
        files.extend(glob(os.path.join(log_dir, pat)))
    files = list(set(files))

    for f in files:
        basename = os.path.basename(f)
        rid = None
        if basename.startswith('r') and '_' in basename:
            try:
                rid = int(basename.split('_')[0][1:])
            except ValueError:
                pass
        if rid is None:
            parts = basename.replace('.jsonl', '').split('_')
            for i, p in enumerate(parts):
                if p in ('robot', 'raft') and i + 1 < len(parts):
                    try:
                        rid = int(parts[i + 1])
                    except ValueError:
                        pass
                    break
        if rid is None:
            continue

        logs[rid] = []
        with open(f, 'r') as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    logs[rid].append(json.loads(line))
                except (json.JSONDecodeError, ValueError):
                    pass
        print(f"  Loaded {len(logs[rid])} records for robot {rid} from {basename}")

    return logs


def interpolate_robot_log(logs: Dict[int, List[dict]], t: float) -> Dict[int, dict]:
    """Get nearest robot-side state at time t for each robot."""
    result = {}
    for rid, records in logs.items():
        if not records:
            continue
        best = min(records, key=lambda r: abs(r.get('t', 0) - t))
        if abs(best.get('t', 0) - t) < 5.0:
            result[rid] = best
    return result


# ============== Main pipeline ==============
def main():
    parser = argparse.ArgumentParser(
        description="REIP Post-Processing Video Overlay",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--trial-dir', type=str, required=True,
                        help="Server trial directory containing overhead.mp4 + robot_states.jsonl")
    parser.add_argument('--robot-logs', type=str, default=None,
                        help="Directory with per-robot JSONL logs (from run_trial.py)")
    parser.add_argument('--meta', type=str, default=None,
                        help="Path to trial_meta.json for fault injection timing")
    parser.add_argument('--output', type=str, default=None,
                        help="Output video path (default: <trial-dir>/overlay.mp4)")
    parser.add_argument('--fps', type=float, default=None,
                        help="Output FPS (default: match source video)")
    parser.add_argument('--start', type=float, default=0,
                        help="Skip this many seconds from the start")
    parser.add_argument('--duration', type=float, default=None,
                        help="Max duration to render (seconds)")
    parser.add_argument('--no-exploration', action='store_true',
                        help="Disable exploration coverage overlay")
    parser.add_argument('--no-arena', action='store_true',
                        help="Disable arena contour/grid overlay")
    parser.add_argument('--no-legend', action='store_true',
                        help="Disable legend")
    args = parser.parse_args()

    # --- Locate files ---
    video_path = os.path.join(args.trial_dir, "overhead.mp4")
    states_path = os.path.join(args.trial_dir, "robot_states.jsonl")
    if not os.path.isfile(video_path):
        print(f"ERROR: Video not found: {video_path}")
        return
    if not os.path.isfile(states_path):
        print(f"ERROR: State log not found: {states_path}")
        return

    output_path = args.output or os.path.join(args.trial_dir, "overlay.mp4")

    # --- Load trial metadata (fault timing) ---
    fault_events = []  # list of (time_offset_sec, description)
    trial_start_time = None
    if args.meta:
        with open(args.meta, 'r') as f:
            meta = json.load(f)
        trial_start_time = meta.get('start_time')
        if meta.get('fault1_actual_time') is not None:
            fault_events.append((meta['fault1_actual_time'],
                                 f"FAULT #{1}: {meta.get('fault_type', '?')} on R{meta.get('fault1_robot', '?')}"))
        if meta.get('fault2_actual_time') is not None:
            fault_events.append((meta['fault2_actual_time'],
                                 f"FAULT #{2}: {meta.get('fault_type', '?')} on R{meta.get('fault2_robot', '?')}"))
        print(f"Trial metadata: {len(fault_events)} fault event(s)")

    # --- Load robot-side logs (optional) ---
    rbot_logs = {}
    if args.robot_logs and os.path.isdir(args.robot_logs):
        print(f"Loading robot-side logs from {args.robot_logs}...")
        rbot_logs = load_robot_logs(args.robot_logs)
        # Auto-detect meta if not explicitly provided
        if not args.meta:
            meta_auto = os.path.join(args.robot_logs, 'trial_meta.json')
            if os.path.isfile(meta_auto):
                with open(meta_auto, 'r') as f:
                    meta = json.load(f)
                trial_start_time = meta.get('start_time')
                if meta.get('fault1_actual_time') is not None:
                    fault_events.append((meta['fault1_actual_time'],
                                         f"FAULT #1: {meta.get('fault_type', '?')} on R{meta.get('fault1_robot', '?')}"))
                if meta.get('fault2_actual_time') is not None:
                    fault_events.append((meta['fault2_actual_time'],
                                         f"FAULT #2: {meta.get('fault_type', '?')} on R{meta.get('fault2_robot', '?')}"))
                print(f"  Auto-loaded trial_meta.json: {len(fault_events)} fault event(s)")

    # --- Read JSONL: first line = calibration header, rest = per-frame state ---
    print(f"Loading state log: {states_path}")
    state_lines = []
    cal = Calibration()
    with open(states_path, 'r') as f:
        for i, line in enumerate(f):
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except (json.JSONDecodeError, ValueError):
                continue
            if rec.get('type') == 'calibration':
                cal = load_calibration(rec)
                print(f"  Calibration loaded (homography={'yes' if cal.homography is not None else 'no'})")
                continue
            state_lines.append(rec)

    print(f"  {len(state_lines)} state frames loaded")

    if cal.inv_homography is None:
        print("WARNING: No homography in calibration -- overlays will use raw coords")

    # --- Open source video ---
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"ERROR: Cannot open video {video_path}")
        return

    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    vid_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    vid_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    out_fps = args.fps or src_fps

    print(f"  Video: {vid_w}x{vid_h} @ {src_fps:.1f} fps, {total_frames} frames")
    print(f"  State lines: {len(state_lines)}  (expect ~{total_frames})")

    if len(state_lines) != total_frames and len(state_lines) > 0:
        print(f"  NOTE: Frame count mismatch -- will use min({total_frames}, {len(state_lines)})")

    # --- Build cell cache ---
    cell_polys, num_explorable = build_cell_cache(cal)

    # --- Determine time range ---
    skip_frames = int(args.start * src_fps)
    max_frames = int(args.duration * src_fps) if args.duration else total_frames
    t0 = state_lines[0]['t'] if state_lines else 0

    # --- Open output ---
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(output_path, fourcc, out_fps, (vid_w, vid_h))
    print(f"\nRendering to {output_path}...")

    # --- Accumulate visited cells ---
    visited: Set[Tuple[int, int]] = set()
    frame_idx = 0
    written = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if frame_idx < skip_frames:
            frame_idx += 1
            continue
        if written >= max_frames:
            break

        # Get matching state line
        state_idx = frame_idx
        if state_idx < len(state_lines):
            rec = state_lines[state_idx]
        elif state_lines:
            rec = state_lines[-1]
        else:
            rec = {'t': t0, 'poses': {}, 'robot_states': {}}

        elapsed = rec.get('t', t0) - t0
        poses = rec.get('poses', {})
        rstates = rec.get('robot_states', {})

        # Merge robot-side log data if available
        if rbot_logs:
            rside = interpolate_robot_log(rbot_logs, rec.get('t', 0))
            for rid, rdata in rside.items():
                rid_str = str(rid)
                if rid_str not in rstates:
                    rstates[rid_str] = {}
                for key in ('fault', 'classified_fault_type', 'bad_commands_received',
                            'first_bad_command_time', 'impeachment_time'):
                    if key in rdata and rdata[key] is not None:
                        rstates[rid_str][key] = rdata[key]

        # Accumulate visited cells
        for rid_str, st in rstates.items():
            cells = st.get('visited_cells')
            if cells:
                for cell in cells:
                    if isinstance(cell, (list, tuple)) and len(cell) == 2:
                        visited.add((int(cell[0]), int(cell[1])))

        # Also mark cells from pose positions
        for rid_str, pose in poses.items():
            cx = int(pose.get('x', 0) / cal.cell_size)
            cy = int(pose.get('y', 0) / cal.cell_size)
            if 0 <= cx < int(cal.arena_w / cal.cell_size) and \
               0 <= cy < int(cal.arena_h / cal.cell_size) and \
               not cal.is_wall_cell(cx, cy):
                visited.add((cx, cy))

        # --- Draw layers ---
        if not args.no_exploration:
            draw_exploration(frame, cell_polys, visited, num_explorable)

        if not args.no_arena:
            draw_arena(frame, cal)

        draw_robots(frame, cal, poses, rstates)

        if not args.no_legend:
            draw_legend(frame)

        coverage_pct = len(visited) / max(1, num_explorable) * 100

        # Compute active fault banners (show for 5 seconds after injection)
        active_fault_texts = []
        for ft, fdesc in fault_events:
            if 0 <= (elapsed - ft) < 5.0:
                alpha = 1.0 - (elapsed - ft) / 5.0
                draw_fault_banner(frame, fdesc, alpha)
            if elapsed >= ft:
                active_fault_texts.append(fdesc)

        draw_stats(frame, rstates, coverage_pct, elapsed, active_fault_texts)

        # Top-center timestamp
        h, w = frame.shape[:2]
        ts_text = f"t = {elapsed:.1f}s"
        ts_sz = cv2.getTextSize(ts_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
        cv2.putText(frame, ts_text, ((w - ts_sz[0]) // 2, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, C_WHITE, 2)

        writer.write(frame)
        written += 1
        frame_idx += 1

        if written % 100 == 0:
            print(f"  Frame {written}/{min(total_frames, len(state_lines))}"
                  f"  t={elapsed:.1f}s  coverage={coverage_pct:.0f}%")

    writer.release()
    cap.release()

    print(f"\nDone! {written} frames written to {output_path}")
    print(f"Final coverage: {len(visited)} cells ({len(visited)/max(1,num_explorable)*100:.0f}%)")

    # --- Save key frames for poster/paper ---
    keyframe_dir = os.path.join(args.trial_dir, "keyframes")
    os.makedirs(keyframe_dir, exist_ok=True)
    keyframe_times = [0, 0.25, 0.5, 0.75, 1.0]
    cap2 = cv2.VideoCapture(output_path)
    out_total = int(cap2.get(cv2.CAP_PROP_FRAME_COUNT))
    for i, frac in enumerate(keyframe_times):
        target_frame = min(int(frac * out_total), out_total - 1)
        cap2.set(cv2.CAP_PROP_POS_FRAMES, target_frame)
        ret, kf = cap2.read()
        if ret:
            path = os.path.join(keyframe_dir, f"keyframe_{i+1}.png")
            cv2.imwrite(path, kf)
            print(f"  Keyframe {i+1}: frame {target_frame} -> {path}")
    cap2.release()


if __name__ == "__main__":
    main()
