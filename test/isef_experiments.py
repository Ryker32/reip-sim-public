#!/usr/bin/env python3
"""
ISEF Experimental Data Collection - Rigorous Comparison Suite

Runs systematic experiments comparing REIP vs RAFT vs Decentralized baselines
under various fault conditions with statistical rigor:
  - N repeated trials per condition (randomized starting positions)
  - Real-time fault detection timing (tracks leader changes in broadcasts)
  - Two arena layouts (open + multiroom) for environmental diversity
  - Statistical summary: mean +/- std for all metrics

Usage:
  python test/isef_experiments.py                    # Full suite (all layouts, N=10)
  python test/isef_experiments.py --layout open      # Single layout
  python test/isef_experiments.py --trials 5         # Fewer trials (faster)
  python test/isef_experiments.py --quick            # Quick check: N=3, open only
"""

import subprocess
import time
import json
import socket
import os
import sys
import math
import random
import re
from collections import Counter
from datetime import datetime
from dataclasses import dataclass, field, asdict
from typing import Any, Iterable, List, Dict, Optional, Tuple
from collections import defaultdict
import heapq

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from hardware_fidelity import (
    
    DEFAULT_ARENA,
    DEFAULT_ARUCO,
    HARDWARE_CLONE_STARTS,
    SIM_MOTOR_PORT,
    SIM_PEER_RELAY_PORT,
    SIM_SENSOR_PORT,
    TOF_SENSOR_ANGLES_DEG,
)
from adversary_agent import AdaptiveAdversary, AdversaryAction
# ==================== Configuration ====================
REIP_SCRIPT = "robot/reip_node.py"
RAFT_SCRIPT = "robot/baselines/raft_node.py"
NUM_ROBOTS = 5
EXPERIMENT_DURATION = 120   # seconds per trial
FAULT_INJECT_TIME = 10      # first fault injection (earlier = more impact)
FAULT_INJECT_TIME_2 = 30    # second bad_leader injection on current leader
TRIALS_PER_CONDITION = 10   # repeated trials for statistics

# After initial leader election settles (~5s), leader changes before
# fault injection count as false positives.
SETTLE_TIME = 5

UDP_POSITION_PORT = 5100
UDP_PEER_PORT = 5200
UDP_FAULT_PORT = 5300
SIM_PORT_OFFSET_BASE = 1000

ARENA_WIDTH = DEFAULT_ARENA.width_mm
ARENA_HEIGHT = DEFAULT_ARENA.height_mm
CELL_SIZE = DEFAULT_ARENA.cell_size_mm

# Each layout: { "walls": [...], "width": W, "height": H, "start_zone": (x0,x1,y0,y1) }
# width/height default to ARENA_WIDTH/ARENA_HEIGHT if omitted.
# start_zone defines where robots spawn (defaults to full arena).
ARENA_LAYOUTS = {
    "open": [],
    "multiroom": [
        (
            DEFAULT_ARENA.interior_wall_x_left_mm,
            0,
            DEFAULT_ARENA.interior_wall_x_left_mm,
            DEFAULT_ARENA.interior_wall_y_end_mm,
        ),
        (
            DEFAULT_ARENA.interior_wall_x_right_mm,
            0,
            DEFAULT_ARENA.interior_wall_x_right_mm,
            DEFAULT_ARENA.interior_wall_y_end_mm,
        ),
    ],
    # ---- Scaled complexity layouts (sim-only, post hardware validation) ----
    # Same arena size as hardware, but L-shaped wall creates 3 zones
    # Tests sustained coordination demand in matched dimensions
    "complex_L": [
        (1000, 0, 1000, 1200),     # Vertical wall (same as multiroom)
        (1000, 600, 1600, 600),     # Horizontal extension creating alcove
    ],
    # Larger arena: 4x area, 4 rooms with connecting passages
    # Tests REIP scalability beyond hardware testbed
    # Note: uses custom dimensions (4000x3000), passed to robot nodes
    "large_office": [
        # Central vertical wall (passage at y=1200-1500 and y=2700-3000)
        (2000, 0, 2000, 1200),      # Lower vertical segment
        (2000, 1500, 2000, 2700),   # Upper vertical segment
        # Left horizontal wall (passage at x=800-1200)
        (0, 1500, 800, 1500),       # Left segment
        (1200, 1500, 2000, 1500),   # Right segment up to center
        # Right horizontal wall (passage at x=2800-3200)
        (2000, 1500, 2800, 1500),   # Left segment from center
        (3200, 1500, 4000, 1500),   # Right segment
    ],
}

# Per-layout arena dimensions (overrides for scaled layouts)
ARENA_DIMENSIONS = {
    "large_office": (4000, 3000),
}

# Per-layout starting zones: (x_min, x_max, y_min, y_max)
ARENA_START_ZONES = {
    "multiroom": (150, 850, 150, 1350),       # Room A (left of wall)
    "complex_L": (150, 850, 150, 1350),        # Room A (left of wall)
    "large_office": (150, 1800, 150, 1350),    # Lower-left quadrant (Room 1)
}

START_PRESETS = {
    "hardware_clone_20260308": HARDWARE_CLONE_STARTS,
}

# ==================== Data Classes ====================
@dataclass
class ExperimentResult:
    name: str
    controller: str           # 'reip', 'raft', 'decentralized'
    fault_type: Optional[str]
    fault_robot: Optional[int]
    layout: str
    trial: int
    seed: int
    final_coverage: float
    time_to_50: Optional[float]            # seconds to reach 50% coverage
    time_to_60: Optional[float]            # seconds to reach 60% coverage
    time_to_80: Optional[float]            # seconds to reach 80% coverage
    time_to_detection: Optional[float]     # seconds from fault 1 to first impeachment
    time_to_first_suspicion: Optional[float]  # seconds from fault 1 to first trust decay
    time_to_detection_2: Optional[float]   # seconds from fault 2 to second impeachment
    num_leader_changes: int
    false_positives: int                   # leader changes between settle and fault
    coverage_at_90s: Optional[float]       # coverage at 90s mark
    coverage_timeline: List[Tuple[float, float]] = field(default_factory=list)
    fault_schedule_name: Optional[str] = None
    fault_events: List[Dict[str, Any]] = field(default_factory=list)
    ablation: Optional[str] = None        # 'no_trust', 'no_causality', 'no_direction', or None
    leader_settle_time: Optional[float] = None
    divider_crossing_time: Optional[float] = None
    mission_complete_time: Optional[float] = None
    peer_emergency_count: int = 0
    min_peer_separation_mm: Optional[float] = None
    min_wall_clearance_mm: Optional[float] = None
    coverage_order: List[Tuple[int, int]] = field(default_factory=list)


@dataclass
class ImpairmentProfile:
    name: str
    startup_blind_s: float = 0.0
    position_delay_s: float = 0.0
    position_jitter_s: float = 0.0
    position_loss_prob: float = 0.0
    position_burst_prob: float = 0.0
    position_burst_len: Tuple[int, int] = (0, 0)
    position_noise_mm: float = 0.0
    heading_noise_rad: float = 0.0
    radial_bias_scale: float = 1.0
    divider_occlusion: bool = False
    peer_delay_s: float = 0.0
    peer_jitter_s: float = 0.0
    peer_loss_prob: float = 0.0
    peer_burst_prob: float = 0.0
    peer_burst_len: Tuple[int, int] = (0, 0)
    fault_delay_s: float = 0.0
    fault_jitter_s: float = 0.0
    fault_loss_prob: float = 0.0
    divider_queue_half_width_mm: float = 220.0
    divider_queue_depth_mm: float = 220.0
    divider_queue_drag: float = 0.0
    crowd_range_mm: float = 280.0
    crowd_position_noise_mm: float = 0.0
    crowd_heading_noise_rad: float = 0.0
    crowd_position_pull: float = 0.0
    crowd_occlusion_prob: float = 0.0
    peer_tof_mask_prob: float = 0.0
    peer_tof_inflate_mm: float = 0.0
    base_drive_scale: float = 1.0
    turn_drive_drag: float = 0.0
    wall_drive_drag: float = 0.0
    turn_position_loss_prob: float = 0.0
    turn_position_noise_mm: float = 0.0
    turn_heading_noise_rad: float = 0.0


@dataclass(frozen=True)
class FaultEventSpec:
    time_s: float
    fault_type: str
    target_mode: str = "fixed_robot"   # fixed_robot | current_leader
    robot_id: Optional[int] = None
    label: Optional[str] = None


@dataclass
class SimRobotPlant:
    x: float
    y: float
    theta: float
    left_pwm: float = 0.0
    right_pwm: float = 0.0
    left_mm_s: float = 0.0
    right_mm_s: float = 0.0
    encoder_left: int = 0
    encoder_right: int = 0
    contact_drag: float = 0.0
    contact_turn_bias_mm_s: float = 0.0
    contact_memory_s: float = 0.0
    turn_metric: float = 0.0
    wall_proximity: float = 0.0


PEER_EMERGENCY_REFRACTORY_S = 3.0


def _count_peer_emergency_incidents(records: Iterable[dict], refractory_s: float = PEER_EMERGENCY_REFRACTORY_S) -> int:
    """Count distinct peer-emergency incidents instead of raw mode toggles.

    The controller can briefly alternate between `peer_emergency` and escape /
    recovery commands while resolving one physical congestion event. Using a
    short refractory window keeps the sim and hardware metrics aligned to
    incident-level behavior rather than edge flicker in the low-level loop.
    """
    count = 0
    prev_active = False
    last_incident_t = None
    for rec in records:
        active = (
            rec.get("command_source") == "peer_emergency" or
            rec.get("stop_reason") == "peer_emergency"
        )
        if active and not prev_active:
            rec_t = rec.get("t")
            current_t = float(rec_t) if rec_t is not None else None
            if current_t is None or last_incident_t is None or (current_t - last_incident_t) > refractory_s:
                count += 1
                last_incident_t = current_t
        prev_active = active
    return count


IMPAIRMENT_PRESETS = {
    "ideal": ImpairmentProfile(name="ideal"),
    "hardware_clone_clean": ImpairmentProfile(
        name="hardware_clone_clean",
        startup_blind_s=DEFAULT_ARUCO.stable_corner_frames / DEFAULT_ARUCO.position_rate_hz,
        position_delay_s=0.025,
        position_jitter_s=0.015,
        position_loss_prob=0.015,
        position_burst_prob=0.01,
        position_burst_len=(2, 4),
        position_noise_mm=4.0,
        heading_noise_rad=math.radians(2.0),
        radial_bias_scale=1.002,
        peer_delay_s=0.06,
        peer_jitter_s=0.025,
        peer_loss_prob=0.008,
        peer_burst_prob=0.01,
        peer_burst_len=(2, 4),
        fault_delay_s=0.03,
        fault_jitter_s=0.01,
        divider_queue_half_width_mm=270.0,
        divider_queue_depth_mm=320.0,
        divider_queue_drag=0.04,
        crowd_range_mm=320.0,
        crowd_position_noise_mm=7.0,
        crowd_heading_noise_rad=math.radians(2.2),
        crowd_position_pull=0.14,
        crowd_occlusion_prob=0.02,
        peer_tof_mask_prob=0.0,
        peer_tof_inflate_mm=2.0,
        base_drive_scale=0.91,
        turn_drive_drag=0.30,
        wall_drive_drag=0.07,
        turn_position_loss_prob=0.02,
        turn_position_noise_mm=6.0,
        turn_heading_noise_rad=math.radians(2.0),
    ),
    "hardware_clone_occlusion": ImpairmentProfile(
        name="hardware_clone_occlusion",
        startup_blind_s=DEFAULT_ARUCO.stable_corner_frames / DEFAULT_ARUCO.position_rate_hz,
        position_delay_s=0.03,
        position_jitter_s=0.02,
        position_loss_prob=0.02,
        position_burst_prob=0.02,
        position_burst_len=(2, 5),
        position_noise_mm=5.0,
        heading_noise_rad=math.radians(2.5),
        radial_bias_scale=1.003,
        divider_occlusion=True,
        peer_delay_s=0.09,
        peer_jitter_s=0.04,
        peer_loss_prob=0.015,
        peer_burst_prob=0.015,
        peer_burst_len=(2, 5),
        fault_delay_s=0.04,
        fault_jitter_s=0.015,
        divider_queue_half_width_mm=290.0,
        divider_queue_depth_mm=340.0,
        divider_queue_drag=0.34,
        crowd_range_mm=340.0,
        crowd_position_noise_mm=34.0,
        crowd_heading_noise_rad=math.radians(6.0),
        crowd_position_pull=0.88,
        crowd_occlusion_prob=0.20,
        peer_tof_mask_prob=0.08,
        peer_tof_inflate_mm=32.0,
        base_drive_scale=0.80,
        turn_drive_drag=0.60,
        wall_drive_drag=0.24,
        turn_position_loss_prob=0.14,
        turn_position_noise_mm=20.0,
        turn_heading_noise_rad=math.radians(5.0),
    ),
    "hardware_clone_blackout": ImpairmentProfile(
        name="hardware_clone_blackout",
        startup_blind_s=DEFAULT_ARUCO.stable_corner_frames / DEFAULT_ARUCO.position_rate_hz + 0.35,
        position_delay_s=0.05,
        position_jitter_s=0.03,
        position_loss_prob=0.12,
        position_burst_prob=0.12,
        position_burst_len=(4, 9),
        position_noise_mm=9.0,
        heading_noise_rad=math.radians(4.0),
        radial_bias_scale=1.004,
        divider_occlusion=True,
        peer_delay_s=0.16,
        peer_jitter_s=0.08,
        peer_loss_prob=0.22,
        peer_burst_prob=0.10,
        peer_burst_len=(4, 10),
        fault_delay_s=0.06,
        fault_jitter_s=0.02,
        fault_loss_prob=0.02,
        divider_queue_half_width_mm=320.0,
        divider_queue_depth_mm=360.0,
        divider_queue_drag=0.46,
        crowd_range_mm=360.0,
        crowd_position_noise_mm=42.0,
        crowd_heading_noise_rad=math.radians(7.5),
        crowd_position_pull=0.95,
        crowd_occlusion_prob=0.28,
        peer_tof_mask_prob=0.14,
        peer_tof_inflate_mm=40.0,
        base_drive_scale=0.77,
        turn_drive_drag=0.66,
        wall_drive_drag=0.28,
        turn_position_loss_prob=0.18,
        turn_position_noise_mm=24.0,
        turn_heading_noise_rad=math.radians(6.0),
    ),
}


FAULT_SCHEDULE_CHOICES = ("default", "cyber_burst", "adaptive")


def build_fault_schedule(
    fault_type: Optional[str],
    fault_robot: int,
    fault_schedule_name: Optional[str],
) -> List[FaultEventSpec]:
    """Build the timed fault events for a trial.

    `default` reproduces the historical behavior:
      - one injection at 10s for all non-clean faults
      - a second injection at 30s on the current leader for leader-centric faults

    `cyber_burst` applies repeated attacks every 3 seconds after the initial hit,
    targeting the currently active leader to model sustained adversarial pressure.
    """
    if fault_type in (None, "", "none", "clear"):
        return []

    schedule_name = (fault_schedule_name or "default").lower()
    if schedule_name not in FAULT_SCHEDULE_CHOICES:
        raise ValueError(
            f"Unsupported fault schedule '{fault_schedule_name}'. "
            f"Choose from {', '.join(FAULT_SCHEDULE_CHOICES)}."
        )

    events = [
        FaultEventSpec(
            time_s=FAULT_INJECT_TIME,
            fault_type=fault_type,
            target_mode="fixed_robot",
            robot_id=fault_robot,
            label="Fault 1",
        )
    ]

    if schedule_name == "default":
        if fault_type in ("bad_leader", "freeze_leader"):
            events.append(FaultEventSpec(
                time_s=FAULT_INJECT_TIME_2,
                fault_type=fault_type,
                target_mode="current_leader",
                label="Fault 2",
            ))
        return events

    for idx, event_t in enumerate((13.0, 16.0, 19.0, 22.0, 25.0, 28.0, 31.0, 34.0), start=2):
        events.append(FaultEventSpec(
            time_s=event_t,
            fault_type=fault_type,
            target_mode="current_leader",
            label=f"Burst {idx - 1}",
        ))
    return events


def _schedule_to_metadata(events: List[FaultEventSpec]) -> List[Dict[str, Any]]:
    return [
        {
            "time_s": ev.time_s,
            "fault_type": ev.fault_type,
            "target_mode": ev.target_mode,
            "robot_id": ev.robot_id,
            "label": ev.label,
        }
        for ev in events
    ]


def _schedule_summary(events: List[FaultEventSpec]) -> str:
    if not events:
        return "none"
    return ", ".join(
        f"{ev.time_s:.0f}s:{ev.fault_type}@"
        f"{'leader' if ev.target_mode == 'current_leader' else f'R{ev.robot_id}'}"
        for ev in events
    )


# ==================== Experiment Runner ====================
class ExperimentRunner:
    def __init__(self, output_dir: str = "experiments", port_offset: int = 0,
                 sim_mode: str = "hardware_clone",
                 impairment_preset: str = "hardware_clone_clean"):
        self._fixed_positions_for_next_trial = None
        self.output_dir = output_dir
        self.layout = "open"
        self.walls: List[Tuple[int, int, int, int]] = []
        self.port_offset = port_offset  # For parallel execution
        self.sim_mode = sim_mode
        self.geometry = DEFAULT_ARENA
        self.aruco = DEFAULT_ARUCO
        self.impairment = IMPAIRMENT_PRESETS.get(
            impairment_preset,
            IMPAIRMENT_PRESETS["hardware_clone_clean"],
        )
        os.makedirs(output_dir, exist_ok=True)

        # Port bases (offset for parallel workers)
        self._pos_port = UDP_POSITION_PORT + port_offset
        self._peer_port = UDP_PEER_PORT + port_offset
        self._fault_port = UDP_FAULT_PORT + port_offset
        self._sim_motor_port = SIM_MOTOR_PORT + port_offset
        self._sim_peer_relay_port = SIM_PEER_RELAY_PORT + port_offset
        self._sim_sensor_port = SIM_SENSOR_PORT + port_offset

        # Sockets
        self.pos_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sensor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.fault_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.state_socket.bind(('127.0.0.1', self._sim_peer_relay_port))
        self.state_socket.setblocking(False)
        self.motor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.motor_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.motor_socket.bind(('127.0.0.1', self._sim_motor_port))
        self.motor_socket.setblocking(False)

        self.robot_processes: List[subprocess.Popen] = []
        self.sim_positions: Dict[int, Tuple[float, float, float]] = {}
        self.robot_plants: Dict[int, SimRobotPlant] = {}
        self.stuck_counters: Dict[int, int] = {}
        self.prev_positions: Dict[int, Tuple[float, float]] = {}
        self.stuck_overrides: Dict[int, Optional[list]] = {}  # locked override targets
        self.motor_faults: Dict[int, Optional[str]] = {}  # rid -> 'spin', 'stop', etc.
        self.path_waypoints: Dict[int, list] = {}       # rid -> [(x,y), ...] waypoints
        self.path_target_key: Dict[int, tuple] = {}     # rid -> (tx,ty) target that generated the path
        self.motor_intents: Dict[int, Tuple[float, float]] = {}
        self._scheduled_messages: List[Tuple[float, int, Tuple[str, Tuple[str, int], bytes]]] = []
        self._message_seq = 0
        self._position_burst_remaining = 0
        self._peer_burst_remaining = 0
        self._next_position_publish = 0.0

        # Per-experiment state (reset each trial)
        self.visited_cells: set = set()
        self.coverage_history: List[Tuple[float, float]] = []
        self.tracked_leaders: Dict[int, Optional[int]] = {}  # rid -> leader they report
        self.last_targets: Dict[int, Optional[list]] = {}  # rid -> last known nav target
        self.last_states_by_robot: Dict[int, dict] = {}
        self.coverage_order: List[Tuple[int, int]] = []

    def set_layout(self, layout: str):
        """Switch arena layout between experiments."""
        self.layout = layout
        self.walls = list(ARENA_LAYOUTS.get(layout, []))
        # Per-layout arena dimensions (for scaled layouts)
        dims = ARENA_DIMENSIONS.get(layout)
        if self.sim_mode == "hardware_clone" and layout == "multiroom":
            self.arena_width, self.arena_height = self.geometry.width_mm, self.geometry.height_mm
        elif dims:
            self.arena_width, self.arena_height = dims
        else:
            self.arena_width, self.arena_height = ARENA_WIDTH, ARENA_HEIGHT

    def _record_visited_cell(self, cell: Optional[Tuple[int, int]]):
        if cell is None:
            return
        cell = (int(cell[0]), int(cell[1]))
        if cell not in self.visited_cells:
            self.visited_cells.add(cell)
            self.coverage_order.append(cell)

    def _current_peer_separation_mm(self) -> Optional[float]:
        if self.sim_mode == "hardware_clone":
            positions = list(self.robot_plants.values())
        else:
            positions = [type("Pos", (), {"x": x, "y": y}) for x, y, _ in self.sim_positions.values()]
        if len(positions) < 2:
            return None
        best = None
        for idx, a in enumerate(positions):
            for b in positions[idx + 1:]:
                dist = math.hypot(b.x - a.x, b.y - a.y)
                best = dist if best is None else min(best, dist)
        return best

    def _current_wall_clearance_mm(self) -> Optional[float]:
        if self.sim_mode == "hardware_clone" and self.layout == "multiroom":
            clearances = [
                self.geometry.distance_to_config_space_obstacle(plant.x, plant.y)
                for plant in self.robot_plants.values()
            ]
        else:
            clearances = []
            for x, y, _ in self.sim_positions.values():
                dists = [x, self.arena_width - x, y, self.arena_height - y]
                clearances.append(min(dists))
        return min(clearances) if clearances else None

    def _parse_robot_behavior_metrics(self) -> Dict[str, int]:
        log_dir = getattr(self, '_current_log_dir', os.path.join(self.output_dir, "logs"))
        peer_emergency_count = 0
        repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        for i in range(1, NUM_ROBOTS + 1):
            stdout_log_path = os.path.join(log_dir, f"robot_{i}.log")
            if not os.path.exists(stdout_log_path):
                continue
            jsonl_path = None
            try:
                with open(stdout_log_path, 'r', errors='replace') as f:
                    for line in f:
                        if "Logging to:" not in line:
                            continue
                        rel_path = line.split("Logging to:", 1)[1].strip()
                        candidate = rel_path if os.path.isabs(rel_path) else os.path.join(repo_root, rel_path)
                        if os.path.exists(candidate):
                            jsonl_path = candidate
                        break
                if not jsonl_path:
                    continue
                records = []
                with open(jsonl_path, 'r', errors='replace') as f:
                    for line in f:
                        line = line.strip()
                        if not line.startswith('{'):
                            continue
                        try:
                            rec = json.loads(line)
                        except json.JSONDecodeError:
                            continue
                        records.append(rec)
                peer_emergency_count += _count_peer_emergency_incidents(records)
            except OSError:
                continue
        return {
            'peer_emergency_count': peer_emergency_count,
        }

    def _wait_for_initial_leader_consensus(self, timeout_s: float = 8.0) -> Optional[int]:
        """Wait for a majority leader report before opening the trial gate.

        Hardware trials intentionally pause for localization plus leader election
        before the trial clock starts. Mirroring that here keeps the clean sim
        runs from measuring startup election churn that the hardware logs exclude.
        """
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            self.receive_motor_intents()
            self._flush_scheduled_messages()
            self.send_sensor_feedback()
            self._emit_position_packets()
            self._flush_scheduled_messages()
            states = self.receive_states()
            leader_reports = [
                st.get('leader_id')
                for st in (list(states.values()) or list(self.last_states_by_robot.values()))
                if st.get('leader_id') is not None
            ]
            if leader_reports:
                counts = Counter(leader_reports)
                leader_id, count = counts.most_common(1)[0]
                if leader_id is not None and count >= max(1, NUM_ROBOTS // 2 + 1):
                    return leader_id
            time.sleep(0.05)
        return None

    # -------------------- Process Management --------------------
    def start_robots(self, script: str, extra_args: List[str] = None,
                     seed: int = 42, experiment_name: str = "",
                     fixed_positions: Dict[int, Tuple[float, float, float]] = None):
        """Start robot processes with seed-based random starting positions."""
        # Use per-experiment log directory to avoid overwriting
        if experiment_name:
            log_dir = os.path.join(self.output_dir, "logs", experiment_name)
        else:
            log_dir = os.path.join(self.output_dir, "logs")
        os.makedirs(log_dir, exist_ok=True)
        self._current_log_dir = log_dir  # Save for log parsing

        rng = random.Random(seed)

        if fixed_positions is None and self.sim_mode == "hardware_clone" and self.layout == "multiroom":
            fixed_positions = START_PRESETS.get("hardware_clone_20260308")

        for i in range(1, NUM_ROBOTS + 1):
            log_file = open(os.path.join(log_dir, f"robot_{i}.log"), 'w')
            cmd = [sys.executable, "-u", script, str(i), "--sim"]
            if self.port_offset:
                cmd.extend(["--port-base", str(self.port_offset)])
            if extra_args:
                cmd.extend(extra_args)
            proc = subprocess.Popen(
                cmd,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            )
            self.robot_processes.append(proc)

            # Starting position: use fixed (hardware) positions if provided,
            # otherwise randomize (seed-controlled for reproducibility).
            if fixed_positions and i in fixed_positions:
                sx, sy, theta = fixed_positions[i]
            else:
                start_zone = ARENA_START_ZONES.get(self.layout)
                if start_zone:
                    sx = rng.uniform(start_zone[0], start_zone[1])
                    sy = rng.uniform(start_zone[2], start_zone[3])
                else:
                    clearance = self.geometry.outer_wall_margin_mm + 20
                    sx = rng.uniform(clearance, self.arena_width - clearance)
                    sy = rng.uniform(clearance, self.arena_height - clearance)
                theta = rng.uniform(0, 2 * math.pi)

                # Reject positions inside walls
                while self._collides_with_wall(sx, sy):
                    if start_zone:
                        sx = rng.uniform(start_zone[0], start_zone[1])
                        sy = rng.uniform(start_zone[2], start_zone[3])
                    else:
                        clearance = self.geometry.outer_wall_margin_mm + 20
                        sx = rng.uniform(clearance, self.arena_width - clearance)
                        sy = rng.uniform(clearance, self.arena_height - clearance)

            self.sim_positions[i] = (sx, sy, theta)
            self.robot_plants[i] = SimRobotPlant(x=sx, y=sy, theta=theta)
            self.stuck_counters[i] = 0
            self.prev_positions[i] = (sx, sy)
            self.motor_intents[i] = (0.0, 0.0)

        time.sleep(2)  # Let processes initialize

    def stop_robots(self):
        """Stop all robot processes"""
        for proc in self.robot_processes:
            proc.terminate()
        for proc in self.robot_processes:
            try:
                proc.wait(timeout=2)
            except Exception:
                proc.kill()
        self.robot_processes = []

    # -------------------- Communication --------------------
    def _should_drop(self, channel: str) -> bool:
        if channel == 'position':
            if self._position_burst_remaining > 0:
                self._position_burst_remaining -= 1
                return True
            if random.random() < self.impairment.position_burst_prob:
                lo, hi = self.impairment.position_burst_len
                self._position_burst_remaining = random.randint(lo, hi) if hi > 0 else 0
                return True
            return random.random() < self.impairment.position_loss_prob
        if channel == 'peer':
            if self._peer_burst_remaining > 0:
                self._peer_burst_remaining -= 1
                return True
            if random.random() < self.impairment.peer_burst_prob:
                lo, hi = self.impairment.peer_burst_len
                self._peer_burst_remaining = random.randint(lo, hi) if hi > 0 else 0
                return True
            return random.random() < self.impairment.peer_loss_prob
        return random.random() < self.impairment.fault_loss_prob

    def _schedule_message(self, channel: str, payload: dict, target: Tuple[str, int],
                          delay_s: float, jitter_s: float):
        if self._should_drop(channel):
            return
        deliver_at = time.time() + max(0.0, delay_s + random.uniform(-jitter_s, jitter_s))
        self._message_seq += 1
        heapq.heappush(
            self._scheduled_messages,
            (deliver_at, self._message_seq, (channel, target, json.dumps(payload).encode())),
        )

    def _flush_scheduled_messages(self):
        now = time.time()
        while self._scheduled_messages and self._scheduled_messages[0][0] <= now:
            _, _, (_, target, payload) = heapq.heappop(self._scheduled_messages)
            if target[1] >= self._sim_sensor_port:
                self.sensor_socket.sendto(payload, target)
            elif target[1] >= self._fault_port and target[1] < self._fault_port + 1000:
                self.fault_socket.sendto(payload, target)
            else:
                self.pos_socket.sendto(payload, target)

    def _position_occluded(self, rid: int, x: float, y: float) -> bool:
        if not self.impairment.divider_occlusion:
            return False
        near_divider = (
            y < self.geometry.interior_wall_y_end_mm + 40.0 and
            self.geometry.interior_wall_x_left_mm - 60.0 <= x <= self.geometry.interior_wall_x_right_mm + 60.0
        )
        return near_divider

    def _divider_queue_state(self, rid: int) -> Tuple[bool, int, Optional[int], Optional[float], float]:
        plant = self.robot_plants[rid]
        x_band = self.impairment.divider_queue_half_width_mm
        y_band = self.impairment.divider_queue_depth_mm
        center_x = self.geometry.interior_wall_x_mm
        tip_y = self.geometry.interior_wall_y_end_mm
        in_queue = (
            abs(plant.x - center_x) <= x_band and
            abs(plant.y - tip_y) <= y_band
        )
        queue_count = 0
        nearest_peer_id = None
        nearest_peer_dist = None
        for other_id, other in self.robot_plants.items():
            if other_id == rid:
                continue
            dist = math.hypot(other.x - plant.x, other.y - plant.y)
            if nearest_peer_dist is None or dist < nearest_peer_dist:
                nearest_peer_dist = dist
                nearest_peer_id = other_id
            if (
                abs(other.x - center_x) <= x_band and
                abs(other.y - tip_y) <= y_band
            ):
                queue_count += 1
        if in_queue:
            queue_count += 1
        crowding = 0.0
        if in_queue and queue_count > 1:
            crowding = max(crowding, min(1.0, (queue_count - 1) / 3.0))
        if nearest_peer_dist is not None and nearest_peer_dist < self.impairment.crowd_range_mm:
            peer_crowding = (self.impairment.crowd_range_mm - nearest_peer_dist) / max(self.impairment.crowd_range_mm, 1.0)
            crowding = max(crowding, min(1.0, peer_crowding))
        return in_queue, queue_count, nearest_peer_id, nearest_peer_dist, crowding

    def _emit_position_packets(self):
        if time.time() < self._next_position_publish:
            return
        self._next_position_publish = time.time() + 1.0 / self.aruco.position_rate_hz
        calibration_ready = self._trial_start_wall_time + self.impairment.startup_blind_s
        for rid, plant in self.robot_plants.items():
            if time.time() < calibration_ready or self._position_occluded(rid, plant.x, plant.y):
                continue
            in_queue, queue_count, nearest_peer_id, nearest_peer_dist, crowding = self._divider_queue_state(rid)
            if crowding > 0.0 and random.random() < self.impairment.crowd_occlusion_prob * crowding:
                continue
            turn_loss_prob = self.impairment.turn_position_loss_prob * (0.35 + 0.65 * plant.turn_metric)
            if (plant.turn_metric > 0.25 or plant.contact_memory_s > 0.0) and random.random() < turn_loss_prob:
                continue
            dx = plant.x - self.aruco.camera_center_x_mm
            dy = plant.y - self.aruco.camera_center_y_mm
            biased_x = self.aruco.camera_center_x_mm + dx * self.impairment.radial_bias_scale
            biased_y = self.aruco.camera_center_y_mm + dy * self.impairment.radial_bias_scale
            turn_noise_mm = self.impairment.turn_position_noise_mm * plant.turn_metric * (1.0 + 0.5 * plant.wall_proximity)
            turn_heading_noise = self.impairment.turn_heading_noise_rad * plant.turn_metric
            noise_mm = self.impairment.position_noise_mm + self.impairment.crowd_position_noise_mm * crowding + turn_noise_mm
            heading_noise = self.impairment.heading_noise_rad + self.impairment.crowd_heading_noise_rad * crowding + turn_heading_noise
            x = biased_x + random.gauss(0.0, noise_mm)
            y = biased_y + random.gauss(0.0, noise_mm)
            theta = plant.theta + random.gauss(0.0, heading_noise)
            if nearest_peer_id is not None and crowding > 0.0:
                other = self.robot_plants[nearest_peer_id]
                peer_scale = 1.0
                if nearest_peer_dist is not None:
                    peer_scale = max(0.45, min(1.0, 1.25 - nearest_peer_dist / max(self.impairment.crowd_range_mm, 1.0)))
                peer_pull = self.impairment.crowd_position_pull * crowding * peer_scale
                x = x * (1.0 - peer_pull) + other.x * peer_pull
                y = y * (1.0 - peer_pull) + other.y * peer_pull
            if in_queue and queue_count > 1:
                divider_pull = min(0.55, 0.18 * crowding + 0.06 * max(0, queue_count - 1))
                x = x * (1.0 - divider_pull) + self.geometry.interior_wall_x_mm * divider_pull
            cell = self.geometry.pos_to_cell(x, y)
            if cell is not None and not self.geometry.is_wall_cell(*cell):
                self._record_visited_cell(cell)
            msg = {
                'type': 'position',
                'robot_id': rid,
                'x': x,
                'y': y,
                'theta': theta,
                'timestamp': time.time(),
            }
            self._schedule_message(
                'position',
                msg,
                ('127.0.0.1', self._pos_port + rid),
                self.impairment.position_delay_s,
                self.impairment.position_jitter_s,
            )

    def inject_fault(self, robot_id: int, fault_type: str):
        """Inject fault into robot through the same delayed channel the hardware uses."""
        msg = {
            'type': 'fault_inject',
            'robot_id': robot_id,
            'fault': fault_type,
            'timestamp': time.time()
        }
        self._schedule_message(
            'fault',
            msg,
            ('127.0.0.1', self._fault_port + robot_id),
            self.impairment.fault_delay_s,
            self.impairment.fault_jitter_s,
        )

    def receive_motor_intents(self):
        try:
            while True:
                data, _ = self.motor_socket.recvfrom(8192)
                msg = json.loads(data.decode())
                if msg.get('type') != 'motor_intent':
                    continue
                rid = msg.get('robot_id')
                if rid:
                    self.motor_intents[rid] = (
                        float(msg.get('left', 0.0)),
                        float(msg.get('right', 0.0)),
                    )
        except (socket.timeout, BlockingIOError, ConnectionResetError, OSError):
            pass

    def receive_states(self) -> Dict[int, dict]:
        """Receive state broadcasts from robots and relay them with impairments."""
        states = {}
        try:
            while True:
                data, _ = self.state_socket.recvfrom(8192)
                msg = json.loads(data.decode())
                msg_type = msg.get('type')
                if msg_type not in ('peer_state', 'leader_ack', 'heartbeat', 'heartbeat_ack', 'vote_request', 'vote_response'):
                    continue
                src = (
                    msg.get('robot_id')
                    or msg.get('voter_id')
                    or msg.get('leader_id')
                    or msg.get('candidate_id')
                )
                if msg_type == 'peer_state':
                    rid = msg.get('robot_id')
                    if rid:
                        states[rid] = msg
                        self.last_states_by_robot[rid] = msg
                        if self.sim_mode != "hardware_clone":
                            for cell in msg.get('visited_cells', []):
                                self._record_visited_cell(tuple(cell))
                for peer_id in range(1, NUM_ROBOTS + 1):
                    if peer_id == src:
                        continue
                    self._schedule_message(
                        'peer',
                        msg,
                        ('127.0.0.1', self._peer_port + peer_id),
                        self.impairment.peer_delay_s,
                        self.impairment.peer_jitter_s,
                    )
        except (socket.timeout, BlockingIOError, ConnectionResetError, OSError):
            pass
        return states

    def send_sensor_feedback(self):
        for rid, plant in self.robot_plants.items():
            msg = {
                'type': 'sim_sensor',
                'robot_id': rid,
                'tof': self._compute_tof_for_robot(rid),
                'encoders': [plant.encoder_left, plant.encoder_right],
                'timestamp': time.time(),
            }
            self.sensor_socket.sendto(
                json.dumps(msg).encode(),
                ('127.0.0.1', self._sim_sensor_port + rid),
            )

    def _send_bootstrap_packets(self):
        """Give each robot one immediate bootstrap packet on every local channel."""
        now = time.time()
        for rid, plant in self.robot_plants.items():
            start_msg = {
                'type': 'fault_inject',
                'robot_id': 0,
                'fault': 'start',
                'timestamp': now,
            }
            position_msg = {
                'type': 'position',
                'robot_id': rid,
                'x': plant.x,
                'y': plant.y,
                'theta': plant.theta,
                'timestamp': now,
            }
            sensor_msg = {
                'type': 'sim_sensor',
                'robot_id': rid,
                'tof': self._compute_tof_for_robot(rid),
                'encoders': [plant.encoder_left, plant.encoder_right],
                'timestamp': now,
            }
            self.fault_socket.sendto(
                json.dumps(start_msg).encode(),
                ('127.0.0.1', self._fault_port + rid),
            )
            self.pos_socket.sendto(
                json.dumps(position_msg).encode(),
                ('127.0.0.1', self._pos_port + rid),
            )
            self.sensor_socket.sendto(
                json.dumps(sensor_msg).encode(),
                ('127.0.0.1', self._sim_sensor_port + rid),
            )

    def _wait_for_robot_ready_states(self, timeout_s: float = 8.0) -> set[int]:
        """Wait until each robot has broadcast at least one peer state.

        On Windows localhost runs, the subprocesses can finish importing and print
        "Running..." before their network threads have actually started draining
        sockets. Sending the one-shot start packet before that point makes the
        canonical run nondeterministically stall at (0,0).
        """
        ready_ids = set(self.last_states_by_robot.keys())
        deadline = time.time() + timeout_s
        while time.time() < deadline and len(ready_ids) < NUM_ROBOTS:
            ready_ids.update(self.receive_states().keys())
            time.sleep(0.05)
        return ready_ids

    # -------------------- Physics / Wall Collision --------------------
    def _line_circle_collision(self, x1, y1, x2, y2, cx, cy, r):
        """Check if circle (cx,cy,r) overlaps line segment (x1,y1)-(x2,y2).
        Tangent contact (distance == radius) is NOT collision."""
        dx, dy = x2 - x1, y2 - y1
        fx, fy = x1 - cx, y1 - cy
        a = dx * dx + dy * dy
        if a < 1e-6:
            return math.hypot(cx - x1, cy - y1) < r
        b = 2 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - r * r
        disc = b * b - 4 * a * c
        if disc <= 0:
            return False
        disc = math.sqrt(disc)
        t1 = (-b - disc) / (2 * a)
        t2 = (-b + disc) / (2 * a)
        return (0 <= t1 <= 1) or (0 <= t2 <= 1) or (t1 < 0 and t2 > 1)

    def _collides_with_wall(self, x, y, radius=75):
        if self.sim_mode == "hardware_clone" and self.layout == "multiroom":
            return not self.geometry.is_free_point(x, y, wall_clearance_mm=self.geometry.outer_wall_margin_mm)
        for wx1, wy1, wx2, wy2 in self.walls:
            if self._line_circle_collision(wx1, wy1, wx2, wy2, x, y, radius):
                return True
        return False

    @staticmethod
    def _ccw(ax, ay, bx, by, cx, cy):
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)

    def _path_crosses_wall(self, x1, y1, x2, y2):
        """Check if direct path crosses any wall segment."""
        for wx1, wy1, wx2, wy2 in self.walls:
            d1 = self._ccw(x1, y1, x2, y2, wx1, wy1)
            d2 = self._ccw(x1, y1, x2, y2, wx2, wy2)
            d3 = self._ccw(wx1, wy1, wx2, wy2, x1, y1)
            d4 = self._ccw(wx1, wy1, wx2, wy2, x2, y2)
            if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
               ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
                return True
        return False

    def _find_reachable_unexplored(self, x, y):
        """Find nearest unexplored cell reachable via BFS (path distance, not Euclidean)."""
        cols, rows = self._build_nav_grid()
        sc = (max(0, min(cols - 1, int(x / CELL_SIZE))),
              max(0, min(rows - 1, int(y / CELL_SIZE))))

        # BFS from robot cell - returns cells in order of path distance
        from collections import deque
        queue = deque([sc])
        visited_bfs = {sc}
        while queue:
            cx, cy = queue.popleft()
            if (cx, cy) not in self.visited_cells:
                tx = (cx + 0.5) * CELL_SIZE
                ty = (cy + 0.5) * CELL_SIZE
                return (tx, ty)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if (0 <= nx < cols and 0 <= ny < rows
                        and (nx, ny) not in visited_bfs
                        and not self._wall_blocks_move(cx, cy, nx, ny)):
                    visited_bfs.add((nx, ny))
                    queue.append((nx, ny))

        # Absolute fallback - pick any unexplored cell
        for cx in range(cols):
            for cy in range(rows):
                if (cx, cy) not in self.visited_cells:
                    return ((cx + 0.5) * CELL_SIZE, (cy + 0.5) * CELL_SIZE)
        return None

    # -------------------- A* Pathfinding on Cell Grid --------------------
    def _wall_blocks_move(self, cx, cy, nx, ny):
        """Check if moving between adjacent cells crosses a wall.
        Walls are zero-thickness line segments; cells are always passable
        but movement BETWEEN cells may be blocked."""
        ax = (cx + 0.5) * CELL_SIZE
        ay = (cy + 0.5) * CELL_SIZE
        bx = (nx + 0.5) * CELL_SIZE
        by = (ny + 0.5) * CELL_SIZE
        return self._path_crosses_wall(ax, ay, bx, by)

    def _build_nav_grid(self):
        """Return grid dimensions.  All cells are passable (walls are
        zero-thickness and block movement between cells, not cells
        themselves).  Use _wall_blocks_move() for adjacency checks."""
        cols = self.arena_width // CELL_SIZE
        rows = self.arena_height // CELL_SIZE
        return cols, rows

    def _astar_cell(self, start_cell, goal_cell, cols, rows):
        """A* on the cell grid.  Returns list of (cx, cy) from start to goal
        (inclusive), or None if no path exists.  Uses _wall_blocks_move()
        for adjacency instead of a passability grid."""
        if start_cell == goal_cell:
            return [start_cell]
        sx, sy = start_cell
        gx, gy = goal_cell

        if not (0 <= sx < cols and 0 <= sy < rows):
            return None
        if not (0 <= gx < cols and 0 <= gy < rows):
            return None

        open_set = [(abs(gx - sx) + abs(gy - sy), 0, sx, sy)]
        came_from = {}
        g_score = {start_cell: 0}

        while open_set:
            _, g, cx, cy = heapq.heappop(open_set)
            if (cx, cy) == goal_cell:
                # Reconstruct
                path = []
                cur = goal_cell
                while cur is not None:
                    path.append(cur)
                    cur = came_from.get(cur)
                path.reverse()
                return path
            if g > g_score.get((cx, cy), float('inf')):
                continue
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if (0 <= nx < cols and 0 <= ny < rows
                        and not self._wall_blocks_move(cx, cy, nx, ny)):
                    ng = g + 1
                    if ng < g_score.get((nx, ny), float('inf')):
                        g_score[(nx, ny)] = ng
                        h = abs(gx - nx) + abs(gy - ny)
                        heapq.heappush(open_set, (ng + h, ng, nx, ny))
                        came_from[(nx, ny)] = (cx, cy)
        return None  # no path

    def _get_path_waypoints(self, x, y, target_x, target_y):
        """Return a list of world-coordinate waypoints from (x,y) to
        (target_x, target_y), routed around walls via A*."""
        cols, rows = self._build_nav_grid()
        sc = (max(0, min(cols - 1, int(x / CELL_SIZE))),
              max(0, min(rows - 1, int(y / CELL_SIZE))))
        gc = (max(0, min(cols - 1, int(target_x / CELL_SIZE))),
              max(0, min(rows - 1, int(target_y / CELL_SIZE))))

        path = self._astar_cell(sc, gc, cols, rows)
        if path is None or len(path) <= 1:
            # Fallback: drive direct (existing behaviour)
            return [(target_x, target_y)]

        # Convert cell path to world-coordinate waypoints (cell centres),
        # but skip the first cell (robot is already there) and use the
        # actual target position for the last waypoint.
        raw = []
        for i, (cx, cy) in enumerate(path[1:], 1):
            if i == len(path) - 1:
                raw.append((target_x, target_y))
            else:
                raw.append(((cx + 0.5) * CELL_SIZE,
                            (cy + 0.5) * CELL_SIZE))

        # ---- Line-of-sight smoothing: skip intermediate waypoints ----
        # If the robot can reach waypoint i+2 without crossing a wall,
        # drop waypoint i+1.  This makes paths much shorter.
        if len(raw) <= 2:
            return raw
        smoothed = [raw[0]]
        i = 0
        while i < len(raw) - 1:
            # Try to skip ahead as far as possible
            farthest = i + 1
            for j in range(i + 2, len(raw)):
                if not self._path_crosses_wall(smoothed[-1][0], smoothed[-1][1],
                                                raw[j][0], raw[j][1]):
                    farthest = j
            smoothed.append(raw[farthest])
            i = farthest
        return smoothed

    def _ray_segment_distance(self, ox, oy, dx, dy, x1, y1, x2, y2):
        rx = x2 - x1
        ry = y2 - y1
        denom = dx * ry - dy * rx
        if abs(denom) < 1e-9:
            return None
        qx = x1 - ox
        qy = y1 - oy
        t = (qx * ry - qy * rx) / denom
        u = (qx * dy - qy * dx) / denom
        if t >= 0.0 and 0.0 <= u <= 1.0:
            return t
        return None

    def _ray_circle_distance(self, ox, oy, dx, dy, cx, cy, radius):
        fx = ox - cx
        fy = oy - cy
        a = dx * dx + dy * dy
        b = 2.0 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - radius * radius
        disc = b * b - 4.0 * a * c
        if disc < 0.0:
            return None
        sqrt_disc = math.sqrt(disc)
        t1 = (-b - sqrt_disc) / (2.0 * a)
        t2 = (-b + sqrt_disc) / (2.0 * a)
        hits = [t for t in (t1, t2) if t >= 0.0]
        return min(hits) if hits else None

    def _compute_tof_for_robot(self, rid: int) -> Dict[str, int]:
        plant = self.robot_plants[rid]
        readings: Dict[str, int] = {}
        _, _, _, _, crowding = self._divider_queue_state(rid)
        peer_tof_radius = self.geometry.body_radius_mm
        for sensor_name, rel_deg in TOF_SENSOR_ANGLES_DEG.items():
            angle = plant.theta + math.radians(rel_deg)
            ox = plant.x + self.geometry.tof_sensor_offset_mm * math.cos(angle)
            oy = plant.y + self.geometry.tof_sensor_offset_mm * math.sin(angle)
            dx = math.cos(angle)
            dy = math.sin(angle)
            best = self.geometry.tof_range_mm + 1.0
            for seg in self.geometry.wall_segments():
                dist = self._ray_segment_distance(ox, oy, dx, dy, *seg)
                if dist is not None and dist < best:
                    best = dist
            for other_id, other in self.robot_plants.items():
                if other_id == rid:
                    continue
                dist = self._ray_circle_distance(ox, oy, dx, dy, other.x, other.y, peer_tof_radius)
                if dist is None:
                    continue
                if crowding > 0.0 and dist < self.impairment.crowd_range_mm:
                    if random.random() < self.impairment.peer_tof_mask_prob * crowding:
                        continue
                    dist += self.impairment.peer_tof_inflate_mm * crowding
                if dist < best:
                    best = dist
            readings[sensor_name] = int(round(best)) if best <= self.geometry.tof_range_mm else 9999
        return readings

    @staticmethod
    def _wrap_angle(theta: float) -> float:
        while theta > math.pi:
            theta -= 2 * math.pi
        while theta < -math.pi:
            theta += 2 * math.pi
        return theta

    def _plant_forward_speed(self, plant: SimRobotPlant) -> float:
        return 0.5 * (plant.left_mm_s + plant.right_mm_s)

    def _escape_blocked(self, plant: SimRobotPlant, nx: float, ny: float) -> bool:
        """Return True when the robot cannot back away from a contact cleanly.

        This approximates real jams at the divider/walls where body contact does
        not immediately resolve by sliding both robots apart.
        """
        probe = 18.0
        test_x = plant.x + nx * probe
        test_y = plant.y + ny * probe
        if self._collides_with_wall(test_x, test_y, radius=self.geometry.robot_radius_mm):
            return True
        if self.sim_mode == "hardware_clone" and self.layout == "multiroom":
            return self.geometry.distance_to_config_space_obstacle(plant.x, plant.y) < 30.0
        return False

    def _contact_turn_bias(self, plant: SimRobotPlant, nx: float, ny: float, magnitude: float) -> float:
        fx = math.cos(plant.theta)
        fy = math.sin(plant.theta)
        tangent_x = -ny
        tangent_y = nx
        handedness = fx * tangent_x + fy * tangent_y
        if abs(handedness) < 1e-3:
            handedness = 1.0 if math.sin(plant.theta) >= 0.0 else -1.0
        return math.copysign(magnitude, handedness)

    def _integrate_hardware_clone(self, rid: int, dt: float):
        plant = self.robot_plants[rid]
        left_pwm, right_pwm = self.motor_intents.get(rid, (0.0, 0.0))
        plant.left_pwm = left_pwm
        plant.right_pwm = right_pwm
        in_queue, queue_count, nearest_peer_id, nearest_peer_dist, crowding = self._divider_queue_state(rid)
        turn_metric = abs(left_pwm - right_pwm) / max(abs(left_pwm) + abs(right_pwm), 1.0)
        wall_clearance = self.geometry.distance_to_config_space_obstacle(plant.x, plant.y)
        wall_proximity = max(0.0, min(1.0, (220.0 - wall_clearance) / 220.0))
        plant.turn_metric = turn_metric
        plant.wall_proximity = wall_proximity

        wheel_base_mm = self.geometry.body_half_width_mm * 2.0
        # Calibrated from clean hardware logs: straight segments typically land
        # around 0.8-0.9 mm/s per commanded PWM, with noticeably slower ramp-up
        # than the earlier idealized plant. Keeping this conservative makes the
        # divider crossing and mission-complete timing much closer to hardware.
        pwm_to_mm_s = 0.9
        max_accel_mm_s2 = 450.0
        queue_drag = 0.0
        queue_turn_bias = 0.0
        if in_queue and queue_count > 1:
            queue_drag = self.impairment.divider_queue_drag * crowding
            queue_turn_bias = (8.0 + 10.0 * crowding) * (1.0 if rid % 2 else -1.0)
            if nearest_peer_dist is not None and nearest_peer_dist < 180.0:
                queue_drag += 0.10
                queue_turn_bias *= 1.35
        drive_scale = self.impairment.base_drive_scale
        drive_scale *= (1.0 - self.impairment.turn_drive_drag * turn_metric)
        drive_scale *= (1.0 - self.impairment.wall_drive_drag * wall_proximity)
        drive_scale = max(0.32, drive_scale)
        drag = max(0.0, min(0.92, max(plant.contact_drag, queue_drag)))
        turn_bias = max(-55.0, min(55.0, plant.contact_turn_bias_mm_s + queue_turn_bias))
        target_left = left_pwm * pwm_to_mm_s * drive_scale * (1.0 - drag) - turn_bias
        target_right = right_pwm * pwm_to_mm_s * drive_scale * (1.0 - drag) + turn_bias
        if drag > 0.0 and left_pwm * right_pwm >= 0.0:
            straight_drag = max(0.15, 1.0 - 0.75 * drag)
            target_left *= straight_drag
            target_right *= straight_drag

        def step_velocity(current: float, target: float) -> float:
            delta = target - current
            limit = max_accel_mm_s2 * dt
            if delta > limit:
                delta = limit
            elif delta < -limit:
                delta = -limit
            return current + delta

        plant.left_mm_s = step_velocity(plant.left_mm_s, target_left)
        plant.right_mm_s = step_velocity(plant.right_mm_s, target_right)

        substeps = max(1, int(max(dt, 0.0) / 0.01))
        sub_dt = dt / substeps if substeps > 0 else 0.0
        for _ in range(substeps):
            linear_mm_s = 0.5 * (plant.left_mm_s + plant.right_mm_s)
            angular_rad_s = (plant.right_mm_s - plant.left_mm_s) / max(wheel_base_mm, 1.0)
            new_theta = plant.theta + angular_rad_s * sub_dt
            mid_theta = plant.theta + 0.5 * angular_rad_s * sub_dt
            cand_x = plant.x + linear_mm_s * math.cos(mid_theta) * sub_dt
            cand_y = plant.y + linear_mm_s * math.sin(mid_theta) * sub_dt

            moved = False
            if not self._collides_with_wall(cand_x, cand_y, radius=self.geometry.robot_radius_mm):
                plant.x, plant.y = cand_x, cand_y
                moved = True
            else:
                if not self._collides_with_wall(cand_x, plant.y, radius=self.geometry.robot_radius_mm):
                    plant.x = cand_x
                    moved = True
                if not self._collides_with_wall(plant.x, cand_y, radius=self.geometry.robot_radius_mm):
                    plant.y = cand_y
                    moved = True
            if moved:
                plant.theta = new_theta
            else:
                plant.left_mm_s *= 0.5
                plant.right_mm_s *= 0.5
                plant.theta = new_theta

            plant.x, plant.y = self.geometry.clamp_to_bounds(plant.x, plant.y)
            plant.theta = self._wrap_angle(plant.theta)

            plant.encoder_left += int(round(plant.left_mm_s * sub_dt))
            plant.encoder_right += int(round(plant.right_mm_s * sub_dt))

        decay = math.exp(-max(dt, 0.0) / 0.35) if dt > 0.0 else 0.0
        plant.contact_drag *= decay
        plant.contact_turn_bias_mm_s *= decay
        plant.contact_memory_s = max(0.0, plant.contact_memory_s - dt)
        self.sim_positions[rid] = (plant.x, plant.y, plant.theta)

    def _resolve_peer_contacts(self):
        ids = sorted(self.robot_plants)
        min_sep = max(
            2.0 * self.geometry.body_radius_mm - 8.0,
            2.0 * self.geometry.body_half_width_mm + 10.0,
        )
        for idx, rid in enumerate(ids):
            a = self.robot_plants[rid]
            for other_id in ids[idx + 1:]:
                b = self.robot_plants[other_id]
                dx = b.x - a.x
                dy = b.y - a.y
                dist = math.hypot(dx, dy)
                if dist >= min_sep:
                    continue
                if dist < 1e-6:
                    nx, ny = 1.0, 0.0
                else:
                    nx = dx / dist
                    ny = dy / dist
                overlap = min_sep - max(dist, 1e-6)
                a_into = max(0.0, self._plant_forward_speed(a)) * max(0.0, math.cos(a.theta) * nx + math.sin(a.theta) * ny)
                b_into = max(0.0, self._plant_forward_speed(b)) * max(0.0, -(math.cos(b.theta) * nx + math.sin(b.theta) * ny))
                closing_speed = a_into + b_into
                a_pinned = self._escape_blocked(a, -nx, -ny)
                b_pinned = self._escape_blocked(b, nx, ny)

                # Real robots do not instantly "pop apart" on contact. Keep some
                # overlap for a few ticks, especially when one body is pinned by
                # the divider or an outer wall.
                separation_fraction = 0.28 + 0.10 * min(1.0, overlap / 20.0)
                if a_pinned or b_pinned:
                    separation_fraction *= 0.45
                correction = overlap * separation_fraction

                if a_pinned and not b_pinned:
                    a_share, b_share = 0.05, 0.95
                elif b_pinned and not a_pinned:
                    a_share, b_share = 0.95, 0.05
                else:
                    total_push = a_into + b_into + 1.0
                    a_share = max(0.2, min(0.8, b_into / total_push))
                    b_share = 1.0 - a_share

                ax = a.x - nx * correction * a_share
                ay = a.y - ny * correction * a_share
                bx = b.x + nx * correction * b_share
                by = b.y + ny * correction * b_share
                if not self._collides_with_wall(ax, ay, radius=self.geometry.robot_radius_mm):
                    a.x, a.y = self.geometry.clamp_to_bounds(ax, ay)
                if not self._collides_with_wall(bx, by, radius=self.geometry.robot_radius_mm):
                    b.x, b.y = self.geometry.clamp_to_bounds(bx, by)

                drag_base = min(0.9, 0.18 + overlap / 55.0 + closing_speed / 240.0)
                if a_pinned or b_pinned:
                    drag_base = min(0.95, drag_base + 0.18)
                a.contact_drag = max(a.contact_drag, drag_base + (0.10 if a_pinned else 0.0))
                b.contact_drag = max(b.contact_drag, drag_base + (0.10 if b_pinned else 0.0))
                a.contact_memory_s = max(a.contact_memory_s, 0.35 if (a_pinned or b_pinned) else 0.20)
                b.contact_memory_s = max(b.contact_memory_s, 0.35 if (a_pinned or b_pinned) else 0.20)

                bias_mag = min(26.0, 5.0 + overlap * 0.45 + closing_speed * 0.06)
                if a_into > 5.0:
                    a.contact_turn_bias_mm_s += self._contact_turn_bias(a, nx, ny, bias_mag)
                if b_into > 5.0:
                    b.contact_turn_bias_mm_s += self._contact_turn_bias(b, -nx, -ny, bias_mag)
                if a_pinned or b_pinned:
                    a.theta = self._wrap_angle(a.theta + self._contact_turn_bias(a, nx, ny, 0.03))
                    b.theta = self._wrap_angle(b.theta + self._contact_turn_bias(b, -nx, -ny, 0.03))

                if overlap > 10.0:
                    stall_scale = 0.55 if (a_pinned or b_pinned) else 0.72
                    a.left_mm_s *= stall_scale
                    a.right_mm_s *= stall_scale
                    b.left_mm_s *= stall_scale
                    b.right_mm_s *= stall_scale
                self.sim_positions[rid] = (a.x, a.y, a.theta)
                self.sim_positions[other_id] = (b.x, b.y, b.theta)

    def _update_positions_hardware_clone(self, dt: float):
        for rid in range(1, NUM_ROBOTS + 1):
            self._integrate_hardware_clone(rid, dt)
        self._resolve_peer_contacts()

    # -------------------- Movement Simulation --------------------
    def update_positions(self, states: Dict[int, dict], dt: float):
        if self.sim_mode == "hardware_clone":
            self._update_positions_hardware_clone(dt)
            return
        """Update simulated positions based on each robot's decided target.

        Uses navigation_target from each robot's broadcast - this is the
        position the robot actually chose to go to (leader assignment if
        trusting, local fallback if not).  Includes wall collision,
        wall-following, and stuck detection with locked override.

        Movement is TIME-BASED: robots move at SPEED_PER_SEC * dt pixels per
        call, ensuring consistent behavior regardless of tick rate / CPU load.
        """
        SPEED_PER_SEC = 500       # pixels/sec (~robot body length per second)
        STUCK_TIME = 1.5          # seconds stuck before override fires
        STUCK_MOVE_EPS = 15       # pixels - less than this = stuck
        OVERRIDE_ARRIVE_DIST = 60 # close enough to release override lock
        WALL_RADIUS = 75          # collision radius used in _collides_with_wall
        MAX_STEP = WALL_RADIUS * 0.8  # Max step per sub-tick to prevent tunneling

        total_move = SPEED_PER_SEC * dt  # total pixels to move this tick

        for rid in range(1, NUM_ROBOTS + 1):
            x, y, theta = self.sim_positions[rid]

            # ---- Motor fault simulation (matches visual_sim.py) ----
            motor_fault = self.motor_faults.get(rid)
            if motor_fault == 'stop':
                # Robot is frozen - don't move, don't count coverage
                continue
            elif motor_fault == 'spin':
                # Robot spins in place - only theta changes, no position change
                theta += 0.2 * (dt / 0.05)  # scale by dt so rotation is time-consistent
                self.sim_positions[rid] = (x, y, theta)
                # Still count the cell it's sitting on
                cell = (int(x / CELL_SIZE), int(y / CELL_SIZE))
                self.visited_cells.add(cell)
                continue
            elif motor_fault == 'erratic':
                # Random jitter
                x += random.uniform(-20, 20) * (dt / 0.05)
                y += random.uniform(-20, 20) * (dt / 0.05)
                theta += random.uniform(-0.3, 0.3) * (dt / 0.05)
                x = max(75, min(self.arena_width - 75, x))
                y = max(75, min(self.arena_height - 75, y))
                self.sim_positions[rid] = (x, y, theta)
                cell = (int(x / CELL_SIZE), int(y / CELL_SIZE))
                self.visited_cells.add(cell)
                continue

            # ---- Stuck detection (time-based) ----
            px, py = self.prev_positions.get(rid, (x, y))
            move_since = math.sqrt((x - px)**2 + (y - py)**2)
            if move_since < STUCK_MOVE_EPS:
                self.stuck_counters[rid] = self.stuck_counters.get(rid, 0) + dt
            else:
                self.stuck_counters[rid] = 0
                self.prev_positions[rid] = (x, y)

            # ---- If there's an active stuck override, check if robot arrived ----
            override = self.stuck_overrides.get(rid)
            if override:
                od = math.sqrt((x - override[0])**2 + (y - override[1])**2)
                if od < OVERRIDE_ARRIVE_DIST:
                    self.stuck_overrides[rid] = None
                    self.stuck_counters[rid] = 0

            # ---- Accept robot's navigation target (unless override is active) ----
            if not self.stuck_overrides.get(rid):
                current_state = states.get(rid) or self.last_states_by_robot.get(rid)
                if current_state:
                    nav = current_state.get('navigation_target')
                    if nav:
                        self.last_targets[rid] = nav

                # Fallback: use leader assignments if robot hasn't reported a target yet
                if rid not in self.last_targets or self.last_targets[rid] is None:
                    state_pool = list(states.values()) or list(self.last_states_by_robot.values())
                    for state in state_pool:
                        if state.get('state') == 'leader':
                            assignments = state.get('assignments', {})
                            if str(rid) in assignments:
                                self.last_targets[rid] = assignments[str(rid)]
                                break

            target = self.last_targets.get(rid)

            # ---- Override target if stuck for too long ----
            # IMPORTANT: Only fire when robot is FAR from its target but can't
            # move (wall wedge).  If the robot arrived at its target and is
            # sitting there, that's not "stuck" - it successfully followed its
            # command.  On real hardware a robot told to go to an explored cell
            # would just stay there; only REIP's trust model can recover.
            # Without this guard, stuck detection acts as an oracle that rescues
            # Raft followers from bad leader assignments.
            target_dist = (math.sqrt((target[0] - x)**2 + (target[1] - y)**2)
                           if target else 0)
            if (self.stuck_counters.get(rid, 0) >= STUCK_TIME
                    and target_dist > OVERRIDE_ARRIVE_DIST):
                alt = self._find_reachable_unexplored(x, y)
                if alt:
                    target = alt
                    self.last_targets[rid] = alt
                    self.stuck_overrides[rid] = alt
                self.stuck_counters[rid] = 0
                self.prev_positions[rid] = (x, y)

            if target:
                # ---- A* waypoint navigation ----
                # If the final target changed, recompute the path
                tkey = (round(target[0], 1), round(target[1], 1))
                if tkey != self.path_target_key.get(rid):
                    wps = self._get_path_waypoints(x, y, target[0], target[1])
                    self.path_waypoints[rid] = wps
                    self.path_target_key[rid] = tkey

                # Current waypoint to chase (first in list)
                wps = self.path_waypoints.get(rid) or [(target[0], target[1])]

                # Sub-step to prevent wall tunneling: break large moves
                # into steps no bigger than MAX_STEP (< collision radius)
                remaining = total_move
                while remaining > 0.5:
                    step = min(remaining, MAX_STEP)
                    remaining -= step

                    # Advance through waypoints as we reach them
                    while len(wps) > 1:
                        wd = math.sqrt((wps[0][0] - x)**2 + (wps[0][1] - y)**2)
                        if wd < CELL_SIZE * 0.6:  # close enough to waypoint
                            wps.pop(0)
                        else:
                            break

                    wp = wps[0]
                    dx = wp[0] - x
                    dy = wp[1] - y
                    dist = math.sqrt(dx*dx + dy*dy)

                    if dist <= 20 and len(wps) <= 1:
                        # Arrived - clear stale target so we pick up the
                        # robot's next broadcast target instead of looping
                        # on the same arrived position forever.
                        self.last_targets[rid] = None
                        self.path_target_key[rid] = None
                        break  # arrived at final target

                    if dist <= 1:
                        # On top of waypoint but more remain
                        if len(wps) > 1:
                            wps.pop(0)
                        continue

                    theta = math.atan2(dy, dx)
                    new_x = x + step * math.cos(theta)
                    new_y = y + step * math.sin(theta)

                    if self._collides_with_wall(new_x, new_y):
                        moved = False
                        if not self._collides_with_wall(new_x, y):
                            x = new_x
                            moved = True
                        if not self._collides_with_wall(x, new_y) and abs(new_y - y) > 0.5:
                            y = new_y
                            moved = True
                        if not moved:
                            perp1 = theta + math.pi / 2
                            perp2 = theta - math.pi / 2
                            options = []
                            for perp in [perp1, perp2]:
                                tx = x + step * math.cos(perp)
                                ty = y + step * math.sin(perp)
                                if (not self._collides_with_wall(tx, ty) and
                                        75 <= tx <= self.arena_width - 75 and
                                        75 <= ty <= self.arena_height - 75):
                                    d = math.sqrt((tx - wp[0])**2 +
                                                  (ty - wp[1])**2)
                                    options.append((d, tx, ty))
                            if options:
                                options.sort()
                                _, x, y = options[0]
                            else:
                                break  # completely blocked, stop sub-stepping
                    else:
                        x = new_x
                        y = new_y

                # Update stored waypoints
                self.path_waypoints[rid] = wps

            # Clamp
            x = max(75, min(self.arena_width - 75, x))
            y = max(75, min(self.arena_height - 75, y))

            self.sim_positions[rid] = (x, y, theta)

            # Track coverage using the same center-cell semantics as the
            # robot nodes. The node broadcasts its own visited_cells too, so
            # this is just a local mirror to keep milestone timing aligned.
            # Apply the same wall mask as the hardware_clone path above, so
            # both branches count against one reachable set.
            cell = self.geometry.pos_to_cell(x, y)
            if cell is not None and not self.geometry.is_wall_cell(*cell):
                self._record_visited_cell(cell)

    def get_coverage(self) -> float:
        """Calculate current coverage percentage"""
        if self.sim_mode == "hardware_clone" and self.layout == "multiroom":
            total_cells = self.geometry.explorable_cell_count()
        else:
            total_cells = (self.arena_width // CELL_SIZE) * (self.arena_height // CELL_SIZE)
        return len(self.visited_cells) / total_cells * 100

    # -------------------- Current Leader Detection --------------------
    def _get_current_leader(self) -> int:
        """Find the most commonly reported leader from tracked states."""
        from collections import Counter
        if not self.tracked_leaders:
            return 1
        counts = Counter(v for v in self.tracked_leaders.values() if v is not None)
        return counts.most_common(1)[0][0] if counts else 1

    # -------------------- Run Single Experiment --------------------
    def run_experiment(
        self,
        name: str,
        controller: str,
        fault_type: Optional[str] = None,
        fault_robot: int = 1,
        fault_schedule_name: Optional[str] = None,
        trial_num: int = 1,
        seed: int = 42,
        save_snapshots: bool = False,
        ablation: Optional[str] = None,
    ) -> ExperimentResult:
        """Run a single experiment with real-time leader-change detection."""
        random.seed(seed)
        print(f"\n{'='*60}")
        print(f"Experiment: {name}  (trial {trial_num}, seed {seed})")
        print(f"  Controller: {controller}, Layout: {self.layout}")
        print(f"  Fault: {fault_type or 'none'} on Robot {fault_robot}")
        fault_schedule = build_fault_schedule(fault_type, fault_robot, fault_schedule_name)
        if fault_schedule:
            print(f"  Fault schedule: {fault_schedule_name or 'default'} "
                  f"({_schedule_summary(fault_schedule)})")
        if ablation:
            print(f"  Ablation: {ablation}")
        print(f"{'='*60}")

        # ---- Reset per-trial state ----
        self.visited_cells = set()
        self.coverage_order = []
        self.coverage_history = []
        self.tracked_leaders = {}
        self.stuck_counters = {}
        self.prev_positions = {}
        self.last_targets = {}
        self.last_states_by_robot = {}
        self.stuck_overrides = {}
        self.motor_faults = {}  # Reset motor faults each trial
        self.motor_intents = {}
        self._scheduled_messages = []
        self._message_seq = 0
        self._position_burst_remaining = 0
        self._peer_burst_remaining = 0
        snapshot_frames = []  # Position snapshots for gridworld viz

        # Flush buffered socket data
        try:
            self.state_socket.setblocking(False)
            while True:
                self.state_socket.recvfrom(8192)
        except (BlockingIOError, ConnectionResetError):
            pass
        self.state_socket.setblocking(False)
        try:
            self.motor_socket.setblocking(False)
            while True:
                self.motor_socket.recvfrom(8192)
        except (BlockingIOError, ConnectionResetError):
            pass
        self.motor_socket.setblocking(False)

        # ---- Choose script ----
        extra_args = []
        if controller == 'decentralized':
            script = REIP_SCRIPT
            extra_args = ["--decentralized"]
        elif controller == 'raft':
            script = RAFT_SCRIPT
        else:
            script = REIP_SCRIPT

        # Ablation flags (only applies to REIP controller)
        if ablation and controller == 'reip':
            extra_args.extend(["--ablation", ablation])

        # Pass arena dimensions for scaled layouts
        if self.layout in ARENA_DIMENSIONS:
            extra_args.extend(["--arena-width", str(self.arena_width),
                               "--arena-height", str(self.arena_height)])

        # Use fixed positions if provided (for hardware validation)
        fixed_pos = getattr(self, '_fixed_positions_for_next_trial', None)
        self.start_robots(script, extra_args, seed=seed, experiment_name=name, fixed_positions=fixed_pos)
        self._fixed_positions_for_next_trial = None  # Clear after use

        ready_ids = self._wait_for_robot_ready_states()
        if len(ready_ids) < NUM_ROBOTS:
            print(f"  Warning: only {len(ready_ids)}/{NUM_ROBOTS} robots reported ready before start")
        # Let simulated localization run during the same pre-start election window
        # the hardware controller uses before the trial clock begins.
        self._trial_start_wall_time = time.time()
        self._next_position_publish = self._trial_start_wall_time
        settled_leader = self._wait_for_initial_leader_consensus()
        if settled_leader is not None:
            print(f"  Pre-start leader consensus: Robot {settled_leader}")
        else:
            print("  Warning: leader consensus not reached before start")

        # Send "start" only after the node network threads are broadcasting.
        # Without this readiness gate, the one-shot start packet can be emitted
        # before the localhost sockets are actually draining, leaving the trial
        # stuck at the startup gate with no localization updates applied.
        for rid in range(1, NUM_ROBOTS + 1):
            start_msg = {
                'type': 'fault_inject', 'robot_id': 0,
                'fault': 'start', 'timestamp': time.time()
            }
            self._schedule_message(
                'fault', start_msg, ('127.0.0.1', self._fault_port + rid),
                self.impairment.fault_delay_s, self.impairment.fault_jitter_s,
            )
        self._flush_scheduled_messages()
        time.sleep(0.2)
        # Send twice to compensate for any startup packet loss
        for rid in range(1, NUM_ROBOTS + 1):
            start_msg = {
                'type': 'fault_inject', 'robot_id': 0,
                'fault': 'start', 'timestamp': time.time()
            }
            self._schedule_message(
                'fault', start_msg, ('127.0.0.1', self._fault_port + rid),
                self.impairment.fault_delay_s, self.impairment.fault_jitter_s,
            )
        self._flush_scheduled_messages()
        self._send_bootstrap_packets()

        # Bootstrap localization before the full peer relay starts. Without
        # this warm-up window, five nodes can spend their first few ticks
        # self-electing and flooding peer traffic before they ever receive the
        # initial pose packets that anchor the hardware-faithful control path.
        bootstrap_until = time.time() + max(1.2, self.impairment.startup_blind_s + 0.4)
        while time.time() < bootstrap_until:
            self.receive_motor_intents()
            self.receive_states()
            self._flush_scheduled_messages()
            self.send_sensor_feedback()
            self._emit_position_packets()
            self._flush_scheduled_messages()
            time.sleep(0.05)

        # ---- Tracking variables ----
        start_time = time.time()
        fault_injected = False
        fault_inject_actual_time = None
        fault2_injected = False
        fault2_inject_time = None
        fault2_robot = None
        first_fault_robot = None
        fault_plan_idx = 0
        injected_fault_events: List[Dict[str, Any]] = []
        time_to_detection = None          # Fault -> leader change (impeachment)
        time_to_first_suspicion = None    # Fault -> first trust decay
        time_to_detection_2 = None        # Second fault -> second impeachment
        time_to_50 = None
        time_to_60 = None
        time_to_80 = None
        coverage_at_90s = None
        leader_settle_time = 0.0 if settled_leader is not None else None
        divider_crossing_time = None
        mission_complete_time = None
        min_peer_separation_mm = None
        min_wall_clearance_mm = None
        num_leader_changes = 0
        false_positives = 0
        last_print_sec = -1
        # Track pre-fault trust levels to detect first decay
        pre_fault_trust: Dict[int, float] = {}  # rid -> trust at fault time
        last_states: Dict[int, dict] = {}  # most recent state per robot

                # ---- Set up adversary agent (if using adaptive schedule) ----
        adversary_agent = None
        _adversary_prev_leader = None
        if fault_schedule_name == "adaptive" and fault_type:
            adversary_agent = AdaptiveAdversary(fault_type=fault_type)
            fault_schedule = []  # agent handles injections, not static schedule
            print(f"  Adversary agent: AdaptiveAdversary (budget=4, cooldown=12s)")
 
        self._last_tick_time = time.time()
        try:
            while time.time() - start_time < EXPERIMENT_DURATION:
                elapsed = time.time() - start_time

                # ---- Inject any due scheduled fault events ----
                while fault_plan_idx < len(fault_schedule) and elapsed >= fault_schedule[fault_plan_idx].time_s:
                    event = fault_schedule[fault_plan_idx]
                    target_robot = event.robot_id if event.target_mode == "fixed_robot" else self._get_current_leader()
                    print(f"  [t={elapsed:.1f}s] Injecting {event.fault_type} on Robot "
                          f"{target_robot} ({event.label or f'event {fault_plan_idx + 1}'})")
                    self.inject_fault(target_robot, event.fault_type)
                    inject_wall_time = time.time()
                    injected_fault_events.append({
                        "index": fault_plan_idx + 1,
                        "scheduled_time_s": event.time_s,
                        "actual_elapsed_s": elapsed,
                        "fault_type": event.fault_type,
                        "target_mode": event.target_mode,
                        "target_robot": target_robot,
                        "label": event.label,
                    })
                    if not fault_injected:
                        fault_injected = True
                        first_fault_robot = target_robot
                        fault_inject_actual_time = inject_wall_time
                        for rid, st in last_states.items():
                            pre_fault_trust[rid] = st.get('trust_in_leader', 1.0)
                    elif not fault2_injected:
                        fault2_injected = True
                        fault2_robot = target_robot
                        fault2_inject_time = inject_wall_time
                    fault_plan_idx += 1

                 
                # ---- Adversary agent decision ----
                if adversary_agent is not None:
                    cur_leader = self._get_current_leader()
                    leader_changed = (_adversary_prev_leader is not None
                                      and cur_leader != _adversary_prev_leader)
                    _adversary_prev_leader = cur_leader
                    action = adversary_agent.decide(elapsed, cur_leader, leader_changed)
                    if action is not None:
                        print(f"  [t={elapsed:.1f}s] ADVERSARY: {action.label} "
                              f"-> Robot {action.target_robot}")
                        self.inject_fault(action.target_robot, action.fault_type)
                        injected_fault_events.append({
                            "index": adversary_agent.compromises_used,
                            "scheduled_time_s": elapsed,
                            "actual_elapsed_s": elapsed,
                            "fault_type": action.fault_type,
                            "target_mode": "adversary_agent",
                            "target_robot": action.target_robot,
                            "label": action.label,
                        })
                        if not fault_injected:
                            fault_injected = True
                            first_fault_robot = action.target_robot
                            fault_inject_actual_time = time.time()
                            for rid, st in last_states.items():
                                pre_fault_trust[rid] = st.get('trust_in_leader', 1.0)
 
                # ---- Simulation step (time-based movement) ----
 
                now = time.time()
                dt = now - self._last_tick_time if hasattr(self, '_last_tick_time') else 0.05
                dt = min(dt, 0.15)  # Cap dt to prevent huge jumps after stalls
                self._last_tick_time = now

                self.receive_motor_intents()
                states = self.receive_states()
                self._flush_scheduled_messages()
                self.update_positions(states, dt)
                self.send_sensor_feedback()
                self._emit_position_packets()
                self._flush_scheduled_messages()

                # ---- Track leader changes in real time ----
                for rid, st in states.items():
                    reported_leader = st.get('leader_id')
                    prev_leader = self.tracked_leaders.get(rid)

                    if prev_leader is not None and reported_leader != prev_leader:
                        num_leader_changes += 1

                        # Was the first fault robot deposed?
                        if fault_injected and first_fault_robot is not None and prev_leader == first_fault_robot and \
                                time_to_detection is None:
                            time_to_detection = time.time() - fault_inject_actual_time
                            print(f"  [t={elapsed:.1f}s] IMPEACHED: Robot {rid} saw "
                                  f"leader change {prev_leader}->{reported_leader} "
                                  f"({time_to_detection:.2f}s after fault 1)")

                        # Was the second fault robot deposed?
                        if (fault2_injected and fault2_robot is not None
                                and prev_leader == fault2_robot
                                and time_to_detection_2 is None
                                and fault2_inject_time is not None):
                            time_to_detection_2 = time.time() - fault2_inject_time
                            print(f"  [t={elapsed:.1f}s] IMPEACHED (2nd): Robot {rid} saw "
                                  f"leader change {prev_leader}->{reported_leader} "
                                  f"({time_to_detection_2:.2f}s after fault 2)")

                        # Leader change before fault = potential false positive
                        # (only count after initial election settles)
                        if not fault_injected and elapsed > SETTLE_TIME:
                            false_positives += 1

                    self.tracked_leaders[rid] = reported_leader
                    last_states[rid] = st  # Keep latest state for trust tracking

                    # ---- Detect first trust decay (suspicion) ----
                    if fault_injected and time_to_first_suspicion is None:
                        trust = st.get('trust_in_leader', 1.0)
                        suspicion = st.get('suspicion', 0.0)
                        baseline = pre_fault_trust.get(rid, 1.0)
                        # Trust has dropped noticeably, or suspicion is accumulating
                        if trust < baseline - 0.05 or suspicion > 0.5:
                            time_to_first_suspicion = time.time() - fault_inject_actual_time
                            print(f"  [t={elapsed:.1f}s] FIRST SUSPICION: Robot {rid} "
                                  f"trust={trust:.2f} sus={suspicion:.2f} "
                                  f"({time_to_first_suspicion:.2f}s after fault)")

                leader_reports = [st.get('leader_id') for st in states.values() if st.get('leader_id') is not None]
                if leader_settle_time is None and leader_reports:
                    from collections import Counter
                    counts = Counter(leader_reports)
                    settled_leader, settled_count = counts.most_common(1)[0]
                    if settled_leader is not None and settled_count >= max(1, NUM_ROBOTS // 2 + 1):
                        leader_settle_time = elapsed

                if mission_complete_time is None:
                    for st in states.values():
                        if st.get('mission_complete'):
                            mission_complete_time = elapsed
                            break

                if self.sim_mode == "hardware_clone" and self.layout == "multiroom":
                    if divider_crossing_time is None:
                        crossed = any(
                            plant.x > self.geometry.interior_wall_x_right_mm
                            for plant in self.robot_plants.values()
                        )
                        if crossed:
                            divider_crossing_time = elapsed
                    peer_sep = self._current_peer_separation_mm()
                    if peer_sep is not None:
                        min_peer_separation_mm = peer_sep if min_peer_separation_mm is None else min(min_peer_separation_mm, peer_sep)
                    wall_clearance = self._current_wall_clearance_mm()
                    if wall_clearance is not None:
                        min_wall_clearance_mm = wall_clearance if min_wall_clearance_mm is None else min(min_wall_clearance_mm, wall_clearance)

                # ---- Coverage milestones ----
                coverage = self.get_coverage()
                self.coverage_history.append((elapsed, coverage))

                if time_to_50 is None and coverage >= 50.0:
                    time_to_50 = elapsed
                if time_to_60 is None and coverage >= 60.0:
                    time_to_60 = elapsed
                if time_to_80 is None and coverage >= 80.0:
                    time_to_80 = elapsed

                if coverage_at_90s is None and elapsed >= 90.0:
                    coverage_at_90s = coverage

                # ---- Progress print (every 15s) ----
                sec = int(elapsed)
                if sec % 15 == 0 and sec != last_print_sec and sec > 0:
                    det_str = f", det={time_to_detection:.1f}s" if time_to_detection else ""
                    sus_str = f", sus={time_to_first_suspicion:.1f}s" \
                        if time_to_first_suspicion else ""
                    print(f"  [t={sec}s] Coverage: {coverage:.1f}%{sus_str}{det_str}")
                    last_print_sec = sec

                # ---- Snapshot logging (for gridworld viz) ----
                if save_snapshots and int(elapsed * 4) != getattr(self, '_last_snap', -1):
                    # 4 Hz snapshot rate (every 0.25s)
                    self._last_snap = int(elapsed * 4)
                    robots_snap = {}
                    for rid, (rx, ry, rtheta) in self.sim_positions.items():
                        st = last_states.get(rid, {})
                        # Next A* waypoint = actual direction (around walls)
                        wps = self.path_waypoints.get(rid)
                        next_wp = list(wps[0]) if wps else None
                        robots_snap[str(rid)] = {
                            'x': round(rx, 1), 'y': round(ry, 1),
                            'theta': round(rtheta, 3),
                            'state': st.get('state', 'unknown'),
                            'leader_id': st.get('leader_id'),
                            'target': st.get('navigation_target'),
                            'predicted_target': st.get('predicted_target'),
                            'commanded_target': st.get('commanded_target'),
                            'next_waypoint': next_wp,
                            'trust': round(st.get('trust_in_leader', 1.0), 3),
                            'suspicion': round(st.get('suspicion', 0.0), 3),
                            'omega': round(st.get('omega', 0.0), 4),
                            'classified_fault': st.get('classified_fault'),
                            'motor_fault': self.motor_faults.get(rid),
                        }
                    snapshot_frames.append({
                        't': round(elapsed, 2),
                        'coverage': round(coverage, 2),
                        'visited_cells': [list(c) for c in self.visited_cells],
                        'robots': robots_snap,
                    })

                time.sleep(0.05)  # 20 Hz

        finally:
            self.stop_robots()

        # ---- Also parse robot logs for precise first_decay_time ----
        # Robot logs contain [DETECT] messages with exact timestamps
        if fault_type and fault_inject_actual_time:
            log_suspicion, log_impeachment = self._parse_robot_logs(
                fault_inject_actual_time)
            # Prefer log-based timing if available (more precise)
            if log_suspicion is not None and (time_to_first_suspicion is None
                    or log_suspicion < time_to_first_suspicion):
                time_to_first_suspicion = log_suspicion
            if log_impeachment is not None and (time_to_detection is None
                    or log_impeachment < time_to_detection):
                time_to_detection = log_impeachment
        behavior_metrics = self._parse_robot_behavior_metrics()

        # ---- Final metrics ----
        final_coverage = self.get_coverage()
        if coverage_at_90s is None:
            coverage_at_90s = final_coverage  # experiment was shorter than 90s

        result = ExperimentResult(
            name=name,
            controller=controller,
            fault_type=fault_type,
            fault_robot=fault_robot,
            layout=self.layout,
            trial=trial_num,
            seed=seed,
            final_coverage=final_coverage,
            time_to_50=time_to_50,
            time_to_60=time_to_60,
            time_to_80=time_to_80,
            time_to_detection=time_to_detection,
            time_to_first_suspicion=time_to_first_suspicion,
            time_to_detection_2=time_to_detection_2,
            num_leader_changes=num_leader_changes,
            false_positives=false_positives,
            coverage_at_90s=coverage_at_90s,
            coverage_timeline=self.coverage_history,
            fault_schedule_name=fault_schedule_name or ("default" if fault_schedule else None),
            fault_events=injected_fault_events,
            ablation=ablation,
            leader_settle_time=leader_settle_time,
            divider_crossing_time=divider_crossing_time,
            mission_complete_time=mission_complete_time,
            peer_emergency_count=behavior_metrics['peer_emergency_count'],
            min_peer_separation_mm=min_peer_separation_mm,
            min_wall_clearance_mm=min_wall_clearance_mm,
            coverage_order=self.coverage_order,
        )

        # Save coverage timeline to per-experiment log dir for convergence plots
        if hasattr(self, '_current_log_dir') and self._current_log_dir:
            tl_path = os.path.join(self._current_log_dir, "coverage_timeline.json")
            try:
                with open(tl_path, 'w') as f:
                    json.dump({
                        'name': name, 'controller': controller,
                        'fault_type': fault_type, 'layout': self.layout,
                        'trial': trial_num, 'seed': seed,
                        'fault_schedule_name': fault_schedule_name or ("default" if fault_schedule else None),
                        'fault_schedule': _schedule_to_metadata(fault_schedule),
                        'fault_events': injected_fault_events,
                        'fault_time_1': fault_schedule[0].time_s if len(fault_schedule) >= 1 else None,
                        'fault_time_2': fault_schedule[1].time_s if len(fault_schedule) >= 2 else None,
                        'timeline': self.coverage_history
                    }, f)
            except Exception:
                pass

            # Save position snapshots for gridworld visualization
            if save_snapshots and snapshot_frames:
                snap_path = os.path.join(self._current_log_dir, "snapshots.json")
                try:
                    with open(snap_path, 'w') as f:
                        json.dump({
                            'name': name, 'controller': controller,
                            'fault_type': fault_type, 'layout': self.layout,
                            'trial': trial_num, 'seed': seed,
                            'arena_width': self.arena_width,
                            'arena_height': self.arena_height,
                            'cell_size': CELL_SIZE,
                            'walls': self.geometry.wall_segments() if self.sim_mode == "hardware_clone" and self.layout == "multiroom" else self.walls,
                            'sim_mode': self.sim_mode,
                            'impairment_preset': self.impairment.name,
                            'fault_schedule_name': fault_schedule_name or ("default" if fault_schedule else None),
                            'fault_schedule': _schedule_to_metadata(fault_schedule),
                            'fault_events': injected_fault_events,
                            'fault_time_1': fault_schedule[0].time_s if len(fault_schedule) >= 1 else None,
                            'fault_time_2': fault_schedule[1].time_s if len(fault_schedule) >= 2 else None,
                            'frames': snapshot_frames,
                        }, f)
                    print(f"  Saved {len(snapshot_frames)} snapshots to {snap_path}")
                except Exception as e:
                    print(f"  Warning: failed to save snapshots: {e}")

        t50_str = f"{time_to_50:.1f}s" if time_to_50 else "N/A"
        t60_str = f"{time_to_60:.1f}s" if time_to_60 else "N/A"
        t80_str = f"{time_to_80:.1f}s" if time_to_80 else "N/A"
        sus_str = f"{time_to_first_suspicion:.2f}s" if time_to_first_suspicion else "N/A"
        det_str = f"{time_to_detection:.2f}s" if time_to_detection else "N/A"
        det2_str = f"{time_to_detection_2:.2f}s" if time_to_detection_2 else "N/A"
        print(f"\n  Results: cov={final_coverage:.1f}%, t50={t50_str}, t60={t60_str}, "
              f"t80={t80_str}, suspicion={sus_str}, impeach1={det_str}, "
              f"impeach2={det2_str}, FP={false_positives}")

        return result

    def _parse_robot_logs(self, fault_time: float) -> Tuple[Optional[float], Optional[float]]:
        """Parse robot log files for precise detection timing.

        Looks for [DETECT] messages that contain first_decay_time and
        impeachment_time timestamps, and computes offsets from fault_time.

        Returns (first_suspicion_offset, impeachment_offset) in seconds.
        """
        log_dir = getattr(self, '_current_log_dir',
                          os.path.join(self.output_dir, "logs"))
        first_sus = None
        first_imp = None

        for i in range(1, NUM_ROBOTS + 1):
            log_path = os.path.join(log_dir, f"robot_{i}.log")
            if not os.path.exists(log_path):
                continue
            try:
                with open(log_path, 'r', errors='replace') as f:
                    for line in f:
                        # [DETECT] First trust decay after N bad commands (Xs)
                        if '[DETECT] First trust decay' in line:
                            # Extract the time in parentheses e.g. "(1.23s)"
                            import re
                            m = re.search(r'\((\d+\.?\d*)s\)', line)
                            if m:
                                dt = float(m.group(1))
                                if first_sus is None or dt < first_sus:
                                    first_sus = dt
                        # [DETECT] IMPEACHMENT after N bad commands, Xs from first
                        if '[DETECT] IMPEACHMENT' in line:
                            import re
                            m = re.search(r'(\d+\.?\d*)s from first', line)
                            if m:
                                dt = float(m.group(1))
                                if first_imp is None or dt < first_imp:
                                    first_imp = dt
            except Exception:
                continue

        return first_sus, first_imp

    # -------------------- Persistence --------------------
    def save_results(self, results: List[ExperimentResult], tag: str = ""):
        """Save results to JSON"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = f"results_{tag}_{timestamp}.json" if tag else f"results_{timestamp}.json"
        path = os.path.join(self.output_dir, fname)

        data = []
        for r in results:
            d = asdict(r)
            data.append(d)

        with open(path, 'w') as f:
            json.dump(data, f, indent=2)

        print(f"\nResults saved to: {path}")
        return path


# ==================== Statistical Analysis ====================
def compute_statistics(results: List[ExperimentResult]):
    """Group results by condition (controller x fault x layout x ablation) and compute stats."""
    groups: Dict[str, List[ExperimentResult]] = defaultdict(list)
    for r in results:
        abl = getattr(r, 'ablation', None) or 'none'
        key = f"{r.layout}|{r.controller}|{r.fault_type or 'none'}|{abl}"
        groups[key].append(r)

    stats = {}
    for key, trials in sorted(groups.items()):
        parts = key.split('|')
        layout, ctrl, fault = parts[0], parts[1], parts[2]
        abl = parts[3] if len(parts) > 3 else 'none'
        n = len(trials)

        coverages = [t.final_coverage for t in trials]
        cov_mean = sum(coverages) / n
        cov_std = math.sqrt(sum((c - cov_mean)**2 for c in coverages) / max(n - 1, 1))

        t50s = [t.time_to_50 for t in trials if t.time_to_50 is not None]
        t50_mean = sum(t50s) / len(t50s) if t50s else None
        t50_std = math.sqrt(sum((t - t50_mean)**2 for t in t50s) / max(len(t50s) - 1, 1)) \
            if t50s and len(t50s) > 1 else 0.0

        t60s = [t.time_to_60 for t in trials if t.time_to_60 is not None]
        t60_mean = sum(t60s) / len(t60s) if t60s else None
        t60_std = math.sqrt(sum((t - t60_mean)**2 for t in t60s) / max(len(t60s) - 1, 1)) \
            if t60s and len(t60s) > 1 else 0.0

        t80s = [t.time_to_80 for t in trials if t.time_to_80 is not None]
        t80_mean = sum(t80s) / len(t80s) if t80s else None
        t80_std = math.sqrt(sum((t - t80_mean)**2 for t in t80s) / max(len(t80s) - 1, 1)) \
            if t80s and len(t80s) > 1 else 0.0

        detections = [t.time_to_detection for t in trials if t.time_to_detection is not None]
        det_mean = sum(detections) / len(detections) if detections else None
        det_std = math.sqrt(sum((d - det_mean)**2 for d in detections) / max(len(detections) - 1, 1)) \
            if detections and len(detections) > 1 else 0.0
        det_rate = len(detections) / n  # fraction of trials where fault was detected

        # First suspicion timing (fault -> first trust decay)
        suspicions = [t.time_to_first_suspicion for t in trials
                      if t.time_to_first_suspicion is not None]
        sus_mean = sum(suspicions) / len(suspicions) if suspicions else None
        sus_std = math.sqrt(sum((s - sus_mean)**2 for s in suspicions) /
                            max(len(suspicions) - 1, 1)) \
            if suspicions and len(suspicions) > 1 else 0.0

        fps = [t.false_positives for t in trials]
        fp_mean = sum(fps) / n

        cov90s = [t.coverage_at_90s for t in trials if t.coverage_at_90s is not None]
        cov90_mean = sum(cov90s) / len(cov90s) if cov90s else None

        stats[key] = {
            'layout': layout,
            'controller': ctrl,
            'fault': fault,
            'ablation': abl,
            'n': n,
            'coverage_mean': cov_mean,
            'coverage_std': cov_std,
            'time_to_50_mean': t50_mean,
            'time_to_50_std': t50_std,
            'time_to_60_mean': t60_mean,
            'time_to_60_std': t60_std,
            'time_to_80_mean': t80_mean,
            'time_to_80_std': t80_std,
            'suspicion_mean': sus_mean,
            'suspicion_std': sus_std,
            'detection_mean': det_mean,
            'detection_std': det_std,
            'detection_rate': det_rate,
            'false_positive_mean': fp_mean,
            'coverage_at_90s_mean': cov90_mean,
        }

    return stats


def print_statistics(results: List[ExperimentResult]):
    """Print a publication-ready summary table."""
    stats = compute_statistics(results)

    print("\n" + "=" * 100)
    print("STATISTICAL SUMMARY  (mean +/- std)")
    print("=" * 100)

    header = (f"{'Layout':<10} {'Controller':<14} {'Fault':<12} {'Ablation':<15} {'N':>3}  "
              f"{'Coverage':>14}  {'T->50%':>12}  {'T->60%':>12}  {'T->80%':>12}  "
              f"{'1st Suspicion':>14}  {'Impeachment':>14}  {'Det%':>5}  {'FP':>4}  {'Cov@90s':>8}")
    print(header)
    print("-" * 170)

    for key in sorted(stats):
        s = stats[key]
        abl_label = s.get('ablation', 'none')
        cov = f"{s['coverage_mean']:.1f}+/-{s['coverage_std']:.1f}"
        t50 = f"{s['time_to_50_mean']:.1f}+/-{s['time_to_50_std']:.1f}" \
            if s['time_to_50_mean'] is not None else "N/A"
        t60 = f"{s['time_to_60_mean']:.1f}+/-{s['time_to_60_std']:.1f}" \
            if s['time_to_60_mean'] is not None else "N/A"
        t80 = f"{s['time_to_80_mean']:.1f}+/-{s['time_to_80_std']:.1f}" \
            if s['time_to_80_mean'] is not None else "N/A"
        sus = f"{s['suspicion_mean']:.2f}+/-{s['suspicion_std']:.2f}" \
            if s['suspicion_mean'] is not None else "N/A"
        det = f"{s['detection_mean']:.2f}+/-{s['detection_std']:.2f}" \
            if s['detection_mean'] is not None else "N/A"
        det_pct = f"{s['detection_rate']*100:.0f}%"
        fp = f"{s['false_positive_mean']:.1f}"
        c90 = f"{s['coverage_at_90s_mean']:.1f}" if s['coverage_at_90s_mean'] else "N/A"

        print(f"{s['layout']:<10} {s['controller']:<14} {s['fault']:<12} {abl_label:<15} "
              f"{s['n']:>3}  {cov:>14}  {t50:>12}  {t60:>12}  {t80:>12}  "
              f"{sus:>14}  {det:>14}  {det_pct:>5}  {fp:>4}  {c90:>8}")

    print("=" * 120)

    # ---- Key comparisons ----
    print("\nKEY COMPARISONS:")
    for layout in ["open", "multiroom"]:
        reip = stats.get(f"{layout}|reip|bad_leader|none")
        raft = stats.get(f"{layout}|raft|bad_leader|none")
        dec = stats.get(f"{layout}|decentralized|bad_leader|none")
        reip_clean = stats.get(f"{layout}|reip|none|none")

        if reip and raft:
            delta = reip['coverage_mean'] - raft['coverage_mean']
            print(f"\n  [{layout}] Bad Leader - REIP vs RAFT:")
            print(f"    Coverage: REIP {reip['coverage_mean']:.1f}% vs RAFT {raft['coverage_mean']:.1f}% "
                  f"(+{delta:.1f}pp advantage)")
            if reip['suspicion_mean'] is not None:
                print(f"    REIP: First suspicion in {reip['suspicion_mean']:.2f}s, "
                      f"full impeachment in {reip['detection_mean']:.2f}s"
                      if reip['detection_mean'] else
                      f"    REIP: First suspicion in {reip['suspicion_mean']:.2f}s, no impeachment")
            else:
                print(f"    REIP detection: {reip['detection_rate']*100:.0f}% rate "
                      f"at {reip['detection_mean']:.2f}s avg"
                      if reip['detection_mean'] else
                      f"    REIP detection: 0% rate")
            print(f"    RAFT detection: {raft['detection_rate']*100:.0f}% rate"
                  + (f" at {raft['detection_mean']:.2f}s avg"
                     if raft['detection_mean'] else " (never detects)"))

        if reip and dec:
            delta = reip['coverage_mean'] - dec['coverage_mean']
            direction = "+" if delta >= 0 else ""
            print(f"    REIP vs Decentralized: {direction}{delta:.1f}pp coverage "
                  f"(REIP {reip['coverage_mean']:.1f}% vs Dec {dec['coverage_mean']:.1f}%)")

        if reip and reip_clean:
            delta = reip_clean['coverage_mean'] - reip['coverage_mean']
            print(f"    REIP resilience: clean={reip_clean['coverage_mean']:.1f}%, "
                  f"fault={reip['coverage_mean']:.1f}% "
                  f"(only {delta:.1f}pp penalty from fault)")

        # ---- Speed comparison (time to milestones) ----
        all_conds = {}
        for tag in ['reip', 'raft', 'decentralized']:
            for fault_tag in ['none', 'bad_leader', 'freeze_leader']:
                k = f"{layout}|{tag}|{fault_tag}"
                if k in stats:
                    label = f"{tag}{'(fault)' if fault_tag != 'none' else ''}"
                    all_conds[label] = stats[k]

        if len(all_conds) >= 2:
            print(f"\n  [{layout}] Time-to-Coverage Milestones:")
            for label, s in sorted(all_conds.items()):
                t50 = f"{s['time_to_50_mean']:.1f}s" if s.get('time_to_50_mean') else "---"
                t60 = f"{s['time_to_60_mean']:.1f}s" if s.get('time_to_60_mean') else "---"
                t80 = f"{s['time_to_80_mean']:.1f}s" if s.get('time_to_80_mean') else "---"
                print(f"    {label:<22}  50%: {t50:>7}  60%: {t60:>7}  80%: {t80:>7}")


# ==================== Experiment Conditions ====================
# (condition_tag, controller, fault_type, fault_robot, ablation)
EXPERIMENT_CONDITIONS = [

    # --- Clean performance ---
    ("NoFault",    "reip",          None,          1, None),
    ("NoFault",    "raft",          None,          1, None),
    ("NoFault",    "decentralized", None,          1, None),
    # --- Bad leader (REIP's key differentiator) ---
    ("BadLeader",  "reip",          "bad_leader",  1, None),
    ("BadLeader",  "raft",          "bad_leader",  1, None),
    ("BadLeader",  "decentralized", "bad_leader",  1, None),
    # --- Freeze leader: leader stops updating assignments ---
    ("FreezeLeader", "reip",          "freeze_leader",  1, None),
    ("FreezeLeader", "raft",          "freeze_leader",  1, None),
    ("FreezeLeader", "decentralized", "freeze_leader",  1, None),
]

# Ablation conditions: each removes ONE REIP component under bad_leader
# to prove necessity. Also test no_causality under clean to show
# false positive prevention.
ABLATION_CONDITIONS = [
    # No trust assessment: REIP with elections but no detection/impeachment
    # Expected: plateaus like RAFT under bad_leader (proves trust IS the contribution)
    ("Ablation_NoTrust_BL",    "reip", "bad_leader", 1, "no_trust"),
    # Under clean: should reveal baseline coordination cost without trust logic
    ("Ablation_NoTrust_Clean", "reip", None,         1, "no_trust"),
    # No causality grace: CAUSALITY_GRACE_PERIOD=0
    # Under clean: expect false positives (unnecessary impeachments)
    ("Ablation_NoCausality_Clean", "reip", None,         1, "no_causality"),
    # No causality grace under bad_leader: faster detection but more false positives
    ("Ablation_NoCausality_BL",    "reip", "bad_leader", 1, "no_causality"),
    # No direction consistency check: only three-tier cell check
    # Expected: still detects bad_leader via Tier 1/3, but possibly slower
    ("Ablation_NoDirection_BL",    "reip", "bad_leader", 1, "no_direction"),
    # Under clean: should show baseline cost of removing direction-consistency checks
    ("Ablation_NoDirection_Clean", "reip", None,         1, "no_direction"),
    # No command instability check: disable omega (oscillation) detection
    # Expected: still detects bad_leader via three-tier + direction, but misses oscillation attacks
    ("Ablation_NoInstability_BL",  "reip", "bad_leader", 1, "no_instability"),
    # Reactive baseline: identical evidence/thresholds/impeachment, but
    # suspicion is applied only AFTER a command is fully executed (arrival).
    # Isolates proactive timing as the experimental variable.
    # Expected: detection still occurs but only after wasted motion;
    # slower impeachment, lower coverage under bad_leader.
    ("Reactive_BL",    "reip", "bad_leader",    1, "reactive"),
    ("Reactive_FL",    "reip", "freeze_leader", 1, "reactive"),
    ("Reactive_Clean", "reip", None,            1, "reactive"),
    # CUSUM decision-rule ablation: identical three-tier evidence, one-sided
    # CUSUM in place of REIP's accumulator, alarm routed to the same
    # leader-replacement path. k and h come from REIP_CUSUM_K / REIP_CUSUM_H.
    ("Cusum_BL",    "reip", "bad_leader",    1, "cusum"),
    ("Cusum_FL",    "reip", "freeze_leader", 1, "cusum"),
    ("Cusum_Clean", "reip", None,            1, "cusum"),
]

ADVERSARY_CONDITIONS = [
    ("Adversary_Adaptive_BL", "reip",          "bad_leader", 1, None),
    ("Adversary_Adaptive_BL", "raft",          "bad_leader", 1, None),
    ("Adversary_Adaptive_BL", "decentralized", "bad_leader", 1, None),
]
 

# ==================== Parallel Worker (module-level for pickling) ===========
def _run_worker(args_tuple):
    """Run a sequence of experiments on one worker with one port offset.

    args_tuple: (worker_id, job_list, output_dir, sim_mode, impairment_preset)
      job_list: [(name, layout, controller, fault_type, fault_robot, trial, seed, ablation, fault_schedule_name), ...]
    """
    worker_id, job_list, output_dir, sim_mode, impairment_preset = args_tuple
    port_offset = SIM_PORT_OFFSET_BASE * (worker_id + 1)
    results = []
    runner = ExperimentRunner(
        output_dir,
        port_offset=port_offset,
        sim_mode=sim_mode,
        impairment_preset=impairment_preset,
    )

    for job in job_list:
        if len(job) == 10:
            name, layout, controller, fault_type, fault_robot, trial, seed, ablation, fixed_positions, fault_schedule_name = job
        else:
            name, layout, controller, fault_type, fault_robot, trial, seed, ablation = job[:8]
            fixed_positions = None
            fault_schedule_name = None
        
        runner.set_layout(layout)
        try:
            # Pass fixed_positions to start_robots via run_experiment
            runner._fixed_positions_for_next_trial = fixed_positions
            r = runner.run_experiment(
                name=name, controller=controller,
                fault_type=fault_type, fault_robot=fault_robot,
                fault_schedule_name=fault_schedule_name,
                trial_num=trial, seed=seed, ablation=ablation)
            results.append(asdict(r))
        except Exception as e:
            print(f"  [W{worker_id}][ERROR] {name}: {e}")
            import traceback
            traceback.print_exc()
        time.sleep(1)

    try:
        runner.state_socket.close()
        runner.motor_socket.close()
    except Exception:
        pass
    return worker_id, results


# ==================== Validation: Sim vs Hardware ====================
def _parse_timestamp_key(name: str) -> Optional[int]:
    digits = "".join(ch for ch in name if ch.isdigit())
    return int(digits) if digits else None


def _find_hardware_trial_artifacts(hardware_path: str) -> Tuple[Optional[str], Optional[str], Optional[str]]:
    path = os.path.abspath(hardware_path)
    if os.path.isdir(path):
        trial_dir = path
        meta_path = os.path.join(trial_dir, "trial_meta.json")
    else:
        if os.path.basename(path) == "trial_meta.json":
            trial_dir = os.path.dirname(path)
            meta_path = path
        else:
            return None, None, None

    if not os.path.exists(meta_path):
        return None, None, None

    with open(meta_path) as f:
        meta = json.load(f)

    trials_root = os.path.dirname(trial_dir)
    start_ts = str(meta.get("start_ts", ""))
    target_key = _parse_timestamp_key(start_ts)
    robot_states_path = None
    best_delta = None
    if target_key is not None and os.path.isdir(trials_root):
        for entry in os.listdir(trials_root):
            candidate_dir = os.path.join(trials_root, entry)
            candidate_file = os.path.join(candidate_dir, "robot_states.jsonl")
            if not os.path.isfile(candidate_file):
                continue
            candidate_key = _parse_timestamp_key(entry)
            if candidate_key is None:
                continue
            delta = abs(candidate_key - target_key)
            if best_delta is None or delta < best_delta:
                best_delta = delta
                robot_states_path = candidate_file

    return meta_path, robot_states_path, trial_dir


def _parse_robot_log_metrics(log_dir: str, t0: float) -> Dict[str, object]:
    time_to_first_suspicion = None
    time_to_detection = None
    mission_complete_time = None
    peer_emergency_count = 0

    for i in range(1, NUM_ROBOTS + 1):
        candidates = [
            os.path.join(log_dir, f"r{i}_robot_{i}.jsonl"),
            os.path.join(log_dir, f"robot_{i}.jsonl"),
        ]
        log_path = None
        for candidate in candidates:
            if os.path.exists(candidate):
                log_path = candidate
                break
        if log_path is None:
            matches = [
                name for name in os.listdir(log_dir)
                if name.startswith(f"r{i}_robot_{i}_") and name.endswith(".jsonl")
            ]
            if matches:
                matches.sort()
                log_path = os.path.join(log_dir, matches[0])
        if log_path is None:
            continue

        try:
            records = []
            with open(log_path, "r", errors="replace") as f:
                for line in f:
                    line = line.strip()
                    if not line.startswith("{"):
                        if "[DETECT] First trust decay" in line:
                            match = re.search(r"\((\d+\.?\d*)s\)", line)
                            if match:
                                dt = float(match.group(1))
                                if time_to_first_suspicion is None or dt < time_to_first_suspicion:
                                    time_to_first_suspicion = dt
                        elif "[DETECT] IMPEACHMENT" in line:
                            match = re.search(r"(\d+\.?\d*)s from first", line)
                            if match:
                                dt = float(match.group(1))
                                if time_to_detection is None or dt < time_to_detection:
                                    time_to_detection = dt
                        continue
                    try:
                        rec = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    records.append(rec)

                    rec_t = rec.get("t")
                    if rec_t is None:
                        continue
                    elapsed = max(0.0, float(rec_t) - t0)
                    if mission_complete_time is None and rec.get("command_source") == "leader_complete":
                        mission_complete_time = elapsed
            peer_emergency_count += _count_peer_emergency_incidents(records)
        except OSError:
            continue

    return {
        "time_to_first_suspicion": time_to_first_suspicion,
        "time_to_detection": time_to_detection,
        "mission_complete_time": mission_complete_time,
        "peer_emergency_count": peer_emergency_count,
    }


def _load_hardware_trial_results(hardware_path: str) -> List[ExperimentResult]:
    meta_path, robot_states_path, log_dir = _find_hardware_trial_artifacts(hardware_path)
    if meta_path is None or robot_states_path is None or log_dir is None:
        return []

    with open(meta_path) as f:
        meta = json.load(f)

    geometry = DEFAULT_ARENA
    coverage_order: List[Tuple[int, int]] = []
    visited_cells = set()
    coverage_timeline: List[Tuple[float, float]] = []
    time_to_50 = None
    time_to_60 = None
    time_to_80 = None
    leader_settle_time = None
    divider_crossing_time = None
    min_peer_separation_mm = None
    min_wall_clearance_mm = None
    false_positives = 0
    num_leader_changes = 0
    tracked_leaders: Dict[int, Optional[int]] = {}
    t0 = None
    coverage_last = None

    with open(robot_states_path, "r", errors="replace") as f:
        for raw_line in f:
            raw_line = raw_line.strip()
            if not raw_line:
                continue
            try:
                rec = json.loads(raw_line)
            except json.JSONDecodeError:
                continue

            if rec.get("type") == "calibration":
                continue

            rec_t = rec.get("t")
            if rec_t is None:
                continue
            rec_t = float(rec_t)
            if t0 is None:
                t0 = rec_t
            elapsed = rec_t - t0

            poses = rec.get("poses", {})
            positions = []
            normalized_poses = []
            for rid_str, pose in poses.items():
                try:
                    rid = int(rid_str)
                except (TypeError, ValueError):
                    continue
                if not (1 <= rid <= NUM_ROBOTS):
                    continue
                normalized_poses.append((rid, pose))
            normalized_poses.sort(key=lambda item: item[0])
            for _, pose in normalized_poses:
                x = float(pose["x"])
                y = float(pose["y"])
                positions.append((x, y))
                cell = geometry.pos_to_cell(x, y)
                if cell is not None and not geometry.is_wall_cell(*cell) and cell not in visited_cells:
                    visited_cells.add(cell)
                    coverage_order.append(cell)

            coverage = len(visited_cells) / max(1, geometry.explorable_cell_count()) * 100.0
            if coverage_last is None or abs(coverage - coverage_last) > 1e-9:
                coverage_timeline.append((elapsed, coverage))
                coverage_last = coverage
            if time_to_50 is None and coverage >= 50.0:
                time_to_50 = elapsed
            if time_to_60 is None and coverage >= 60.0:
                time_to_60 = elapsed
            if time_to_80 is None and coverage >= 80.0:
                time_to_80 = elapsed

            if divider_crossing_time is None:
                if any(x > geometry.interior_wall_x_right_mm for x, _ in positions):
                    divider_crossing_time = elapsed

            if len(positions) >= 2:
                for idx, (ax, ay) in enumerate(positions):
                    for bx, by in positions[idx + 1:]:
                        dist = math.hypot(bx - ax, by - ay)
                        min_peer_separation_mm = dist if min_peer_separation_mm is None else min(min_peer_separation_mm, dist)

            for x, y in positions:
                clearance = geometry.distance_to_config_space_obstacle(x, y)
                min_wall_clearance_mm = clearance if min_wall_clearance_mm is None else min(min_wall_clearance_mm, clearance)

            robot_states = rec.get("robot_states", {})
            leader_reports = []
            for rid_str, state in robot_states.items():
                rid = int(rid_str)
                reported_leader = state.get("leader_id")
                prev_leader = tracked_leaders.get(rid)
                if prev_leader is not None and reported_leader != prev_leader:
                    num_leader_changes += 1
                    if elapsed > SETTLE_TIME:
                        false_positives += 1
                tracked_leaders[rid] = reported_leader
                if reported_leader is not None:
                    leader_reports.append(reported_leader)
            if leader_settle_time is None and leader_reports:
                from collections import Counter
                counts = Counter(leader_reports)
                settled_leader, settled_count = counts.most_common(1)[0]
                if settled_leader is not None and settled_count >= max(1, NUM_ROBOTS // 2 + 1):
                    leader_settle_time = elapsed

    if t0 is None:
        return []

    log_metrics = _parse_robot_log_metrics(log_dir, t0)
    final_coverage = len(visited_cells) / max(1, geometry.explorable_cell_count()) * 100.0
    coverage_at_90s = None
    for elapsed, coverage in coverage_timeline:
        if elapsed >= 90.0:
            coverage_at_90s = coverage
            break
    if coverage_at_90s is None:
        coverage_at_90s = final_coverage

    result = ExperimentResult(
        name=meta.get("trial_name", os.path.basename(log_dir)),
        controller=meta.get("controller", "reip"),
        fault_type=meta.get("fault_type"),
        fault_robot=None,
        layout="multiroom",
        trial=int(meta.get("trial_num", 1)),
        seed=0,
        final_coverage=final_coverage,
        time_to_50=time_to_50,
        time_to_60=time_to_60,
        time_to_80=time_to_80,
        time_to_detection=log_metrics["time_to_detection"],
        time_to_first_suspicion=log_metrics["time_to_first_suspicion"],
        time_to_detection_2=None,
        num_leader_changes=num_leader_changes,
        false_positives=false_positives,
        coverage_at_90s=coverage_at_90s,
        coverage_timeline=coverage_timeline,
        leader_settle_time=leader_settle_time,
        divider_crossing_time=divider_crossing_time,
        mission_complete_time=log_metrics["mission_complete_time"],
        peer_emergency_count=int(log_metrics["peer_emergency_count"]),
        min_peer_separation_mm=min_peer_separation_mm,
        min_wall_clearance_mm=min_wall_clearance_mm,
        coverage_order=coverage_order,
    )
    return [result]


def validate_sim_hardware(sim_results: List[ExperimentResult], hardware_json: str):
    """Compare simulation results to hardware results and report validation status.
    
    Returns: (passed: bool, report: str)
    """
    hardware_results = []
    if os.path.exists(hardware_json):
        if os.path.isdir(hardware_json) or os.path.basename(hardware_json) == "trial_meta.json":
            hardware_results = _load_hardware_trial_results(hardware_json)
        else:
            with open(hardware_json) as f:
                hardware_data = json.load(f)
            if isinstance(hardware_data, list):
                for h in hardware_data:
                    try:
                        r = ExperimentResult(**{
                            k: v for k, v in h.items()
                            if k in ExperimentResult.__dataclass_fields__
                        })
                        hardware_results.append(r)
                    except Exception:
                        pass
    
    if not hardware_results:
        return False, f"ERROR: Could not parse hardware results from {hardware_json}"
    
    # Group by condition
    sim_groups = defaultdict(list)
    hw_groups = defaultdict(list)
    
    for r in sim_results:
        key = f"{r.controller}|{r.fault_type or 'none'}"
        sim_groups[key].append(r)
    
    for r in hardware_results:
        key = f"{r.controller}|{r.fault_type or 'none'}"
        hw_groups[key].append(r)
    
    def _mean_optional(values):
        vals = [v for v in values if v is not None]
        return (sum(vals) / len(vals)) if vals else None

    def _normalize_coverage_order(order):
        normalized = []
        for cell in order or []:
            if isinstance(cell, (list, tuple)) and len(cell) == 2:
                normalized.append((int(cell[0]), int(cell[1])))
        return normalized

    def _coverage_prefix_match(sim_trials, hw_trials, prefix_len=25):
        sims = [_normalize_coverage_order(t.coverage_order) for t in sim_trials if t.coverage_order]
        hws = [_normalize_coverage_order(t.coverage_order) for t in hw_trials if t.coverage_order]
        if not sims or not hws:
            return None
        scores = []
        for sim_order in sims:
            for hw_order in hws:
                limit = min(prefix_len, len(sim_order), len(hw_order))
                if limit <= 0:
                    continue
                matches = sum(1 for idx in range(limit) if sim_order[idx] == hw_order[idx])
                scores.append(matches / limit)
        return (sum(scores) / len(scores)) if scores else None

    # Compare each condition
    report_lines = []
    report_lines.append("=" * 80)
    report_lines.append("SIMULATION vs HARDWARE VALIDATION")
    report_lines.append("=" * 80)
    report_lines.append("")
    
    all_passed = True
    tolerance = {
        'coverage': 10.0,      # 10% absolute difference
        'detection_time': 0.5,  # 0.5s difference
        'suspicion_time': 0.3,  # 0.3s difference
        'leader_settle_time': 1.0,
        'divider_crossing_time': 2.0,
        'mission_complete_time': 3.0,
        'peer_emergency_count': 2,
        'min_peer_separation_mm': 80.0,
        'min_wall_clearance_mm': 60.0,
        'coverage_prefix_match': 0.60,
    }
    
    for key in sorted(set(list(sim_groups.keys()) + list(hw_groups.keys()))):
        sim_trials = sim_groups.get(key, [])
        hw_trials = hw_groups.get(key, [])
        
        if not sim_trials:
            report_lines.append(f"  {key}: Hardware only (no sim data)")
            continue
        if not hw_trials:
            report_lines.append(f"  {key}: Sim only (no hardware data)")
            continue
        
        # Compute means
        sim_cov = sum(t.final_coverage for t in sim_trials) / len(sim_trials)
        hw_cov = sum(t.final_coverage for t in hw_trials) / len(hw_trials)
        cov_diff = abs(sim_cov - hw_cov)
        
        sim_det_mean = _mean_optional([t.time_to_detection for t in sim_trials])
        hw_det_mean = _mean_optional([t.time_to_detection for t in hw_trials])
        sim_sus_mean = _mean_optional([t.time_to_first_suspicion for t in sim_trials])
        hw_sus_mean = _mean_optional([t.time_to_first_suspicion for t in hw_trials])
        sim_leader_mean = _mean_optional([t.leader_settle_time for t in sim_trials])
        hw_leader_mean = _mean_optional([t.leader_settle_time for t in hw_trials])
        sim_cross_mean = _mean_optional([t.divider_crossing_time for t in sim_trials])
        hw_cross_mean = _mean_optional([t.divider_crossing_time for t in hw_trials])
        sim_complete_mean = _mean_optional([t.mission_complete_time for t in sim_trials])
        hw_complete_mean = _mean_optional([t.mission_complete_time for t in hw_trials])
        sim_peer_emergency = _mean_optional([float(t.peer_emergency_count) for t in sim_trials])
        hw_peer_emergency = _mean_optional([float(t.peer_emergency_count) for t in hw_trials])
        sim_min_peer_sep = _mean_optional([t.min_peer_separation_mm for t in sim_trials])
        hw_min_peer_sep = _mean_optional([t.min_peer_separation_mm for t in hw_trials])
        sim_min_wall = _mean_optional([t.min_wall_clearance_mm for t in sim_trials])
        hw_min_wall = _mean_optional([t.min_wall_clearance_mm for t in hw_trials])
        coverage_prefix_match = _coverage_prefix_match(sim_trials, hw_trials)
        
        # Check pass/fail
        passed = True
        if cov_diff > tolerance['coverage']:
            passed = False
            all_passed = False
        
        det_diff = abs(sim_det_mean - hw_det_mean) if (sim_det_mean is not None and hw_det_mean is not None) else None
        if det_diff and det_diff > tolerance['detection_time']:
            passed = False
            all_passed = False
        
        sus_diff = abs(sim_sus_mean - hw_sus_mean) if (sim_sus_mean is not None and hw_sus_mean is not None) else None
        if sus_diff and sus_diff > tolerance['suspicion_time']:
            passed = False
            all_passed = False

        leader_diff = abs(sim_leader_mean - hw_leader_mean) if (sim_leader_mean is not None and hw_leader_mean is not None) else None
        if leader_diff is not None and leader_diff > tolerance['leader_settle_time']:
            passed = False
            all_passed = False

        crossing_diff = abs(sim_cross_mean - hw_cross_mean) if (sim_cross_mean is not None and hw_cross_mean is not None) else None
        if crossing_diff is not None and crossing_diff > tolerance['divider_crossing_time']:
            passed = False
            all_passed = False

        complete_diff = abs(sim_complete_mean - hw_complete_mean) if (sim_complete_mean is not None and hw_complete_mean is not None) else None
        if complete_diff is not None and complete_diff > tolerance['mission_complete_time']:
            passed = False
            all_passed = False

        peer_emergency_diff = abs(sim_peer_emergency - hw_peer_emergency) if (sim_peer_emergency is not None and hw_peer_emergency is not None) else None
        if peer_emergency_diff is not None and peer_emergency_diff > tolerance['peer_emergency_count']:
            passed = False
            all_passed = False

        peer_sep_diff = abs(sim_min_peer_sep - hw_min_peer_sep) if (sim_min_peer_sep is not None and hw_min_peer_sep is not None) else None
        if peer_sep_diff is not None and peer_sep_diff > tolerance['min_peer_separation_mm']:
            passed = False
            all_passed = False

        wall_diff = abs(sim_min_wall - hw_min_wall) if (sim_min_wall is not None and hw_min_wall is not None) else None
        if wall_diff is not None and wall_diff > tolerance['min_wall_clearance_mm']:
            passed = False
            all_passed = False

        if coverage_prefix_match is not None and coverage_prefix_match < tolerance['coverage_prefix_match']:
            passed = False
            all_passed = False
        
        status = "PASS PASS" if passed else "FAIL FAIL"
        report_lines.append(f"  {status} {key}:")
        report_lines.append(f"    Coverage: Sim {sim_cov:.1f}% vs HW {hw_cov:.1f}% (diff: {cov_diff:.1f}%)")
        if sim_det_mean is not None and hw_det_mean is not None:
            report_lines.append(f"    Detection: Sim {sim_det_mean:.2f}s vs HW {hw_det_mean:.2f}s (diff: {det_diff:.2f}s)")
        if sim_sus_mean is not None and hw_sus_mean is not None:
            report_lines.append(f"    Suspicion: Sim {sim_sus_mean:.2f}s vs HW {hw_sus_mean:.2f}s (diff: {sus_diff:.2f}s)")
        if leader_diff is not None:
            report_lines.append(f"    Leader settle: Sim {sim_leader_mean:.2f}s vs HW {hw_leader_mean:.2f}s (diff: {leader_diff:.2f}s)")
        if crossing_diff is not None:
            report_lines.append(f"    Divider crossing: Sim {sim_cross_mean:.2f}s vs HW {hw_cross_mean:.2f}s (diff: {crossing_diff:.2f}s)")
        if complete_diff is not None:
            report_lines.append(f"    Mission complete: Sim {sim_complete_mean:.2f}s vs HW {hw_complete_mean:.2f}s (diff: {complete_diff:.2f}s)")
        if peer_emergency_diff is not None:
            report_lines.append(f"    Peer emergency count: Sim {sim_peer_emergency:.1f} vs HW {hw_peer_emergency:.1f} (diff: {peer_emergency_diff:.1f})")
        if peer_sep_diff is not None:
            report_lines.append(f"    Min peer separation: Sim {sim_min_peer_sep:.1f}mm vs HW {hw_min_peer_sep:.1f}mm (diff: {peer_sep_diff:.1f}mm)")
        if wall_diff is not None:
            report_lines.append(f"    Min wall clearance: Sim {sim_min_wall:.1f}mm vs HW {hw_min_wall:.1f}mm (diff: {wall_diff:.1f}mm)")
        if coverage_prefix_match is not None:
            report_lines.append(f"    Coverage-order prefix match: {coverage_prefix_match:.2f}")
        report_lines.append("")
    
    report_lines.append("=" * 80)
    if all_passed:
        report_lines.append("VALIDATION PASSED: Simulation matches hardware within tolerances")
        report_lines.append("  -> Safe to use 100-trial sim results for paper")
    else:
        report_lines.append("VALIDATION FAILED: Significant differences detected")
        report_lines.append("  -> Fix simulation physics before using results")
    report_lines.append("=" * 80)
    
    return all_passed, "\n".join(report_lines)


# ==================== Main ====================
def main():
    import argparse
    from concurrent.futures import ProcessPoolExecutor, as_completed

    parser = argparse.ArgumentParser(description="ISEF Experiment Suite")
    parser.add_argument("--layout", default="all",
                        help="'open', 'multiroom', or 'all' (default: all)")
    parser.add_argument("--trials", type=int, default=TRIALS_PER_CONDITION,
                        help=f"Trials per condition (default: {TRIALS_PER_CONDITION})")
    parser.add_argument("--quick", action="store_true",
                        help="Quick check: N=3, open layout only")
    parser.add_argument("--condition", default=None,
                        help="Run only conditions matching this substring (e.g. 'BadLeader')")
    parser.add_argument("--workers", type=int, default=0,
                        help="Parallel workers (0=auto based on CPU cores, 1=sequential)")
    parser.add_argument("--snapshots", action="store_true",
                        help="Save per-timestep position snapshots for gridworld viz")
    parser.add_argument("--ablation", action="store_true",
                        help="Run ablation study conditions instead of standard conditions")
    parser.add_argument("--duration", type=float, default=EXPERIMENT_DURATION,
                        help=f"Per-trial duration in seconds (default: {EXPERIMENT_DURATION})")
    parser.add_argument("--validate", type=str, metavar="HARDWARE_JSON",
                        help="Validation mode: compare sim results to hardware results from JSON file")
    parser.add_argument("--hardware-positions", type=str, metavar="POSITIONS_JSON",
                        help="Use fixed starting positions from JSON (e.g., from ArUco detection)")
    parser.add_argument("--sim-mode", choices=["hardware_clone", "legacy"], default="hardware_clone",
                        help="Simulation backend to use (default: hardware_clone)")
    parser.add_argument("--impairment-preset", choices=sorted(IMPAIRMENT_PRESETS.keys()),
                        default="hardware_clone_clean",
                        help="Sensor/network impairment preset for hardware_clone mode")
    parser.add_argument("--adversary", action="store_true",
                        help="Run adaptive adversary attack conditions")
    parser.add_argument("--fault-schedule", choices=FAULT_SCHEDULE_CHOICES,
                        default="default",
                        help="Timed fault schedule to apply to non-clean conditions")
    args = parser.parse_args()
    globals()["EXPERIMENT_DURATION"] = args.duration

    # Load hardware positions if provided
    fixed_positions = None
    if args.hardware_positions:
        with open(args.hardware_positions) as f:
            data = json.load(f)
        if 'sim_fixed_positions' in data:
            fixed_positions = {int(k): tuple(v) for k, v in data['sim_fixed_positions'].items()}
            print(f"Loaded {len(fixed_positions)} hardware starting positions")
        elif 'robot_positions' in data:
            import math
            fixed_positions = {}
            for rid, pos in data['robot_positions'].items():
                x, y, heading_deg = pos
                fixed_positions[int(rid)] = (float(x), float(y), math.radians(heading_deg))
            print(f"Loaded {len(fixed_positions)} hardware starting positions (converted)")
        else:
            print(f"WARNING: No position data found in {args.hardware_positions}")
    
    if args.quick:
        n_trials = 3
        layouts = ["open"]
    else:
        n_trials = args.trials
        layouts = list(ARENA_LAYOUTS.keys()) if args.layout == "all" else [args.layout]

    # Filter conditions if requested
    if args.ablation:
        conditions = ABLATION_CONDITIONS
    else:
        conditions = EXPERIMENT_CONDITIONS
    if args.adversary:
        conditions = ADVERSARY_CONDITIONS
    if args.condition and args.condition.lower() != "all":
        conditions = [c for c in conditions if args.condition.lower() in c[0].lower()
                      or args.condition.lower() in c[1].lower()]

    # Build the full job list
    # Each job: (name, layout, controller, fault_type, fault_robot, trial, seed,
    #            ablation, fixed_positions, fault_schedule_name)
    jobs = []
    for layout in layouts:
        for trial in range(1, n_trials + 1):
            seed = trial * 1000
            for cond_tag, controller, fault_type, fault_robot, ablation in conditions:
                name = f"{layout}_{controller}_{cond_tag}_t{trial}"
                # Use fixed positions if provided (only for first trial for validation)
                use_fixed = fixed_positions if (fixed_positions and trial == 1) else None
                if "Adversary_Adaptive" in cond_tag:
                    schedule_name = "adaptive"
                elif fault_type not in (None, "", "none", "clear"):
                    schedule_name = args.fault_schedule
                else:
                    schedule_name = None
                jobs.append((name, layout, controller, fault_type, fault_robot, trial, seed, ablation, use_fixed, schedule_name))

    total = len(jobs)
    if total == 0:
        print("No experiments matched the given filters. Check --condition and --layout.")
        sys.exit(1)

    # Decide number of parallel workers
    # Each experiment uses ~6 processes (5 robots + harness) but they're mostly
    # sleeping (20Hz loop = 95% idle), so we can run many workers per core.
    # Physics is time-based, so results are consistent regardless of load.
    cpu_count = os.cpu_count() or 4
    if args.workers > 0:
        n_workers = args.workers
    else:
        # Default: ~2 workers per physical core (each experiment is I/O-bound)
        n_workers = max(1, min(cpu_count // 2, total))
    n_workers = min(n_workers, total)  # Never more workers than jobs

    seq_hours = total * (EXPERIMENT_DURATION + 8) / 3600
    par_hours = (total / n_workers) * (EXPERIMENT_DURATION + 8) / 3600

    print(f"\nISEF Experiment Suite")
    print(f"  Layouts: {layouts}")
    print(f"  Conditions: {len(conditions)}")
    print(f"  Trials per condition: {n_trials}")
    print(f"  Total experiments: {total}")
    print(f"  Workers: {n_workers} parallel")
    print(f"  Est. time: {par_hours:.1f}h  (sequential would be {seq_hours:.1f}h)")
    print()

    # ---- Create a named run directory ----
    # Format: experiments/run_YYYYMMDD_HHMMSS_<layout>_<N>trials/
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    layout_tag = "+".join(layouts)
    cond_tag = args.condition or "all"
    run_name = f"run_{timestamp}_{layout_tag}_{n_trials}trials_{cond_tag}"
    output_dir = os.path.join("experiments", run_name)
    os.makedirs(output_dir, exist_ok=True)

    print(f"  Output: {output_dir}/")
    print()

    if n_workers <= 1:
        # ---- Sequential mode (simple, for debugging) ----
        runner = ExperimentRunner(
            output_dir,
            port_offset=SIM_PORT_OFFSET_BASE,
            sim_mode=args.sim_mode,
            impairment_preset=args.impairment_preset,
        )
        all_results: List[ExperimentResult] = []
        for i, job in enumerate(jobs):
            name, layout, controller, fault_type, fault_robot, trial, seed, ablation, fixed_positions, fault_schedule_name = job
            print(f"\n>>> Experiment {i+1}/{total}: {name}")
            runner.set_layout(layout)
            runner._fixed_positions_for_next_trial = fixed_positions
            result = runner.run_experiment(
                name=name, controller=controller,
                fault_type=fault_type, fault_robot=fault_robot,
                fault_schedule_name=fault_schedule_name,
                trial_num=trial, seed=seed,
                save_snapshots=args.snapshots,
                ablation=ablation,
            )
            all_results.append(result)
            time.sleep(2)

        final_path = runner.save_results(all_results, tag="final")
        print_statistics(all_results)
    else:
        # ---- Parallel mode ----
        # Partition jobs into N worker queues; each queue runs sequentially
        # within its process, so port offsets never collide.
        worker_queues: List[List[tuple]] = [[] for _ in range(n_workers)]
        for i, job in enumerate(jobs):
            worker_queues[i % n_workers].append(job)

        all_results: List[ExperimentResult] = []
        completed = 0
        start_time = time.time()

        # Build worker args: (worker_id, job_list, output_dir)
        worker_args = [(wid, wq, output_dir, args.sim_mode, args.impairment_preset)
                       for wid, wq in enumerate(worker_queues)]

        with ProcessPoolExecutor(max_workers=n_workers) as executor:
            futures = {executor.submit(_run_worker, wa): wa[0]
                       for wa in worker_args}

            for future in as_completed(futures):
                wid = futures[future]
                try:
                    returned_wid, result_dicts = future.result()
                except Exception as e:
                    import traceback
                    print(f"  [Worker {wid} CRASHED]: {e}")
                    traceback.print_exc()
                    continue
                for rd in result_dicts:
                    completed += 1
                    rd.pop('coverage_timeline', None)
                    r = ExperimentResult(**{
                        k: v for k, v in rd.items()
                        if k in ExperimentResult.__dataclass_fields__
                    })
                    all_results.append(r)
                elapsed = time.time() - start_time
                print(f"  Worker {wid} done: {len(result_dicts)} experiments "
                      f"({elapsed/60:.1f}m elapsed)")

        # Save results directly (no ExperimentRunner needed)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        final_path = os.path.join(output_dir, f"results_final_{timestamp}.json")
        with open(final_path, 'w') as f:
            json.dump([asdict(r) for r in all_results], f, indent=2)
        print(f"\nResults saved to: {final_path}")
        print(f"  ({len(all_results)} experiments collected from {n_workers} workers)")
        print_statistics(all_results)

    # ---- Write a human-readable summary alongside the JSON ----
    summary_path = os.path.join(output_dir, "SUMMARY.txt")
    with open(summary_path, 'w') as f:
        f.write(f"REIP Experiment Run: {run_name}\n")
        f.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M')}\n")
        f.write(f"Layouts: {layouts}\n")
        f.write(f"Conditions: {len(conditions)}\n")
        f.write(f"Trials per condition: {n_trials}\n")
        f.write(f"Total experiments: {total}\n")
        f.write(f"Workers: {n_workers}\n\n")
        # Write the stats table
        import io
        old_stdout = sys.stdout
        sys.stdout = buf = io.StringIO()
        print_statistics(all_results)
        sys.stdout = old_stdout
        f.write(buf.getvalue())

    if args.validate:
        passed, report = validate_sim_hardware(all_results, args.validate)
        validation_path = os.path.join(output_dir, "VALIDATION.txt")
        with open(validation_path, 'w') as f:
            f.write(report)
        print("\n" + report)
        print(f"\nValidation report: {validation_path}")

    print(f"\nDone! {len(all_results)} experiments completed.")
    print(f"Results dir: {output_dir}/")
    print(f"  results_final_*.json  - raw data")
    print(f"  SUMMARY.txt           - human-readable stats")
    print(f"  logs/                 - per-experiment robot logs")
    print(f"\nGenerate graphs:")
    print(f"  python test/generate_poster_graphs.py {output_dir}/results_final_*.json")


if __name__ == "__main__":
    main()
