#!/usr/bin/env python3
"""
RAFT Heartbeat Baseline - Judge-Proof Comparison

Implements a simplified version of the RAFT consensus algorithm
(Ongaro & Ousterhout 2014, "In Search of an Understandable Consensus
Algorithm") adapted for multi-robot frontier exploration.

This is industry-standard failover:
- Leader sends heartbeats
- If heartbeats stop, elect new leader
- NO trust. NO performance reasoning.

Key difference from REIP:
- RAFT catches CRASH faults (leader dies, heartbeats stop)
- REIP catches BYZANTINE faults (leader alive but misbehaving)
  Lamport et al. 1982, "The Byzantine Generals Problem"

A spinning/erratic leader still sends heartbeats -> RAFT never catches it.
REIP's trust model does.

Frontier exploration uses greedy nearest-frontier assignment
(Burgard et al. 2000, "Collaborative multi-robot exploration").

Usage: python3 raft_node.py <robot_id> [--sim]
"""

import socket
import json
import time
import math
import hashlib
import threading
import random
import os
import signal
from dataclasses import dataclass, field
from typing import Dict, Optional, Set, Tuple
from enum import Enum

def _sigterm_handler(signum, frame):
    raise KeyboardInterrupt
signal.signal(signal.SIGTERM, _sigterm_handler)

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in os.sys.path:
    os.sys.path.insert(0, PROJECT_ROOT)

try:
    from hardware_fidelity import (
        BROADCAST_RATE_HZ,
        CONTROL_RATE_HZ,
        DEFAULT_ARENA,
        LEADER_ACK_STALE_S,
        PEER_TIMEOUT_S,
        POSITION_FRESHNESS_S,
        POSITION_STOP_TIMEOUT_S,
        SIM_MOTOR_PORT,
        SIM_PEER_RELAY_PORT,
        SIM_SENSOR_PORT,
    )
except ImportError:
    import math as _math

    BROADCAST_RATE_HZ = 5.0
    CONTROL_RATE_HZ = 10.0
    LEADER_ACK_STALE_S = 1.5
    PEER_TIMEOUT_S = 3.0
    POSITION_FRESHNESS_S = 0.5
    POSITION_STOP_TIMEOUT_S = 2.0
    SIM_MOTOR_PORT = 6100
    SIM_PEER_RELAY_PORT = 5500  # Must match REIP and harness
    SIM_SENSOR_PORT = 6300

    class _ArenaGeometry:
        width_mm = 2000; height_mm = 1500; cell_size_mm = 125
        robot_radius_mm = 110.0; body_radius_mm = 77; body_half_width_mm = 64
        body_front_mm = 55; swept_radius_mm = int(_math.sqrt(77**2 + 64**2))
        tof_sensor_offset_mm = 55; avoid_distance_mm = 280; critical_distance_mm = 160
        interior_wall_x_left_mm = 1000; interior_wall_x_right_mm = 1036
        interior_wall_x_mm = 1018; interior_wall_y_end_mm = 1050
        outer_wall_margin_mm = 110; divider_margin_mm = 64; repulsion_zone_mm = 350

        @property
        def divider_tip_clearance_mm(self):
            return self.swept_radius_mm

        def cell_to_pos(self, cell):
            return ((cell[0] + 0.5) * self.cell_size_mm, (cell[1] + 0.5) * self.cell_size_mm)

        def is_free_point(self, x_mm, y_mm, wall_clearance_mm=None):
            c = self.outer_wall_margin_mm if wall_clearance_mm is None else wall_clearance_mm
            if x_mm < c or x_mm > self.width_mm - c:
                return False
            if y_mm < c or y_mm > self.height_mm - c:
                return False
            if y_mm < self.interior_wall_y_end_mm:
                dc = min(c, self.divider_margin_mm)
                if self.interior_wall_x_left_mm - dc < x_mm < self.interior_wall_x_right_mm + dc:
                    return False
            for tip_x in (self.interior_wall_x_left_mm, self.interior_wall_x_right_mm):
                if _math.hypot(x_mm - tip_x, y_mm - self.interior_wall_y_end_mm) < self.divider_tip_clearance_mm:
                    return False
            return True

        def is_wall_cell(self, cx, cy):
            pos = self.cell_to_pos((cx, cy))
            return not self.is_free_point(pos[0], pos[1], wall_clearance_mm=self.outer_wall_margin_mm)

    DEFAULT_ARENA = _ArenaGeometry()

MAX_RX_MESSAGES_PER_TICK = 32

# Hardware imports
try:
    import serial
    import smbus
    import busio
    import board
    import adafruit_vl53l0x
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
    print("[WARN] Hardware modules not available - simulation mode")

# ============== Configuration ==============
# MUST MATCH robot/reip_node.py
UDP_POSITION_PORT = 5100
UDP_PEER_PORT = 5200
UDP_FAULT_PORT = 5300
UDP_SIM_MOTOR_PORT = SIM_MOTOR_PORT
UDP_SIM_PEER_RELAY_PORT = SIM_PEER_RELAY_PORT
UDP_SIM_SENSOR_PORT = SIM_SENSOR_PORT
BROADCAST_IP = "192.168.20.255"
NUM_ROBOTS = 5

ARENA_WIDTH = DEFAULT_ARENA.width_mm
ARENA_HEIGHT = DEFAULT_ARENA.height_mm
CELL_SIZE = DEFAULT_ARENA.cell_size_mm

ROBOT_RADIUS = DEFAULT_ARENA.robot_radius_mm   # mm, bounding diagonal from ArUco to rear wheel corner
BODY_RADIUS = DEFAULT_ARENA.body_radius_mm     # mm, max body extent behind ArUco (rear edge)
BODY_HALF_WIDTH = DEFAULT_ARENA.body_half_width_mm # mm, half width (128mm / 2)
BODY_FRONT = DEFAULT_ARENA.body_front_mm      # mm, body extent ahead of ArUco (front bumper with ToF)
SWEPT_RADIUS = DEFAULT_ARENA.swept_radius_mm   # mm, sqrt(77^2 + 64^2) - rear corner during rotation
TOF_SENSOR_OFFSET = DEFAULT_ARENA.tof_sensor_offset_mm  # mm, ToF sensors are this far ahead of ArUco center
AVOID_DISTANCE = DEFAULT_ARENA.avoid_distance_mm
CRITICAL_DISTANCE = DEFAULT_ARENA.critical_distance_mm

# Interior wall: 35.7mm thick foam board, left face at x=1000
INTERIOR_WALL_X_LEFT = DEFAULT_ARENA.interior_wall_x_left_mm
INTERIOR_WALL_X_RIGHT = DEFAULT_ARENA.interior_wall_x_right_mm
INTERIOR_WALL_X = DEFAULT_ARENA.interior_wall_x_mm        # center (for legacy/tip)
INTERIOR_WALL_Y_END = DEFAULT_ARENA.interior_wall_y_end_mm

# Outer wall margin: at least ROBOT_RADIUS so frontier assignment never
# sends robots to cells they can't physically reach without hitting the wall.
OUTER_WALL_MARGIN = DEFAULT_ARENA.outer_wall_margin_mm      # ~110mm - excludes perimeter cells
DIVIDER_MARGIN = DEFAULT_ARENA.divider_margin_mm       # 64mm - robot body extends this far from center
WALL_MARGIN = OUTER_WALL_MARGIN       # default used by most code
REPULSION_ZONE = DEFAULT_ARENA.repulsion_zone_mm                  # mm - soft repulsion, wide enough to steer clear

# RAFT parameters - NO TRUST, only heartbeats
HEARTBEAT_INTERVAL = 0.5   # Leader sends heartbeat this often
HEARTBEAT_TIMEOUT = 2.0    # Assume leader dead after this silence
ELECTION_TIMEOUT_MIN = 1.0
ELECTION_TIMEOUT_MAX = 2.0
LEADER_ACK_STALE = LEADER_ACK_STALE_S

CONTROL_RATE = CONTROL_RATE_HZ
BROADCAST_RATE = BROADCAST_RATE_HZ
BASE_SPEED = 80              # must match reip_node.py for fair comparison

TOF_CHANNELS = {0: "right", 1: "front_right", 2: "front", 3: "front_left", 4: "left"}

# ============== Enums ==============
class RaftState(Enum):
    FOLLOWER = "follower"
    CANDIDATE = "candidate"
    LEADER = "leader"

# ============== Hardware (same as reip_node) ==============
class Hardware:
    def __init__(self):
        self._uart_lock = threading.Lock()
        self.hw_ok = False
        self._sim_tof = {name: 9999 for name in TOF_CHANNELS.values()}
        self._sim_encoders = (0, 0)
        self._last_sim_motor_cmd = (0.0, 0.0)
        if not HARDWARE_AVAILABLE:
            print("  Hardware: SIMULATED")
            return

        for attempt in range(3):
            try:
                self.uart = serial.Serial('/dev/serial0', 115200, timeout=0.1)
                time.sleep(0.5)
                self.uart.reset_input_buffer()
                self.bus = smbus.SMBus(1)
                try:
                    self.bus.write_byte(0x70, 0)
                except Exception:
                    pass
                time.sleep(0.2)
                self.i2c = busio.I2C(board.SCL, board.SDA)
                self.MUX_ADDR = 0x70
                self.tof_sensors = {}
                self._init_tof_sensors()
                if len(self.tof_sensors) < 3:
                    raise RuntimeError(f"Only {len(self.tof_sensors)}/5 ToF sensors init'd")
                self._ping_pico()
                self.hw_ok = True
                break
            except Exception as e:
                print(f"  Hardware init attempt {attempt+1}/3 failed: {e}")
                try:
                    self.bus.close()
                except Exception:
                    pass
                try:
                    self.i2c.deinit()
                except Exception:
                    pass
                time.sleep(1.0 + attempt)
        if not self.hw_ok:
            print("  WARNING: Hardware init failed, running with degraded sensors")
    
    def _ping_pico(self):
        if not HARDWARE_AVAILABLE: return True
        for _ in range(3):
            try:
                self.uart.reset_input_buffer()
                self.uart.write(b'PING\n')
                time.sleep(0.1)
                if self.uart.readline().decode().strip() == "PONG":
                    print("  Pico: OK")
                    return True
            except: pass
            time.sleep(0.3)
        print("  Pico: FAILED")
        return False
    
    def _init_tof_sensors(self):
        if not HARDWARE_AVAILABLE: return
        for ch, name in TOF_CHANNELS.items():
            try:
                self.bus.write_byte(self.MUX_ADDR, 1 << ch)
                time.sleep(0.05)
                self.tof_sensors[name] = adafruit_vl53l0x.VL53L0X(self.i2c)
            except Exception as e:
                print(f"  ToF {name}: FAILED ({e})")
    
    def _select_mux(self, ch):
        if not HARDWARE_AVAILABLE: return
        try:
            self.bus.write_byte(self.MUX_ADDR, 1 << ch)
            time.sleep(0.001)  # 1ms, tca9548a switches in <1us
        except: pass
    
    def read_tof_all(self) -> Dict[str, int]:
        if not HARDWARE_AVAILABLE:
            return dict(self._sim_tof)
        distances = {}
        for ch, name in TOF_CHANNELS.items():
            try:
                self._select_mux(ch)
                if name in self.tof_sensors:
                    distances[name] = self.tof_sensors[name].range
                else:
                    distances[name] = -1
            except:
                distances[name] = -1
        return distances
    
    def read_encoders(self) -> tuple:
        if not HARDWARE_AVAILABLE: return self._sim_encoders
        with self._uart_lock:
            try:
                self.uart.reset_input_buffer()
                self.uart.write(b'ENC\n')
                time.sleep(0.02)
                resp = self.uart.readline().decode().strip()
                if ',' in resp:
                    l, r = resp.split(',')
                    return (int(l), int(r))
            except: pass
        return (0, 0)
    
    def set_motors(self, left: float, right: float):
        if not HARDWARE_AVAILABLE:
            self._last_sim_motor_cmd = (left, right)
            return
        with self._uart_lock:
            try:
                self.uart.reset_input_buffer()
                self.uart.write(f"MOT,{left:.1f},{right:.1f}\n".encode())
            except: pass
    
    def stop(self):
        if not HARDWARE_AVAILABLE:
            self._last_sim_motor_cmd = (0.0, 0.0)
            return
        with self._uart_lock:
            try:
                self.uart.reset_input_buffer()
                self.uart.write(b'STOP\n')
            except: pass

    def update_sim_feedback(self, tof: Optional[Dict[str, int]] = None,
                            encoders: Optional[Tuple[int, int]] = None):
        if tof is not None:
            for name in TOF_CHANNELS.values():
                self._sim_tof[name] = int(tof.get(name, self._sim_tof.get(name, 9999)))
        if encoders is not None:
            self._sim_encoders = (int(encoders[0]), int(encoders[1]))

# ============== RAFT Node ==============
class RAFTNode:
    """
    RAFT-style leader election with HEARTBEAT ONLY.
    
    NO trust scores. NO performance checks.
    Leader is replaced ONLY when heartbeats stop.
    
    This is the judge-proof baseline: industry-standard failover.
    REIP should beat this under adversarial faults.
    """
    
    def __init__(self, robot_id: int, sim_mode: bool = False):
        self.robot_id = robot_id
        self.sim_mode = sim_mode
        
        mode_str = "SIM" if sim_mode else "HARDWARE"
        print(f"=== RAFT Baseline - Robot {robot_id} [{mode_str}] ===")
        print(f"NO TRUST. Heartbeat timeout only.\n")
        
        # Position
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.position_timestamp = 0.0
        self.position_rx_mono = 0.0
        self._pos_initialized = False
        
        # Sensors
        self.tof = {}
        self.encoders = (0, 0)
        
        # RAFT state
        self.state = RaftState.FOLLOWER
        self.current_term = 0
        self.voted_for: Optional[int] = None
        self.current_leader: Optional[int] = None
        self.last_heartbeat = time.time()
        self.election_timeout = self._random_election_timeout()
        self.leader_mission_complete = False
        self.active_robot_ids: Set[int] = set(range(1, NUM_ROBOTS + 1))
        self.active_robot_count: int = NUM_ROBOTS
        self._votes_received: Set[int] = set()
        self._heartbeat_seq = 0
        self._heartbeat_digest: Optional[str] = None
        self._heartbeat_ack_sets: Dict[str, Set[int]] = {}
        self._leader_quorum_mono = 0.0
        self._leader_claim_mono = 0.0
        self._last_acked_heartbeat_key: Optional[Tuple[int, int, str]] = None
        
        # Peers
        self.peers: Dict[int, dict] = {}
        self.peer_timeout = 3.0
        self._peer_seq = 0
        
        # Coverage
        self.coverage_width = int(ARENA_WIDTH / CELL_SIZE)
        self.coverage_height = int(ARENA_HEIGHT / CELL_SIZE)
        self.my_visited: Set[tuple] = set()
        self.known_visited: Set[tuple] = set()
        
        # Task from leader
        self.assigned_target: Optional[Tuple[float, float]] = None
        self._prev_assignments: Dict[str, Tuple[float, float]] = {}  # Assignment persistence
        self.current_navigation_target: Optional[Tuple[float, float]] = None
        self._nav_waypoint_cell: Optional[Tuple[int, int]] = None
        self._nav_waypoint_set_mono = 0.0
        self._raw_navigation_target: Optional[Tuple[float, float]] = None
        self.assigned_target_rx_mono = 0.0
        self._last_command_source = "startup"
        self._last_stop_reason = "waiting_for_start"
        self._last_target_angle = 0.0
        self._last_heading_error = 0.0
        self._last_motor_cmd = (0.0, 0.0)
        
        # Logging
        self.log_file = None
        self.init_logging()
        
        # Hardware
        print("Initializing hardware...")
        self.hw = Hardware()
        
        # Sockets
        self._init_sockets()
        
        self.trial_started = False
        self._trial_start_mono = 0.0
        self._trial_max_duration = 180.0

        # Fault injection
        self.injected_fault = None
        self.bad_leader_mode = False
        self.self_injure_leader_mode = False
        self.oscillate_leader_mode = False
        self._oscillate_phase = 0
        self.freeze_leader_mode = False
        self._frozen_assignments: Dict[str, Tuple[float, float]] = {}
        
        self.running = False
        print("Ready!\n")
    
    def _random_election_timeout(self) -> float:
        return random.uniform(ELECTION_TIMEOUT_MIN, ELECTION_TIMEOUT_MAX)

    def _quorum_size(self) -> int:
        return max(1, self.active_robot_count // 2 + 1)

    def _leader_quorum_ok(self) -> bool:
        if self.state != RaftState.LEADER:
            return False
        return (time.monotonic() - self._leader_quorum_mono) <= LEADER_ACK_STALE

    def _trusted_leader_reports_complete(self) -> bool:
        if self.current_leader is None or self.state == RaftState.LEADER:
            return False
        peer = self.peers.get(self.current_leader)
        if peer is None:
            return False
        if time.time() - peer.get('last_seen', 0) >= self.peer_timeout:
            return False
        return bool(peer.get('leader_established', False) and peer.get('mission_complete', False))

    def _heartbeat_digest_for(self, leader_id: int, term: int, heartbeat_seq: int,
                              assignments: Dict[str, Tuple[float, float]]) -> str:
        payload = {
            'leader_id': leader_id,
            'term': term,
            'heartbeat_seq': heartbeat_seq,
            'assignments': assignments,
        }
        raw = json.dumps(payload, sort_keys=True, separators=(',', ':')).encode()
        return hashlib.sha256(raw).hexdigest()[:16]

    def _send_heartbeat_ack(self, leader_id: int, term: int, heartbeat_seq: int, heartbeat_digest: str):
        ack_key = (leader_id, heartbeat_seq, heartbeat_digest)
        if self._last_acked_heartbeat_key == ack_key:
            return
        msg = {
            'type': 'heartbeat_ack',
            'robot_id': self.robot_id,
            'leader_id': leader_id,
            'term': term,
            'heartbeat_seq': heartbeat_seq,
            'heartbeat_digest': heartbeat_digest,
            'timestamp': time.time(),
        }
        self.broadcast_message(msg)
        self._last_acked_heartbeat_key = ack_key

    def _handle_heartbeat_ack(self, msg: dict):
        if self.state != RaftState.LEADER or msg.get('leader_id') != self.robot_id:
            return
        if msg.get('term') != self.current_term:
            return
        if msg.get('heartbeat_digest') != self._heartbeat_digest:
            return
        if msg.get('heartbeat_seq') != self._heartbeat_seq:
            return
        ack_robot = msg.get('robot_id')
        if ack_robot == self.robot_id or ack_robot not in self.active_robot_ids:
            return
        ackers = self._heartbeat_ack_sets.setdefault(self._heartbeat_digest, set())
        ackers.add(ack_robot)
        if 1 + len(ackers) >= self._quorum_size():
            self._leader_quorum_mono = time.monotonic()
    
    def _init_sockets(self):
        if self.sim_mode:
            pos_port = UDP_POSITION_PORT + self.robot_id
            peer_port = UDP_PEER_PORT + self.robot_id
            fault_port = UDP_FAULT_PORT + self.robot_id
        else:
            pos_port = UDP_POSITION_PORT
            peer_port = UDP_PEER_PORT
            fault_port = UDP_FAULT_PORT
        
        self.pos_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pos_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.pos_socket.bind(('', pos_port))
        self.pos_socket.setblocking(False)
        print(f"[INFO] Position socket listening on port {pos_port}")
        
        self.peer_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.peer_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.peer_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.peer_socket.bind(('', peer_port))
        self.peer_socket.setblocking(False)
        print(f"[INFO] Peer socket listening on port {peer_port}")
        
        self.fault_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.fault_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.fault_socket.bind(('', fault_port))
        self.fault_socket.setblocking(False)
        print(f"[INFO] Fault socket listening on port {fault_port}")

        self.sim_motor_socket = None
        self.sim_sensor_socket = None
        if self.sim_mode:
            sensor_port = UDP_SIM_SENSOR_PORT + self.robot_id
            self.sim_sensor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sim_sensor_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sim_sensor_socket.bind(('', sensor_port))
            self.sim_sensor_socket.setblocking(False)
            self.sim_motor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sim_motor_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            print(f"[INFO] Sim sensor socket listening on port {sensor_port}")
    
    def init_logging(self):
        os.makedirs("logs", exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.log_path = f"logs/raft_{self.robot_id}_{timestamp}.jsonl"
        self.log_file = open(self.log_path, 'w')
        print(f"  Logging to: {self.log_path}")
    
    def log_state(self):
        if not self.log_file: return
        record = {
            't': time.time(),
            'robot_id': self.robot_id,
            'x': self.x, 'y': self.y, 'theta': self.theta,
            'state': self.state.value,
            'leader_id': self.current_leader,
            'term': self.current_term,
            'raw_navigation_target': list(self._raw_navigation_target) if self._raw_navigation_target else None,
            'navigation_target': list(self.current_navigation_target) if self.current_navigation_target else None,
            'commanded_target': list(self.assigned_target) if self.assigned_target else None,
            'my_visited_count': len(self.my_visited),
            'known_visited_count': len(self.known_visited),
            'fault': self.injected_fault,
            'position_rx_age': time.monotonic() - self.position_rx_mono if self.position_rx_mono > 0 else None,
            'assigned_target_rx_age': time.monotonic() - self.assigned_target_rx_mono if self.assigned_target_rx_mono > 0 else None,
            'target_angle': self._last_target_angle,
            'heading_error': self._last_heading_error,
            'in_pivot': self._in_pivot if hasattr(self, '_in_pivot') else False,
            'stuck_count': getattr(self, '_stuck_count', 0),
            'escape_active': time.monotonic() < getattr(self, '_escape_until', 0.0),
            'stop_reason': self._last_stop_reason,
            'command_source': self._last_command_source,
            'tof': dict(self.tof),
            'motor_cmd': [self._last_motor_cmd[0], self._last_motor_cmd[1]],
        }
        self.log_file.write(json.dumps(record) + '\n')
        self.log_file.flush()
    
    # ==================== POSITION ====================
    def receive_position(self):
        try:
            for _ in range(MAX_RX_MESSAGES_PER_TICK):
                data, _ = self.pos_socket.recvfrom(8192)
                msg = json.loads(data.decode())
                if msg.get('type') == 'position' and msg.get('robot_id') == self.robot_id:
                    POS_EMA = 0.5
                    raw_x, raw_y = msg['x'], msg['y']
                    if not self._pos_initialized:
                        self.x = raw_x
                        self.y = raw_y
                        self._pos_initialized = True
                    else:
                        self.x += POS_EMA * (raw_x - self.x)
                        self.y += POS_EMA * (raw_y - self.y)
                    self.theta = msg['theta']
                    self.position_timestamp = msg.get('timestamp', time.time())
                    self.position_rx_mono = time.monotonic()

                    cell = self.get_cell(self.x, self.y)
                    if cell is not None and not self._is_wall_cell(*cell):
                        self.my_visited.add(cell)
                        self.known_visited.add(cell)
                elif msg.get('type') == 'peer_state':
                    peer_id = msg.get('robot_id')
                    if peer_id and peer_id != self.robot_id:
                        self.handle_peer_state(msg)
        except (BlockingIOError, ConnectionResetError):
            pass
        except OSError:
            pass

    def receive_sim_feedback(self):
        if not self.sim_mode or self.sim_sensor_socket is None:
            return
        try:
            for _ in range(MAX_RX_MESSAGES_PER_TICK):
                data, _ = self.sim_sensor_socket.recvfrom(8192)
                msg = json.loads(data.decode())
                if msg.get('type') != 'sim_sensor' or msg.get('robot_id') != self.robot_id:
                    continue
                self.hw.update_sim_feedback(
                    tof=msg.get('tof') or {},
                    encoders=tuple(msg.get('encoders', (0, 0))),
                )
        except (BlockingIOError, ConnectionResetError):
            pass
        except OSError:
            pass

    def _publish_motor_intent(self, left: float, right: float):
        if not self.sim_mode or self.sim_motor_socket is None:
            return
        msg = {
            'type': 'motor_intent',
            'robot_id': self.robot_id,
            'left': float(left),
            'right': float(right),
            'stop_reason': self._last_stop_reason,
            'command_source': self._last_command_source,
            'timestamp': time.time(),
        }
        try:
            self.sim_motor_socket.sendto(json.dumps(msg).encode(), ('127.0.0.1', UDP_SIM_MOTOR_PORT))
        except OSError:
            pass
    
    def get_cell(self, x: float, y: float) -> Optional[tuple]:
        cx = int(x / CELL_SIZE)
        cy = int(y / CELL_SIZE)
        if 0 <= cx < self.coverage_width and 0 <= cy < self.coverage_height:
            return (cx, cy)
        return None
    
    def cell_to_pos(self, cell: tuple) -> Tuple[float, float]:
        return ((cell[0] + 0.5) * CELL_SIZE, (cell[1] + 0.5) * CELL_SIZE)
    
    def pos_to_cell(self, pos: Tuple[float, float]) -> tuple:
        return (int(pos[0] / CELL_SIZE), int(pos[1] / CELL_SIZE))
    
    def _is_wall_cell(self, cx: int, cy: int) -> bool:
        """True if the cell center is physically unreachable (inside the robot body
        when pressed against a wall).  Keep this tight - the repulsion system
        handles safe navigation; this only excludes truly impossible cells."""
        return DEFAULT_ARENA.is_wall_cell(cx, cy)

    def distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def path_distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Distance accounting for the interior wall."""
        different_sides = (p1[0] < INTERIOR_WALL_X_LEFT) != (p2[0] < INTERIOR_WALL_X_LEFT)
        if different_sides and (p1[1] < INTERIOR_WALL_Y_END or p2[1] < INTERIOR_WALL_Y_END):
            passage_y = (
                int((INTERIOR_WALL_Y_END + DEFAULT_ARENA.divider_tip_clearance_mm) / CELL_SIZE) + 0.5
            ) * CELL_SIZE
            d1_to_passage = math.sqrt((p1[0] - INTERIOR_WALL_X)**2 + (p1[1] - passage_y)**2)
            d2_to_passage = math.sqrt((p2[0] - INTERIOR_WALL_X)**2 + (p2[1] - passage_y)**2)
            return d1_to_passage + d2_to_passage
        return self.distance(p1, p2)
    
    # ==================== RAFT PROTOCOL ====================
    def broadcast_message(self, msg: dict):
        data = json.dumps(msg).encode()
        try:
            if self.sim_mode:
                self.peer_socket.sendto(data, ('127.0.0.1', UDP_SIM_PEER_RELAY_PORT))
            else:
                # Hardware mode: single broadcast, all robots on same port
                self.peer_socket.sendto(data, (BROADCAST_IP, UDP_PEER_PORT))
        except Exception:
            pass
    
    def send_heartbeat(self):
        """Leader sends heartbeats with task assignments"""
        if self.state != RaftState.LEADER:
            return
        
        assignments = self.compute_task_assignments()
        mission_complete = not assignments and self.assigned_target is None
        self._heartbeat_seq += 1
        heartbeat_digest = self._heartbeat_digest_for(
            self.robot_id, self.current_term, self._heartbeat_seq, assignments
        )
        self._heartbeat_digest = heartbeat_digest
        self._heartbeat_ack_sets = {heartbeat_digest: self._heartbeat_ack_sets.get(heartbeat_digest, set())}
        
        msg = {
            'type': 'heartbeat',
            'leader_id': self.robot_id,
            'term': self.current_term,
            'heartbeat_seq': self._heartbeat_seq,
            'heartbeat_digest': heartbeat_digest,
            'mission_complete': mission_complete,
            'assignments': assignments,
            'timestamp': time.time()
        }
        self.broadcast_message(msg)
    
    def send_state(self):
        """Broadcast state to peers"""
        # If we're leader, include assignments in peer_state for experiment harness
        assignments = {}
        mission_complete = False
        if self.state == RaftState.LEADER:
            assignments = self.compute_task_assignments()
            mission_complete = not assignments and self.assigned_target is None
        self._peer_seq += 1
        has_fresh_position = (
            self.position_rx_mono > 0.0 and
            time.monotonic() - self.position_rx_mono < 1.0
        )
        
        msg = {
            'type': 'peer_state',
            'robot_id': self.robot_id,
            'peer_seq': self._peer_seq,
            'x': self.x, 'y': self.y, 'theta': self.theta,
            'state': self.state.value,
            'term': self.current_term,
            'leader_id': self.current_leader,
            'leader_established': self._leader_quorum_ok(),
            'mission_complete': mission_complete,
            'has_fresh_position': has_fresh_position,
            'coverage_count': len(self.my_visited),
            'visited_cells': list(self.my_visited)[-100:],
            'navigation_target': list(self.current_navigation_target) if self.current_navigation_target else None,
            'commanded_target': list(self.assigned_target) if self.assigned_target else None,
            'assignments': assignments,
            'trial_started': self.trial_started,
            'timestamp': time.time()
        }
        self.broadcast_message(msg)
    
    def request_votes(self):
        """Candidate requests votes"""
        msg = {
            'type': 'vote_request',
            'candidate_id': self.robot_id,
            'term': self.current_term,
            'timestamp': time.time()
        }
        self.broadcast_message(msg)
    
    def send_vote(self, candidate_id: int, term: int):
        """Send vote to candidate"""
        msg = {
            'type': 'vote_response',
            'voter_id': self.robot_id,
            'candidate_id': candidate_id,
            'term': term,
            'granted': True,
            'timestamp': time.time()
        }
        self.broadcast_message(msg)
    
    def receive_messages(self):
        """Process incoming messages"""
        try:
            for _ in range(MAX_RX_MESSAGES_PER_TICK):
                data, _ = self.peer_socket.recvfrom(8192)
                msg = json.loads(data.decode())
                self.handle_message(msg)
        except (BlockingIOError, ConnectionResetError):
            pass
    
    def handle_message(self, msg: dict):
        msg_type = msg.get('type')
        
        if msg_type == 'heartbeat':
            self.handle_heartbeat(msg)
        elif msg_type == 'vote_request':
            self.handle_vote_request(msg)
        elif msg_type == 'vote_response':
            self.handle_vote_response(msg)
        elif msg_type == 'peer_state':
            self.handle_peer_state(msg)
        elif msg_type == 'heartbeat_ack':
            self._handle_heartbeat_ack(msg)
    
    def handle_heartbeat(self, msg: dict):
        """Receive heartbeat from leader - reset election timeout."""
        leader_id = msg.get('leader_id')
        term = msg.get('term', 0)
        
        if term >= self.current_term:
            self.current_term = term
            self.current_leader = leader_id
            self.state = RaftState.FOLLOWER
            self.last_heartbeat = time.time()
            self.election_timeout = self._random_election_timeout()
            self.leader_mission_complete = bool(msg.get('mission_complete', False))
            heartbeat_digest = msg.get('heartbeat_digest')
            heartbeat_seq = msg.get('heartbeat_seq')
            if heartbeat_digest and heartbeat_seq is not None:
                self._send_heartbeat_ack(leader_id, term, heartbeat_seq, heartbeat_digest)
            
            assignments = msg.get('assignments', {})
            if str(self.robot_id) in assignments:
                self.assigned_target = tuple(assignments[str(self.robot_id)])
                self.assigned_target_rx_mono = time.monotonic()
            elif self.leader_mission_complete:
                self.assigned_target = None
                self.assigned_target_rx_mono = 0.0
    
    def handle_vote_request(self, msg: dict):
        """Handle vote request from candidate."""
        candidate_id = msg.get('candidate_id')
        term = msg.get('term', 0)
        
        if term > self.current_term:
            self.current_term = term
            self.voted_for = None
            self.state = RaftState.FOLLOWER
            self.last_heartbeat = time.time()
            self.election_timeout = self._random_election_timeout()
        
        if term >= self.current_term and self.voted_for in (None, candidate_id):
            self.voted_for = candidate_id
            self.last_heartbeat = time.time()
            self.election_timeout = self._random_election_timeout()
            self.send_vote(candidate_id, term)
    
    def handle_vote_response(self, msg: dict):
        """Handle vote response -- check for majority."""
        if self.state != RaftState.CANDIDATE:
            return
        
        if msg.get('granted') and msg.get('term') == self.current_term:
            voter = msg.get('voter_id')
            if voter not in self._votes_received:
                self._votes_received.add(voter)
            
            if len(self._votes_received) >= self._quorum_size():
                self.become_leader()
    
    def handle_peer_state(self, msg: dict):
        """Update peer info"""
        peer_id = msg.get('robot_id')
        if peer_id and peer_id != self.robot_id:
            if peer_id not in self.active_robot_ids:
                return
            if msg.get('trial_started') and not self.trial_started:
                self.trial_started = True
                self._trial_start_mono = time.monotonic()
                self.my_visited.clear()
                self.known_visited.clear()
                self.assigned_target_rx_mono = 0.0
                self.current_navigation_target = None
                self._raw_navigation_target = None
                self._last_stop_reason = "trial_started"
                self._last_command_source = "trial_started"
                self._last_motor_cmd = (0.0, 0.0)
                if hasattr(self, '_nav_history'):
                    self._nav_history.clear()
                print(f"[START] Auto-started via peer R{peer_id} broadcast")
            existing = self.peers.get(peer_id, {})
            peer_seq = msg.get('peer_seq')
            if peer_seq is not None:
                try:
                    peer_seq = int(peer_seq)
                except (TypeError, ValueError):
                    peer_seq = None
            if peer_seq is not None and peer_seq <= existing.get('last_peer_seq', -1):
                return
            self.peers[peer_id] = {
                'x': msg.get('x', 0),
                'y': msg.get('y', 0),
                'theta': msg.get('theta', 0),
                'state': msg.get('state', 'follower'),
                'leader_id': msg.get('leader_id'),
                'leader_established': bool(msg.get('leader_established', False)),
                'mission_complete': bool(msg.get('mission_complete', False)),
                'has_fresh_position': bool(msg.get('has_fresh_position', False)),
                'last_seen': time.time(),
                'last_peer_seq': peer_seq if peer_seq is not None else existing.get('last_peer_seq', -1),
                'visited_cells': existing.get('visited_cells', set())
            }
            # Merge coverage.  Apply the same wall mask used when marking our
            # own position, so a peer cannot introduce cells the arena geometry
            # says are unreachable.  Without this the coverage denominator
            # differs between controllers.
            for cell in msg.get('visited_cells', []):
                cell_tuple = tuple(cell)
                if not self._is_wall_cell(*cell_tuple):
                    self.known_visited.add(cell_tuple)
    
    def check_election_timeout(self):
        """Check if we should start election (leader heartbeat timeout)."""
        if self.state == RaftState.LEADER:
            return
        if time.time() - self.last_heartbeat > self.election_timeout:
            self.start_election()
    
    def start_election(self):
        """Start leader election -- increment term, vote for self, request votes."""
        self.state = RaftState.CANDIDATE
        self.current_term += 1
        self.voted_for = self.robot_id
        self._votes_received = {self.robot_id}
        self.election_timeout = self._random_election_timeout()
        self.last_heartbeat = time.time()
        print(f"[RAFT] Starting election, term {self.current_term}")
        self.request_votes()
    
    def become_leader(self):
        """Transition to leader state after winning election."""
        for pid, peer in self.peers.items():
            if (peer.get('state') == 'leader' and pid < self.robot_id and
                time.time() - peer.get('last_seen', 0) < self.peer_timeout):
                self.state = RaftState.FOLLOWER
                self.current_leader = pid
                self.last_heartbeat = time.time()
                self.election_timeout = self._random_election_timeout()
                print(f"[RAFT] Yielding to lower-ID leader R{pid}")
                return
        
        self.state = RaftState.LEADER
        self.current_leader = self.robot_id
        self._leader_quorum_mono = time.monotonic()
        self._leader_claim_mono = time.monotonic()
        self._heartbeat_seq = 0
        self._heartbeat_digest = None
        self._heartbeat_ack_sets.clear()
        self._prev_assignments.clear()
        print(f"[RAFT] Won election, term {self.current_term}")
        self.send_heartbeat()
    
    # ==================== TASK ASSIGNMENT ====================
    def _side_of_divider(self, pos: Tuple[float, float]) -> int:
        """Classify a position as left/right of the divider, or 0 in the throat."""
        x, y = pos
        if y >= INTERIOR_WALL_Y_END:
            return 0
        if x < INTERIOR_WALL_X_LEFT:
            return -1
        if x > INTERIOR_WALL_X_RIGHT:
            return 1
        return 0

    def _is_in_bottleneck_zone(self, pos: Tuple[float, float]) -> bool:
        """True when the robot is in or immediately approaching the narrow passage."""
        x, y = pos
        throat_half_width = ROBOT_RADIUS + CELL_SIZE * 0.5
        return (INTERIOR_WALL_Y_END - CELL_SIZE <= y <= INTERIOR_WALL_Y_END + CELL_SIZE and
                INTERIOR_WALL_X - throat_half_width <= x <= INTERIOR_WALL_X + throat_half_width)

    def _assignment_requires_bottleneck_crossing(self,
                                                 pos: Tuple[float, float],
                                                 target_pos: Tuple[float, float]) -> bool:
        """Return True when reaching the target requires traversing the divider passage."""
        start_side = self._side_of_divider(pos)
        goal_side = self._side_of_divider(target_pos)
        if start_side == 0 or goal_side == 0 or start_side == goal_side:
            return False
        return pos[1] < INTERIOR_WALL_Y_END or target_pos[1] < INTERIOR_WALL_Y_END

    def _active_bottleneck_crossers(self,
                                    robots: Dict[int, Tuple[float, float]],
                                    assignments: Dict[str, Tuple[float, float]]) -> Set[int]:
        """Robots currently occupying or reserving the passage."""
        active = {rid for rid, pos in robots.items() if self._is_in_bottleneck_zone(pos)}
        for rid_str, tgt in assignments.items():
            rid = int(rid_str)
            pos = robots.get(rid)
            if pos and self._assignment_requires_bottleneck_crossing(pos, tgt):
                active.add(rid)
        return active

    def _best_frontier_for_robot(self, pos, theta, robot_cell, available_frontiers,
                                  assigned_frontiers, still_valid, passage_blocked,
                                  crossing_locked, wall_cx):
        """Pick one frontier for this robot: prefer forward, else nearest. Returns (cx,cy) or None."""
        MIN_TARGET_SPACING = CELL_SIZE * 3
        forward_dir = (math.cos(theta), math.sin(theta))
        best_frontier = None
        best_dist = float('inf')

        def score(frontier, require_forward):
            if frontier in assigned_frontiers or (robot_cell and frontier == robot_cell):
                return None
            fp = self.cell_to_pos(frontier)
            if passage_blocked and (frontier[0] < wall_cx) != (pos[0] < INTERIOR_WALL_X_LEFT):
                return None
            if crossing_locked and self._assignment_requires_bottleneck_crossing(pos, fp):
                return None
            for _, existing_tgt in still_valid.items():
                if self.distance(fp, existing_tgt) < MIN_TARGET_SPACING:
                    return None
            dx, dy = fp[0] - pos[0], fp[1] - pos[1]
            if require_forward and (dx * forward_dir[0] + dy * forward_dir[1]) <= 0:
                return None
            return self.path_distance(pos, fp)

        for f in available_frontiers:
            d = score(f, require_forward=True)
            if d is not None and d < best_dist:
                best_dist = d
                best_frontier = f
        if best_frontier is not None:
            return best_frontier
        for f in available_frontiers:
            d = score(f, require_forward=False)
            if d is not None and d < best_dist:
                best_dist = d
                best_frontier = f
        return best_frontier

    def _ensure_room_has_assignee(self, room_frontiers, room_assigned_count,
                                  still_valid, robots):
        """If this room has no assignee, reassign the robot farthest from its target to this room."""
        if room_assigned_count != 0 or not room_frontiers or len(still_valid) < 2:
            return
        worst_rid = None
        worst_dist = -1
        for rid_str, tgt in still_valid.items():
            rid = int(rid_str)
            if rid == self.robot_id:
                continue
            if rid in robots:
                d = self.path_distance(robots[rid], tgt)
                if d > worst_dist:
                    worst_dist = d
                    worst_rid = rid_str
        if worst_rid is not None:
            pos = robots[int(worst_rid)]
            best_f = min(room_frontiers,
                         key=lambda f: self.path_distance(pos, self.cell_to_pos(f)))
            still_valid[worst_rid] = self.cell_to_pos(best_f)

    def compute_task_assignments(self) -> Dict[str, Tuple[float, float]]:
        """Leader assigns frontiers to robots.
        Algorithm: keep previous assignments if target still unexplored; else
        pick nearest frontier (prefer forward); ensure each room has an assignee when passage clear.
        If bad_leader_mode is active, deliberately sends robots to explored cells."""
        if self.state != RaftState.LEADER:
            return {}
        
        # --- Bad-leader fault: send robots to explored cells ---
        if self.bad_leader_mode:
            return self._compute_bad_assignments()

        if self.self_injure_leader_mode:
            return self._compute_self_injure_assignments()
        
        # --- Oscillate-leader fault: flip-flop targets between corners ---
        if self.oscillate_leader_mode:
            return self._compute_oscillate_assignments()
        
        # --- Freeze-leader fault: stop updating assignments ---
        if self.freeze_leader_mode:
            return self._compute_freeze_assignments()
        
        frontiers = []
        frontier_set = set()
        for cx in range(self.coverage_width):
            for cy in range(self.coverage_height):
                if (cx, cy) not in self.known_visited and not self._is_wall_cell(cx, cy):
                    frontiers.append((cx, cy))
                    frontier_set.add((cx, cy))
        
        if not frontiers:
            self._prev_assignments.clear()
            return {}
        
        # Collect all robot positions and headings (me + peers)
        # REIP doesn't check position freshness here - just uses whatever positions it has
        robots = {self.robot_id: (self.x, self.y)}
        robot_thetas = {self.robot_id: self.theta}
        for pid, peer in self.peers.items():
            if time.time() - peer.get('last_seen', 0) < self.peer_timeout:
                robots[pid] = (peer['x'], peer['y'])
                robot_thetas[pid] = peer.get('theta', 0.0)

        MAX_BOTTLENECK_CROSSERS = 1

        still_valid = {}
        assigned = set()
        for rid_str, prev_target in self._prev_assignments.items():
            rid = int(rid_str)
            if rid in robots:
                cell = self.pos_to_cell(prev_target)
                if cell in frontier_set:
                    pos = robots[rid]
                    dist = math.sqrt((pos[0] - prev_target[0])**2 +
                                     (pos[1] - prev_target[1])**2)
                    if dist < CELL_SIZE:
                        if not self._is_wall_cell(*cell):
                            self.known_visited.add(cell)
                        continue
                    still_valid[rid_str] = prev_target
                    assigned.add(cell)

        active_crossers = self._active_bottleneck_crossers(robots, still_valid)

        need_assignment = [rid for rid in robots if str(rid) not in still_valid]
        available_frontiers = [f for f in frontiers if f not in assigned]
        wall_cx = int(INTERIOR_WALL_X_LEFT / CELL_SIZE)

        if need_assignment and available_frontiers:
            def nearest_frontier_dist(rid):
                pos = robots[rid]
                return min(self.path_distance(pos, self.cell_to_pos(f))
                           for f in available_frontiers)
            sorted_robots = sorted(need_assignment, key=nearest_frontier_dist)

            for rid in sorted_robots:
                pos = robots[rid]
                theta = robot_thetas.get(rid, 0.0)
                robot_cell = self.get_cell(pos[0], pos[1])
                already_crossing = rid in active_crossers
                crossing_locked = (len(active_crossers) >= MAX_BOTTLENECK_CROSSERS and
                                   not already_crossing)
                passage_blocked = crossing_locked
                best = self._best_frontier_for_robot(
                    pos, theta, robot_cell, available_frontiers,
                    assigned, still_valid, passage_blocked, crossing_locked, wall_cx)
                if best:
                    assigned.add(best)
                    target_pos = self.cell_to_pos(best)
                    still_valid[str(rid)] = target_pos
                    if self._assignment_requires_bottleneck_crossing(pos, target_pos):
                        active_crossers.add(rid)

        room_a_n = sum(1 for t in still_valid.values() if int(t[0] / CELL_SIZE) < wall_cx)
        room_b_n = len(still_valid) - room_a_n
        room_a_frontiers = [f for f in available_frontiers if f not in assigned and f[0] < wall_cx]
        room_b_frontiers = [f for f in available_frontiers if f not in assigned and f[0] >= wall_cx]

        if len(active_crossers) == 0:
            self._ensure_room_has_assignee(room_b_frontiers, room_b_n, still_valid, robots)
            self._ensure_room_has_assignee(room_a_frontiers, room_a_n, still_valid, robots)

        self._prev_assignments = dict(still_valid)
        return still_valid
    
    def _compute_bad_assignments(self) -> Dict[str, Tuple[float, float]]:
        """BAD LEADER: deliberately assign already-explored cells.
        Simulates a compromised/hallucinating leader that fixates on
        territory it believes needs coverage.  assignments are persisted -
        each robot gets the SAME bad target every broadcast until cleared.
        raft followers have no way to detect this - they just obey."""
        explored = list(self.known_visited)
        if not explored:
            return {}
        
        robots = {self.robot_id: (self.x, self.y)}
        for pid, peer in self.peers.items():
            if time.time() - peer.get('last_seen', 0) < self.peer_timeout:
                robots[pid] = (peer['x'], peer['y'])
        
        # Persist assignments: pick bad targets ONCE, reuse until fault cleared
        if not hasattr(self, '_bad_assignments_cache'):
            self._bad_assignments_cache = {}
        
        assignments = {}
        for rid in robots:
            if str(rid) not in self._bad_assignments_cache:
                bad_cell = random.choice(explored)
                self._bad_assignments_cache[str(rid)] = self.cell_to_pos(bad_cell)
            
            target = self._bad_assignments_cache[str(rid)]
            assignments[str(rid)] = target
            # Leader stores its own bad assignment so compute_motor_command uses it
            if rid == self.robot_id:
                self.assigned_target = target
        
        print(f"[BAD_LEADER] Raft leader sending {len(assignments)} robots "
              f"to EXPLORED cells (persistent, no detection possible)")
        return assignments

    def _compute_self_injure_assignments(self) -> Dict[str, Tuple[float, float]]:
        """Raft leader commands followers into its occupied zone."""
        robots = {self.robot_id: (self.x, self.y)}
        for pid, peer in self.peers.items():
            if time.time() - peer.get('last_seen', 0) < self.peer_timeout:
                robots[pid] = (peer['x'], peer['y'])

        leader_target = self.get_my_frontier() or (self.x, self.y)
        attack_target = (self.x, self.y)
        assignments = {}
        for rid in robots:
            target = leader_target if rid == self.robot_id else attack_target
            assignments[str(rid)] = target
            if rid == self.robot_id:
                self.assigned_target = target

        print(f"[SELF_INJURE] Raft leader attracting {max(0, len(assignments) - 1)} follower(s) "
              f"into occupied zone at ({attack_target[0]:.0f}, {attack_target[1]:.0f})")
        return assignments
    
    def _compute_oscillate_assignments(self) -> Dict[str, Tuple[float, float]]:
        """OSCILLATE LEADER: flip-flop all followers between two far corners.
        Every broadcast cycle, targets alternate between corner A and B.
        raft has no trust model - followers flip-flop forever, coverage stalls."""
        margin = ROBOT_RADIUS + CELL_SIZE
        corner_a = (margin, margin)
        corner_b = (ARENA_WIDTH - margin, ARENA_HEIGHT - margin)
        
        self._oscillate_phase = 1 - self._oscillate_phase
        target = corner_a if self._oscillate_phase == 0 else corner_b
        
        robots = {self.robot_id: (self.x, self.y)}
        for pid, peer in self.peers.items():
            if time.time() - peer.get('last_seen', 0) < self.peer_timeout:
                robots[pid] = (peer['x'], peer['y'])
        
        assignments = {}
        for rid in robots:
            assignments[str(rid)] = target
            if rid == self.robot_id:
                self.assigned_target = target
        
        print(f"[OSCILLATE] Raft phase {self._oscillate_phase}: all {len(assignments)} "
              f"robots -> ({target[0]:.0f}, {target[1]:.0f}) -- NO DETECTION POSSIBLE")
        return assignments
    
    def _compute_freeze_assignments(self) -> Dict[str, Tuple[float, float]]:
        """FREEZE LEADER: stop updating assignments entirely.
        Snapshots current assignments on first call, then returns the same stale
        targets forever.  raft has no trust model - followers obey the stale
        assignments indefinitely, coverage stalls once targets are explored."""
        if not self._frozen_assignments:
            # First call: snapshot current assignments or compute fresh ones
            if self._prev_assignments:
                self._frozen_assignments = dict(self._prev_assignments)
            else:
                # No previous assignments - temporarily disable freeze mode to compute fresh assignments
                # This prevents the bug where everyone gets assigned to leader's starting position
                # and avoids infinite recursion (compute_task_assignments calls _compute_freeze_assignments)
                self.freeze_leader_mode = False
                try:
                    fresh_assignments = self.compute_task_assignments()
                    if fresh_assignments:
                        self._frozen_assignments = fresh_assignments
                    else:
                        # Fallback: assign everyone to leader position (shouldn't happen in normal operation)
                        robots = {self.robot_id: (self.x, self.y)}
                        for pid, peer in self.peers.items():
                            if time.time() - peer.get('last_seen', 0) < self.peer_timeout:
                                robots[pid] = (peer['x'], peer['y'])
                        for rid in robots:
                            self._frozen_assignments[str(rid)] = (self.x, self.y)
                finally:
                    self.freeze_leader_mode = True
            
            print(f"[FREEZE_LEADER] Raft frozen {len(self._frozen_assignments)} assignments "
                  f"(will never update -- NO DETECTION POSSIBLE)")
        
        # Always return the same stale assignments
        my_key = str(self.robot_id)
        if my_key in self._frozen_assignments:
            self.assigned_target = self._frozen_assignments[my_key]
        
        return dict(self._frozen_assignments)
    
    def get_my_frontier(self) -> Optional[Tuple[float, float]]:
        """Find nearest unexplored cell using path-aware distance.
        Crossing the interior wall adds the actual detour through the
        passage so robots explore their own side first."""
        my_cell = self.get_cell(self.x, self.y)
        if not my_cell:
            return None

        wall_cell_x_left = int(INTERIOR_WALL_X_LEFT / CELL_SIZE)
        wall_cell_y_end = int(INTERIOR_WALL_Y_END / CELL_SIZE)
        robot_on_left = my_cell[0] < wall_cell_x_left

        PASSAGE_Y_START = INTERIOR_WALL_Y_END
        PASSAGE_X_MIN = INTERIOR_WALL_X_LEFT - ROBOT_RADIUS
        PASSAGE_X_MAX = INTERIOR_WALL_X_RIGHT + ROBOT_RADIUS
        peer_in_passage = False
        me_in_passage = (self.y > PASSAGE_Y_START and
                         PASSAGE_X_MIN < self.x < PASSAGE_X_MAX)
        if not me_in_passage:
            for p in self.peers.values():
                if (time.time() - p.get('last_seen', 0) < self.peer_timeout and
                    p.get('y', 0) > PASSAGE_Y_START and
                    PASSAGE_X_MIN < p.get('x', 0) < PASSAGE_X_MAX):
                    peer_in_passage = True
                    break

        forward = (math.cos(self.theta), math.sin(self.theta))
        best_dist = float('inf')
        best_cell = None
        best_forward = False

        for cx in range(self.coverage_width):
            for cy in range(self.coverage_height):
                cell = (cx, cy)
                if cell in self.known_visited or self._is_wall_cell(cx, cy):
                    continue
                if cell == my_cell:
                    continue

                cell_on_left = cx < wall_cell_x_left
                if peer_in_passage and cell_on_left != robot_on_left:
                    continue

                dist = abs(cx - my_cell[0]) + abs(cy - my_cell[1])
                if robot_on_left != cell_on_left and cy < wall_cell_y_end:
                    detour_up = max(0, wall_cell_y_end + 2 - my_cell[1])
                    detour_down = max(0, wall_cell_y_end + 2 - cy)
                    dist += detour_up + detour_down + 3

                cell_pos = self.cell_to_pos(cell)
                dx = cell_pos[0] - self.x
                dy = cell_pos[1] - self.y
                is_forward = (dx * forward[0] + dy * forward[1]) > 0
                if (is_forward, -dist) > (best_forward, -best_dist) or best_cell is None:
                    best_dist = dist
                    best_cell = cell
                    best_forward = is_forward

        if best_cell:
            return self.cell_to_pos(best_cell)
        return None
    
    # ==================== NAVIGATION ====================
    def _init_stuck_detection(self):
        self._nav_history = []
        self._escape_until = 0.0
        self._escape_start = 0.0
        self._escape_angle = 0.0
        self._stuck_count = 0
        self._tof_emergency_count = 0

    def _init_heading_pd(self):
        self._smooth_theta = None
        self._prev_heading_err = None
        self._prev_heading_t = 0.0
        self._in_pivot = False
        self._pivot_start_mono = 0.0

    def _finalize_motor_command(self, left: float, right: float,
                                stop_reason: Optional[str] = None) -> Tuple[float, float]:
        if stop_reason in ("escape_reverse", "stuck_escape"):
            pass
        elif stop_reason in ("peer_emergency", "tof_emergency", "escape_pivot"):
            MAX_PWM_STEP = 50.0
            prev_left, prev_right = self._last_motor_cmd
            left = max(prev_left - MAX_PWM_STEP, min(prev_left + MAX_PWM_STEP, left))
            right = max(prev_right - MAX_PWM_STEP, min(prev_right + MAX_PWM_STEP, right))
        else:
            MAX_PWM_STEP = 20.0
            prev_left, prev_right = self._last_motor_cmd
            left = max(prev_left - MAX_PWM_STEP, min(prev_left + MAX_PWM_STEP, left))
            right = max(prev_right - MAX_PWM_STEP, min(prev_right + MAX_PWM_STEP, right))
        if abs(left) < 1e-6:
            left = 0.0
        if abs(right) < 1e-6:
            right = 0.0
        self._last_motor_cmd = (left, right)
        if stop_reason is not None:
            self._last_stop_reason = stop_reason
        return (left, right)

    def _set_motion_override_target(self, angle: float, distance: float = 250.0):
        """Expose reactive motion intent to the sim harness."""
        tx = self.x + distance * math.cos(angle)
        ty = self.y + distance * math.sin(angle)
        tx = max(75, min(ARENA_WIDTH - 75, tx))
        ty = max(75, min(ARENA_HEIGHT - 75, ty))
        self._nav_waypoint_cell = None
        self._nav_waypoint_set_mono = 0.0
        self.current_navigation_target = (tx, ty)

    def _check_stuck(self):
        """Detect if robot is physically stuck (wall-grinding, spinning wheels).
        Skips during pivot turns - the robot intentionally stays in place."""
        if self._in_pivot:
            self._nav_history.clear()
            return False

        now = time.monotonic()

        if self.position_rx_mono == 0.0 or now - self.position_rx_mono > 1.0:
            self._nav_history.clear()
            return False

        self._nav_history.append((now, self.x, self.y))
        self._nav_history = [(t, x, y) for t, x, y in self._nav_history
                             if now - t < 2.5]
        if len(self._nav_history) < 2:
            return False
        oldest_t, oldest_x, oldest_y = self._nav_history[0]
        elapsed = now - oldest_t
        moved = math.sqrt((self.x - oldest_x)**2 + (self.y - oldest_y)**2)
        return elapsed > 0.8 and moved < 15

    def _nearest_navigable_cell(self, cx, cy):
        """BFS from a wall cell to find the closest non-wall cell."""
        from collections import deque
        W, H = self.coverage_width, self.coverage_height
        q = deque([(cx, cy)])
        seen = {(cx, cy)}
        while q:
            x, y = q.popleft()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = x + dx, y + dy
                if 0 <= nx < W and 0 <= ny < H and (nx, ny) not in seen:
                    seen.add((nx, ny))
                    if not self._is_wall_cell(nx, ny):
                        return (nx, ny)
                    q.append((nx, ny))
        return None

    def _segment_is_clear(self, p1, p2):
        """Check whether a straight segment stays in free space."""
        dist = self.distance(p1, p2)
        steps = max(2, int(dist / max(35.0, CELL_SIZE / 3)))
        for i in range(1, steps + 1):
            t = i / steps
            sx = p1[0] + (p2[0] - p1[0]) * t
            sy = p1[1] + (p2[1] - p1[1]) * t
            cell = self.get_cell(sx, sy)
            if cell is None or self._is_wall_cell(*cell):
                return False
        return True

    def _select_path_waypoint(self, path_cells):
        """Choose a stable, farther-ahead waypoint from an A* path."""
        LOOKAHEAD_CELLS = 5
        COMMIT_DIST = CELL_SIZE * 0.55
        COMMIT_TIME = 0.8

        if self.current_navigation_target and self._nav_waypoint_cell in path_cells:
            dist_to_commit = self.distance((self.x, self.y), self.current_navigation_target)
            recently_set = (time.monotonic() - self._nav_waypoint_set_mono) < COMMIT_TIME
            if dist_to_commit > COMMIT_DIST and recently_set:
                return self.current_navigation_target

        best_cell = path_cells[1] if len(path_cells) > 1 else path_cells[0]
        max_idx = min(len(path_cells) - 1, LOOKAHEAD_CELLS)
        origin = (self.x, self.y)
        for idx in range(1, max_idx + 1):
            candidate = path_cells[idx]
            candidate_pos = self.cell_to_pos(candidate)
            if self._segment_is_clear(origin, candidate_pos):
                best_cell = candidate
            else:
                break

        self._nav_waypoint_cell = best_cell
        self._nav_waypoint_set_mono = time.monotonic()
        return self.cell_to_pos(best_cell)

    def _route_around_wall(self, target):
        """A* pathfinding on the cell grid with visible-path lookahead."""
        import heapq

        my_cell = self.get_cell(self.x, self.y)
        tgt_cell = self.get_cell(target[0], target[1])
        if my_cell is None or tgt_cell is None or my_cell == tgt_cell:
            return target

        W = self.coverage_width
        H = self.coverage_height

        if self._is_wall_cell(*my_cell):
            escape = self._nearest_navigable_cell(*my_cell)
            if escape:
                return self.cell_to_pos(escape)
            return target

        if self._is_wall_cell(*tgt_cell):
            snap = self._nearest_navigable_cell(*tgt_cell)
            if snap:
                tgt_cell = snap
                target = self.cell_to_pos(snap)

        def neighbors(cx, cy):
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < W and 0 <= ny < H and not self._is_wall_cell(nx, ny):
                    yield (nx, ny)

        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        start = my_cell
        goal = tgt_cell
        open_set = [(heuristic(start, goal), 0, start)]
        came_from = {start: None}
        g_score = {start: 0}

        while open_set:
            _, g, cur = heapq.heappop(open_set)
            if cur == goal:
                path_cells = []
                node = goal
                while node is not None:
                    path_cells.append(node)
                    node = came_from.get(node)
                path_cells.reverse()
                return self._select_path_waypoint(path_cells)
            if g > g_score.get(cur, float('inf')):
                continue
            for nb in neighbors(*cur):
                ng = g + 1
                if ng < g_score.get(nb, float('inf')):
                    g_score[nb] = ng
                    came_from[nb] = cur
                    heapq.heappush(open_set, (ng + heuristic(nb, goal), ng, nb))

        return target

    def _wall_slide_heading(self, desired_angle):
        """Modify heading to avoid walls.

        The hard clamp (wall-slide) must trigger BEFORE physical contact.
        ROBOT_RADIUS=110mm means the edge touches at 110mm from the wall.
        We clamp heading at ROBOT_RADIUS + 60mm so the robot starts
        sliding along the wall with clearance to spare.  The soft
        repulsion zone is just an early nudge for diagonal approaches.
        """
        hx = math.cos(desired_angle)
        hy = math.sin(desired_angle)

        SLIDE_DIST = ROBOT_RADIUS + 60
        DIV_SLIDE_DIST = BODY_HALF_WIDTH + 60

        def _repel(dist_to_wall, zone):
            if dist_to_wall < zone:
                n = (zone - dist_to_wall) / zone
                return n * n
            return 0.0

        repel_x, repel_y = 0.0, 0.0
        repel_x += _repel(self.x, REPULSION_ZONE)
        repel_x -= _repel(ARENA_WIDTH - self.x, REPULSION_ZONE)
        repel_y += _repel(self.y, REPULSION_ZONE)
        repel_y -= _repel(ARENA_HEIGHT - self.y, REPULSION_ZONE)

        if self.y < INTERIOR_WALL_Y_END:
            d_right = self.x - INTERIOR_WALL_X_RIGHT
            d_left = INTERIOR_WALL_X_LEFT - self.x
            div_zone = REPULSION_ZONE
            if 0 < d_left < div_zone:
                repel_x -= _repel(d_left, div_zone)
            if 0 < d_right < div_zone:
                repel_x += _repel(d_right, div_zone)

        for tip_x in [INTERIOR_WALL_X_LEFT, INTERIOR_WALL_X_RIGHT]:
            tip_dx = self.x - tip_x
            tip_dy = self.y - INTERIOR_WALL_Y_END
            tip_dist = math.sqrt(tip_dx * tip_dx + tip_dy * tip_dy)
            TIP_ZONE = DIV_SLIDE_DIST
            if tip_dist < TIP_ZONE and tip_dist > 5 and self.y < INTERIOR_WALL_Y_END + 50:
                tip_strength = _repel(tip_dist, TIP_ZONE)
                repel_x += (tip_dx / tip_dist) * tip_strength
                repel_y += (tip_dy / tip_dist) * tip_strength

        REPEL_GAIN = 4.0
        hx += repel_x * REPEL_GAIN
        hy += repel_y * REPEL_GAIN

        if self.x < SLIDE_DIST and hx < 0:
            hx = 0
        if self.x > ARENA_WIDTH - SLIDE_DIST and hx > 0:
            hx = 0
        if self.y < SLIDE_DIST and hy < 0:
            hy = 0
        if self.y > ARENA_HEIGHT - SLIDE_DIST and hy > 0:
            hy = 0
        if self.y < INTERIOR_WALL_Y_END:
            d_from_right = self.x - INTERIOR_WALL_X_RIGHT
            d_from_left = INTERIOR_WALL_X_LEFT - self.x
            if 0 < d_from_right < DIV_SLIDE_DIST and hx < 0:
                hx = 0
            elif 0 < d_from_left < DIV_SLIDE_DIST and hx > 0:
                hx = 0

        if abs(hx) < 0.01 and abs(hy) < 0.01:
            cx, cy = ARENA_WIDTH / 2, ARENA_HEIGHT / 2
            return math.atan2(cy - self.y, cx - self.x)

        return math.atan2(hy, hx)

    def _peer_avoidance_heading(self, desired_angle) -> float:
        """Steer away from nearby peers using their broadcast positions."""
        PEER_AVOID_DIST = 400
        PEER_REPEL_GAIN = 4.0

        hx = math.cos(desired_angle)
        hy = math.sin(desired_angle)

        repel_x, repel_y = 0.0, 0.0
        closest_dist = 9999.0
        n_close = 0
        for pid, peer in self.peers.items():
            if time.time() - peer.get('last_seen', 0) > self.peer_timeout:
                continue
            dx = self.x - peer['x']
            dy = self.y - peer['y']
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < closest_dist:
                closest_dist = dist
            if dist < PEER_AVOID_DIST and dist > 5:
                n_close += 1
                strength = (PEER_AVOID_DIST - dist) / PEER_AVOID_DIST
                if dist < 2 * ROBOT_RADIUS:
                    strength *= 6.0
                elif dist < 2 * ROBOT_RADIUS + 100:
                    strength *= 3.0
                repel_x += (dx / dist) * strength
                repel_y += (dy / dist) * strength

        if n_close >= 2 and (abs(repel_x) + abs(repel_y)) < 0.5:
            offset_angle = (self.robot_id * 1.2566)
            repel_x += math.cos(offset_angle) * 0.8
            repel_y += math.sin(offset_angle) * 0.8

        if abs(repel_x) < 0.01 and abs(repel_y) < 0.01:
            return desired_angle

        if closest_dist < 2 * ROBOT_RADIUS:
            return math.atan2(repel_y, repel_x)

        hx += repel_x * PEER_REPEL_GAIN
        hy += repel_y * PEER_REPEL_GAIN
        return math.atan2(hy, hx)

    # ==================== MOTOR CONTROL ====================
    def compute_motor_command(self) -> Tuple[float, float]:
        if not hasattr(self, '_nav_history'):
            self._init_stuck_detection()
        if not hasattr(self, '_smooth_theta'):
            self._init_heading_pd()

        # Fault injection
        if self.injected_fault == 'spin':
            self._last_command_source = "fault_spin"
            return self._finalize_motor_command(BASE_SPEED, -BASE_SPEED, "fault_spin")
        elif self.injected_fault == 'stop':
            self._last_command_source = "fault_stop"
            return self._finalize_motor_command(0.0, 0.0, "fault_stop")
        elif self.injected_fault == 'erratic':
            self._last_command_source = "fault_erratic"
            return self._finalize_motor_command(
                random.uniform(-BASE_SPEED, BASE_SPEED),
                random.uniform(-BASE_SPEED, BASE_SPEED),
                "fault_erratic"
            )

        # Escape mode: pivot toward escape angle, then drive away.
        # When in a wall zone, skip blind reverse - reversing pushes
        # the robot deeper into the wall it's already stuck against.
        if time.monotonic() < self._escape_until:
            self._last_command_source = "escape_mode"
            phase_elapsed = time.monotonic() - self._escape_start
            escape_angle = getattr(self, '_escape_angle', math.atan2(
                ARENA_HEIGHT / 2 - self.y, ARENA_WIDTH / 2 - self.x))
            self._set_motion_override_target(escape_angle, 300.0)

            in_wall_zone = (
                self.x < OUTER_WALL_MARGIN + 50 or
                self.x > ARENA_WIDTH - OUTER_WALL_MARGIN - 50 or
                self.y < OUTER_WALL_MARGIN + 50 or
                self.y > ARENA_HEIGHT - OUTER_WALL_MARGIN - 50 or
                (self.y < INTERIOR_WALL_Y_END and
                 abs(self.x - INTERIOR_WALL_X) < DIVIDER_MARGIN + 80)
            )

            reverse_dur = 0.0 if in_wall_zone else 0.55
            pivot_end = reverse_dur + 0.5

            if phase_elapsed < reverse_dur:
                return self._finalize_motor_command(-BASE_SPEED * 0.75, -BASE_SPEED * 0.75, "escape_reverse")
            elif phase_elapsed < pivot_end:
                diff = escape_angle - self.theta
                while diff > math.pi: diff -= 2 * math.pi
                while diff < -math.pi: diff += 2 * math.pi
                if abs(diff) < 0.25:
                    return self._finalize_motor_command(BASE_SPEED * 0.6, BASE_SPEED * 0.6, "escape_drive")
                # Sharp arc (both wheels same sign): actual rotation. L,-R causes teetering on hardware.
                pivot_slow = max(35, BASE_SPEED * 0.28)
                if diff > 0:
                    return self._finalize_motor_command(pivot_slow, BASE_SPEED, "escape_pivot")
                return self._finalize_motor_command(BASE_SPEED, pivot_slow, "escape_pivot")
            else:
                safe_angle = self._wall_slide_heading(escape_angle)
                diff = safe_angle - self.theta
                while diff > math.pi: diff -= 2 * math.pi
                while diff < -math.pi: diff += 2 * math.pi
                turn = max(-1, min(1, diff / (math.pi / 4)))
                left = BASE_SPEED * (1 - turn * 0.5)
                right = BASE_SPEED * (1 + turn * 0.5)
                self._stuck_count = max(0, self._stuck_count - 1)
                return self._finalize_motor_command(left, right, "escape_drive")

        # ToF: front-facing sensors. When critically close, pivot in place away (sharp arc).
        # Immediate pivot prevents wall-grinding; no waiting for 3 frames.
        front_d = self.tof.get('front', 9999)
        fl_d = self.tof.get('front_left', 9999)
        fr_d = self.tof.get('front_right', 9999)
        left_d = self.tof.get('left', 9999)
        right_d = self.tof.get('right', 9999)
        if front_d <= 0: front_d = 9999
        if fl_d <= 0: fl_d = 9999
        if fr_d <= 0: fr_d = 9999
        if left_d <= 0: left_d = 9999
        if right_d <= 0: right_d = 9999
        min_front = min(front_d, fl_d, fr_d)

        if 20 < min_front < CRITICAL_DISTANCE:
            turn_away = 1 if (left_d > right_d) else -1
            pivot_slow = max(35, BASE_SPEED * 0.28)
            self._last_command_source = "tof_emergency"
            if turn_away > 0:
                return self._finalize_motor_command(pivot_slow, BASE_SPEED, "tof_emergency")
            return self._finalize_motor_command(BASE_SPEED, pivot_slow, "tof_emergency")

        TOF_EMERGENCY_DIST = 120
        if min_front > 20 and min_front < TOF_EMERGENCY_DIST:
            self._tof_emergency_count += 2
            if self._tof_emergency_count >= 4:
                self._tof_emergency_count = 0
                self._stuck_count += 1
                escape_time = min(2.0 + self._stuck_count * 0.5, 4.0)
                self._escape_until = time.monotonic() + escape_time
                self._escape_start = time.monotonic()
                self._nav_history.clear()
                flee_x, flee_y = 0.0, 0.0
                if self.x < 300: flee_x += 1.0
                if ARENA_WIDTH - self.x < 300: flee_x -= 1.0
                if self.y < 300: flee_y += 1.0
                if ARENA_HEIGHT - self.y < 300: flee_y -= 1.0
                if self.y < INTERIOR_WALL_Y_END:
                    if 0 < INTERIOR_WALL_X_LEFT - self.x < 300:
                        flee_x -= 1.0
                    if 0 < self.x - INTERIOR_WALL_X_RIGHT < 300:
                        flee_x += 1.0
                if abs(flee_x) < 0.01 and abs(flee_y) < 0.01:
                    flee_x = math.cos(self.robot_id * 1.2566)
                    flee_y = math.sin(self.robot_id * 1.2566)
                self._escape_angle = math.atan2(flee_y, flee_x)
                self._last_command_source = "tof_emergency"
                return self._finalize_motor_command(-BASE_SPEED * 0.7, -BASE_SPEED * 0.7, "tof_emergency")
        else:
            self._tof_emergency_count = max(0, self._tof_emergency_count - 1)

        PEER_HARD_CONTACT_DIST = 2 * ROBOT_RADIUS + 20
        PEER_FORWARD_CONTACT_DIST = 2 * ROBOT_RADIUS + 50
        _closest_peer_dist = 9999.0
        _flee_px, _flee_py = 0.0, 0.0
        _emergency_peer_count = 0
        heading_x = math.cos(self.theta)
        heading_y = math.sin(self.theta)
        for pid, peer in self.peers.items():
            if time.time() - peer.get('last_seen', 0) > self.peer_timeout:
                continue
            pdx = self.x - peer['x']
            pdy = self.y - peer['y']
            pdist = math.sqrt(pdx * pdx + pdy * pdy)
            if pdist < _closest_peer_dist:
                _closest_peer_dist = pdist
            if pdist <= 5:
                continue
            peer_ahead_proj = ((peer['x'] - self.x) * heading_x + (peer['y'] - self.y) * heading_y) / pdist
            in_forward_corridor = peer_ahead_proj > 0.45
            should_emergency = (
                pdist < PEER_HARD_CONTACT_DIST or
                (pdist < PEER_FORWARD_CONTACT_DIST and in_forward_corridor)
            )
            if should_emergency:
                _emergency_peer_count += 1
                _flee_px += pdx / pdist
                _flee_py += pdy / pdist
        if _emergency_peer_count > 0 and (abs(_flee_px) > 0.01 or abs(_flee_py) > 0.01):
            self._last_command_source = "peer_emergency"
            flee_angle = math.atan2(_flee_py, _flee_px)
            self._set_motion_override_target(flee_angle, 220.0)
            diff = flee_angle - self.theta
            while diff > math.pi: diff -= 2 * math.pi
            while diff < -math.pi: diff += 2 * math.pi
            if abs(diff) > math.pi * 0.3:
                pivot_slow = max(35, BASE_SPEED * 0.28)
                if diff > 0:
                    return self._finalize_motor_command(pivot_slow, BASE_SPEED, "peer_emergency")
                else:
                    return self._finalize_motor_command(BASE_SPEED, pivot_slow, "peer_emergency")
            else:
                return self._finalize_motor_command(BASE_SPEED * 0.5, BASE_SPEED * 0.5, "peer_emergency")

        target = None
        if self.state == RaftState.LEADER:
            if self.bad_leader_mode or self.oscillate_leader_mode or self.freeze_leader_mode:
                target = self.assigned_target if self.assigned_target else self.get_my_frontier()
            else:
                target = self.get_my_frontier()
            self._last_command_source = "leader_frontier"
        elif self.assigned_target:
            # Check if assignment is stale (like REIP does)
            assignment_age = time.monotonic() - self.assigned_target_rx_mono if self.assigned_target_rx_mono > 0 else float('inf')
            ASSIGNMENT_STALE_TIME = 5.0  # Must survive WiFi drops (leader broadcasts at 5Hz)
            if assignment_age < ASSIGNMENT_STALE_TIME:
                target = self.assigned_target  # Follow assignment
                self._last_command_source = "leader_assigned"
            else:
                # Assignment is stale - fall back to autonomous exploration
                target = self.get_my_frontier()
                self._last_command_source = "predicted_frontier"
                self.assigned_target = None  # Clear stale assignment
        elif self._trusted_leader_reports_complete():
            target = None
            self._last_command_source = "leader_complete"
        else:
            target = self.get_my_frontier()
            self._last_command_source = "predicted_frontier"

        if not target:
            self.current_navigation_target = None
            self._nav_waypoint_cell = None
            self._nav_waypoint_set_mono = 0.0
            self._raw_navigation_target = None
            return self._finalize_motor_command(0.0, 0.0, "no_target")

        ARRIVAL_RADIUS = CELL_SIZE
        dist_to_target = math.sqrt((self.x - target[0])**2 + (self.y - target[1])**2)
        if dist_to_target < ARRIVAL_RADIUS:
            cell = self.get_cell(target[0], target[1])
            now = time.monotonic()
            if cell and cell not in self.my_visited:
                self.my_visited.add(cell)
            if cell and cell not in self.known_visited:
                self.known_visited.add(cell)
            self._stuck_count = 0
            if self.state == RaftState.LEADER:
                self.assigned_target = None

        # Clamp target outside the wall-slide zone so the robot never
        # aims at a point it will be prevented from reaching.
        SLIDE_DIST = ROBOT_RADIUS + 60
        DIV_SLIDE_DIST = BODY_HALF_WIDTH + 60
        tx = max(SLIDE_DIST, min(ARENA_WIDTH - SLIDE_DIST, target[0]))
        ty = max(SLIDE_DIST, min(ARENA_HEIGHT - SLIDE_DIST, target[1]))
        if ty < INTERIOR_WALL_Y_END and (
                INTERIOR_WALL_X_LEFT - DIV_SLIDE_DIST < tx < INTERIOR_WALL_X_RIGHT + DIV_SLIDE_DIST):
            if tx < INTERIOR_WALL_X:
                tx = INTERIOR_WALL_X_LEFT - DIV_SLIDE_DIST
            else:
                tx = INTERIOR_WALL_X_RIGHT + DIV_SLIDE_DIST
        target = (tx, ty)
        self._raw_navigation_target = target

        # Route around interior wall
        target = self._route_around_wall(target)
        self.current_navigation_target = target

        if (self.x > OUTER_WALL_MARGIN + 50 and
            self.x < ARENA_WIDTH - OUTER_WALL_MARGIN - 50 and
            self.y > OUTER_WALL_MARGIN + 50 and
            self.y < ARENA_HEIGHT - OUTER_WALL_MARGIN - 50):
            if self._stuck_count > 0:
                self._stuck_count = max(0, self._stuck_count - 1)

        # Stuck detection -> escape: pivot away, then drive to open space
        if self._check_stuck():
            self._stuck_count += 1
            escape_time = min(1.5 + self._stuck_count * 0.3, 3.0)
            self._escape_until = time.monotonic() + escape_time
            self._escape_start = time.monotonic()
            self._nav_history.clear()

            flee_x, flee_y = 0.0, 0.0
            for pid, peer in self.peers.items():
                if time.time() - peer.get('last_seen', 0) > self.peer_timeout:
                    continue
                d = math.sqrt((self.x - peer['x'])**2 + (self.y - peer['y'])**2)
                if d < 400 and d > 5:
                    flee_x += (self.x - peer['x']) / d
                    flee_y += (self.y - peer['y']) / d
            flee_range = 300.0
            if self.x < flee_range:
                flee_x += (flee_range - self.x) / flee_range
            if ARENA_WIDTH - self.x < flee_range:
                flee_x -= (flee_range - (ARENA_WIDTH - self.x)) / flee_range
            if self.y < flee_range:
                flee_y += (flee_range - self.y) / flee_range
            if ARENA_HEIGHT - self.y < flee_range:
                flee_y -= (flee_range - (ARENA_HEIGHT - self.y)) / flee_range
            if self.y < INTERIOR_WALL_Y_END:
                d_to_left = abs(self.x - INTERIOR_WALL_X_LEFT)
                d_to_right = abs(self.x - INTERIOR_WALL_X_RIGHT)
                if d_to_left < flee_range:
                    flee_x += -(flee_range - d_to_left) / flee_range if self.x > INTERIOR_WALL_X_LEFT else (flee_range - d_to_left) / flee_range
                if d_to_right < flee_range:
                    flee_x += (flee_range - d_to_right) / flee_range if self.x > INTERIOR_WALL_X_RIGHT else -(flee_range - d_to_right) / flee_range

            if abs(flee_x) < 0.01 and abs(flee_y) < 0.01:
                flee_x = math.cos(self.robot_id * 1.2566)
                flee_y = math.sin(self.robot_id * 1.2566)

            self._escape_angle = math.atan2(flee_y, flee_x)
            self._last_command_source = "stuck_escape"
            self._set_motion_override_target(self._escape_angle, 300.0)
            return self._finalize_motor_command(-BASE_SPEED * 0.6, -BASE_SPEED * 0.6, "stuck_escape")

        # Navigate to target
        dx = target[0] - self.x
        dy = target[1] - self.y
        target_angle = math.atan2(dy, dx)

        # Wall-sliding and peer avoidance
        target_angle = self._wall_slide_heading(target_angle)
        target_angle = self._peer_avoidance_heading(target_angle)
        self._last_target_angle = target_angle

        EMA_ALPHA = 0.25
        if self._smooth_theta is None:
            self._smooth_theta = self.theta
        else:
            delta = self.theta - self._smooth_theta
            while delta > math.pi: delta -= 2 * math.pi
            while delta < -math.pi: delta += 2 * math.pi
            self._smooth_theta += EMA_ALPHA * delta
            while self._smooth_theta > math.pi: self._smooth_theta -= 2 * math.pi
            while self._smooth_theta < -math.pi: self._smooth_theta += 2 * math.pi

        diff = target_angle - self._smooth_theta
        while diff > math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi

        KP = 1.0 / (math.pi / 2)
        HEADING_DEADBAND = 0.17  # ~10 deg - reject ArUco jitter
        if abs(diff) < HEADING_DEADBAND:
            diff = 0.0
        self._last_heading_error = diff

        turn = max(-1, min(1, KP * diff))

        # ToF-based reactive turn bias (includes front sensor)
        TOF_BIAS_DIST = 300
        fl = self.tof.get('front_left', 9999)
        fr = self.tof.get('front_right', 9999)
        front_tof = self.tof.get('front', 9999)
        left_tof = self.tof.get('left', 9999)
        right_tof = self.tof.get('right', 9999)
        if fl <= 0: fl = 9999
        if fr <= 0: fr = 9999
        if front_tof <= 0: front_tof = 9999
        if left_tof <= 0: left_tof = 9999
        if right_tof <= 0: right_tof = 9999
        tof_bias = 0.0
        if fl < TOF_BIAS_DIST and fl > 20:
            tof_bias += 0.15 * (TOF_BIAS_DIST - fl) / TOF_BIAS_DIST
        if left_tof < TOF_BIAS_DIST and left_tof > 20:
            tof_bias += 0.08 * (TOF_BIAS_DIST - left_tof) / TOF_BIAS_DIST
        if fr < TOF_BIAS_DIST and fr > 20:
            tof_bias -= 0.15 * (TOF_BIAS_DIST - fr) / TOF_BIAS_DIST
        if right_tof < TOF_BIAS_DIST and right_tof > 20:
            tof_bias -= 0.08 * (TOF_BIAS_DIST - right_tof) / TOF_BIAS_DIST
        if front_tof < TOF_BIAS_DIST and front_tof > 20:
            steer_sign = 1.0 if (fl > fr or left_tof > right_tof) else -1.0
            tof_bias += steer_sign * 0.2 * (TOF_BIAS_DIST - front_tof) / TOF_BIAS_DIST
        turn = max(-1, min(1, turn + tof_bias))

        SPEED_TAPER_ZONE = 250
        wall_dists = [self.x, ARENA_WIDTH - self.x,
                      self.y, ARENA_HEIGHT - self.y]
        if self.y < INTERIOR_WALL_Y_END:
            if self.x < INTERIOR_WALL_X_LEFT:
                wall_dists.append(INTERIOR_WALL_X_LEFT - self.x)
            elif self.x > INTERIOR_WALL_X_RIGHT:
                wall_dists.append(self.x - INTERIOR_WALL_X_RIGHT)
        nearest_wall = min(wall_dists)
        sf_wall = 1.0
        if nearest_wall < SPEED_TAPER_ZONE:
            sf_wall = 0.45 + 0.55 * (nearest_wall / SPEED_TAPER_ZONE)

        min_front_tof = min(front_d, fl_d, fr_d)
        sf_tof = 1.0
        if 20 < min_front_tof < 250:
            sf_tof = max(0.5, min_front_tof / 250)

        in_tight_space = (
            self._is_in_bottleneck_zone((self.x, self.y)) or
            self.x < 260 or self.x > ARENA_WIDTH - 260 or
            self.y < 260 or self.y > ARENA_HEIGHT - 260
        )
        if (20 < min_front_tof < (CRITICAL_DISTANCE + 18) and in_tight_space) or (
            min(fl, fr) < (CRITICAL_DISTANCE + 15) and min(left_tof, right_tof) < 260
        ):
            sf_tof = min(sf_tof, max(0.22, min_front_tof / 250))

        min_peer_dist = 9999.0
        min_peer_ahead_dist = 9999.0
        heading_x = math.cos(self._smooth_theta)
        heading_y = math.sin(self._smooth_theta)
        for pid, peer in self.peers.items():
            if time.time() - peer.get('last_seen', 0) > self.peer_timeout:
                continue
            rel_x = peer['x'] - self.x
            rel_y = peer['y'] - self.y
            d = math.sqrt(rel_x**2 + rel_y**2)
            min_peer_dist = min(min_peer_dist, d)
            if d > 5:
                forward_proj = (rel_x * heading_x + rel_y * heading_y) / d
                lateral_sep = abs(-rel_x * heading_y + rel_y * heading_x)
                if forward_proj > 0.35 and lateral_sep < CELL_SIZE * 1.35:
                    min_peer_ahead_dist = min(min_peer_ahead_dist, d)

        PEER_CONTACT_DIST = 2 * ROBOT_RADIUS + 60
        sf_peer = 1.0
        if min_peer_dist < PEER_CONTACT_DIST:
            sf_peer = 0.10
        elif min_peer_dist < 400:
            sf_peer = 0.3 + 0.7 * (min_peer_dist / 400)

        sf_peer_yield = 1.0
        if min_peer_ahead_dist < 420:
            sf_peer_yield = max(0.05, (min_peer_ahead_dist - PEER_CONTACT_DIST) / (420 - PEER_CONTACT_DIST))

        MIN_MOTOR_PWM = 25
        hazard_scale = min(sf_wall, sf_tof, sf_peer, sf_peer_yield)
        effective_speed = BASE_SPEED * hazard_scale
        if hazard_scale >= 0.22:
            effective_speed = max(MIN_MOTOR_PWM + 5, effective_speed)

        turn_mix = 0.55

        # Pivot (sharp arc) for turns > ~63 deg so we don't command one wheel near zero (teetering).
        PIVOT_ENTER = math.pi * 0.35
        PIVOT_EXIT  = math.pi * 0.19   # ~35 deg
        if abs(diff) > PIVOT_ENTER and not self._in_pivot:
            self._in_pivot = True
            self._pivot_start_mono = time.monotonic()
        if abs(diff) < PIVOT_EXIT:
            self._in_pivot = False
        if self._in_pivot and self._pivot_start_mono > 0 and time.monotonic() - self._pivot_start_mono > 1.0:
            self._in_pivot = False
            self._escape_start = time.monotonic()
            self._escape_until = self._escape_start + 0.8
            self._escape_angle = target_angle
            self._set_motion_override_target(target_angle, 250.0)
            return self._finalize_motor_command(-BASE_SPEED * 0.4, -BASE_SPEED * 0.4, "pivot_timeout_escape")
        if self._in_pivot:
            pivot_fast = BASE_SPEED
            pivot_slow = max(MIN_MOTOR_PWM + 10, BASE_SPEED * 0.28)
            self._set_motion_override_target(target_angle, 200.0)
            if diff > 0:
                left_speed, right_speed = pivot_slow, pivot_fast
            else:
                left_speed, right_speed = pivot_fast, pivot_slow
            self._last_turn = turn
            return self._finalize_motor_command(left_speed, right_speed, "pivot_turn")

        left_speed = effective_speed * (1 - turn * turn_mix)
        right_speed = effective_speed * (1 + turn * turn_mix)
        if abs(turn) > 0.1:
            if abs(left_speed) > 0 and abs(left_speed) < MIN_MOTOR_PWM:
                left_speed = math.copysign(MIN_MOTOR_PWM, left_speed)
            if abs(right_speed) > 0 and abs(right_speed) < MIN_MOTOR_PWM:
                right_speed = math.copysign(MIN_MOTOR_PWM, right_speed)

        self._last_turn = turn
        return self._finalize_motor_command(left_speed, right_speed, "tracking")
    
    # ==================== FAULT INJECTION ====================
    def receive_fault_injection(self):
        try:
            for _ in range(MAX_RX_MESSAGES_PER_TICK):
                data, _ = self.fault_socket.recvfrom(1024)
                msg = json.loads(data.decode())
                target = msg.get('robot_id')
                if target == self.robot_id or target == 'all' or target == 0:
                    fault = msg.get('fault', msg.get('cmd', 'none'))
                    if fault in ('set_trial_robots', 'start'):
                        active_robot_ids = msg.get('active_robot_ids')
                        if active_robot_ids:
                            try:
                                parsed_ids = {int(rid) for rid in active_robot_ids}
                            except (TypeError, ValueError):
                                parsed_ids = set()
                            if parsed_ids:
                                parsed_ids.add(self.robot_id)
                                self.active_robot_ids = parsed_ids
                                self.active_robot_count = len(parsed_ids)
                    if fault == 'start':
                        if self.trial_started:
                            continue
                        self.trial_started = True
                        self._trial_start_mono = time.monotonic()
                        self.my_visited.clear()
                        self.known_visited.clear()
                        self.assigned_target_rx_mono = 0.0
                        self.current_navigation_target = None
                        self._nav_waypoint_cell = None
                        self._nav_waypoint_set_mono = 0.0
                        self._raw_navigation_target = None
                        self._escape_until = 0.0
                        self._escape_start = 0.0
                        self._stuck_count = 0
                        self._last_stop_reason = "trial_started"
                        self._last_command_source = "trial_started"
                        self._last_motor_cmd = (0.0, 0.0)
                        if hasattr(self, '_nav_history'):
                            self._nav_history.clear()
                        for p in self.peers.values():
                            p['visited_cells'] = set()
                        print(f"[START] Trial started - coverage reset, motors engaged")
                        continue
                    elif fault in ('none', 'clear'):
                        self.injected_fault = None
                        self.bad_leader_mode = False
                        self.self_injure_leader_mode = False
                        self.oscillate_leader_mode = False
                        self.freeze_leader_mode = False
                        self._frozen_assignments.clear()
                        if hasattr(self, '_bad_assignments_cache'):
                            self._bad_assignments_cache.clear()
                        print(f"[FAULT] Cleared")
                    elif fault == 'bad_leader':
                        # Leadership fault: send robots to explored cells.
                        # Raft has NO trust model, so it will follow blindly.
                        self.bad_leader_mode = True
                        self.self_injure_leader_mode = False
                        self.oscillate_leader_mode = False
                        self.freeze_leader_mode = False
                        self.injected_fault = None
                        print(f"[FAULT] BAD_LEADER mode - will send bad assignments (Raft CANNOT detect this)")
                    elif fault in ('self_injure_leader', 'peer_collision_leader', 'ram_peer'):
                        self.self_injure_leader_mode = True
                        self.bad_leader_mode = False
                        self.oscillate_leader_mode = False
                        self.freeze_leader_mode = False
                        self.injected_fault = None
                        print(f"[FAULT] SELF_INJURE_LEADER mode - will command followers into occupied peer zone (Raft CANNOT detect this)")
                    elif fault == 'oscillate_leader':
                        # Oscillation fault: flip-flop targets.
                        # Raft has NO trust model, so followers oscillate forever.
                        self.oscillate_leader_mode = True
                        self.bad_leader_mode = False
                        self.self_injure_leader_mode = False
                        self.freeze_leader_mode = False
                        self.injected_fault = None
                        self._oscillate_phase = 0
                        print(f"[FAULT] OSCILLATE_LEADER mode - will flip-flop targets (Raft CANNOT detect this)")
                    elif fault == 'freeze_leader':
                        # Freeze fault: leader stops updating assignments.
                        # Raft has NO trust model, so followers stall indefinitely.
                        self.freeze_leader_mode = True
                        self.bad_leader_mode = False
                        self.self_injure_leader_mode = False
                        self.oscillate_leader_mode = False
                        self.injected_fault = None
                        self._frozen_assignments.clear()  # Will be populated on first call
                        print(f"[FAULT] FREEZE_LEADER mode - will stop updating (Raft CANNOT detect this)")
                    else:
                        self.injected_fault = fault
                        self.bad_leader_mode = False
                        self.self_injure_leader_mode = False
                        self.oscillate_leader_mode = False
                        self.freeze_leader_mode = False
                        print(f"[FAULT] Injected: {fault}")
        except (BlockingIOError, ConnectionResetError):
            pass
    
    # ==================== MAIN LOOPS ====================
    def sensor_loop(self):
        while self.running:
            self.tof = self.hw.read_tof_all()
            # encoder reads removed - positions come from camera,
            # and UART contention with motor commands trips Pico watchdog.
            time.sleep(0.02)
    
    def network_loop(self):
        last_broadcast = 0
        last_heartbeat = 0
        
        while self.running:
            self.receive_position()
            self.receive_sim_feedback()
            self.receive_fault_injection()
            self.receive_messages()
            
            now = time.time()
            now_mono = time.monotonic()
            
            # State broadcast
            if now - last_broadcast > 1.0 / BROADCAST_RATE:
                self.send_state()
                last_broadcast = now
            
            # Raft election timeout check
            self.check_election_timeout()

            # Leader heartbeat
            if self.state == RaftState.LEADER and now - last_heartbeat > HEARTBEAT_INTERVAL:
                self.send_heartbeat()
                last_heartbeat = now
            
            # Prune peers
            dead = [p for p, d in self.peers.items() if now - d.get('last_seen', 0) > self.peer_timeout]
            for p in dead:
                del self.peers[p]
            
            time.sleep(0.01)
    
    def control_loop(self):
        interval = 1.0 / CONTROL_RATE
        last_print = 0
        last_log = 0
        printed_waiting = False
        
        while self.running:
            start = time.time()

            if not self.trial_started and not printed_waiting:
                print(f"[R{self.robot_id}] Waiting for 'start' command...")
                printed_waiting = True

            if (self.trial_started and self._trial_start_mono > 0 and
                time.monotonic() - self._trial_start_mono > self._trial_max_duration):
                print(f"[R{self.robot_id}] Trial duration exceeded {self._trial_max_duration}s, auto-stopping")
                self.hw.stop()
                self.running = False
                break

            left, right = 0.0, 0.0
            pos_age = time.monotonic() - self.position_rx_mono if self.position_rx_mono > 0 else float('inf')
            if self.trial_started and self.position_rx_mono > 0 and pos_age < 1.5:
                left, right = self.compute_motor_command()
                self.hw.set_motors(left, right)
            else:
                if not self.trial_started:
                    self._last_stop_reason = "trial_not_started"
                elif self.position_rx_mono == 0:
                    self._last_stop_reason = "no_position"
                else:
                    self._last_stop_reason = "stale_position"
                self._last_motor_cmd = (0.0, 0.0)
                self.hw.stop()
            self._publish_motor_intent(left, right)
            
            if time.time() - last_log > 0.2:
                self.log_state()
                last_log = time.time()
            
            if time.time() - last_print > 2.0:
                cov = len(self.my_visited) / (self.coverage_width * self.coverage_height) * 100
                total = len(self.known_visited) / (self.coverage_width * self.coverage_height) * 100
                print(f"[R{self.robot_id}] state={self.state.value} "
                      f"leader={self.current_leader} term={self.current_term} "
                      f"cov={cov:.1f}%/{total:.1f}%")
                last_print = time.time()
            
            elapsed = time.time() - start
            if elapsed < interval:
                time.sleep(interval - elapsed)
    
    def run(self):
        self.running = True
        
        sensor_thread = threading.Thread(target=self.sensor_loop, daemon=True)
        network_thread = threading.Thread(target=self.network_loop, daemon=True)
        
        sensor_thread.start()
        network_thread.start()
        
        print("Running RAFT baseline... Press Ctrl+C to stop\n")
        
        try:
            self.control_loop()
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.running = False
            self.hw.stop()
            if self.log_file:
                self.log_file.close()
            print("Stopped.")

# ============== Entry ==============
if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python3 raft_node.py <robot_id> [--sim] [--port-base N]")
        sys.exit(1)
    
    robot_id = int(sys.argv[1])
    sim_mode = "--sim" in sys.argv
    
    # Parse --port-base for parallel experiment support
    if "--port-base" in sys.argv:
        idx = sys.argv.index("--port-base")
        port_offset = int(sys.argv[idx + 1])
        UDP_POSITION_PORT += port_offset
        UDP_PEER_PORT += port_offset
        UDP_FAULT_PORT += port_offset
        UDP_SIM_PEER_RELAY_PORT += port_offset  # Must match harness relay port
        UDP_SIM_MOTOR_PORT += port_offset       # Must match harness motor port
        UDP_SIM_SENSOR_PORT += port_offset      # Must match harness sensor port
    
    # Parse --arena-width / --arena-height for scaled layouts
    if "--arena-width" in sys.argv:
        idx = sys.argv.index("--arena-width")
        ARENA_WIDTH = int(sys.argv[idx + 1])
    if "--arena-height" in sys.argv:
        idx = sys.argv.index("--arena-height")
        ARENA_HEIGHT = int(sys.argv[idx + 1])
    
    node = RAFTNode(robot_id, sim_mode=sim_mode)
    node.run()
