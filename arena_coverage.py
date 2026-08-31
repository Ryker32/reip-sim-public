"""Single source of truth for the coverage metric.

The denominator used to be a literal (135) copied between analysis scripts.  It
had no geometric basis: the arena geometry the robot code enforces,
``DEFAULT_ARENA.is_wall_cell()``, rejects 70 of the 16x12=192 cells, so a robot
can only ever occupy 122.  Dividing by 135 understated every hardware coverage
figure by about ten percent and made a fully covered trial read as 90.4%.

This module derives the reachable set from ``DEFAULT_ARENA`` itself, so the
denominator cannot drift from the geometry again.  Import it rather than
hardcoding a cell count.

Coverage is the union of cells occupied by at least one robot over the trial,
which is the definition the paper states: "the percentage of traversable cells
visited by at least one robot".  Reconstructing it from the logged (x, y)
trajectories rather than from each node's ``known_visited_count`` also makes the
measure independent of which peer messages happened to arrive, so controllers
that gossip differently are still measured identically.
"""
import contextlib
import io
import os
import sys

_ROBOT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'robot')
if _ROBOT_DIR not in sys.path:
    sys.path.insert(0, _ROBOT_DIR)

# reip_node prints a hardware-availability warning at import; keep it out of the
# analysis scripts' stdout.
with contextlib.redirect_stdout(io.StringIO()):
    from reip_node import DEFAULT_ARENA  # noqa: E402

ARENA = DEFAULT_ARENA
CELL_SIZE_MM = ARENA.cell_size_mm
GRID_COLS = int(ARENA.width_mm / CELL_SIZE_MM)
GRID_ROWS = int(ARENA.height_mm / CELL_SIZE_MM)

#: Cells a robot can physically occupy, per the arena geometry.
REACHABLE_CELLS = frozenset(
    (cx, cy)
    for cx in range(GRID_COLS)
    for cy in range(GRID_ROWS)
    if not ARENA.is_wall_cell(cx, cy)
)
TOTAL_REACHABLE_CELLS = len(REACHABLE_CELLS)


def cell_of(x_mm, y_mm):
    """Grid cell containing a position, or None if outside the reachable set."""
    if x_mm is None or y_mm is None:
        return None
    cell = (int(x_mm / CELL_SIZE_MM), int(y_mm / CELL_SIZE_MM))
    return cell if cell in REACHABLE_CELLS else None


def covered_cells(entries_by_robot):
    """Union of reachable cells occupied by any robot.

    ``entries_by_robot`` maps robot id -> iterable of log entries carrying
    'x' and 'y' in mm.
    """
    covered = set()
    for entries in entries_by_robot.values():
        for e in entries:
            cell = cell_of(e.get('x'), e.get('y'))
            if cell is not None:
                covered.add(cell)
    return covered


def coverage_pct(entries_by_robot):
    """Percentage of the reachable arena covered by at least one robot."""
    return len(covered_cells(entries_by_robot)) / TOTAL_REACHABLE_CELLS * 100
