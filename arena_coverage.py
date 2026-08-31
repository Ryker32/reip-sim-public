"""Single source of truth for the arena geometry and the coverage metric.

Three copies of the arena geometry used to exist independently:

* ``hardware_fidelity.ArenaGeometry`` -- used by the simulation harness.
* ``robot/reip_node.py`` -- a standalone copy, because deployment copies that
  one file to each Pi and it cannot import from the rest of the tree.
* ``pc/visualize_vectors.py`` -- a third implementation with entirely different
  constants (outer margin 0, divider margin 20, body radius 77).

Nothing kept them in agreement, and the coverage denominator was worse still: a
literal ``135`` copied between analysis scripts, with no geometric basis at all.
``DEFAULT_ARENA.is_wall_cell()`` rejects 70 of the 16x12=192 cells, leaving 122,
so dividing by 135 understated every hardware coverage figure by about ten
percent and made a fully covered trial read as 90.4%.

This module is now the single import point for PC-side code.  It takes
``hardware_fidelity`` as canonical and *verifies* that the standalone robot copy
still agrees, raising at import if it has drifted.  Import ``ARENA`` from here
rather than defining geometry or hardcoding a cell count.

Coverage is the union of reachable cells occupied by at least one robot, which
is the definition the paper states: "the percentage of traversable cells visited
by at least one robot".  Reconstructing it from logged (x, y) trajectories
rather than from each node's ``known_visited_count`` also makes the measure
independent of which peer gossip arrived, so controllers that share coverage
differently are still measured identically.
"""
import contextlib
import io
import os
import sys

_ROOT = os.path.dirname(os.path.abspath(__file__))
for _p in (_ROOT, os.path.join(_ROOT, 'robot')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from hardware_fidelity import DEFAULT_ARENA as ARENA  # noqa: E402

CELL_SIZE_MM = ARENA.cell_size_mm
GRID_COLS = ARENA.cols
GRID_ROWS = ARENA.rows

#: Cells a robot can physically occupy, per the arena geometry.
REACHABLE_CELLS = frozenset(
    (cx, cy)
    for cx in range(GRID_COLS)
    for cy in range(GRID_ROWS)
    if not ARENA.is_wall_cell(cx, cy)
)
TOTAL_REACHABLE_CELLS = len(REACHABLE_CELLS)

# Fields that must match between the canonical geometry and the standalone copy
# deployed to the robots.
_SHARED_FIELDS = (
    'width_mm', 'height_mm', 'cell_size_mm', 'outer_wall_margin_mm',
    'divider_margin_mm', 'divider_tip_clearance_mm',
    'interior_wall_x_left_mm', 'interior_wall_x_right_mm',
    'interior_wall_y_end_mm',
)


def _verify_robot_copy():
    """Fail loudly if robot/reip_node.py's standalone geometry has drifted.

    That copy cannot import this module (it is deployed to the Pis on its own),
    so agreement is checked rather than enforced by construction.
    """
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            from reip_node import DEFAULT_ARENA as ROBOT_ARENA
    except Exception as exc:  # pragma: no cover - robot deps missing
        raise ImportError(
            "arena_coverage could not import robot/reip_node.py to verify the "
            f"deployed arena geometry still matches hardware_fidelity: {exc}"
        ) from exc

    mismatches = [
        (f, getattr(ARENA, f), getattr(ROBOT_ARENA, f, None))
        for f in _SHARED_FIELDS
        if getattr(ARENA, f) != getattr(ROBOT_ARENA, f, None)
    ]
    robot_reachable = sum(
        1 for cx in range(GRID_COLS) for cy in range(GRID_ROWS)
        if not ROBOT_ARENA.is_wall_cell(cx, cy)
    )
    if robot_reachable != TOTAL_REACHABLE_CELLS:
        mismatches.append(
            ('reachable cell count', TOTAL_REACHABLE_CELLS, robot_reachable))
    if mismatches:
        detail = "\n".join(
            f"    {name}: hardware_fidelity={a!r} reip_node={b!r}"
            for name, a, b in mismatches)
        raise AssertionError(
            "Arena geometry has drifted between hardware_fidelity.py and "
            "robot/reip_node.py. Coverage numbers from the simulation and the "
            "robots are no longer comparable:\n" + detail)


_verify_robot_copy()


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
