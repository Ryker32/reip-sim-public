#!/usr/bin/env python3
"""Pin the published detection bounds to a simulation of the real update rule.

Theorem 1 in the paper is a closed form.  These tests do not check the closed
form against itself: they replay the actual trust update from
``reip_node._apply_suspicion_update`` step by step and assert the constants
exported by ``reip_node`` match what the simulation does.

They exist because the impeachment bound was previously stated as
``C * n_detect``, which multiplies by an already-rounded quantity and rounds
twice.  Under Tier-1 evidence that gave 8 commands where the update rule
actually impeaches at 6, because a threshold crossing subtracts sigma and
carries the residual rather than resetting the accumulator to zero.

Run directly (``python test/test_trust_bounds.py``) or under pytest.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'robot'))

import reip_node as rn  # noqa: E402


def simulate(weight_per_command, n_commands=2000, clean_fraction=0.0):
    """Replay _apply_suspicion_update exactly.

    Returns (commands_to_first_decay, commands_to_impeachment, max_S).
    ``clean_fraction`` spaces cleanly-verifying commands evenly among the
    flagged ones, which is how an adversary would evade accumulation.
    """
    S, T = 0.0, 1.0
    first_decay = impeachment = None
    max_S = 0.0
    clean_due = 0.0
    for n in range(1, n_commands + 1):
        clean_due += clean_fraction
        is_clean = clean_due >= 1.0
        if is_clean:
            clean_due -= 1.0

        # --- mirrors _apply_suspicion_update ---
        if not is_clean:
            S += weight_per_command
        else:
            S = max(0.0, S - rn.RECOVERY_RATE)
        max_S = max(max_S, S)
        if S >= rn.SUSPICION_THRESHOLD:          # `if`, not `while` -- intentional
            T = max(rn.MIN_TRUST, T - rn.TRUST_DECAY_RATE)
            S -= rn.SUSPICION_THRESHOLD          # carry the residual
            if first_decay is None:
                first_decay = n
            if impeachment is None and T < rn.IMPEACHMENT_THRESHOLD:
                impeachment = n
    return first_decay, impeachment, max_S


def test_detection_bound_matches_simulation():
    """n_detect = ceil(sigma / w) is exact for every tier."""
    for weight, const in ((rn.WEIGHT_PERSONAL, rn.WORST_CASE_DETECT_T1),
                          (rn.WEIGHT_TOF, rn.WORST_CASE_DETECT_T1),
                          (rn.WEIGHT_PEER, rn.WORST_CASE_DETECT_T3)):
        first_decay, _, _ = simulate(weight)
        assert first_decay == const, (
            f"w={weight}: simulation first decays at command {first_decay}, "
            f"constant says {const}")


def test_impeachment_bound_matches_simulation():
    """n_imp = ceil(C * sigma / w), not C * ceil(sigma / w)."""
    for weight, const in ((rn.WEIGHT_PERSONAL, rn.WORST_CASE_IMPEACH_T1),
                          (rn.WEIGHT_PEER, rn.WORST_CASE_IMPEACH_T3)):
        _, impeachment, _ = simulate(weight)
        assert impeachment == const, (
            f"w={weight}: simulation impeaches at command {impeachment}, "
            f"constant says {const}")


def test_impeachment_bound_is_tight_not_the_old_loose_form():
    """Guard against reintroducing C * n_detect."""
    loose = rn.THRESHOLD_CROSSINGS_TO_IMPEACH * rn.WORST_CASE_DETECT_T1
    assert rn.WORST_CASE_IMPEACH_T1 < loose, (
        "Tier-1 impeachment bound has reverted to the doubly-rounded form")
    assert rn.WORST_CASE_IMPEACH_T1 == 6 and loose == 8


def test_closed_form_agrees_across_weights():
    """ceil(C * sigma / w) tracks the simulation across plausible weights.

    Exact equality is asserted for the deployed tier weights.  For other weights
    the simulation may lag the closed form by one command: the accumulator is a
    float, so a residual that should land exactly on sigma can land just under
    it.  At w=0.2, command 30 gives S=1.4999999999999996 and the crossing slips
    to command 31, where exact rational arithmetic gives 30.  This is a property
    of the implementation, not of the bound, and it does not affect the
    published Tier-1 or Tier-3 figures -- both are exact.
    """
    deployed = (rn.WEIGHT_PERSONAL, rn.WEIGHT_TOF, rn.WEIGHT_PEER)
    for weight in (1.0, 0.9, 0.5, 0.3, 0.25, 0.2, 0.15):
        _, impeachment, _ = simulate(weight)
        closed = math.ceil(rn.THRESHOLD_CROSSINGS_TO_IMPEACH
                           * rn.SUSPICION_THRESHOLD / weight)
        if weight in deployed:
            assert impeachment == closed, (
                f"deployed weight w={weight}: simulation {impeachment}, "
                f"closed form {closed} -- these must agree exactly")
        else:
            assert closed <= impeachment <= closed + 1, (
                f"w={weight}: simulation {impeachment}, closed form {closed}; "
                f"more than one command of float slack means the formula is wrong")


def test_single_command_maximum_is_two_sigma():
    """The per-command contributions cap at exactly 2*sigma.

    Tier evidence is mutually exclusive (max 1.0); peer-safety adds at most once
    (it breaks out of the peer loop); MPC-severe caps at 0.5 + 0.3; omega caps at
    OMEGA_WEIGHT.  Their sum is what makes the paper's floor operator differ from
    this code's single decrement, so it is pinned here.
    """
    max_tier = max(rn.WEIGHT_PERSONAL, rn.WEIGHT_TOF, rn.WEIGHT_PEER)
    max_mpc = 0.8   # 0.5 + (pi - SEVERE)/(pi - SEVERE) * 0.3, see _compute_mpc_direction_error
    max_omega = 0.3  # OMEGA_WEIGHT, local to assess_leader_command
    total = max_tier + rn.WEIGHT_PEER_SAFETY + max_mpc + max_omega
    assert abs(total - 2 * rn.SUSPICION_THRESHOLD) < 1e-9, (
        f"max single-command suspicion is {total}, no longer 2*sigma; the "
        f"divergence from the paper's floor operator has changed magnitude")


def test_single_decrement_is_conservative():
    """`if` can only decay trust more slowly than the paper's floor operator."""
    S, T_if, T_floor = 0.0, 1.0, 1.0
    for _ in range(200):
        S += 1.9  # tier + peer-safety, the largest combination seen in the logs
        crossings = int(S // rn.SUSPICION_THRESHOLD)
        if S >= rn.SUSPICION_THRESHOLD:
            T_if = max(rn.MIN_TRUST, T_if - rn.TRUST_DECAY_RATE)
            S -= rn.SUSPICION_THRESHOLD
        T_floor = max(rn.MIN_TRUST, T_floor - crossings * rn.TRUST_DECAY_RATE)
        assert T_if >= T_floor - 1e-9, "single decrement decayed faster than floor"


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for t in tests:
        t()
        print(f"PASS  {t.__name__}")
    print(f"\n{len(tests)} tests passed.")
    print(f"n_detect: T1={rn.WORST_CASE_DETECT_T1}  T3={rn.WORST_CASE_DETECT_T3}")
    print(f"n_imp:    T1={rn.WORST_CASE_IMPEACH_T1}  T3={rn.WORST_CASE_IMPEACH_T3}")
    print(f"At {5} Hz: Tier-1 impeachment within "
          f"{rn.WORST_CASE_IMPEACH_T1 / 5:.1f}s, Tier-3 within "
          f"{rn.WORST_CASE_IMPEACH_T3 / 5:.1f}s")
