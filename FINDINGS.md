# Audit findings

A reproducibility audit of this repository, carried out 2026-08-30/31, against
the URTC 2026 paper's Tables I–III and Theorem 1.

Every issue below was found by reading the code and the raw logs, not by
comparing against expected answers. Each entry records what was wrong, how it
surfaced, what changed in the code, and what still needs an edit to the paper.

**Bottom line:** no result reversed. Tables I and II reproduce exactly and were
untouched. Table III's comparative claims survived — all four REIP-vs-Raft gaps
held or widened when both controllers were finally measured the same way. What
was wrong was the measurement apparatus around the numbers: a denominator with
no geometric basis, a clock read in the wrong units, an aggregator that silently
discarded half its input, and two mismatches between the paper's stated
mathematics and the implementation.

Relevant commits: `6b087b09`, `0e3f7a95`, `7bcb9ef3`, `e6ac5d60`.

---

## 0. Table I's build was never committed

**What was wrong.** The paper states that all raw results, seeds and scripts are
public and that every statistic was verified against them. The simulator version
that produced Table I was never committed. The campaigns ran 2026-02-28; the
last commit before them is `8c7f547b` (2026-02-27), whose harness contains no
`freeze_leader` fault at all, and the next commit touching those files is
2026-03-10, by which time the physics model had been replaced. The code that ran
sits between them, in a working tree that was overwritten.

**How it was found.** Walking `git log` and `git reflog` around the campaign
timestamps. The reflog shows no commit on 2026-02-28, which is stronger evidence
than file timestamps.

**What the raw data still proves.** The build is identifiable even though the
tree is gone. Two independent fingerprints in the preserved logs pin it to
`8c7f547b` plus an uncommitted freeze-leader fault:

* Replaying `random.Random(seed)` through `8c7f547b`'s `start_robots` reproduces
  the logged starting positions of all five robots exactly, in both campaigns
  and across controllers (trial 1: `R1=(694,954) R2=(397,711) R3=(835,306)
  R4=(405,737) R5=(616,423)`).
* Every 2026-02-28 robot log prints `Detection Bounds: T1=2 cmds, T3=8 cmds |
  Impeachment: T1=8 cmds, T3=32 cmds`, which is `8c7f547b`'s bound formula.
  Today's formula prints `T3=5` and `T3=20`.

**Status.** Not fixed and not fixable — the tree is gone. `verify_from_raw.py`
reproduces every Table I statistic from the committed raw data, which
establishes that the published numbers follow from the published data, but not
that a reader can regenerate that data.

**Paper edit needed.** The reproducibility claim in Section VI-A overstates what
the repository supports. Either scope it to the raw results and analysis
(accurate), or re-run the campaign on a tagged commit.

**Related.** Simulation results are not bit-reproducible in any case: robots run
as separate processes over UDP and `dt` comes from wall-clock, so absolute
coverage is machine-dependent. Measured on two machines, the same build and
seeds differed by roughly 28 percentage points — larger than the paper's
headline effect. This deserves a limitations sentence.

---

## 1. Coverage denominator: 135, with no geometric basis

**What was wrong.** Coverage was computed as `visited / 135`. The arena geometry
the robot code enforces, `DEFAULT_ARENA.is_wall_cell()`, rejects 70 of the
16×12 = 192 cells, so a robot can only ever occupy **122**. Dividing by 135
understated every hardware coverage figure by about ten percent and made a fully
covered trial read as 90.4%, which is the ceiling 122/135, not a measurement.

**How it was found.** Four trials reported coverage above 100% (up to 142.2%),
which is impossible under any correct denominator. Chasing that anomaly exposed
the constant.

135 is not derivable from the geometry at any clearance assumption — sweeping
clearance yields only 190, 181 or 122 free cells. It was introduced in
`a6829b3f` (2026-03-14, "added some data pipelines") as a bare literal commented
only `# explorable cells in the multiroom arena`, and propagated by copying into
`aggregate_hardware_results.py` and `parse_hardware_trials.py`.

**What changed.** `arena_coverage.py` derives the reachable set from the
geometry, and all three analysis scripts import it. No literal remains. Coverage
is now the union of reachable cells occupied by at least one robot,
reconstructed from logged trajectories — the definition the paper states, and
unlike each node's `known_visited_count` it does not depend on which peer gossip
arrived.

The four impossible trials are all from 2026-03-03, before the 2026-03-05 fix
(`bf78bc1b`) that stopped marking a five-cell plus-shape around the robot on
every position update. They are now rejected with a loud warning.

**Effect on Table III:**

| | published (÷135) | corrected (÷122, trajectory) |
|---|---|---|
| REIP clean | 91.0 (N=5) | **91.8** (N=4) |
| REIP bad leader | 86.5 | **86.7** |
| REIP freeze | 87.3 | **87.7** |
| REIP self-injure | 90.1 | **92.5** |
| Raft clean | 89.5 | **90.7** |
| Raft bad leader | 52.1 | **52.0** |
| Raft freeze | 61.9 | **60.8** |
| Raft self-injure | 55.0 | **56.7** |

**The comparative claims hold.** Gaps: clean +0.9 → **+1.1**, bad leader
+34.4 → **+34.8**, freeze +25.4 → **+26.9**, self-injure +35.1 → **+35.7**.

**Paper edits needed.** Replace the Table III body; the caption must say N=4 for
REIP clean, whose trial selection is inferred rather than recorded. Update the
three gap figures. Rewrite the hardware-versus-simulation transfer sentence: both
sides now share a denominator and measure, so the comparison is legitimate, but
the numbers moved. The setup text says "192 cells of 125 mm" while coverage is
over 122; suggested wording:

> The arena is discretised into 125 mm cells on a 16×12 grid (192 cells), of
> which 122 are traversable: the remainder lie within the robot's body clearance
> of a wall or the divider and cannot be occupied by a robot's centre. Coverage
> is the percentage of those 122 traversable cells entered by at least one robot.

**Related, fixed at the same time.** Neither controller filtered wall cells on
the peer-merge path, and `raft_node.py` added cells on target arrival with no
filter at all, so Raft accumulated up to 124 cells against REIP's 122 — the two
were measured against different reachable sets. The asymmetry worked against
REIP. All three sites now apply the same mask.

The geometry was also defined three times: `hardware_fidelity.py` (simulation),
`robot/reip_node.py` (standalone, because deployment copies that one file to
each Pi), and `pc/visualize_vectors.py`, the last using outer margin 0, divider
margin 20 and body radius 77 — none matching the 110/64/100 the robots use, so
the overlay drew a different reachable set than every reported number was
computed against. `arena_coverage.py` is now the single import point, takes
`hardware_fidelity` as canonical, and raises at import if the robot copy drifts.

---

## 2. Detection times computed on the wrong clock

**What was wrong.** `trial_meta.json` stores `fault1_actual_time` as **seconds
since `start_time`** (e.g. `20.02`). The aggregator's comment asserted the
opposite — "fault1_time is already absolute" — and its lookup compared a ~1.77e9
epoch against ~20.0:

```python
if abs(e['t'] - fault1_time) < 5.0:   # never true
...
fault_t_rel = fault1_time - t0 if fault1_time > t0 else 20.0   # always 20.0
```

The lookup never matched for any trial, so execution always fell through to a
hardcoded `20.0`. That literal was measured from `t0`, the first log line, which
is 1.7–2.2 s after `start_time`, so the fault was placed later than it occurred
and every detection interval came out that much too short.

**How it was found.** The paper's detection figures (15.8 / 17.0 / 8.6 s) did not
match the aggregator's (12.1 / 15.6 / 13.4 s). Testing candidate event
definitions against the logs required reading the fault time, which exposed the
unit mismatch.

**What changed.** The fault time is now `start_time + fault1_actual_time`. A
fault trial with no recorded injection is reported loudly and excluded from
detection statistics rather than silently assigned the literal. That fires
exactly once, on `raft_bad_leader_t1_20260315_201207` — the only one of 57
fault-condition trials missing the field, and independently the trial the
contemporaneous run log excluded.

The summary now reports both of Table I's definitions instead of one ambiguous
"Detect" column.

**Corrected values (medians, seconds):**

| Condition | first suspicion | impeachment | previously published |
|---|---|---|---|
| Bad leader | **2.0** | **17.7** | 15.8 |
| Freeze | **8.8** | **20.5** | 17.0 |
| Self-injure | **12.4** | **18.3** | 8.6 |

Of the three published figures, only bad-leader's 15.8 was ever reproducible,
and it corresponded to impeachment (mean 15.8). Freeze and self-injure match
nothing under any of four tested definitions, mean or median.

**Paper edits needed.** Replace the detection column, stating which event it
measures. **Delete or re-ground the claim that self-injure is caught fastest
because Tier-2 sensing triggers immediately** — under every definition tested it
is not: first suspicion 12.4 s against bad-leader's 2.0 s, and middle on
impeachment. Note also that the hardware/simulation detection contrast is an
order of magnitude (simulation impeaches in 1.2–1.7 s, hardware in 17.7–20.5 s),
which is starker than the current framing implies.

---

## 3. The aggregator silently dropped all 28 Raft hardware trials

**What was wrong.** `parse_trial_dir` looked only for `r*_robot_*.jsonl`. Raft
trials name their per-robot logs `r1_raft_1_<ts>.jsonl`, so the glob never
matched and the function returned `None` — **silently**, with no warning. Every
Raft row in Table III was therefore unreproducible from the repository's own
tooling, even though the data was present and complete.

Of 198 trial directories: 119 used `r*_<controller>_*.jsonl` (missed), 60 used
`r*_robot_*.jsonl` (matched), 19 had metadata but no logs (missed).

**How it was found.** Running the repository's own aggregator from a clean clone
produced three REIP rows and no Raft rows at all.

**What changed.** All naming conventions are tried in turn, and every skipped
trial now reports why. Trial selection follows `HARDWARE_RUN_GUIDE.md`, the
contemporaneous run log naming the exact directory each numbered trial produced,
rather than an inferred heuristic; conditions the log does not cover are
labelled as inferred.

**A note on method.** The selection rule was initially reverse-engineered by
searching for the subset of trials whose mean matched each published value. That
is circular and cannot fail. It was replaced with the run log, which is
independent evidence: it names `raft_bad_leader_t1_20260315_201736` at 65.9%
coverage over an alternative attempt at 91.9%, a choice no results-driven
selection would make. The two agree on all 35 trials the log covers.

**Also fixed.** `hardware_results_summary.txt` was a stale UTF-16 aggregate over
all 198 directories, reporting REIP clean as N=34 at 67.3% and containing
coverage values up to 142%. It has been regenerated.

**Paper edit needed.** None beyond the Table III replacement in §1 — the Raft
values were correct, they simply could not be reproduced.

---

## 4. The impeachment bound was rounded twice

**What was wrong.** Theorem 1 gave `n_imp = ⌈(T₀ − τ_imp)/Δ⌉ · n_detect`, which
multiplies by an already-rounded `n_detect = ⌈σ/w⌉`. A crossing subtracts σ and
carries the residual rather than resetting, so crossings after the first arrive
sooner. Under Tier-1 evidence the update rule impeaches at command 6, not 8:

```
cmd 1: S=1.0
cmd 2: S=0.5  <- crossing, trust=0.8
cmd 3: S=0.0  <- crossing, trust=0.6      (one command, not two)
cmd 4: S=1.0
cmd 5: S=0.5  <- crossing, trust=0.4
cmd 6: S=0.0  <- crossing, trust=0.2      impeachment
```

**How it was found.** Simulating the actual update rule rather than evaluating
the closed form.

**What changed.** The constants now use the tightened form. They appear only in
log lines, never in control flow, so no runtime behaviour changed.

```
n_imp = ⌈ ⌈(T₀ − τ_imp)/Δ_decay⌉ · σ / w_k ⌉
```

Tier 1: 8 → **6** commands, **1.2 s** at 5 Hz rather than 1.6 s. Tier 3
unchanged at 20 and 4.0 s, because σ/w is integral there. `n_detect = ⌈σ/w_k⌉`
was already exact.

`test/test_trust_bounds.py` pins both bounds against a step-by-step replay of
the real update rule and guards against the doubly-rounded form returning.

**Paper edits needed.** Replace the `n_imp` formula and its proof sentence;
update the deployed-parameters line to 6 commands and 1.2 s. **Drop the
"consistent with the empirical medians of 1.21–1.67 s" clause** — at 1.2 s the
bound no longer sits above the 1.21 s observed median, and the two are not
strictly comparable since the bound counts commands while the median includes
broadcast jitter.

**Also.** The evasion threshold `r/(w_k + r)` is an asymptotic drift condition,
not a mission-duration one. Within 120 s at 5 Hz, a Tier-3 adversary at 26%
still evades despite positive drift; the effective threshold is nearer 30%.
Conversely a randomly-scheduled adversary is caught well below its own bound
(13/20 trials at 4% under Tier-1), which favours REIP. The sentence needs
"asymptotically".

---

## 5. Eq. 2's floor operator does not describe the code

**What was wrong.** The paper writes
`T ← max(T_min, T − ⌊S/σ⌋ · Δ_decay)`. The code uses `if`, not `while`: at most
one decrement per update, subtracting a single σ and carrying the rest.

The two differ whenever S ≥ 2σ, and that is reachable. Per-command contributions
cap at tier 1.0 + peer-safety 0.9 + MPC-severe 0.8 + omega 0.3 = **3.0, exactly
2σ**. The accumulator also climbs across commands, because each update removes at
most one σ while additions continue.

**How it was found.** Bounding the maximum single-command suspicion by
enumerating every contributing block, then scanning the logs for what was
actually reached.

**Measured in the recorded campaigns:** largest single addition **1.90**;
largest accumulator value **9.50**; **716** of 230,759 flagged commands left the
accumulator at or above 2σ. At S = 9.5 the floor operator prescribes six
decrements where the code applies one.

**What changed.** Nothing in the runtime. `while` would alter behaviour on
results already verified against the raw data. `_apply_suspicion_update` now
documents that one decrement per update is deliberate, and that the divergence
is **conservative** — `if` decays trust more slowly than the floor operator, so
it cannot manufacture a false impeachment, and the detection bound still holds.

**Paper edit needed.** Amend Eq. 2 to state that one decrement is applied per
update, or note explicitly that the floor form upper-bounds the decay the code
applies.

**Incidental.** The accumulator is a float, so for weights outside the deployed
tiers a residual that should land exactly on σ can land just under it, slipping
a crossing by one command (w = 0.2 gives 31 where exact arithmetic gives 30).
The deployed weights (1.0, 0.3) are exact.

---

## Open items

**Credentials in git history — both repositories.** The robot SSH password was
committed in plaintext across 28 scripts here and in `run_trial.py` in
`REIP-RESEARCH-FINAL`, which has been public since 2026-03-16. It has been
rotated on all five robots and removed from both working trees, but **remains in
both histories**. Removing it needs `git filter-repo --replace-text` and a
force-push; note that old commits stay reachable by SHA on GitHub afterwards
unless Support garbage-collects them. The rotation is what actually protects the
robots.

**Absolute paths in logs.** Thirteen blobs under
`experiments/run_20260227_*/logs/multiroom_raft_Oscillate_*/` contain
`C:\Users\ryker\research\reip-sim-public\...` inside Python tracebacks, leaking
the Windows username and directory layout. Low severity; fold into the same
`filter-repo` pass. Those tracebacks are also crashes in the Raft oscillate
baseline (`_compute_oscillate_assignments` failing on `self.arena_width`) —
oscillate is not in Table I, so no published number is affected.

**Nothing else was found.** A sweep of all 57,154 non-source objects across all
61 commits found no API keys, tokens, cloud credentials, private keys, WiFi
credentials, email addresses, phone numbers or machine names.

**Line-ending churn.** `.gitignore` shows as modified with a CRLF-versus-LF
difference and no content change. Left alone; it will surface on a future commit.

---

## Verification

`verify_from_raw.py` reproduces every published statistic in Tables I–III from
the raw data in this repository, and exits non-zero if any check fails:

```bash
git clone https://github.com/Ryker32/reip-sim-public.git
cd reip-sim-public
python verify_from_raw.py        # exits 0
python test/test_trust_bounds.py # 6 tests
```
