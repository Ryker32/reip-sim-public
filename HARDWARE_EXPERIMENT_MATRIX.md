# [ARCHIVED WORKING NOTES] Hardware Experiment Matrix

> **This is a planning checklist from partway through the hardware campaign and is
> retained only for provenance. The coverage/speed/detection numbers below are
> interim values from early trials and are superseded by the final results in the
> paper's Table III, which were computed from the complete trial logs in `trials/`.
> The bad-leader hardware condition flagged as missing below was subsequently run
> and is included in the final results.**

## Current Status in Paper (Table 2)

| Controller | Fault Type | N | Coverage | Speed (mm/s) | Detect (s) | Status |
|-----------|------------|---|----------|--------------|------------|--------|
| REIP | None (clean) | 5 | 63.5% | 42 | --- | [x] DONE |
| REIP | Freeze Ldr | 5 | 63.5% | 45 | 8.8 | [x] DONE |
| REIP | Self-Injure | 5 | 61.6% | 46 | 3.3 | [x] DONE |
| Raft | None (clean) | 5 | 66.3% | 19 | --- | [x] DONE |
| Raft | Freeze Ldr | 5 | 33.3% | 19 | --- | [x] DONE |
| Raft | Self-Injure | 5 | 21.5% | 19 | --- | [x] DONE |

## [!] POTENTIAL GAPS / QUESTIONS:

1. **"Self-Injure" fault type**: [x] **CONFIRMED** - This is `self_injure_leader` fault:
   - Leader commands followers into leader's occupied peer bubble (causes collisions)
   - Different from `bad_leader` (sends to explored cells)
   - Different from `freeze_leader` (stops updating assignments)

2. **Missing `bad_leader` hardware results**: [!] **POTENTIAL GAP**
   - Simulation has `bad_leader` as the KEY differentiator (90.9% vs Raft 66.4%)
   - Hardware table shows "Freeze Ldr" and "Self-Injure" but NOT `bad_leader`
   - **Question**: Do you have `bad_leader` hardware results? If not, this is the most important one to run!

3. **Decentralized baseline**: Not in hardware table. Paper only shows REIP vs Raft. Is this intentional?

## RECOMMENDED COMPLETE MATRIX (for ISEF submission):

### Minimum Required (if you have the 6 above):
[x] **You have all 6 conditions reported in the paper**

### [!] CRITICAL MISSING (if not already run):
- **REIP + `bad_leader` fault** - THIS IS THE KEY DEMO (simulation shows 90.9% vs Raft 66.4%)
- **Raft + `bad_leader` fault** - Shows Raft's failure (simulation shows 66.4% coverage)

### Optional Additional (for stronger validation):
- Decentralized + None (clean baseline)
- Decentralized + `bad_leader` (if you want to show it's immune)

## QUICK VERIFICATION CHECKLIST:

Before submitting, verify:

1. [x] All 6 conditions in Table 2 have data
2. [x] N=5 trials per condition (or more)
3. [x] Coverage, Speed, and Detect times are calculated
4. [x] Fault injection timing matches paper description
5. [x] "Self-Injure" is clearly defined (what fault command?)
6. [x] Speed-normalized comparison table (Table 3) has matching data

## FAULT INJECTION COMMANDS REFERENCE:

From `EXPERIMENT_PROCEDURE.md`:
- `bad_leader <id>` - Byzantine: leader sends to explored cells
- `freeze_leader <id>` - Leader stops updating assignments
- `stop <id>` - Motor fault: robot freezes
- `spin <id>` - Motor fault: robot spins

**"Self-Injure" might be**: Leader stops updating assignments AND stops moving = `freeze_leader` + `stop`?

## TIME ESTIMATE:

- Each trial: 120 seconds
- Setup between trials: ~2-3 minutes
- **6 conditions * 5 trials = 30 trials**
- **Total time: ~2-3 hours** (if robots are already set up)

## ACTION ITEMS (PRIORITY ORDER):

1. ** CRITICAL: Check if `bad_leader` hardware results exist**
   - This is the KEY differentiator in simulation (90.9% vs 66.4%)
   - If missing, this is the #1 priority to run
   - Fault command: `bad_leader 1` (inject at t=10s)

2. **Confirm all 6 existing conditions have N>=5 trials** - Paper shows N=5

3. **Double-check all numbers in Table 2** - Coverage, Speed, Detect times

4. **Verify Table 3 (speed-normalized) calculations** - Based on Table 2 data

5. **If you add `bad_leader` results, update Table 2** - Add 2 more rows (REIP + Raft)

---

## IF YOU NEED TO RUN MORE TRIALS:

###  CRITICAL PRIORITY (if `bad_leader` is missing):
1. **REIP + bad_leader** - THIS IS THE KEY DEMO (simulation: 90.9% coverage)
   - Fault injection: `bad_leader 1` at t=10s
   - Minimum: 5 trials (N=5)
   - Expected: Coverage ~60-65%, Detection ~1-3s

2. **Raft + bad_leader** - Shows Raft's failure (simulation: 66.4% coverage)
   - Fault injection: `bad_leader 1` at t=10s
   - Minimum: 5 trials (N=5)
   - Expected: Coverage ~30-40% (much worse than REIP)

### Lower Priority:
3. **More trials for existing conditions** (N=5 is minimum, N=10 is better for stats)

### Quick Run Command (if using automated system):
```bash
# Run bad_leader condition (if missing)
python test/isef_experiments.py --layout multiroom --trials 5 --condition BadLeader
```

### Manual Hardware Run (for bad_leader):
1. Start position server
2. Start all robots (REIP or Raft)
3. Wait for leader election
4. At t=10s, inject fault: `bad_leader 1` (in fault injector terminal)
5. Let run for 120s
6. Collect logs
7. Repeat 5 times per controller

### Manual Hardware Run:
See `EXPERIMENT_PROCEDURE.md` for step-by-step instructions.
