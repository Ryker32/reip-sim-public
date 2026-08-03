# REIP Supplemental Materials

**Proactive Trust-Based Detection and Impeachment of Compromised Leaders in Multi-Robot Exploration**

William R. Kollmyer -- Olympia High School, Olympia, WA

This folder holds the reproduction package for the paper. The experiment harness and analysis scripts live at the repository root (`test/`); the raw campaign data the paper's tables cite lives in `experiments/` and `trials/`.

---

## Reproducing the Paper's Tables and Figures

### Prerequisites

- Python 3.10+
- `pip install matplotlib numpy scipy`

### Main simulation campaign (Table I, Fig. 3, detection statistics)

```bash
python test/isef_experiments.py --layout multiroom --trials 100 --workers 20
```

Runs REIP, Raft, and Decentralized under clean, bad-leader, and freeze-leader conditions (N=100 each; 900 trials). Seeds are deterministic: seed = trial index x 1000 (the full list is in `seeds/seeds.json`). The campaigns the paper's tables were generated from are preserved in `experiments/`:

- `run_20260228_014625_multiroom_100trials_all` -- Table I coverage + detection statistics
- `run_20260228_135632_multiroom_100trials_FreezeLeader` -- freeze-leader condition
- `run_20260228_102956_multiroom_30trials_all` -- ablation study (Table II, N=30 per variant)
- `run_20260731_233210` + `run_20260801_000344` -- paired reactive-vs-proactive comparison (Section IV-D)

### Reactive-vs-proactive comparison (Section IV-D)

```bash
python test/isef_experiments.py --layout multiroom --trials 100 --ablation --condition Reactive --workers 20
python test/isef_experiments.py --layout multiroom --trials 100 --condition reip --workers 20
python test/analyze_paired_reactive.py <proactive_results.json> <reactive_results.json>
```

### Figures

```bash
python test/_generate_paper_figures.py
```

Regenerates the paper's coverage figure as a vector PDF directly from the raw campaign JSONs.

### Hardware results (Table III)

Hardware trials cannot be re-run without the physical robots; the per-robot JSONL logs for every trial behind Table III are preserved in `trials/` at the repository root (directories named `<controller>_<fault>_t<trial>_<timestamp>`).

---

## Folder Contents

```
REIP_Supplemental/
├── raw_data/
│   ├── multiroom_n30/          <- earlier N=30 verification campaign (270 trials)
│   ├── snapshots/              <- vector snapshot JSONs for visualization
│   └── gridworld_2000/         <- legacy gridworld prototype data (superseded)
├── seeds/
│   └── seeds.json              <- the 100 seeds used by the N=100 campaigns
├── figures/                    <- charts as PNG + PDF
├── hardware/
│   ├── bom.csv                 <- bill of materials with costs ($84.56/robot)
│   ├── aruco_markers/          <- printable ArUco marker PDFs (robots + arena corners)
│   └── cad/                    <- chassis CAD drawings (isometric + dimensioned views)
└── README.md                   <- this file
```

Note: `raw_data/multiroom_n30` and `raw_data/gridworld_2000` are retained for provenance; the paper's numbers come from the N=100 campaigns in `experiments/` listed above.

## Experiment Conditions

| Condition     | Controllers    | Fault              | Injection            |
|---------------|----------------|--------------------|----------------------|
| Clean         | REIP/Raft/Dec  | none               | --                   |
| Bad Leader    | REIP/Raft/Dec  | `bad_leader`       | t=10 s, t=30 s (dual)|
| Freeze Leader | REIP/Raft/Dec  | `freeze_leader`    | t=10 s, t=30 s (dual)|
| Self-Injure   | REIP/Raft (hw) | `self_injure_leader` | t=10 s (hardware only) |

**Dual sequential injection**: the first fault hits Robot 1 at t=10 s; at t=30 s a second fault targets whoever is currently the leader, testing repeated-attack survival.

## ArUco Marker Layout

| Marker ID | Assignment         | Position             |
|-----------|--------------------|----------------------|
| 0--4      | Robots 1--5        | Affixed to robot top |
| 10        | Arena corner (TL)  | (0, 0) mm            |
| 11        | Arena corner (TR)  | (2000, 0) mm         |
| 12        | Arena corner (BL)  | (0, 1500) mm         |
| 13        | Arena corner (BR)  | (2000, 1500) mm      |

Marker size: 50 mm x 50 mm, dictionary `DICT_4X4_50`. The overhead camera (Logitech C922, 1080p 30 fps) computes a homography from the corner markers to map pixel coordinates to arena millimeters; robots are localized via this homography. The localization service provides position only; all trust, election, and coordination logic runs distributed on the robots.

## Key Results (matching the paper)

- **Simulation (N=100/condition)**: REIP holds 100% median coverage under clean, bad-leader, and freeze-leader conditions; Raft's median falls to 72.7% and 60.4%. Detection: 97--99% of faults, median first suspicion 0.20--0.21 s, median impeachment 1.21--1.67 s.
- **Reactive twin (paired, same build/seeds)**: 47--54% detection at median 62--81 s versus proactive's 100% at 1.4--2.0 s.
- **Hardware (N=5/condition)**: REIP 86.5--91.0% coverage across all fault types; Raft 52.1--61.9% under faults.

## Contact

rykerkollmyer@gmail.com
