# Orca 🐋

> **Universal Maritime Autonomy Engine v0.2.0**
> Surface vessels · Submarines · Yachts · Boats · Autonomous USV — single engine

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue)](https://python.org)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)
[![Tests](https://img.shields.io/badge/Tests-82_passed-brightgreen)](tests/)
[![Stdlib](https://img.shields.io/badge/Core-stdlib_only-lightgrey)](marine_autonomy/)

**한국어 버전:** [README.md](README.md)

---

## What it is

Orca is a **universal autonomous maritime control engine** in pure Python.

Sibling package to [Autonomy_Runtime_Stack](https://github.com/qquartsco-svg/Autonomy_Runtime_Stack) — same Edge AI design principles, adapted for marine environments. Runs on edge devices (Jetson Nano, Raspberry Pi) with zero external dependencies.

```
Core layers (v0.2.0):
  Nonlinear Fossen 3-DOF dynamics  — Coriolis matrix + nonlinear damping + RK4
  Disturbance model                — Wave (sinusoidal) + Wind (dynamic pressure) + Current
  LOS path following               — Line-of-Sight heading + waypoint sequencing
  Multi-vessel COLREGs FSM         — All contacts simultaneously + max avoidance angle
  Maritime A* path planning        — Depth-chart obstacle avoidance + depth bonus cost
  Marine EKF state estimation      — GPS + heading + speed fusion (4-state, pure Python)
  Omega (Ω) safety verdict         — Risk · fuel · visibility · depth · contacts
  Universal hull classes           — Surface / Submarine (4-DOF) / Yacht / Boat / USV
```

> **Disclaimer:** COLREGs logic is an operational heuristic, not a certified compliance system.

---

## Hull Classes

| HullClass | DOF | Vessel | Default Preset |
|-----------|-----|--------|----------------|
| `SURFACE_VESSEL` | 3 | 10m patrol vessel | harbor / coastal / ocean / river |
| `SUBMARINE` | 4 | 50m submarine | sub_shallow / sub_deep |
| `YACHT` | 3 | 12m sailing yacht | yacht_racing / yacht_cruising |
| `BOAT` | 3 | 6m speedboat | boat_patrol / boat_harbor |
| `AUTONOMOUS_USV` | 3 | 3m unmanned surface vehicle | usv_survey |

---

## Core Equations

### 1. Nonlinear Fossen 3-DOF

```
M·ν̇ = τ_ctrl + τ_dist − C(ν)·ν − D(ν)·ν

Coriolis C(ν):  [[0, 0, -m22v-m26r],
                 [0, 0,  m11u      ],
                 [m22v+m26r, -m11u, 0]]

Nonlinear damping: Xu·u + Xuu·|u|·u  (surge)
                   Yv·v + Yvv·|v|·v  (sway)
                   Nr·r + Nrr·|r|·r  (yaw)
```

### 2. Disturbance Forces

```
Wave (sinusoidal): τ_wave = k·Hs²·Awp·cos(ψ_wave−ψ)·sin(2π·t/Tp)
Wind (dyn. pressure): τ_wind = 0.5·ρ_air·Vw²·C·A·cos/sin(ψ_wind−ψ)
Current: relative velocity correction u_r, v_r
```

### 3. Submarine Depth Control (4-DOF)

```
e_z = target_depth − depth
ẇ   = Bz·e_z − Kz·w
depth' = depth + w·dt
```

### 4. LOS Guidance

```
ψ_d = α_k + atan2(−e, Δ) + δ_colregs
δ_rudder = Kp·(ψ_d − ψ) − Kd·r
```

### 5. Multi-vessel COLREGs

```
Priority: EMERGENCY_STOP > GIVE_WAY (max angle) > STAND_ON > CRUISE
HEAD_ON → +20° stbd  |  CROSSING_GW → +15°  |  OVERTAKING → +10°
Returns: situations[], dominant_contact, avoid_heading_offset_rad
```

### 6. Maritime A*

```
cost = step_cost + depth_penalty
depth_penalty = max(0, 1−(depth−draft)/10) × 0.2
passable if: depth[r][c] ≥ draft + min_depth_m
```

---

## Quick Start

```bash
git clone https://github.com/qquartsco-svg/Orca.git
cd Orca
pip install -e .
```

```python
from marine_autonomy import VesselOrchestrator, MarineTickContext, get_preset
from marine_autonomy.contracts.schemas import (
    MarinePerception, ContactVessel, DisturbanceState, HullClass
)

orch = VesselOrchestrator(preset=get_preset("coastal"))
ctx  = MarineTickContext(
    hull_class=HullClass.SURFACE_VESSEL,
    waypoints=((0.0, 0.0), (500.0, 200.0), (1000.0, 0.0)),
    perception=MarinePerception(
        contacts=(
            ContactVessel(id="TGT-01", range_m=400.0, bearing_rad=0.1,
                          cog_rad=3.14, sog_ms=6.0),
            ContactVessel(id="TGT-02", range_m=600.0, bearing_rad=1.2,
                          cog_rad=1.5,  sog_ms=4.0),
        ),
        depth_m=25.0,
    ),
    disturbance=DisturbanceState(
        wave_height_m=1.5, wave_period_s=8.0,
        wind_speed_ms=8.0, wind_dir_rad=1.0,
        t_s=0.0,
    ),
)

for step in range(200):
    ctx = orch.tick(ctx, dt_s=0.1)
    print(f"[{step:3d}] {ctx.colregs_state:15s} Ω={ctx.omega:.3f} {ctx.verdict}")
```

---

## Tests

```bash
pytest tests/ -v
# 82 passed, 0 failed (stdlib only)
```

---

## Related Repos

| Repo | Role |
|------|------|
| [Autonomy_Runtime_Stack](https://github.com/qquartsco-svg/Autonomy_Runtime_Stack) | AV foundation engine |
| [SYD_DRIFT](https://github.com/qquartsco-svg/SYD_DRIFT) | AV + SHA-256 audit chain |
| [marine-propulsion-engine](https://github.com/qquartsco-svg/marine-propulsion-engine) | Shaft wear control + audit |
| **Orca** | Universal maritime autonomy engine (this repo) |

---

## License

MIT

---

*Orca — the orca whale. The most precise navigator in the ocean.*
