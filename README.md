# inverted-pendulum-fuzzy-control

Swing-up, balancing **and cart position regulation** of an inverted pendulum
on a cart, using fuzzy logic controllers benchmarked against LQR and classical
PID baselines.

Every component of the fuzzy system — membership function shape and partition,
rule base origin, inference operator, defuzzification method — is a
configurable design variable. The non-fuzzy controllers exist to quantify what
the fuzzy design actually contributes.

**[View the results report →](https://miladnikmand.github.io/inverted-pendulum-fuzzy-control/report.html)**

---

## The problem

A cart of mass `mc` slides along a horizontal track. A pendulum of length `l`
and bob mass `m` hangs from the cart pivot and swings in the vertical plane.

```
            Z ↑
              │   bob   (upright: z = +l)
              ○
              │   rod, length l
   ───────────●───────────   cart, mass mc
              │
          track / ring
```

State `x = [xc; ẋc; θ; θ̇]`, input `F` (horizontal force on the cart).
`θ = 0` is upright and unstable; `θ = π` is hanging and stable.

The controller has to do three things:

1. **Swing up** — pump energy until the pendulum reaches the upright zone
2. **Balance** — hold `θ → 0` against gravity
3. **Regulate position** — bring the cart to `xc → xc_ref` and keep it there

The third is what makes this more than a textbook exercise. The system is
**underactuated**: two outputs to place, one force to place them with. Every
force that corrects the angle also moves the cart.

---

## Equations of motion

Derived via Lagrangian mechanics, used in full nonlinear form:

```
(mc+m)·ẍc  +  m·l·θ̈·cos θ  =  F  −  b·ẋc  −  m·l·θ̇²·sin θ
m·l²·θ̈    +  m·l·ẍc·cos θ  =  m·g·l·sin θ  −  bp·θ̇
```

The coupling `m·l` is non-zero at equilibrium, so the linearised system is
**rank-4 controllable** and LQR is a valid baseline.

**Error convention: `e = +θ`.** Since `M⁻¹₂₁ < 0`, positive force produces
negative angular acceleration, so a positive angle error needs positive force
to correct it.

---

## The cascade governor

The obvious way to add position control is to bolt a correction onto the
controller output:

```
F = F_θ(θ, θ̇) + kp_xc·(xc − xc_ref) + kd_xc·ẋc      ← does not work
```

This fails structurally, not through bad tuning. Near equilibrium the angle
loop is very stiff — a tenth of a degree already commands several newtons — so
the position term is either negligible against it, or large enough to topple
the pendulum. Raising the gain trades one failure for the other.

The working architecture never adds the two objectives. It separates them by
timescale and lets the fast loop serve the slow one:

```
θ_ref = clamp( −kp_gov·(xc − xc_ref) − kd_gov·ẋc ,  ±θ_max )    outer, slow
F     = F_θ( θ − θ_ref , θ̇ )                                    inner, fast
```

The inner loop is unchanged — it still regulates an angle to a reference. The
outer loop moves that reference. Tilting the pendulum a degree or two puts a
horizontal component of gravity on the bob, the cart accelerates that way, and
the inner loop holds the tilt while it happens. The coupling that makes the
problem hard is the coupling the solution exploits.

At steady state `xc = xc_ref` gives `θ_ref = 0`, and the controller reduces
exactly to pure balancing.

**Stability gate.** The governor stays off until `|θ| < 5°` has held for 1.5 s
continuously. If the angle later leaves that band the governor disengages and
re-arms only once balance returns. Position is always the lower-priority
objective, which is what stops it ever costing the pendulum.

**Gain derivation.** The plant is augmented with the integral of position
error, `xa = [xc, ẋc, θ, θ̇, ∫(xc − xc_ref)]`, and an LQR solved on that
5-state system. The cascade gains are read off the resulting `K₅`:

```
kp_gov = |K₅[1]/K₅[3]|·180/π        kd_gov = |K₅[2]/K₅[3]|·180/π
```

So the analytical design already works before any tuning, and the optimiser
reports improvement relative to it.

---

## Controllers

| Controller | Type | Role |
|---|---|---|
| Fuzzy PID — Sugeno | Fuzzy | Weighted-average defuzzification |
| Fuzzy PID — Mamdani | Fuzzy | Centroid defuzzification |
| Fuzzy PID — Type-2 | Fuzzy | Interval MFs, Karnik-Mendel reduction |
| Fuzzy SMC | Fuzzy | Sliding mode, fuzzy-scheduled switching gain |
| Adaptive Fuzzy | Fuzzy | Online gradient-descent rule update |
| LQR | Baseline | Augmented 5-state, integral position action |
| Classical PID | Baseline | Fixed gains, same cascade structure |

All non-LQR controllers use the same governor, so the comparison isolates the
stabilising law rather than the architecture.

Near equilibrium the fuzzy controllers use LQR-matched base gains and scale
them up for large errors (×1.5 at 5°, ×4.0 beyond 20°). The fuzzy advantage
lies in the transient right after capture, where the linearised LQR is least
valid.

**On the SMC:** it underperforms on position, and the reason is structural.
The sliding surface drives `s = λe + ė → 0`, which defines a *trajectory*
toward the origin, not the origin itself. Without integral action it settles at
a small non-zero angle whenever the cart is moving, so the governor never sees
the stable band it needs. This is reported rather than hidden — it is a real
property of the method.

---

## Swing-up

| Strategy | Law | Guarantee |
|---|---|---|
| Energy | `F = k·(E − E_target)·cos θ·θ̇` | Lyapunov, `V̇ ≤ 0` |
| Backstepping | Two-level cascade, `ẍc_desired → F` | Composite Lyapunov |
| Sinusoidal | Phase-aware bang-bang at `ωₙ` | None — empirical |

The energy law is proportional in `θ̇` with **no sign function**. A
`sign(θ̇·cos θ)` formulation chatters at `dt = 0.005 s` because the sign flips
every step near the hanging position, where `θ̇` crosses zero. The proportional
form also scales itself down as `E → E_target`, giving a smooth handoff.

---

## Disturbances

Five types, each applied to the **already-settled** system by warm-starting
from a baseline run, so the test measures rejection rather than swing-up.

| Type | Mechanism | Channel |
|---|---|---|
| Impulse kick | Gaussian force pulse on the cart | `ẋc` |
| Wind gust | Aerodynamic drag on the rod, ramp/hold/release | `θ` torque |
| Track friction | Coulomb + viscous spike opposing cart motion | `ẋc` |
| Parametric mass | Bob mass steps up mid-run | both, via `M` |
| Track tilt | Gravity component along the track, from `t = 0` | `ẋc` |

---

## Quick start

```matlab
>> check_setup          % verify all files present
>> pendulum_launcher    % GUI — does everything
```

The launcher has six tabs (physical setup, fuzzy system, controllers,
simulation, disturbances, run & results). On the last tab: **Run Simulation**,
then **Animate baseline**, **Run disturbance suite**, **Refresh list** →
tick types → **Animate selected** / **Export selected**, then **Build report**.

Script equivalent:

```matlab
>> run_all              % baseline → Pendulum_Control_Run_NNN/
>> run_disturbance      % → <run>/disturbance/<type>/
>> animate_pendulum     % auto-picks the latest run
>> fp_build_report      % → report_data.js, opens report.html
```

### Gain optimisation (optional)

```matlab
>> opt_run              % Nelder-Mead, warm-started from the analytical design
>> opt_visualise        % convergence, Pareto front, gain comparison
>> opt_apply            % → optimised_config.m
```

Then set `use_optimised_gains = true` at the top of `run_all.m` and re-run.

### Publishing

```matlab
>> fp_export_web        % → docs/  (~10 MB, committable)
```

Raw run folders reach hundreds of megabytes and are gitignored.
`fp_export_web` copies the stills, shrinks a curated set of animations to web
size, rewrites the asset paths and writes `docs/` for GitHub Pages.

---

## Output layout

```
Pendulum_Control_Run_007/          gitignored — large
  results.mat  README.txt  run_summary.json
  figures/     <Ctrl>_response.png, _phase_plane.png, _gains.png, _lyapunov.png
  animation/   <Ctrl>_pendulum.gif, <Ctrl>_graphs.gif
  disturbance/
    kick/      results.mat, figures/, animation/
    wind/      ...

docs/                              committed — served by Pages
  index.html  report_data.js  assets/
```

---

## Requirements

**MATLAB R2019b or later.** No toolboxes required — `care()` comes from
Control System Toolbox if available, with a fallback otherwise.

---

## File structure

```
Model
  pendulum_params.m          Presets, derived quantities
  pendulum_dynamics.m        Full nonlinear cart-pendulum ODE
  pendulum_linearise.m       4-state and augmented 5-state linearisation

Fuzzy engine
  fp_membership.m            Triangular / Gaussian / trapezoidal, T1 and T2
  fp_inference.m             Sugeno and Mamdani, min/product
  fp_rule_base.m             kp/kd/ki tables — 5/7/9 sets, 4 presets
  fp_gain_map.m              Raw fuzzy output → bounded PID gains

Controllers
  ctrl_fuzzy_pid.m           Type-1 Sugeno / Mamdani
  ctrl_type2_fuzzy_pid.m     Type-2 with KM type reduction
  ctrl_fuzzy_smc.m           Fuzzy sliding mode
  ctrl_adaptive_fuzzy.m      Online rule adaptation
  ctrl_fuzzy_swingup.m       Hybrid energy + rule-based swing-up
  ctrl_lqr.m                 Augmented LQR with integral position action
  ctrl_classical_pid.m       Fixed-gain baseline
  ctrl_xc_governor.m         Outer position loop (the cascade governor)

Swing-up
  swingup_energy.m           Lyapunov energy law
  swingup_backstepping.m     Two-level cascade
  swingup_sinusoidal.m       Bang-bang baseline

Simulation
  fp_run_analysis.m          RK4 engine, phase logic, governor, checkpointing
  fp_disturbance.m           Five disturbance models
  fp_metrics.m               Angle and position metrics
  fp_lyapunov.m              V(x) and V̇(x) for both phases
  fp_checkpoint.m            Crash-safe resume
  fp_rule_designer.m         Data-driven rule suggestion
  fp_sensitivity.m           MF / rule / parameter sweeps

Optimisation
  opt_cost.m                 Scalar cost from one simulation
  opt_run.m                  Nelder-Mead driver, analytical warm start
  opt_visualise.m            Convergence, Pareto front, gain comparison
  opt_apply.m                Writes optimised_config.m

Output
  fp_export_figures.m        PNG export
  fp_write_summary.m         Console, README.txt, JSON
  fp_build_report.m          Builds report_data.js
  fp_export_web.m            Builds the committable docs/ bundle
  animate_pendulum.m         3D pendulum GIF + 6-panel graphs GIF

Entry points
  pendulum_launcher.m        Six-tab GUI
  run_all.m                  Batch baseline
  run_disturbance.m          Disturbance suite
  check_setup.m              File verification
```

---

## License

MIT. Equations of motion derived via Lagrangian mechanics; Sugeno fuzzy
inference structure based on the original project by M. Nikmand (2026).
