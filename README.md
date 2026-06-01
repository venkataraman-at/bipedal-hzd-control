# Bipedal Locomotion via Hybrid Zero Dynamics

**EECE 7398 — Legged Robots | Northeastern University**  
Athmavidya Venkataraman, Guru Vignesh Madhavan, Anushka Pradeep Chauhan, Amir Shahzad

---

## Overview

Complete modeling, gait optimization, and control implementation for a **planar three-link bipedal robot** using the **Hybrid Zero Dynamics (HZD)** framework. The robot is modeled as a kinematic chain with four point masses (stance leg, swing leg, hip, torso) and three degrees of freedom — two actuated joints and one unactuated cyclic variable governing stance-leg progression.

The pipeline covers:
- Euler-Lagrange equations of motion in control-affine state-space form
- Instantaneous rigid-body impact map for foot-strike velocity discontinuities
- Gait design via 4th-order Bézier polynomial virtual constraints
- Constrained optimization using MATLAB `fmincon` with periodicity constraint
- Output feedback linearization with PD compensation to enforce virtual constraints
- Poincaré section analysis for orbital stability assessment

---

## System Model

| Parameter | Value | Description |
|-----------|-------|-------------|
| m₁ = m₂ | 5 kg | Mass of each leg |
| Mₕ | 15 kg | Mass of hip |
| Mₜ | 10 kg | Mass of torso |
| r | 1 m | Leg length |
| l | 0.5 m | Torso length |
| g | 9.81 m/s² | Gravitational acceleration |
| **Total** | **35 kg** | System mass |

**Generalized coordinates:** `q = [q1, q2, q3]ᵀ`  
- `q1` — stance leg angle (unactuated cyclic variable)  
- `q2` — swing leg angle relative to stance leg (actuated)  
- `q3` — torso angle relative to stance leg (actuated)

**Equations of motion:**
```
D(q)q̈ + C(q,q̇)q̇ + G(q) = Bu
```

---

## Gait Design

Virtual holonomic constraints are imposed on the actuated joints:
```
y = h(q,s) = qb - b(s),    qb = [q2, q3]ᵀ
```

Reference trajectories `b(s)` are parameterized as **4th-order Bézier polynomials** in the gait timing variable `s(q1) ∈ [0,1]`. Bézier polynomials are chosen for their C∞ continuity, direct boundary-condition enforcement, and suitability for gradient-based optimization.

**Optimization (MATLAB `fmincon`):**
```
min  J = Σ‖u(t)‖²    s.t.  z(tf) = z(t0)
```

Optimized parameter vector:
```
f* = [-0.4448, -2.3191, 0.1745, 0.7588, 0.6279, 2.7925, 2.8580, 3.0543]
```

---

## Controller

Output feedback linearization with PD virtual input drives `y → 0` exponentially:
```
u = (LgLfh)⁻¹(v - L²fh)
v = -Kp·y - Kd·ẏ
```

Gains: `Kp = diag(5000, 5000)`, `Kd = diag(100, 100)`

The controller successfully drives actuated joints onto their Bézier references throughout the simulation, sustaining forward locomotion across 10 steps even without a perfectly periodic orbit.

---

## Results

- Robot sustains **forward locomotion over 10 simulated steps**
- Actuated joints `q2`, `q3` closely track Bézier reference trajectories
- Phase portrait shows progressive loop drift — optimizer converged to an infeasible point, periodicity constraint `z(tf) = z(t0)` not fully satisfied
- Mechanical Cost of Transport (MCOT) computed per step using normalized energy expenditure

---

## Repository Structure

```
├── autogen/                  # Auto-generated symbolic dynamics functions
│   ├── func_compute_D_C_G_B.m       # Inertia, Coriolis, gravity matrices
│   ├── func_compute_De_E_dY_dq.m    # Extended inertia + impact Jacobian
│   ├── func_compute_beta1.m         # Zero dynamics coefficient
│   ├── func_compute_dLfh.m          # Second-order Lie derivative
│   └── func_compute_pMh_pMt_...m    # Forward kinematics
│
├── util/
│   ├── bezier.m                     # Bézier polynomial evaluation
│   ├── d_ds_bezier.m                # Bézier derivative w.r.t. s
│   ├── func_gait_timing.m           # Gait timing variable s(q1)
│   ├── func_map_z_x.m               # Zero dynamics ↔ full state map
│   └── func_model_params.m          # System parameters
│
├── func_feedback.m           # Feedback linearizing controller
├── func_zero_dynamics.m      # Zero dynamics ODE
├── func_impact_map.m         # Rigid-body impact map
├── func_full_dynamics.m      # Full hybrid system dynamics
├── func_compute_control_action.m
├── Optimize.m                # Bézier gait optimization (fmincon)
├── sim_and_plot_ZD.m         # Simulate + plot zero dynamics
├── sim_and_plot_full_dynamics.m  # Simulate + plot full system
├── animate_results.m         # Animate bipedal gait
├── poincare_analysis.m       # Poincaré section / orbital stability
└── plot_trajectories.m       # Joint angle/velocity plots
```

---

## Running the Simulation

**Requirements:** MATLAB (tested on R2023a+)

```matlab
% 1. Set up path
run('set_path.m')

% 2. Run gait optimization
run('Optimize.m')

% 3. Simulate full dynamics
run('sim_and_plot_full_dynamics.m')

% 4. Animate the gait
run('animate_results.m')

% 5. Poincaré analysis
run('poincare_analysis.m')
```

---

## References

1. Westervelt, Grizzle, Koditschek — *Hybrid Zero Dynamics of Planar Biped Walkers*, IEEE TAC 2003
2. Ramezani et al. — *Performance Analysis and Feedback Control of ATRIAS*, JDSC 2013
3. Reher, Ma, Ames — *Dynamic Walking with Compliance on Cassie*, ICRA 2019
4. Slotine & Li — *Applied Nonlinear Control*, Prentice-Hall 1991

---

## Course

EECE 7398 — Legged Robots | Northeastern University | Spring 2025  
Instructor: Prof. Alireza Ramezani
