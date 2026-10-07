| Test | Metric | Result | Unit |
|------|--------|--------|------|
| Startup | Rise time (10-90 %) | 47.73 | ms |
| Startup | Overshoot | 0.141 | % |
| Startup | Settling time (2 % band) | 76.94 | ms |
| Startup | Steady-state speed error | -0.065 | rpm |
| Startup | Peak phase current | 4.33 | A |
| Startup | Peak electromagnetic torque | 3.63 | N*m |
| Startup | Steady-state torque | 0.355 | N*m |
| Startup | RMS phase current (steady state) | 0.366 | A |
| Load step | Maximum speed drop | 22.53 | rpm |
| Load step | Time to maximum drop | 4.85 | ms |
| Load step | Recovery time (+/-0.5 % band) | 20.13 | ms |
| Load step | Steady-state speed error after step | 0.012 | rpm |
| Load step | Mean torque before step | 0.350 | N*m |
| Load step | Mean torque after step | 1.36 | N*m |
| Load step | RMS phase current before step | 0.361 | A |
| Load step | RMS phase current after step | 1.36 | A |
| Load step | Effective torque constant after step | 0.835 | N*m/A |
| Reference change | 0 -> 1000 rpm: rise time | 38.01 | ms |
| Reference change | 0 -> 1000 rpm: overshoot | 0.279 | % |
| Reference change | 0 -> 1000 rpm: settling time | 63.31 | ms |
| Reference change | 0 -> 1000 rpm: steady-state error | 0.050 | rpm |
| Reference change | 1000 -> 1500 rpm: rise time | 27.35 | ms |
| Reference change | 1000 -> 1500 rpm: overshoot | 0.444 | % |
| Reference change | 1000 -> 1500 rpm: settling time | 45.64 | ms |
| Reference change | 1000 -> 1500 rpm: steady-state error | 0.025 | rpm |
| Anti-windup | Undershoot below 1000 rpm, without anti-windup | 343 | rpm |
| Anti-windup | Time at duty limit, without anti-windup | 172 | ms |
| Anti-windup | Minimum integrator state, without anti-windup | -0.703 | - |
| Anti-windup | Settling time (2 % of step), without anti-windup | 310 | ms |
| Anti-windup | Undershoot below 1000 rpm, with anti-windup | 205 | rpm |
| Anti-windup | Time at duty limit, with anti-windup | 88.20 | ms |
| Anti-windup | Minimum integrator state, with anti-windup | 0.000 | - |
| Anti-windup | Settling time (2 % of step), with anti-windup | 254 | ms |

**Sanity checks**

- **PASS** Speed reaches the 1500 rpm reference (|error| < 0.5 %) (error = -0.07 rpm)
- **PASS** Rotor turns in the commanded direction (reverse creep at start < 5 rpm) (min speed = -0.99 rpm)
- **PASS** DC-link voltage equals Vdc (polarity check) (mean v_dc = 160.0 V)
- **PASS** Electromagnetic torque follows the load step (within 10 %) (torque increase = 1.005 N*m)
- **PASS** Phase current increases with load (0.361 A -> 1.360 A rms)
- **PASS** Torque per ampere matches Kt = 2*p*lambda within 5 % (correct commutation alignment) (0.835 N*m/A vs 0.840 N*m/A)
- **PASS** Load step causes a temporary dip that the PI removes (dip 22.5 rpm, final error 0.01 rpm)
- **PASS** Anti-windup reduces the undershoot after saturation (343.2 rpm -> 205.3 rpm)
- **PASS** All logged signals are finite (no NaN/Inf) in every run (0 non-finite samples)
- **PASS** Duty command stays within its saturation limits (range 0.000 ... 0.861)
- **PASS** Only legal Hall states (001...110) occur while rotating (285710 samples, 0 illegal)
- **PASS** Hall states advance in the order 5-4-6-2-3-1 (no skipped or reversed steps) (1400 forward transitions, 0 other)
- **PASS** No leg has its upper and lower switch on together (0 of 2900005 gate samples)
- **PASS** At most one upper and one lower switch conduct at a time (0 of 2900005 gate samples)
