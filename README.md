# Automatic Reverse Parking System for Software Defined Vehicles
**Using Virtual Distance Sensors — SENSE → ANALYZE → DECIDE → CONTROL → PARK**

A 100% software, in-browser simulation (HTML + CSS + JavaScript). No Arduino, ESP32, Raspberry Pi, camera or physical sensors.

## Run it
Open `index.html` in any modern browser. No build step, no server, no internet needed.

## Objective and problem statement
Parallel reverse parking is error-prone: drivers misjudge the slot length, steering timing and the distance to neighbouring cars. The objective is a software-defined control loop that measures a slot, decides whether the car fits, and parks it autonomously while monitoring distance sensors for every moment of the manoeuvre.

## SDV relevance
The behaviour is entirely software: sensing, planning, control and safety are separate software modules that read virtual signals and write actuator commands (gear, speed, steering). Changing the vehicle size, safety margin or sensor layout changes behaviour by editing configuration (`CFG`), not hardware, which is the core SDV idea.

## Architecture (`script.js`)
| Module | Role |
|---|---|
| `CFG` | Vehicle size, steering limits, margins, slot sizes |
| `World` / geometry | Parked cars, curbs, dynamic obstacles; ray and hull intersection maths |
| `Vehicle` | Kinematic bicycle model, rate-limited steering, gear shifting, acceleration limits |
| `Sensors` | Four ray-cast fan sensors (FL, FR, RL, RR) |
| `Slot` | Side-scan slot measurement, suitability check, safe boundary |
| `Safety` | Collision envelope and emergency-stop logic |
| `Ctl` | State machine and steering control |
| `Manual` | Keyboard / on-screen driving |
| `App` + view | Fixed-step simulation loop, canvas rendering, dashboard |

## Four virtual sensors
Each sensor is a fan of five rays cast from a bumper corner against the obstacle rectangles; the shortest hit is the reading (max 250 cm). Front sensors angle forward, rear sensors backward, both slightly outward. Colours: **green** ≥ 60 cm, **yellow** 25–60 cm, **red** < 25 cm. A separate left-facing side-scan ray measures the slot.

## Slot detection and safe boundary
While driving past the row at 5 km/h the side-scan ray reads a short distance beside a parked car and a long one across a gap. The odometer positions at those two transitions give the **measured slot length**.

`Required length = vehicle length + 2 × safety margin = 450 + 2 × 50 = 550 cm`

If measured ≥ required the slot is *Suitable*, otherwise *Slot Too Small* and the manoeuvre aborts. The **safe boundary** is the slot inset by 10 cm at both ends; the final pose is placed inside it and drawn as a dashed rectangle.

## Parking algorithm and state machine
`SEARCHING → SPACE_DETECTED → POSITIONING → REVERSE → TURNING → ALIGNING → PARKED`, with `EMERGENCY_STOP` reachable from any moving state.

* **POSITIONING** — from the measured lateral offset *d* and the minimum turning radius *R = wheelbase / tan(max steer)*, the planner computes the arc angle `θ = acos(1 − d / 2R)` and therefore the stopping point from which two full-lock arcs end at the target.
* **REVERSE** — full steering lock toward the bay. It hands over when the remaining lateral error equals what one opposite full-lock arc can still deliver (braking distance included).
* **TURNING** — opposite lock; the car straightens as it settles into the bay.
* **ALIGNING** — heading/lateral hold while centring fore-aft inside the safe boundary.

The car is never moved to a destination: every position change comes from the bicycle model. Steering moves at a limited rate, the car stops before changing gear, and it steers first and rolls after, as a real car would.

## Collision detection and emergency stop
The car hull (rotated rectangle) is tested against every obstacle each step. Emergency stop triggers when a sensor in the direction of travel reads under 10 cm or the hull comes within 4 cm of anything. Warning shows within 25 cm. In manual mode the car cannot be driven through an obstacle.

## Demonstrations
1. **Standard parking** — choose *650 cm · medium* and press **START AUTOMATIC PARKING**.
2. **Tight slot** — choose *600 cm · tight*. It parks with about 10 cm minimum clearance. Slots under about 595 cm pass the 550 cm requirement but exceed what a single two-arc manoeuvre can do, so the envelope guard stops the car safely rather than allowing contact.
3. **Slot too small** — choose *520 cm*; the system rejects it.
4. **Emergency** — press **EMERGENCY DEMO**: an obstacle appears behind the car during reverse and the rear-left sensor triggers EMERGENCY STOP. You can also start parking and press **ADD OBSTACLE BEHIND CAR** at any time.
5. **Manual drive** — press **MANUAL DRIVE**, then ↑/W, ↓/S, ←/→ or A/D, Space to brake.

## Technologies
HTML5, CSS3 (grid, custom properties), JavaScript (ES2020), Canvas 2D.

## Future enhancements
Multi-point shuffle manoeuvres for slots under 595 cm, perpendicular and angled bays, moving pedestrians, noisy sensor models, a speed multiplier, and exporting telemetry as CSV.
