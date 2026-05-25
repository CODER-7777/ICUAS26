# Improved Repulsion in `applyRepulsion` — Detailed Walkthrough

This document explains every change made to the swarm-repulsion logic in
`move.cpp`, why each one is there, and what its tuning knobs do.

The changes target the **inter-drone oscillation** that appeared when two
drones approached the same waypoint: they would push each other, drift apart,
get pulled back toward their goals, push again, and so on indefinitely.

The fix is layered — six independent improvements that each address one
contributing factor.

---

## 0. Architecture recap

`applyRepulsion` is called once per drone per control tick (50 ms). It takes
the drone's planned next waypoint (`target`), the drone's current position
(`current`), and a snapshot of the swarm, and returns a *nudged* target that
pushes the drone away from its neighbours.

Three new pieces of persistent state were added at the class level so
information can flow across ticks:

| Member | Purpose |
| --- | --- |
| `prev_poses_for_vel_` | Last tick's pose snapshot, used to finite-difference per-drone velocities. |
| `prev_vel_tick_` | Wall-clock timestamp of the previous control tick, used to scale the finite difference correctly. |
| `repulsion_active_pairs_` | Boolean state per ordered pair of drones (`id_a < id_b`) — the hysteresis bit that remembers whether this pair is currently in the "actively repelling" state. |
| `last_pub_cmd_` | Per-drone last published x/y command, used by the output low-pass filter. |

All of these are updated serially in `publishCommands`, *before* the
`#pragma omp parallel for` section. The parallel loop only reads them and
writes to its own entry in `last_pub_cmd_`. This avoids races on the
underlying `std::map` tree structure.

---

## 1. Relative-velocity term (closing-speed look-ahead)

### Why it matters

The original APF was purely position-based: it only acted when two drones were
*already* inside the safe radius. By that point, both have momentum carrying
them further toward each other, so they overshoot, get repelled hard, and
oscillate.

The fix is to make the field aware of *closing velocity* — so two drones
heading at each other start being pushed apart **before** they collide,
proportional to how fast they are converging.

### Code: per-tick velocity finite-difference

Added in `publishCommands`, right after the pose snapshot:

```cpp
std::map<std::string, geometry_msgs::msg::Vector3> velocities;
{
    rclcpp::Time now_t = this->now();
    double dt = 0.05;  // nominal control-loop period (matches control_timer_)
    if (prev_vel_tick_.nanoseconds() > 0) {
        try {
            double measured = (now_t - prev_vel_tick_).seconds();
            if (measured > 1e-3 && measured < 1.0) dt = measured;
        } catch (const std::exception&) {
            // Clock-source mismatch — fall back to nominal dt
        }
    }
    for (const auto& [id, pose] : current_poses) {
        auto pit = prev_poses_for_vel_.find(id);
        if (pit == prev_poses_for_vel_.end()) continue;     // first tick: no velocity
        geometry_msgs::msg::Vector3 v;
        v.x = (pose.pose.position.x - pit->second.pose.position.x) / dt;
        v.y = (pose.pose.position.y - pit->second.pose.position.y) / dt;
        v.z = (pose.pose.position.z - pit->second.pose.position.z) / dt;
        velocities[id] = v;
    }
    prev_poses_for_vel_ = current_poses;
    prev_vel_tick_      = now_t;
}
```

Word by word:

- `rclcpp::Time now_t = this->now();` — node-clock timestamp for this tick.
- `dt = 0.05` — default to the nominal 50 ms control loop. We use this on the
  very first tick (when `prev_vel_tick_` has never been set) and as a fallback
  if the measured interval is unreasonable.
- `if (prev_vel_tick_.nanoseconds() > 0)` — protects against the
  default-constructed `rclcpp::Time` (zero stamp); we have no usable previous
  tick yet.
- `try { measured = (now_t - prev_vel_tick_).seconds(); }` — `rclcpp::Time`
  subtraction throws if the two `Time` objects come from different clock
  sources (e.g. ROS time vs system time). The `catch` swallows that and falls
  back to nominal `dt`.
- `if (measured > 1e-3 && measured < 1.0) dt = measured;` — sanity clamp.
  Sub-millisecond is unphysical and >1 s means the loop stalled; in either
  case we trust the nominal period more than the noisy measurement.
- The loop builds `velocities[id]` by simple finite difference against the
  last snapshot. Any drone present this tick but not last tick (e.g. just
  registered) is skipped — its velocity is implicitly zero in the consumer.
- `prev_poses_for_vel_ = current_poses;` — at the end, the current snapshot
  becomes the "previous" snapshot for next time.

### Code: using closing speed inside `applyRepulsion`

```cpp
double vel_strength = 0.0;
if (auto it = velocities.find(other_id); it != velocities.end()) {
    const auto& vother = it->second;
    double v_close = (vother.x - vme.x) * rhx + (vother.y - vme.y) * rhy;
    if (v_close > V_CLOSE_CLAMP) v_close = V_CLOSE_CLAMP;
    if (v_close > 0.0) {
        double falloff = 1.0 - dist / V_LOOKAHEAD;
        vel_strength = kv * v_close * falloff;
    }
}
```

- `rhx, rhy` is the unit vector pointing *from the other drone toward me*.
- `(vother - vme) · r_hat` is the closing speed along this direction:
  - Positive → the other drone is moving toward me faster than I'm moving away.
  - Negative → we are diverging; no push needed.
- `V_CLOSE_CLAMP = 2.0 m/s` — sanity clamp. Finite-differenced velocity is
  noisy; without the clamp a single jittery pose can spike the push.
- `falloff = 1 - dist / V_LOOKAHEAD` — smoothly fades the push to zero at the
  look-ahead radius (`safe_radius + hysteresis_margin + 0.3 m ≈ 1.2 m` in
  normal flight). So a slow approach at long range gets only a tiny push;
  a fast approach at medium range gets the full term.
- The position-based and velocity-based contributions are **summed** into
  `strength`, then clipped together.

### Why this is the biggest oscillation killer

Oscillation is fundamentally a feedback delay: forces only kick in after the
drones are already too close. The velocity term **eliminates the delay** — as
soon as the drones start converging, the field starts pushing back, so they
decelerate *before* hitting the hard boundary instead of after.

### Tuning knobs

| Knob | Default | Effect of increasing |
| --- | --- | --- |
| `kv` | 0.35 | Stronger velocity push; reacts harder to fast approaches. |
| `V_LOOKAHEAD` | `release_radius + 0.3` | Starts pushing from further away. |
| `V_CLOSE_CLAMP` | 2.0 m/s | Cap on closing-speed contribution (raises only if your max drone speed is high and you trust the velocity estimate). |

---

## 2. Hysteresis on activation radius

### Why it matters

The original code activated repulsion when `dist < safe_radius` and turned it
off the instant `dist ≥ safe_radius`. A drone hovering right at the boundary
toggles on/off at the 20 Hz control rate, producing a jittery sawtooth in the
command output.

Hysteresis fixes that with a Schmitt-trigger-style band:

- **Activate** when `dist < safe_radius` (0.7 m).
- **Release** only when `dist > safe_radius + REPULSION_HYSTERESIS_MARGIN`
  (0.9 m).
- **Hold previous state** when in the band [0.7, 0.9].

### Code: pair-state update

Persistent state:

```cpp
std::map<std::pair<std::string,std::string>, bool> repulsion_active_pairs_;
```

The key is a *canonical ordered pair* — always `(id_a < id_b)` — so both
perspectives map to the same bit.

Updated once per tick in `publishCommands`, between velocity computation and
the parallel publish loop:

```cpp
const double safe_radius_now = (current_state_ == SwarmState::RETURN_TO_HOME) ? 0.0 : 0.7;
const double release_radius  = safe_radius_now + REPULSION_HYSTERESIS_MARGIN;

std::vector<std::string> ids;
ids.reserve(current_poses.size());
for (const auto& kv : current_poses) ids.push_back(kv.first);

for (size_t a = 0; a < ids.size(); ++a) {
    for (size_t b = a + 1; b < ids.size(); ++b) {
        const auto& pa = current_poses.at(ids[a]).pose.position;
        const auto& pb = current_poses.at(ids[b]).pose.position;
        double d = std::hypot(pa.x - pb.x, pa.y - pb.y);

        auto key = std::make_pair(ids[a], ids[b]);
        auto it  = repulsion_active_pairs_.find(key);
        bool prev = (it != repulsion_active_pairs_.end()) ? it->second : false;

        bool now_active = prev;
        if (d < safe_radius_now)        now_active = true;
        else if (d > release_radius)    now_active = false;
        // else: hold previous state (the hysteresis band)

        repulsion_active_pairs_[key] = now_active;
        if (now_active) {
            active_peers_by_drone[ids[a]].insert(ids[b]);
            active_peers_by_drone[ids[b]].insert(ids[a]);
        }
    }
}
```

- `ids` is built once so the nested loop iterates by index — `a < b` guarantees
  canonical pair ordering without sorting strings.
- `prev` defaults to `false` if this pair has never been seen.
- The three-way branch encodes the Schmitt trigger.
- `active_peers_by_drone[id]` is a per-drone `std::unordered_set` of
  neighbours currently in the active state, built once and passed into
  `applyRepulsion` for fast `O(1)` membership tests in the inner loop.

### Code: using hysteresis inside `applyRepulsion`

The position-based APF magnitude formula was changed from
`gain * (1/dist - 1/safe_radius)` to use `release_radius` instead:

```cpp
double pos_strength = 0.0;
if (active_peers.count(other_id) && dist < release_radius) {
    pos_strength = gain * (1.0 / dist - 1.0 / release_radius);
    if (pos_strength < 0.0) pos_strength = 0.0;
}
```

Three details:

1. The `active_peers.count(other_id)` gate is the hysteresis check itself —
   the formula only fires when the pair is in the "active" state.
2. The reference radius for the APF formula is the **release** radius, not
   the activation radius. Why: when a pair is active and the drones drift
   slightly past `safe_radius`, the original formula went *negative*
   (attractive!). Using `release_radius` makes the formula naturally taper to
   zero at the release boundary, with the safety clamp `if (pos_strength < 0)
   pos_strength = 0;` as a belt-and-suspenders guard.
3. `dist < release_radius` is a cheap early-out so we never compute the
   division when the pair is already separated past the release point.

### Tuning knobs

| Knob | Default | Effect of increasing |
| --- | --- | --- |
| `REPULSION_HYSTERESIS_MARGIN` | 0.2 m | Wider deadband. Repulsion stays on longer after activation; less chatter but slower release. |

---

## 3. Tangential bias

### Why it matters

A pure radial repulsion on two drones heading directly at each other gives a
push along the line connecting them — *anti-parallel to their motion*. They
decelerate but don't sidestep, so they end up "magnetised" head-to-head and
the system stalls.

A tangential component (perpendicular to the line of approach) makes them
slip past each other on opposite sides.

### Code

```cpp
const double K_TAN = 0.35;  // Tangential blend factor

// Tangential bias: rotate r_hat 90° CCW.
double tx = -rhy;
double ty =  rhx;

dx_rep += strength * ((1.0 - K_TAN) * rhx + K_TAN * tx);
dy_rep += strength * ((1.0 - K_TAN) * rhy + K_TAN * ty);
```

- `(tx, ty) = (-rhy, rhx)` is `r_hat` rotated 90° counter-clockwise (standard
  2D rotation `R(90°) = [[0,-1],[1,0]]`).
- The final push is a weighted blend: 65% radial, 35% tangential.

### Why no per-drone sign flip is needed

A common mistake is to think both drones need to *agree* on which way to go.
They don't, because each drone computes `r_hat` from *its own* perspective:

- Drone A sees B: `r_hat_A = (A - B) / dist`.
- Drone B sees A: `r_hat_B = (B - A) / dist = -r_hat_A`.

If both apply the *same* rotation (90° CCW) to *their own* `r_hat`, their
world-space tangents are also opposites:

```
rot90(r_hat_A)  =  (-r_hat_A.y,  r_hat_A.x)
rot90(r_hat_B)  =  (-r_hat_B.y,  r_hat_B.x)
              =  ( r_hat_A.y, -r_hat_A.x)
              = -rot90(r_hat_A)
```

So A pushes itself "left" and B pushes itself "right" (or vice versa) — they
slip past each other automatically, no negotiation needed.

This is exactly the "right-hand rule" passing convention used by ships and
aircraft.

### Tuning knobs

| Knob | Default | Effect of increasing |
| --- | --- | --- |
| `K_TAN` | 0.35 | More sideways slip, less head-on braking. Push it too high (≥0.7) and drones loop around each other instead of separating. |

---

## 4. Asymmetric priority

### Why it matters

If both drones in a conflict apply the *same* amount of avoidance, they
over-correct — A shifts left, B shifts right, both shift more than necessary,
they overshoot in opposite directions, then have to swing back. This is the
symmetric component of the oscillation that the velocity term alone can't fix.

The remedy is to make one drone yield more than the other. Then the conflict
resolves with one drone doing 75–80% of the avoidance work and the other only
making small corrections.

### Code

```cpp
const double YIELD_LOW  = 0.3;  // Lower-priority drone yields less
const double YIELD_HIGH = 1.0;  // Higher-priority drone does most of the avoidance

double yield = (my_id < other_id) ? YIELD_LOW : YIELD_HIGH;
strength *= yield;
```

- The tie-break is the drone ID's lexicographic ordering. It's deterministic,
  cheap, and requires no communication.
- "Lower ID = right of way" is an arbitrary convention; what matters is that
  both drones reach *opposite* conclusions about who yields.

When A (`cf_1`) and B (`cf_2`) meet:
- A's perspective: `"cf_1" < "cf_2"` → A multiplies its push by `YIELD_LOW` (0.3).
- B's perspective: `"cf_2" < "cf_1"` is false → B multiplies its push by `YIELD_HIGH` (1.0).

Net: A barely budges, B does most of the dodging, and the symmetric loop is
broken.

### Tuning knobs

| Knob | Default | Effect of decreasing `YIELD_LOW` (toward 0) |
| --- | --- | --- |
| `YIELD_LOW` | 0.3 | More extreme asymmetry. At 0, the low-priority drone holds its course completely; at 1, no asymmetry. |
| `YIELD_HIGH` | 1.0 | Standard full push. Could be raised >1 to overcompensate, but rarely useful. |

### Future enhancement

If you want priority to be dynamic instead of ID-based — e.g. yield based on
battery, distance-to-goal, or current speed — replace the comparison with
whatever boolean expression you want. The architecture doesn't change.

---

## 5. Low-pass filter (LPF) on the output command

### Why it matters

Even with everything above, the *commanded* setpoint can still jump tick to
tick whenever the planner re-emits a new waypoint or the repulsion crosses a
threshold. The Crazyflie's onboard tracking controller will chase those
jumps, producing visible wobble.

A first-order IIR filter on the x/y command smooths out the high-frequency
content while preserving the underlying trajectory.

### Code: persistent state

```cpp
struct LpfState { bool valid = false; double x = 0.0; double y = 0.0; };
std::map<std::string, LpfState> last_pub_cmd_;
```

### Code: pre-populate before the parallel loop

```cpp
for (const auto& item : batch_buffer) {
    last_pub_cmd_.try_emplace(item.id);
}
```

This is **important for thread safety**. `std::map::operator[]` and `insert`
restructure the underlying red-black tree, which is not safe under
`#pragma omp parallel for`. By calling `try_emplace` serially first, we
guarantee that every key the parallel loop will touch already exists —
the loop then only modifies values, never the tree structure. libstdc++'s
`std::map` allows concurrent modification of values at *distinct existing
keys* without contention.

### Code: filter application

```cpp
{
    const double ALPHA = 0.5;       // history weight
    const double JUMP_RESET = 0.5;  // m — beyond this, bypass filter
    auto& st = last_pub_cmd_.at(item.id);
    if (st.valid && std::hypot(safe_cmd.x - st.x, safe_cmd.y - st.y) < JUMP_RESET) {
        safe_cmd.x = (1.0 - ALPHA) * safe_cmd.x + ALPHA * st.x;
        safe_cmd.y = (1.0 - ALPHA) * safe_cmd.y + ALPHA * st.y;
    }
    st.x = safe_cmd.x;
    st.y = safe_cmd.y;
    st.valid = true;
}
```

- `ALPHA = 0.5` — equal weight on history and current input. Time constant of
  the filter is roughly `dt / (1 - ALPHA) = 0.05 / 0.5 = 0.1 s`, so it damps
  noise above ~10 Hz.
- `JUMP_RESET = 0.5 m` — if the new command differs from the last by more
  than 50 cm, we *bypass* the filter entirely for that tick. This prevents
  the filter from dragging the drone toward a stale setpoint when:
  - A new path segment kicks in (planner replan).
  - RTH begins and the target changes from "mission waypoint" to "home".
  - The drone is teleported (sim restart).
- `st.valid` defaults to `false`, so the very first command for each drone is
  published raw — no filter lag on takeoff.
- Only `x` and `y` are smoothed. `z` is left raw because altitude is
  separately commanded by the planner and we don't want vertical lag during
  takeoff/landing. `yaw` is also untouched because it's angular and would
  need an angle-aware blend (atan2 of sin/cos), which isn't worth the
  complexity for this use case.

### Tuning knobs

| Knob | Default | Effect of increasing |
| --- | --- | --- |
| `ALPHA` | 0.5 | Heavier smoothing; less jitter but more lag. ≥0.8 starts to feel sluggish; ≤0.2 has little effect. |
| `JUMP_RESET` | 0.5 m | Filter bypass threshold. Raise it if your planner emits frequent small jumps you want smoothed; lower it if the filter is masking real maneuvers. |

---

## 6. Staleness guard

### Why it matters

`swarm_poses_` is populated by per-drone pose subscriptions. If a drone's
publisher dies, drops out, or its rosbag pauses, the entry sits in the map
forever as a *phantom* — a stationary fake drone that still generates
repulsion fields and pushes live drones around for no reason.

### Code

Inside the neighbour loop of `applyRepulsion`:

```cpp
const double STALE_TIMEOUT = 0.5;  // seconds
const rclcpp::Time now_t = this->now();

for (const auto& [other_id, other_pose] : all_poses) {
    if (other_id == my_id) continue;

    const rclcpp::Time stamp(other_pose.header.stamp);
    if (stamp.nanoseconds() > 0) {
        try {
            if ((now_t - stamp).seconds() > STALE_TIMEOUT) continue;
        } catch (const std::exception&) {
            // Clock-source mismatch — fall through and trust the pose
        }
    }
    ...
}
```

Word by word:

- `stamp.nanoseconds() > 0` — `header.stamp` is zero-default. If the
  publisher never set it, treating "now − 0 = a huge number" as stale would
  exclude every pose. So we only check staleness when the stamp is non-zero;
  poses without a stamp are trusted.
- `rclcpp::Time` subtraction throws on clock-source mismatch (ROS time vs
  system time). The `try/catch` makes the guard *fail-open*: if we can't tell
  how old the stamp is, we trust the pose rather than excluding it. This
  avoids the worst-case of "all neighbours invisible because of a clock
  config error".
- `STALE_TIMEOUT = 0.5 s` — at the simulator's typical 30–100 Hz pose
  publication, this allows ~15–50 missed messages before declaring a
  drone stale. That's tolerant to brief network hiccups but quick enough to
  drop a genuinely dead publisher within half a second.

### Tuning knobs

| Knob | Default | Effect of decreasing |
| --- | --- | --- |
| `STALE_TIMEOUT` | 0.5 s | Faster drop-off of dead publishers, but more sensitive to network hiccups. |

---

## Master tuning cheat sheet

| Symptom | First knob to try | Direction |
| --- | --- | --- |
| Drones still oscillate around each other | `kv` (velocity gain) | Increase |
| Drones stop too far apart / mission slow | `safe_radius` (in code) | Decrease |
| Edge chatter in/out of avoidance | `REPULSION_HYSTERESIS_MARGIN` | Increase |
| Head-on stall (drones brake but don't sidestep) | `K_TAN` | Increase |
| Both drones still over-correct | `YIELD_LOW` | Decrease (toward 0) |
| Visible command jitter on the wire | `ALPHA` | Increase (toward 0.7) |
| Drone keeps "snapping" between setpoints | `JUMP_RESET` | Increase |
| Phantom drone pushing live ones | `STALE_TIMEOUT` | Decrease |

---

## Files touched

- `src/graph_traversal/src/move.cpp`
  - **Class members** (~lines 179–187): added `prev_poses_for_vel_`,
    `prev_vel_tick_`, `repulsion_active_pairs_`, `LpfState`, `last_pub_cmd_`.
  - **`applyRepulsion`** (~lines 1277–1420): new signature accepting
    `velocities` and `active_peers`; added velocity term, hysteresis gate,
    tangential bias, yield weighting, staleness guard.
  - **`publishCommands`** (~lines 1373–1560): added per-tick velocity
    computation, per-pair hysteresis update, pre-populated LPF state, LPF
    application before publish.
- `src/graph_traversal/src/improved_repulsion.md` — this document.

No CMake or package.xml changes were needed; the only new header is
`<geometry_msgs/msg/vector3.hpp>`, which `geometry_msgs` already provides.

---

## What this does *not* do

Out of scope but worth knowing about for future work:

- **No obstacle-awareness in the nudge.** The shifted target isn't checked
  against `cached_grid_`. In tight rooms, repulsion can push a drone into a
  wall; the planner will catch it on the next plan tick but a same-tick
  validation would be safer.
- **No 3D push.** Repulsion is still XY-only. Within the Z-gate band, two
  drones at slightly different altitudes still try to dodge horizontally
  instead of vertically.
- **No tuning via ROS parameters.** Every knob is a `const double` inside the
  function. Promote them to `declare_parameter` calls if you want runtime
  tuning.
- **No telemetry.** When repulsion fires, nothing is logged. A
  throttled `RCLCPP_DEBUG` showing peer ID, distance, and applied strength
  would make tuning much faster — currently you have to read intent from
  rqt_plot of the published positions.
