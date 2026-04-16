# On Acquiescence — Whip Control System
## Project context for Claude (VS Code)

---

## What this project is

A 1.5m robotic arm with a metal chain whip, rotating in a horizontal plane. Hardware stack:

- **Mac browser** → HTTP/WebSocket → **Raspberry Pi 4** (headless, on local network)
- **Pi4** → serial (115200 baud, `/dev/ttyACM0`) → **Raspberry Pi Pico W**
- **Pico** → step pulses → **JSS57P2N stepper** (5:1 gearbox, 400 steps/rev output)
- **Motor** → drives a 1.5m counterweighted arm → chain whip dragging on the floor

The chain has significant mass and floor friction. It lags behind the arm and tangles if you reverse direction too quickly. The main design constraint is **avoiding tangles**.

---

## Current file state

### `/home/claude/whip/static/index.html` — Safari-compatible standalone HTML
- Opens directly from Finder (double-click), no server needed
- Enter Pi4 IP to connect for live motor control
- COMPOSE tab: block-based sequence builder (but currently has bugs — see below)
- PERFORM tab: live sliders/buttons (requires Pi4)
- LIBRARY tab: saved sequences
- Visualizer: top-down arm canvas + timeline with tangle risk highlighting
- localStorage wrapped in try/catch for Safari file:// restriction
- Uses `AbortController` instead of `AbortSignal.timeout` (Safari compat)

### `/mnt/user-data/outputs/preview.jsx` — React artifact (in-browser preview)
- Block-based choreographer UI
- Global parameters: speed, angle, rest time, braking, soft-start
- Block types: pendulum, wave, spin, buildup, pause
- Each block inherits globals but can override individually
- Live preview — updates automatically on any change
- **Currently has bugs** — see below

### `/home/claude/whip/server.py` — Pi4 Python server
- HTTP (port 8080) + WebSocket (port 8081) + serial bridge + sequence engine
- Runs sequences autonomously (survives browser disconnect)
- Systemd service (`whip.service`) for boot autostart
- API: `/api/status`, `/api/sequences`, `/api/cmd`, `/api/stop`, etc.

---

## The core problem to solve

The two codebases (index.html and preview.jsx) have diverged and have conflicting approaches. The index.html uses a **linear step list** (explicit `["speed", 2000]`, `["brakeon"]` steps). The preview.jsx uses a **block-based approach** (high-level moves like "pendulum 4×"). These need to be unified.

The right approach is **block-based** because:
1. Users shouldn't have to manually write config steps like speed/braking
2. Global parameters should apply everywhere without repetition
3. Blocks map to actual artistic moves (pendulum swing, wave buildup, spin)

---

## Simulator logic (verified correct)

```js
const PPR = 400, GR = 5.0; // pulses per rev, gear ratio

function moveTime(deg, spd, brk, soft) {
  const steps = (Math.abs(deg) * GR / 360) * PPR;
  if (!steps) return 0;
  let t = steps / Math.max(spd, 1);
  if (soft && steps > 150) t += 0.15; // soft-start ramp
  if (brk  && steps > 150) t += 0.15; // braking ramp
  return t;
}
```

Tangle risk: two consecutive moves in opposite directions with gap < 250ms.

Spin events use linear interpolation; pendulum/wave moves use ease-in-out.

---

## Block types

```
pendulum  — N reps of left/right swings, returns to centre
wave      — starts small+fast, grows to full angle over N reps  
spin      — continuous rotation: CW, CCW, or both (CW then CCW)
buildup   — ramps angle and speed from 0→full over N reps
pause     — hold position for N ms
```

Each block has:
- `reps` — repetitions (pendulum/wave/buildup)
- `revs` — revolution count (spin only)
- `dir` — 1 (CW), -1 (CCW), or "both" (spin only)
- `ms` — pause duration (pause only)
- `angle`, `speed`, `restMs` — optional per-block overrides (null = use global)

---

## Pico commands (serial protocol)

```
on / off          — motor power
stop              — emergency stop
left / right      — move by current angle setting
neutral           — return to zero
cw / ccw          — continuous rotation
speed <sps>       — steps per second (motor shaft)
angle <deg>       — swing angle in degrees
delay <ms>        — pendulum dwell time
brakeon/off       — proportional braking
softstarton/off   — 150-step ramp at start
calibrate         — zero position
```

The block system should compile down to these commands when sending to Pi4.

---

## What needs to be built

A single clean React component (or clean HTML file) that:

1. **Global parameter sliders** — speed, angle, rest time, braking toggle, soft-start toggle
2. **Block list** — add pendulum/wave/spin/buildup/pause blocks, reorder, delete
3. **Per-block controls** — click to expand: reps/revs/direction + optional overrides
4. **Live visualizer** — top-down arm canvas auto-updates on any change
5. **Timeline** — shows angle over time, colour-coded by block, tangle risks in red
6. **Save/load** — JSON export/import, named sequences
7. **Pi4 connection** (HTML version only) — enter IP, enables live PERFORM tab and RUN buttons

The preview.jsx has the right architecture but has bugs that need fixing from scratch with a clear head rather than iterative patches.

---

## Known bugs in current preview.jsx

1. Spin "Both" direction was broken (generates dir:null → NaN position)
2. `addBlock` had stale closure — `setSelIdx(blocks.length)` captured wrong value
3. Layout doesn't fill full viewport in iframe (position:fixed fights with artifact sandbox)
4. The `useEffect` that auto-previews depends on `startAnim` which changes reference each render, causing infinite loops in some cases
5. The HTML and JSX codebases have diverged — HTML still uses old step-list format

---

## Hardware reference

- Motor: JSS57P2N NEMA23 closed-loop stepper
- Gear ratio: 5:1 (400 steps/rev at output shaft)  
- Arm: ~1.5m, counterweighted, chain attaches at tip
- Chain: metal link, ~1–1.5m, drags on floor
- PEND pin: GP5 on Pico (move-complete signal, used for pendulum endpoint detection)
- Soft-start ramp: 150 steps
- Typical speed range: 500–5000 sps (motor shaft), = 15–150 rpm output
