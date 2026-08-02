# cart_api — Python control library for the FollowRTK golf cart

A small, organized Python library to **control** and **read** the self-driving
golf cart: RTK GPS, steering, gas, and brake — through one simple interface.

```python
from cartlib import Cart

with Cart() as cart:
    cart.arm()                       # GPS streaming; pedals armed (out of failsafe)
    print(cart.snapshot())           # live gps + pedals + steering state

    cart.steering.enable()           # energize steering motor
    cart.steering.set_angle(15)      # +15° at the steering column
    cart.pedals.set_brake(0.2)       # engage brake (safe)
    cart.stop()                      # release pedals
```

## Hardware map

| Subsystem | Device                         | Port (auto-detected by-id) | Protocol                          |
|-----------|--------------------------------|----------------------------|-----------------------------------|
| RTK GPS   | u-blox ZED-F9x GNSS            | `/dev/ttyACM0`             | NMEA (GGA) read; NTRIP for RTK    |
| Steering  | ODrive S1 + M8325s (3:1 belt)  | `/dev/ttyACM1`             | ODrive **ASCII** over USB-CDC     |
| Gas+Brake | Arduino Mega 2560 (2 actuators)| `/dev/ttyACM2`             | `pedal_control.ino` G/B/S/H/D     |

Ports are resolved from the stable `/dev/serial/by-id/` symlinks, so the
library keeps working even if the `ttyACM` numbers shuffle on reboot.

## Install

The only dependency is **pyserial** (already on the Jetson). No `odrive`
package needed — steering uses the ODrive ASCII protocol directly.

```bash
pip install -r requirements.txt   # pyserial>=3.5
```

## Layout

```
cart_api/
├── cartlib/                 the library
│   ├── config.py            ports, baud rates, limits (single source of truth)
│   ├── gps.py               GpsReceiver — read RTK GPS fixes
│   ├── ntrip.py             NtripClient — optional RTK corrections feeder
│   ├── pedals.py            PedalController — gas + brake (+ heartbeat watchdog)
│   ├── steering.py          SteeringController — ODrive S1 steering
│   ├── cart.py              Cart — unifies all three
│   └── percept/             pedestrian & vehicle speed governor
│       ├── calib.py         loads calibration/front_camera.json
│       ├── camera.py        threaded V4L2 grabber (newest frame only)
│       ├── detector.py      YOLO wrapper
│       ├── geometry.py      pixels -> metres (ground contact point)
│       ├── track.py         Kalman tracks + constant-velocity forecast
│       ├── policy.py        the safety core: one number, v_allowed_mph
│       ├── telemetry.py     minimap payloads
│       └── service.py       the live loop
├── calibration/
│   └── front_camera.json    intrinsics (from PRODUCTION) + mount geometry
├── examples/
│   ├── read_all.py          live read-only dashboard
│   └── actuation_demo.py    opt-in brake / steer / gas demos (MOVES HARDWARE)
├── selftest.py              read-only verification of all 3 subsystems
└── requirements.txt
```

## Verify the hardware (no motion)

```bash
python3 selftest.py
```

Opens each device and reads live state — GPS fix, gas/brake pot positions,
e-stop/failsafe flags, steering angle, ODrive bus voltage. Exit code `0` =
everything detected and readable. Last run on the cart:

```
[GPS]      fix=GPS sats=12 hdop=0.54  lat=37.426562 lon=-122.164102      PASS
[PEDALS]   gas_pot=0.007 brake_pot=0.009  failsafe=True estop=False      PASS
[STEERING] bus_voltage=47.5 V  angle=-0.01°  state=IDLE  errors=0        PASS
```

## Perception (pedestrian & vehicle speed governor)

The camera never steers and never touches a pedal. It produces exactly one
number — `v_allowed_mph` — and the follower takes `min(its own target, that)`.
Everything else is diagnostics.

Order of operations, and none of it can be skipped:

**1. Measure the mount.** The intrinsics come from PRODUCTION's ChArUco run
(0.17 px RMS), but no calibration board can tell you how high the lens is or
how far it tilts down, and every range comes from those two numbers. Park on
flat ground, put markers straight ahead at known distances (measured from the
ground directly *below the lens*), spanning near to far — 3 m and 25 m, not
5 m and 8 m:

```bash
python3 tools/percept_ground_calib.py --fit-focal     # look at the fit
python3 tools/percept_ground_calib.py --fit-focal --write
```

Until this is done, `--percept-live` refuses to arm.

**2. Watch it, parked.** Nothing is opened but GPS and the camera — this tool
cannot move the cart:

```bash
python3 tools/percept_live.py --no-gps --print
# UI: NEXT_PUBLIC_GPS_WS_URL=ws://localhost:8766
```

Check ranges against a tape measure, and that a standing person holds still on
the minimap instead of drifting.

**3. Shadow mode, driving.** Real drives, real decisions, minimap live, but the
speed cap is reported and ignored:

```bash
python3 -m cartlib.server --percept
```

**4. Live.** Perception may now slow and stop the cart:

```bash
python3 -m cartlib.server --percept-live
```

Behaviour at each layer, in the order they override each other (lowest wins):
L0 reflex stops for anything confirmed inside `reflex_range_m`; L1 holds an RSS
speed envelope against every tracked object's forecast corridor; L2 drops to
`degraded_speed_mph` when the detector, the frame age or the ranging cues stop
agreeing. `min` of the three, then a jerk limiter, then the follower.

Offline, without a camera:

```bash
python3 tools/percept_demo.py --list        # replay simulated scenarios
python3 -m pytest tests/ -q                 # 231 tests incl. closed-loop sim
```

## API reference

### `GpsReceiver` (`cartlib.gps`)
- `open()` / `close()` (or use as a context manager)
- `.latest` → `{lat, lon, fix_type, fix_code, sats, hdop, alt, ts}`
- `.wait_for_fix(timeout=10)`

### `NtripClient` (`cartlib.ntrip`)
- `NtripClient(gps).start()` / `.stop()` — feeds RTCM corrections to the
  receiver so it can reach **RTK Fixed**. **Note:** this opens an *outbound*
  connection to an NTRIP caster and sends the cart's position; it's opt-in.

### `PedalController` (`cartlib.pedals`)
- `arm()` — leave failsafe (starts the 20 Hz heartbeat keeping the Mega armed)
- `set_gas(v)` — `0..gas_cap` (**drives the cart**; default cap = `FSD_GAS_LIMIT` 0.25)
- `set_brake(v)` — `0..0.45` (always safe — only stops the cart)
- `stop()` — release both pedals · `disarm()` — graceful shutdown
- `.telemetry` → `{gas, brake, gas_target, brake_target, heartbeat_ms, failsafe, estop}`

### `SteeringController` (`cartlib.steering`)
- `enable()` — closed-loop, motor energized · `idle()` — de-energize
- `set_angle(deg)` — steering-column angle relative to connect() (clamped to ±90° default)
- `.angle_deg()`, `.bus_voltage()`, `.status()`, `clear_errors()`

### `Cart` (`cartlib.cart`)
- `arm()`, `stop()`, `emergency_brake()`, `snapshot()`
- `.gps`, `.pedals`, `.steering` — the subsystem objects

## Safety notes

- The Arduino boots in **FAILSAFE** and re-trips it if the host heartbeat
  stops for >300 ms (gas released, **brake slammed on**). `PedalController`
  runs the heartbeat for you while armed; closing it sends a graceful disarm.
- A hardware **e-stop** forces full brake / zero gas at the firmware level and
  surfaces as `telemetry["estop"]`.
- Gas is governed by a layered cap hierarchy (`config.effective_gas_cap`):
  hardware `GAS_POT_MAX` 0.68 → `GLOBAL_SPEED_LIMIT` 0.45 → mode cap.
- `examples/actuation_demo.py` moves real hardware and is fully opt-in; the
  `--gas` demo additionally requires `--i-understand-this-drives`.
- Limits in `config.py` mirror the production firmware
  (`PRODUCTION/limits.py`, `sketches/common/cart_limits.h`). Keep them in sync.
```
