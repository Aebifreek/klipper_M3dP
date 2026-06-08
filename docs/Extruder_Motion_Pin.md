# Extruder Motion Pin

This document describes the `extrude_motion_pin` feature added to the
extruder module (`klippy/kinematics/extruder.py`).  It allows a digital
output pin to follow the extruder's extrusion state: HIGH while filament
is actively being pushed forward, LOW when the extruder is idle or
retracting.  A common use-case is signalling a filament-sensor, a relay,
or an external controller that extrusion is in progress.
Also possible is to completly change from the ExtruderStepper to the extrude_motion_pin by not set dir_pin, step_pin or rotation_distance.

## What has changed in extruder.py

### `AsyncGPIOController` class

A new background-thread GPIO controller was added to handle the case
where the motion pin is a Raspberry Pi BCM GPIO (e.g. `gpio17`).

MCU-scheduled commands are not suitable for host-side GPIOs because the
Klipper reactor is single-threaded and the MCU command pipeline would
have to carry a host GPIO write as a timed event.  Instead,
`AsyncGPIOController` runs a dedicated worker thread that:

1. Receives `(gpio_num, value, print_time)` tuples from the reactor thread
   via a thread-safe queue.
2. Converts `print_time` to a wall-clock monotonic time using
   `mcu.estimated_print_time()`.
3. Sleeps until that wall time and then calls `RPi.GPIO.output()`.

Two safety mechanisms are built in:

| Mechanism | Default | Purpose |
|---|---|---|
| **Debounce** | 200 ms | A `1→0` followed by a `0→1` on the same pin within 200 ms cancels both events, suppressing brief LOW glitches between consecutive extrusion moves. |
| **Minimum toggle interval** | 50 ms | Any two consecutive state changes on the same pin are guaranteed to be at least 50 ms apart, preventing `Timer too close` errors on single-core hosts such as a Raspberry Pi Zero. |

If `RPi.GPIO` is not installed the controller falls back to a simulated
mode and logs every state change — useful for testing on a non-Pi host.

### `PrinterExtruder` changes

The following new config keys were added to `PrinterExtruder`:

| Config key | Default | Description |
|---|---|---|
| `extrude_motion_pin` | *(none)* | Pin name or BCM GPIO descriptor. If omitted the feature is disabled. |
| `extrude_motion_pin_off_delay` | `mcu.min_schedule_time()` | Seconds to subtract from the move start time when setting an MCU digital-out pin HIGH (not used for RPi GPIO). |
| `extrude_motion_pin_idle_delay` | `0.050` | Wall-clock seconds of no positive-extrusion `process_move` calls before the idle timer fires LOW. |
| `extrude_motion_pin_low_print_gap` | `0.500` | Minimum gap (in print-time seconds) between the end of the last extrusion move and the toolhead's current print_time before the idle timer commits a LOW. Prevents false LOW signals during lookahead batch gaps. |

**Pin selection logic:**

* If the pin name matches the pattern `gpio<N>` (optionally prefixed with
  a port like `rpi:gpio17` or with an invert flag `!gpio17`) the BCM GPIO
  path is taken and `AsyncGPIOController` is used.
* Any other pin name is treated as a normal MCU digital output and goes
  through Klipper's standard `pins` / `digital_out` path.

**State machine:**

* `process_move` is called for every queued move.  When a positive
  extrusion move is detected (`axis_d > 0`) the pin is driven HIGH (once)
  and a reactor timer is armed for `_pin_idle_threshold` seconds.
* The reactor timer re-checks two conditions before sending LOW:
  1. Wall-time idle ≥ `extrude_motion_pin_idle_delay`
  2. `toolhead.print_time − last_extrude_end_print_time` ≥
     `extrude_motion_pin_low_print_gap`
* A retract or zero-extrusion move drives the pin LOW immediately and
  disarms the idle timer.

## Configuring printer.cfg

### Using a Raspberry Pi BCM GPIO pin

Install `RPi.GPIO` on the host if not already present:

```bash
pip3 install RPi.GPIO
```

In `printer.cfg`, reference the pin with its BCM number using the `gpio<N>`
syntax:

```ini
[extruder]
step_pin: ...
dir_pin: ...
# ... other extruder settings ...

# Signal gpio17 HIGH during forward extrusion, LOW when idle/retracting.
extrude_motion_pin: gpio17

# Optional tuning (defaults shown):
# extrude_motion_pin_idle_delay: 0.050
# extrude_motion_pin_low_print_gap: 0.500
```

To reference the pin via the `rpi` MCU alias (as defined in
[RPi microcontroller](RPi_microcontroller.md)):

```ini
extrude_motion_pin: rpi:gpio17
```

An inverted pin (active-LOW hardware) is expressed with a `!` prefix:

```ini
extrude_motion_pin: !gpio17
```

> **Note:** The BCM GPIO path bypasses the MCU entirely.  The pin is
> toggled directly from the host process using `RPi.GPIO`, so no
> `[mcu rpi]` section is required.

### Using a standard MCU digital output pin

For a pin that is wired to a microcontroller (e.g. an unused endstop
header on a control board) use its normal Klipper pin alias:

```ini
[extruder]
# ... other extruder settings ...
extrude_motion_pin: PA4          # MCU pin alias
extrude_motion_pin_off_delay: 0.010
```

The pin is managed as a `digital_out` object through Klipper's standard
pin scheduler and will be synchronised with the MCU clock.

### Tuning advice

* **`extrude_motion_pin_idle_delay`** — Increase this value (e.g.
  `0.100`) if the pin flickers LOW during fast travel moves between
  extrusion segments.  Decrease it if you need a faster LOW response
  after the print finishes a segment.

* **`extrude_motion_pin_low_print_gap`** — The default `0.500` s works
  well for most printers.  If you see spurious LOWs during high-speed
  printing with many short moves, try increasing to `1.000`.  If layer
  changes are unusually fast (e.g. on a very small object) you may need
  to reduce it slightly.

* **`extrude_motion_pin_off_delay`** — Only applies to MCU digital-out
  pins.  Leave at the default unless you observe `Timer too close`
  errors; in that case increase by `0.010` increments.

## Security note

When driving high-power devices (spindles, relays, solenoids) through
this pin, always add appropriate flyback diodes and ensure the GPIO
current rating is not exceeded.  Use an opto-isolator or MOSFET driver
stage rather than connecting loads directly to a Pi GPIO.
