# LX4000 XY Stage Driver

Python driver for the ASI LX4000 XY stage over serial.

## Usage

```python
from xy_stage import XYStage

with XYStage("/dev/ttyUSB0") as stage:
    x, y = stage.where()
    stage.move(5000, 5000)
    stage.move_relative(1000, -500)
    stage.move_to_center()
    stage.home()
    stage.halt()
    busy = stage.is_busy()
```

Serial settings are 115200 8N1.

Position units are **0.1 µm**.

The measured travel range is:

| Axis | Min | Max | Range |
|------|-----|-----|-------|
| X | -465270 | 424914 | ~890 mm |
| Y | -347158 | 534573 | ~880 mm |

## API

**Motion**

| Method | Description |
|--------|-------------|
| `where() -> (x, y)` | Read current position |
| `move(x, y)` | Absolute move (bounds-checked) |
| `move_relative(dx, dy)` | Relative move |
| `move_to_center()` | Move to (5000, 5000) |
| `home()` | Drive to home (positive limit switch) |
| `halt()` | Stop all motion immediately |
| `is_busy() -> bool` | True if stage is moving |

**Configuration**

| Method | Description |
|--------|-------------|
| `speed(x, y)` | Set max velocity for each axis |
| `accel(x, y)` | Set ramp time in ms for each axis |
| `backlash(x, y)` | Set backlash correction for each axis |
| `zero()` | Set current position as origin (0, 0) |
| `here(x, y)` | Redefine current position without moving |

**Calibration**

| Method | Description |
|--------|-------------|
| `aalign()` | Auto-align motor drive circuit (stage will move) |
| `aalign_query() -> (x, y)` | Query current potentiometer values |
| `aalign_set(x, y)` | Write potentiometer values directly (no motion) |
| `azero()` | Auto-adjust zero balance of motor drive card |

**Utility**

| Method | Description |
|--------|-------------|
| `info() -> str` | Dump full axis info from controller |
| `firmware_date() -> str` | Firmware compile date/time |
| `reset()` | Reset controller (reopen port afterwards) |

## Protocol

Commands are addressed to the XY stage with the `2H` prefix (e.g. `2H MOVE X=1000 Y=2000\r`). The controller replies `:A [data]` on success or `:N<code>` on error.

| Error | Meaning |
|-------|---------|
| -1 | Unknown command |
| -2 | Unrecognized axis parameter |
| -3 | Missing parameters |
| -4 | Parameter out of range |
| -5 | Operation failed |
| -6 | Undefined error |
| -21 | Command halted by HALT |

## Hardware

Tested with a **Gearmo USB 2.0 RS-232 Serial Adapter** using the **Future Technology Devices International FT232 Serial (UART) IC**. On Linux this enumerates as `/dev/ttyUSB0` and is detected automatically by the driver via its FTDI manufacturer string.

## References

- [LX-4000 stage controller notes](https://41j.com/blog/2021/03/lx-4000-stage-controller-notes/)
- [ASI MS-2000 Operations and Programming Manual](https://www.asiimaging.com/downloads/manuals/Operations_and_Programming_Manual.pdf)
