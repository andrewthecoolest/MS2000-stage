import time
import serial
from typing import Tuple

_PREFIX = "2H"
_EOL = b"\r"

_ERRORS = {
    -1:  "Unknown command",
    -2:  "Unrecognized axis parameter",
    -3:  "Missing parameters",
    -4:  "Parameter out of range",
    -5:  "Operation failed",
    -6:  "Undefined error",
    -21: "Command halted by HALT",
}

# Measured hard limits (stage units, 0.1 µm)
X_MIN, X_MAX = -465270, 424914
Y_MIN, Y_MAX = -347158, 534573
X_CENTER = 5000
Y_CENTER = 5000


class XYStageError(Exception):
    pass


class XYStage:
    """
    Driver for the LX4000 XY stage (MS-2000-compatible protocol, address 2H).

    Position units are stage units (0.1 µm by default unless the controller
    has been reconfigured).
    """

    def __init__(self, port: str, timeout: float = 2.0):
        self._ser = serial.Serial(
            port=port,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )

    def close(self) -> None:
        self._ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def _send(self, cmd: str) -> str:
        """Send a prefixed command and return the payload after ':A'."""
        self._ser.reset_input_buffer()
        self._ser.write(f"{_PREFIX} {cmd}\r".encode("ascii"))
        reply = self._ser.read_until(_EOL)
        # A glitched/noisy serial frame can return non-ASCII bytes. Treat that
        # as a comms failure (like a timeout) so callers retry, rather than
        # letting a UnicodeDecodeError escape and abort the whole scan.
        try:
            decoded = reply.decode("ascii")
        except UnicodeDecodeError:
            raise XYStageError(f"Corrupted response to {cmd!r}: {reply!r}")
        # The controller frames some replies with ETX/control bytes
        # (e.g. '\x03:A X=10 Y=10\r\n'). Drop anything non-printable so the
        # ':A'/':N' prefix detection below works.
        raw = "".join(c for c in decoded if c.isprintable()).strip()
        if not raw:
            raise XYStageError(f"No response to {cmd!r} (timeout)")
        if raw.startswith(":N"):
            code = int(raw[2:])
            desc = _ERRORS.get(code, f"Reserved/unknown code {code}")
            raise XYStageError(f"{desc} (error {code}): {cmd!r}")
        return raw[3:].strip() if raw.startswith(":A") else raw

    def move(self, x: float, y: float) -> None:
        """Absolute move to (x, y). Returns as soon as the controller acknowledges."""
        if not (X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX):
            raise XYStageError(f"Target ({x}, {y}) outside stage bounds")
        self._send(f"MOVE X={x:.0f} Y={y:.0f}")

    def move_relative(self, dx: float, dy: float) -> None:
        """Relative move by (dx, dy)."""
        self._send(f"MOVREL X={dx:.0f} Y={dy:.0f}")

    def move_to_center(self) -> None:
        """Move to the center of the stage's travel range."""
        self.move(X_CENTER, Y_CENTER)

    def where(self) -> Tuple[float, float]:
        """Return current (x, y) position."""
        data = self._send("WHERE X Y")
        try:
            x, y = data.split()
            return float(x), float(y)
        except ValueError:
            raise XYStageError(f"Unexpected WHERE response: {data!r}")

    def home(self) -> None:
        """Home both axes. The controller acknowledges immediately; motion continues in background."""
        self._send("HOME X Y")

    def halt(self) -> None:
        """Stop all motion immediately (global interrupt, no axis prefix)."""
        self._ser.reset_input_buffer()
        self._ser.write(b"\\\r")
        self._ser.read_until(_EOL)

    def is_busy(self) -> bool:
        """True if the stage is currently moving."""
        return "B" in self._send("/")

    def speed(self, x: float, y: float) -> None:
        """Set max velocity for each axis."""
        self._send(f"SPEED X={x} Y={y}")

    def accel(self, x: float, y: float) -> None:
        """Set ramp time in ms for each axis."""
        self._send(f"ACCEL X={x} Y={y}")

    def backlash(self, x: float, y: float) -> None:
        """Set backlash correction for each axis."""
        self._send(f"BACKLASH X={x} Y={y}")

    def zero(self) -> None:
        """Set current position as origin (0, 0)."""
        self._send("ZERO X Y")

    def here(self, x: float, y: float) -> None:
        """Redefine current position as (x, y) without moving."""
        self._send(f"HERE X={x:.0f} Y={y:.0f}")

    def info(self) -> str:
        """Return axis info string from the controller (multi-line)."""
        self._ser.reset_input_buffer()
        self._ser.write(f"{_PREFIX} INFO X Y\r".encode("ascii"))
        lines = []
        while True:
            line = self._ser.read_until(_EOL).decode("ascii").strip()
            if not line:
                break
            lines.append(line)
        return "\n".join(lines)

    def reset(self) -> None:
        """Reset the controller. Re-open the serial port after calling this."""
        self._ser.write(b"~\r")

    def firmware_date(self) -> str:
        """Return the firmware compile date/time."""
        return self._send("CDATE")

    def aalign(self, settle: float = 1.5, timeout: float = 90.0) -> None:
        """Auto-align motor drive circuit for both axes. WARNING: stage will move.

        The controller streams 'Val:.. EC:..' diagnostic lines throughout the
        alignment and clears its busy flag before the stream ends, so we issue
        the command and then drain output until the line stays quiet for
        ``settle`` seconds. This guarantees the buffer is clean (and alignment
        finished) before any follow-up query.
        """
        self._ser.reset_input_buffer()
        self._ser.write(f"{_PREFIX} AALIGN X Y\r".encode("ascii"))
        deadline = time.monotonic() + timeout
        last_activity = time.monotonic()
        while time.monotonic() - last_activity < settle:
            line = self._ser.read_until(_EOL)
            if line:
                last_activity = time.monotonic()
                text = "".join(c for c in line.decode("ascii", errors="ignore")
                               if c.isprintable()).strip()
                if text.startswith(":N"):
                    code = int(text[2:])
                    desc = _ERRORS.get(code, f"Reserved/unknown code {code}")
                    raise XYStageError(f"{desc} (error {code}): AALIGN")
            if time.monotonic() > deadline:
                raise XYStageError(f"AALIGN did not settle within {timeout:.0f}s")

    def aalign_query(self) -> Tuple[float, float]:
        """Query current potentiometer values set by AALIGN. Returns (x, y)."""
        data = self._send("AALIGN X? Y?")
        try:
            parts = dict(t.split("=") for t in data.split())
            return float(parts["X"]), float(parts["Y"])
        except (ValueError, KeyError):
            raise XYStageError(f"Unexpected AALIGN query response: {data!r}")

    def aalign_set(self, x: float, y: float) -> None:
        """Write potentiometer values directly. Skips auto-alignment motion."""
        self._send(f"AALIGN X={x} Y={y}")

    def azero(self) -> None:
        """Auto-adjust zero balance of motor drive card for both axes."""
        self._send("AZERO X Y")
