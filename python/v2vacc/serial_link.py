"""
serial_link.py -- non-blocking serial reader for the telemetry stream.

The dashboard must redraw at a steady frame rate regardless of what the
serial port is doing. A blocking readline() in the plotting loop makes
the window freeze every time a board resets or a cable twitches, which
looks broken in a demo even when nothing is wrong.

So: a daemon thread owns the port and pushes parsed records into a
bounded queue; the plotting loop drains whatever is there and moves on.
A bounded queue also means that if the consumer stalls we drop old
samples rather than growing memory without limit.

Includes `list_ports()` and auto-detection, because "which COM port is
the follower on today" is a question nobody should answer by hand twice.
"""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass

from .telemetry import parse_line, CommentRecord

__all__ = ["SerialTelemetry", "list_ports", "autodetect_port", "ReplaySource"]


def list_ports() -> list[tuple[str, str]]:
    """Return [(device, description), ...] for every serial port present."""
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        return []
    return [(p.device, p.description or "") for p in lp.comports()]


def autodetect_port(prefer: str | None = None) -> str | None:
    """Pick the most likely ESP32 port.

    Matches the usual USB-UART bridges found on ESP32 dev boards:
    CP210x (Silicon Labs), CH340/CH910x (WCH), FTDI.
    """
    ports = list_ports()
    if not ports:
        return None
    if prefer:
        for dev, _ in ports:
            if dev.lower() == prefer.lower():
                return dev
    needles = ("cp210", "silicon labs", "ch340", "ch910", "ftdi",
               "usb serial", "usb-serial", "uart")
    for dev, desc in ports:
        if any(n in desc.lower() for n in needles):
            return dev
    return ports[0][0]


# ---------------------------------------------------------------------
@dataclass
class _Stats:
    lines: int = 0
    parsed: int = 0
    dropped: int = 0
    errors: int = 0
    connected: bool = False
    last_rx: float = 0.0


class SerialTelemetry:
    """Threaded reader. Use as a context manager or call start()/stop().

        with SerialTelemetry("COM5") as link:
            for rec in link.drain():
                ...
    """

    def __init__(self, port: str, baud: int = 115200, maxsize: int = 20000,
                 reconnect: bool = True, name: str = "node"):
        self.port = port
        self.baud = baud
        self.name = name
        self.reconnect = reconnect
        self._q: queue.Queue = queue.Queue(maxsize=maxsize)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.stats = _Stats()
        self.banner: list[str] = []

    # ----- lifecycle --------------------------------------------------
    def start(self) -> "SerialTelemetry":
        if self._thread is not None:
            return self
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name=f"serial-{self.name}")
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def __enter__(self) -> "SerialTelemetry":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    # ----- consumer ---------------------------------------------------
    def drain(self, limit: int = 5000) -> list:
        """Pop everything currently queued. Never blocks."""
        out = []
        for _ in range(limit):
            try:
                out.append(self._q.get_nowait())
            except queue.Empty:
                break
        return out

    @property
    def alive(self) -> bool:
        return self.stats.connected

    @property
    def age(self) -> float:
        """Seconds since the last byte arrived. Large = the node is quiet."""
        return time.monotonic() - self.stats.last_rx if self.stats.last_rx else float("inf")

    # ----- worker -----------------------------------------------------
    def _run(self) -> None:
        try:
            import serial  # pyserial
        except ImportError:
            self.stats.errors += 1
            return

        while not self._stop.is_set():
            ser = None
            try:
                ser = serial.Serial(self.port, self.baud, timeout=0.2)
                self.stats.connected = True
                # Give the ESP32 time to finish its boot banner.
                time.sleep(0.2)
                ser.reset_input_buffer()

                while not self._stop.is_set():
                    raw = ser.readline()
                    if not raw:
                        continue
                    self.stats.lines += 1
                    self.stats.last_rx = time.monotonic()

                    text = raw.decode("utf-8", errors="replace")
                    rec = parse_line(text)
                    if rec is None:
                        continue
                    if isinstance(rec, CommentRecord):
                        self.banner.append(rec.text)
                        del self.banner[:-40]
                        continue

                    self.stats.parsed += 1
                    try:
                        self._q.put_nowait(rec)
                    except queue.Full:
                        # drop the oldest to keep the stream live
                        try:
                            self._q.get_nowait()
                            self._q.put_nowait(rec)
                        except queue.Empty:
                            pass
                        self.stats.dropped += 1

            except Exception:
                self.stats.errors += 1
                self.stats.connected = False
                if not self.reconnect:
                    return
                self._stop.wait(1.0)
            finally:
                self.stats.connected = False
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass


# ---------------------------------------------------------------------
class ReplaySource:
    """Drop-in stand-in for SerialTelemetry that replays a recorded log.

    Lets the dashboard be developed, demonstrated and screenshotted with
    no boards attached -- and lets a run be replayed at the review if the
    hardware misbehaves on the day.
    """

    def __init__(self, path, speed: float = 1.0, loop: bool = False):
        from .telemetry import parse_file
        follow, lead = parse_file(path)
        self._records = sorted(follow + lead, key=lambda r: r.t_ms)
        self._i = 0
        self.speed = speed
        self.loop = loop
        self._t0 = None
        self.stats = _Stats(connected=True)
        self.banner = [f"replay: {path} ({len(self._records)} records)"]
        self.name = "replay"

    def start(self) -> "ReplaySource":
        self._t0 = time.monotonic()
        return self

    def stop(self) -> None:
        pass

    def __enter__(self) -> "ReplaySource":
        return self.start()

    def __exit__(self, *exc) -> None:
        pass

    @property
    def alive(self) -> bool:
        return self._i < len(self._records) or self.loop

    @property
    def age(self) -> float:
        return 0.0

    def drain(self, limit: int = 5000) -> list:
        if self._t0 is None or not self._records:
            return []
        elapsed = (time.monotonic() - self._t0) * self.speed * 1000.0
        out = []
        while self._i < len(self._records) and len(out) < limit:
            if self._records[self._i].t_ms <= elapsed:
                out.append(self._records[self._i])
                self._i += 1
            else:
                break
        if self._i >= len(self._records) and self.loop:
            self._i = 0
            self._t0 = time.monotonic()
        return out
