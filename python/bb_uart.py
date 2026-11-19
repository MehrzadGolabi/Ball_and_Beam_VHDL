#!/usr/bin/env python3
"""Host tool for the ball-and-beam FPGA (115200 8N1 over the FT2232H USB-UART).

Protocol (see simulink/mlfb/cmd_regs.m and telemetry.m)
  host -> FPGA  : A5 ADDR HI LO CHK            CHK = A5^ADDR^HI^LO
                  ADDR bit7 = 1 -> read request
  FPGA -> host  : telemetry  5A 10 SEQ POS ECHO SP SERVO ERR I D FLAGS CHK  (19 bytes)
                  read reply 5A 20 ADDR HI LO CHK                          (6 bytes)
  16-bit fields big-endian, ERR/I/D signed, CHK = XOR of preceding bytes.

Examples
  python bb_uart.py ports
  python bb_uart.py -p COM7 dump
  python bb_uart.py -p COM7 set kp 3.0 kd 50 setpoint 300
  python bb_uart.py -p COM7 get kp
  python bb_uart.py -p COM7 monitor
  python bb_uart.py -p COM7 plot --log run1.csv
  python bb_uart.py -p COM7 shell
  python bb_uart.py selftest
"""
from __future__ import annotations

import argparse
import csv
import collections
import struct
import sys
import threading
import time
from dataclasses import dataclass

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # selftest works without pyserial
    serial = None

BAUD = 115200

# name -> (address, kind, help)   kind: 'int' raw 16-bit, 'q88' gain value/256
REGS = {
    "setpoint": (0x00, "int", "setpoint, mm from the sensor"),
    "kp":       (0x01, "q88", "proportional gain, us per mm"),
    "ki":       (0x02, "q88", "integral gain, us per mm per sample"),
    "kd":       (0x03, "q88", "derivative gain, us per (mm/sample)"),
    "center":   (0x04, "int", "servo pulse that levels the beam, us"),
    "smin":     (0x05, "int", "servo pulse minimum, us"),
    "smax":     (0x06, "int", "servo pulse maximum, us"),
    "mode":     (0x07, "int", "bit0 loop on, bit1 invert, bit2 telemetry, bit3 median"),
    "manual":   (0x08, "int", "servo pulse when loop is off, us"),
    "dshift":   (0x09, "int", "derivative filter: alpha = 1/2^dshift (0..7)"),
    "ilim":     (0x0A, "int", "integrator clamp, us"),
    "trig_ms":  (0x0B, "int", "sensor / control period, ms (>=10)"),
    "cmd":      (0x0C, "int", "write 1: reset integrator"),
    "version":  (0x0D, "ro",  "design version (0xBB01)"),
    "n_ok":     (0x0E, "ro",  "good command frames received"),
    "n_bad":    (0x0F, "ro",  "command frames with bad checksum"),
}
ADDR2NAME = {a: n for n, (a, _, _) in REGS.items()}
MODE_BITS = {"loop": 1, "invert": 2, "telemetry": 4, "median": 8}


def xor(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
    return c


def cmd_frame(addr: int, value: int = 0, read: bool = False) -> bytes:
    a = (addr | 0x80) if read else addr
    hi, lo = (value >> 8) & 0xFF, value & 0xFF
    body = bytes([0xA5, a, hi, lo])
    return body + bytes([xor(body)])


def encode_value(name: str, value: float) -> int:
    _, kind, _ = REGS[name]
    if kind == "ro":
        raise ValueError(f"{name} is read-only")
    v = round(value * 256) if kind == "q88" else int(round(value))
    if not 0 <= v <= 0xFFFF:
        raise ValueError(f"{name}={value} does not fit in 16 bits")
    return v


def decode_value(name: str, raw: int) -> float:
    return raw / 256.0 if REGS[name][1] == "q88" else raw


@dataclass
class Telemetry:
    t: float
    seq: int
    pos: int
    echo_us: int
    setpoint: int
    servo: int
    err: int
    iterm: int
    dterm: int
    flags: int

    @property
    def flag_text(self) -> str:
        s = []
        if self.flags & 1: s.append("TIMEOUT")
        if self.flags & 2: s.append("RANGE")
        s.append("LOOP" if self.flags & 4 else "MANUAL")
        if self.flags & 8: s.append("INV")
        if self.flags & 16: s.append("MED")
        return ",".join(s)


class FrameParser:
    """Byte-stream parser; resynchronises on bad checksums."""

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data: bytes):
        self.buf.extend(data)
        out = []
        while True:
            i = self.buf.find(0x5A)
            if i < 0:
                self.buf.clear()
                break
            del self.buf[:i]
            if len(self.buf) < 2:
                break
            typ = self.buf[1]
            n = {0x10: 19, 0x20: 6}.get(typ)
            if n is None:
                del self.buf[0]
                continue
            if len(self.buf) < n:
                break
            f = bytes(self.buf[:n])
            if xor(f[:-1]) != f[-1]:
                self.bad += 1
                del self.buf[0]
                continue
            del self.buf[:n]
            if typ == 0x10:
                seq = f[2]
                pos, echo, sp, servo = struct.unpack(">HHHH", f[3:11])
                err, it, dt = struct.unpack(">hhh", f[11:17])
                out.append(Telemetry(time.time(), seq, pos, echo, sp, servo, err, it, dt, f[17]))
            else:
                out.append(("reg", f[2], (f[3] << 8) | f[4]))
        return out


class BallBeam:
    def __init__(self, port: str):
        if serial is None:
            sys.exit("pyserial is required: pip install pyserial")
        self.ser = serial.Serial(port, BAUD, timeout=0.05)
        self.parser = FrameParser()
        self.replies = collections.deque()
        self.telemetry = collections.deque(maxlen=100000)
        self.callbacks = []
        self._lock = threading.Lock()
        self._stop = False
        self._rx = threading.Thread(target=self._reader, daemon=True)
        self._rx.start()

    def close(self):
        self._stop = True
        self._rx.join(timeout=1)
        self.ser.close()

    def _reader(self):
        while not self._stop:
            data = self.ser.read(256)
            if not data:
                continue
            for item in self.parser.feed(data):
                if isinstance(item, Telemetry):
                    self.telemetry.append(item)
                    for cb in self.callbacks:
                        cb(item)
                else:
                    with self._lock:
                        self.replies.append(item)

    # --- registers ---
    def write(self, name: str, value: float):
        addr = REGS[name][0]
        self.ser.write(cmd_frame(addr, encode_value(name, value)))
        self.ser.flush()
        time.sleep(0.002)

    def read(self, name: str, timeout: float = 0.5) -> float:
        addr = REGS[name][0]
        with self._lock:
            self.replies.clear()
        self.ser.write(cmd_frame(addr, read=True))
        self.ser.flush()
        t_end = time.time() + timeout
        while time.time() < t_end:
            with self._lock:
                while self.replies:
                    _, a, v = self.replies.popleft()
                    if a == addr:
                        return decode_value(name, v)
            time.sleep(0.005)
        raise TimeoutError(f"no reply reading {name} - check port, baud, wiring, FPGA bitstream")

    def set_mode_bit(self, bit: str, on: bool):
        m = int(self.read("mode"))
        m = (m | MODE_BITS[bit]) if on else (m & ~MODE_BITS[bit])
        self.write("mode", m)


# ------------------------------------------------------------------ commands
def print_telemetry(t: Telemetry):
    print(f"#{t.seq:3d} pos={t.pos:4d}mm sp={t.setpoint:4d} err={t.err:+5d} "
          f"servo={t.servo:4d}us I={t.iterm:+5d} D={t.dterm:+5d} echo={t.echo_us:5d}us [{t.flag_text}]")


def cmd_dump(bb: BallBeam):
    for name, (addr, kind, hlp) in REGS.items():
        if name == "cmd":
            continue
        v = bb.read(name)
        vs = f"{v:.4f}" if kind == "q88" else (f"0x{int(v):04X}" if name in ("version", "mode") else f"{int(v)}")
        print(f"  {addr:02X} {name:9s} = {vs:>10s}   {hlp}")


def cmd_set(bb: BallBeam, pairs):
    if len(pairs) % 2:
        sys.exit("usage: set NAME VALUE [NAME VALUE ...]")
    for name, val in zip(pairs[::2], pairs[1::2]):
        if name in MODE_BITS:
            bb.set_mode_bit(name, val.lower() in ("1", "on", "true", "yes"))
            print(f"mode.{name} = {val}")
            continue
        if name not in REGS:
            sys.exit(f"unknown register {name}; known: {', '.join(REGS)} or mode bits {', '.join(MODE_BITS)}")
        v = float(int(val, 0)) if val.lower().startswith("0x") else float(val)
        bb.write(name, v)
        rb = bb.read(name) if REGS[name][1] != "ro" and name != "cmd" else v
        print(f"{name} = {rb}")


def cmd_monitor(bb: BallBeam, log: str | None):
    writer, fh = open_log(log)
    def cb(t):
        print_telemetry(t)
        if writer:
            write_row(writer, t)
    bb.callbacks.append(cb)
    print("monitoring, Ctrl+C to stop")
    try:
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        if fh:
            fh.close()


def open_log(path):
    if not path:
        return None, None
    fh = open(path, "w", newline="")
    w = csv.writer(fh)
    w.writerow(["t", "seq", "pos_mm", "echo_us", "setpoint", "servo_us", "err", "iterm", "dterm", "flags"])
    return w, fh


def write_row(w, t: Telemetry):
    w.writerow([f"{t.t:.4f}", t.seq, t.pos, t.echo_us, t.setpoint, t.servo, t.err, t.iterm, t.dterm, t.flags])


def cmd_plot(bb: BallBeam, window: float, log: str | None):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    writer, fh = open_log(log)
    if writer:
        bb.callbacks.append(lambda t: write_row(writer, t))
    t0 = time.time()
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    l_pos, = ax1.plot([], [], label="ball position")
    l_sp, = ax1.plot([], [], "--", label="setpoint")
    ax1.set_ylabel("mm from sensor"); ax1.legend(loc="upper left"); ax1.grid(True)
    l_srv, = ax2.plot([], [], color="tab:red", label="servo pulse")
    ax2.set_ylabel("us"); ax2.set_xlabel("s"); ax2.grid(True); ax2.legend(loc="upper left")
    txt = ax1.text(0.99, 0.95, "", transform=ax1.transAxes, ha="right", va="top", family="monospace")

    def update(_):
        data = [x for x in list(bb.telemetry) if x.t >= time.time() - window]
        if not data:
            return l_pos, l_sp, l_srv, txt
        ts = [x.t - t0 for x in data]
        l_pos.set_data(ts, [x.pos for x in data])
        l_sp.set_data(ts, [x.setpoint for x in data])
        l_srv.set_data(ts, [x.servo for x in data])
        ax1.set_xlim(max(0, ts[-1] - window), max(window, ts[-1]))
        lo = min(min(x.pos for x in data), min(x.setpoint for x in data))
        hi = max(max(x.pos for x in data), max(x.setpoint for x in data))
        ax1.set_ylim(lo - 20, hi + 20)
        ax2.set_ylim(min(x.servo for x in data) - 50, max(x.servo for x in data) + 50)
        last = data[-1]
        txt.set_text(f"pos {last.pos} mm  err {last.err:+d}  [{last.flag_text}]\n"
                     f"{len(data) / max(1e-3, ts[-1] - ts[0]):.1f} Hz  bad frames {bb.parser.bad}")
        return l_pos, l_sp, l_srv, txt

    _anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)  # noqa: F841
    fig.suptitle("Ball & Beam telemetry (close window to exit)")
    plt.tight_layout()
    plt.show()
    if fh:
        fh.close()


SHELL_HELP = """commands:
  set NAME VALUE ...   write registers (kp/ki/kd take real gains), e.g. set kp 3 kd 50
  get NAME             read a register
  dump                 read all registers
  loop on|off          enable / disable the controller (off -> servo = manual)
  invert on|off        flip the control direction
  median on|off        median-of-3 filter on the sensor
  servo US             manual mode at pulse width US (for finding 'center')
  reset_i              reset the integrator
  mon [N]              print the next N telemetry frames (default 10)
  quit"""


def cmd_shell(bb: BallBeam):
    print(SHELL_HELP)
    while True:
        try:
            line = input("bb> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        p = line.split()
        try:
            if p[0] in ("quit", "exit", "q"):
                break
            elif p[0] == "help":
                print(SHELL_HELP)
            elif p[0] == "set":
                cmd_set(bb, p[1:])
            elif p[0] == "get":
                print(f"{p[1]} = {bb.read(p[1])}")
            elif p[0] == "dump":
                cmd_dump(bb)
            elif p[0] in ("loop", "invert", "median"):
                bb.set_mode_bit(p[0], p[1] == "on")
            elif p[0] == "servo":
                bb.write("manual", float(p[1]))
                bb.set_mode_bit("loop", False)
            elif p[0] == "reset_i":
                bb.write("cmd", 1)
            elif p[0] == "mon":
                n = int(p[1]) if len(p) > 1 else 10
                start = len(bb.telemetry)
                t_end = time.time() + 10
                while len(bb.telemetry) < start + n and time.time() < t_end:
                    time.sleep(0.02)
                for t in list(bb.telemetry)[start:start + n]:
                    print_telemetry(t)
            else:
                print("unknown command; type help")
        except (ValueError, KeyError, TimeoutError, IndexError, SystemExit) as e:
            print(f"error: {e}")


def selftest():
    """Offline check of framing against the HDL definitions."""
    f = cmd_frame(0x0B, 10)
    assert f == bytes([0xA5, 0x0B, 0x00, 0x0A, 0xA5 ^ 0x0B ^ 0x0A]), f.hex()
    assert cmd_frame(0x0D, read=True)[1] == 0x8D
    assert encode_value("kp", 3.0) == 768 and decode_value("kp", 768) == 3.0
    body = bytes([0x5A, 0x10, 7]) + struct.pack(">HHHHhhh", 149, 874, 330, 2296, 181, 3, -13) + bytes([20])
    tel = body + bytes([xor(body)])
    rd = bytes([0x5A, 0x20, 0x0D, 0xBB, 0x01])
    rd += bytes([xor(rd)])
    p = FrameParser()
    out = p.feed(b"\x00\x5a\x99" + tel[:7]) + p.feed(tel[7:] + rd + b"\x5a\x10" + b"\x00" * 17 + b"\x01")
    assert isinstance(out[0], Telemetry) and out[0].pos == 149 and out[0].dterm == -13
    assert out[0].flag_text == "LOOP,MED", out[0].flag_text
    assert out[1] == ("reg", 0x0D, 0xBB01)
    assert p.bad >= 1
    print("selftest OK")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", help="serial port, e.g. COM7 or /dev/ttyUSB1")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ports", help="list serial ports")
    sub.add_parser("selftest", help="offline protocol test")
    sub.add_parser("dump", help="read all registers")
    s = sub.add_parser("set", help="write registers: set NAME VALUE [...]")
    s.add_argument("pairs", nargs="+")
    g = sub.add_parser("get", help="read one register")
    g.add_argument("name", choices=list(REGS))
    m = sub.add_parser("monitor", help="print telemetry")
    m.add_argument("--log", help="CSV file")
    pl = sub.add_parser("plot", help="live plot")
    pl.add_argument("--window", type=float, default=20.0, help="seconds shown")
    pl.add_argument("--log", help="CSV file")
    sub.add_parser("shell", help="interactive console")
    a = ap.parse_args()

    if a.cmd == "selftest":
        return selftest()
    if a.cmd == "ports":
        if serial is None:
            sys.exit("pip install pyserial")
        for p in serial.tools.list_ports.comports():
            print(f"{p.device:10s} {p.description}")
        print("(the FT2232H shows two ports; the UART is usually the second / higher one)")
        return
    if not a.port:
        sys.exit("--port is required (see: bb_uart.py ports)")
    bb = BallBeam(a.port)
    try:
        if a.cmd == "dump":
            cmd_dump(bb)
        elif a.cmd == "set":
            cmd_set(bb, a.pairs)
        elif a.cmd == "get":
            print(f"{a.name} = {bb.read(a.name)}")
        elif a.cmd == "monitor":
            cmd_monitor(bb, a.log)
        elif a.cmd == "plot":
            cmd_plot(bb, a.window, a.log)
        elif a.cmd == "shell":
            cmd_shell(bb)
    finally:
        bb.close()


if __name__ == "__main__":
    main()
