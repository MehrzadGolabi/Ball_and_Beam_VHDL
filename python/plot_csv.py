#!/usr/bin/env python3
"""Plot a telemetry CSV: either build/closed_loop.csv from the GHDL test,
or a log recorded with `bb_uart.py monitor/plot --log file.csv`.

    python plot_csv.py ../build/closed_loop.csv [-o out.png]
"""
import argparse
import csv

import matplotlib.pyplot as plt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("-o", "--out", help="save PNG instead of showing")
    a = ap.parse_args()
    with open(a.csv, newline="") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise SystemExit("empty CSV")
    tkey = "t_s" if "t_s" in rows[0] else "t"
    t = [float(r[tkey]) for r in rows]
    t = [x - t[0] for x in t] if tkey == "t" else t
    pos = [int(r["pos_mm"]) for r in rows]
    sp = [int(r["setpoint"]) for r in rows]
    srv = [int(r["servo_us"]) for r in rows]
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    if "ball_mm" in rows[0]:
        ax1.plot(t, [float(r["ball_mm"]) for r in rows], label="true ball (plant)", alpha=0.6)
    ax1.plot(t, pos, label="measured position (FPGA)")
    ax1.plot(t, sp, "--", label="setpoint")
    ax1.set_ylabel("mm from sensor"); ax1.grid(True); ax1.legend()
    ax2.plot(t, srv, color="tab:red"); ax2.set_ylabel("servo pulse, us")
    ax2.set_xlabel("s"); ax2.grid(True)
    fig.suptitle(a.csv)
    plt.tight_layout()
    if a.out:
        fig.savefig(a.out, dpi=110)
        print("saved", a.out)
    else:
        plt.show()


if __name__ == "__main__":
    main()
