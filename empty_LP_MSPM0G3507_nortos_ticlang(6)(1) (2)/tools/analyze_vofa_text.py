#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""VOFA UART2 text capture script.

Captures CSV text lines from zdt_motor_test_task() via UART2,
saves to CSV with absolute + relative timestamps.

9-channel format: rx,raw,ball,target,vel,cmd,rod,lost,run
Source: VOFA_SendString() -> UART_2_INST
"""

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

try:
    import serial
    from serial import SerialException
except ImportError:
    serial = None
    SerialException = Exception

# ============================================================
# 9-channel field definitions (guide sec 4.2)
#   rx     = g_rx_pulse                   UART1 vision rx count
#   raw    = g_vision_x_offset            raw vision x offset
#   ball   = ball_pos_px                  filtered ball position
#   target = target_pos_px                target position (usually 0)
#   vel    = ball_vel_px                  ball velocity estimate
#   cmd    = last_target_pulse            control output (before limit)
#   rod    = RodActuator_GetTargetPulse() actual rod target (after limit)
#   lost   = vision_lost_count            vision lost frame count
#   run    = question3_running            question 3 active flag
# ============================================================
FIELD_NAMES = ["rx", "raw", "ball", "target", "vel", "cmd", "rod", "lost", "run"]


def parse_args():
    p = argparse.ArgumentParser(
        description="VOFA UART2 text capture (9ch question 3 debug)"
    )
    p.add_argument("--port", default="COM13")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--csv", default="",
                   help="CSV path (default: auto logs/q3_vofa_YYYYMMDD_HHMMSS.csv)")
    p.add_argument("--duration", type=float, default=0,
                   help="seconds to capture; 0 = until Ctrl+C")
    p.add_argument("--no-wait", action="store_true",
                   help="start timer immediately (default: wait for first data)")
    p.add_argument("--verbose", action="store_true",
                   help="print every line (default: 1-second summary)")
    p.add_argument("--quiet", action="store_true",
                   help="no output except final summary")
    return p.parse_args()


def make_csv_path(user_path):
    if user_path:
        return user_path
    log_dir = Path(__file__).resolve().parent / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return str(log_dir / f"q3_vofa_{stamp}.csv")


def open_serial(port, baud):
    if serial is None:
        print("ERROR: pyserial not installed. Run: python -m pip install pyserial")
        return None
    try:
        return serial.Serial(port, baud, timeout=0.5)
    except SerialException as e:
        print(f"ERROR: cannot open {port}: {e}")
        print("  Check: device manager port, USB-TTL connected, not occupied by VOFA/other tool")
        return None


def parse_line(text):
    """Parse comma-separated line -> float list. Returns None for empty lines."""
    if not text:
        return None
    parts = text.split(",")
    if len(parts) != len(FIELD_NAMES):
        raise ValueError(f"expected {len(FIELD_NAMES)} cols, got {len(parts)}")
    return [float(x) for x in parts]


def fmt_vals(values):
    """Compact single-line display of all field values."""
    return " ".join(f"{n}={v:.1f}" for n, v in zip(FIELD_NAMES, values))


def wait_for_data(ser):
    """Block until first valid data line arrives. Returns (text, values)."""
    n = 0
    print("waiting for data... (start question 3 on device)")
    while True:
        try:
            raw = ser.readline()
        except SerialException as e:
            print(f"serial error: {e}")
            sys.exit(1)
        if not raw:
            n += 1
            if n % 20 == 1:  # ~every 10s
                print("  still waiting...")
            continue
        text = raw.decode("utf-8", errors="ignore").strip()
        if not text:
            continue
        try:
            vals = parse_line(text)
            if vals is not None:
                print(f"  first line: {fmt_vals(vals)}\n")
                return text, vals
        except ValueError:
            pass
        n += 1


def main():
    args = parse_args()
    csv_path = make_csv_path(args.csv)

    # ---- startup info ----
    print(f"port={args.port} baud={args.baud} duration={args.duration or 'unlimited'}")
    print(f"fields: {','.join(FIELD_NAMES)}")
    print(f"csv:    {csv_path}")
    print(f"wait={'off' if args.no_wait else 'on'}  "
          f"verbose={args.verbose}  quiet={args.quiet}")
    print()

    ser = open_serial(args.port, args.baud)
    if ser is None:
        sys.exit(1)

    try:
        with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(["timestamp", "time_s", "line_index", *FIELD_NAMES, "raw_text"])

            line_idx = 0
            bad_lines = 0

            # ---- wait mode (default: on) ----
            if not args.no_wait:
                first_text, first_vals = wait_for_data(ser)
                t0 = time.time()
                line_idx = 1
                ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                w.writerow([ts, "0.000", line_idx,
                            *[f"{v:.3f}" for v in first_vals], first_text])
                f.flush()
                if not args.quiet:
                    print(f"#{line_idx:<6} t=  0.000s {fmt_vals(first_vals)}")
            else:
                t0 = time.time()

            last_summary = 0.0
            no_data_since = t0

            # ---- main loop ----
            while args.duration <= 0 or time.time() - t0 < args.duration:
                try:
                    raw = ser.readline()
                except SerialException as e:
                    print(f"serial error: {e}")
                    break

                now = time.time()

                if not raw:
                    if now - no_data_since > 10 and line_idx > 0:
                        print("WARNING: no data for 10s (check device, USE_VOFA_DEBUG=1, "
                              "question 3 running)")
                        no_data_since = now
                    continue

                no_data_since = now
                text = raw.decode("utf-8", errors="ignore").strip()
                if not text:
                    continue

                ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                t_rel = now - t0

                try:
                    vals = parse_line(text)
                    if vals is None:
                        continue
                    line_idx += 1

                    w.writerow([ts, f"{t_rel:.3f}", line_idx,
                                *[f"{v:.3f}" for v in vals], text])
                    f.flush()  # safety: write each line to disk immediately

                    if args.verbose:
                        print(f"#{line_idx:<6} t={t_rel:7.3f}s {fmt_vals(vals)}")
                    elif not args.quiet and t_rel - last_summary >= 1.0:
                        print(f"#{line_idx:<6} t={t_rel:7.3f}s {fmt_vals(vals)}")
                        last_summary = t_rel

                except ValueError:
                    bad_lines += 1
                    if not args.quiet:
                        print(f"  [skip #{bad_lines}] {text}")

    except KeyboardInterrupt:
        print("\nstopped (Ctrl+C)")

    finally:
        ser.close()
        elapsed = time.time() - t0 if line_idx > 0 else 0
        print(f"\n--- done ---")
        print(f"  duration:   {elapsed:.1f}s")
        print(f"  valid:      {line_idx}")
        print(f"  bad:        {bad_lines}")
        if elapsed > 0:
            print(f"  rate:       {line_idx / elapsed:.1f} lines/s")
        print(f"  csv:        {csv_path}")


if __name__ == "__main__":
    main()
