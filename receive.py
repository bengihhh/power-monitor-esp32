import argparse
import csv
import math
import time
from pathlib import Path


def receive(source, output, clock=time.time):
    writer = csv.writer(output, lineterminator="\n")
    writer.writerow(("time_s", "epoch_s", "current_uA"))
    previous = None
    anchor = 0
    for line in source:
        if not line.endswith(b"\n") or not line.startswith(b"PM,"):
            continue
        try:
            _, stamp, reading = line.split(b",")
            instant, current = int(stamp), float(reading)
        except ValueError:
            continue
        if instant < 0 or not math.isfinite(current):
            continue
        if previous is None or instant < previous:
            anchor = clock() - instant / 1e6
        previous = instant
        writer.writerow((f"{instant / 1e6:.6f}", f"{anchor + instant / 1e6:.6f}",
                         f"{current:.3f}"))
        output.flush()


def main():
    import serial

    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="ESP32 USB-to-UART COM port")
    parser.add_argument("csv", type=Path)
    args = parser.parse_args()
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    try:
        with serial.Serial(baudrate=921600, timeout=1.0) as port:
            port.dtr = False
            port.rts = False
            port.port = args.port
            port.open()
            with args.csv.open("x", encoding="utf-8", newline="") as output:
                print(f"{args.port} -> {args.csv}", flush=True)
                receive(iter(port.readline, None), output)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
