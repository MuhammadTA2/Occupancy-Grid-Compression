#!/usr/bin/env python3
"""
Stand-in for the real base-station path-planning computer -- lets you test
the BASE ESP32's Serial2 grid-output/instructions-input path (and the whole
LoRa round trip) without waiting on the real A* implementation.

Connects to the BASE's Serial2 link (via a USB-to-serial adapter wired to
GPIO16/17+GND, NOT the ESP32's USB programming port -- that's a separate
UART carrying only debug text), waits for the decompressed grid the base
forwards after receiving it over LoRa from the rover, prints a summary, then
sends back a canned waypoint list as a stand-in for a real computed path.

Usage: python simulate_basestation.py <COM port or /dev/ttyUSBx>
"""
import sys

try:
    import serial
except ImportError:
    print("Missing dependency: pip install pyserial")
    sys.exit(1)

BAUD = 115200

# Canned path -- stand-in for real A* output. Must match what main.cpp's
# buildSyntheticPath() used to hardcode, or anything else you want to test.
FAKE_PATH = [(2, 2), (5, 8), (10, 15), (18, 18)]


def encode_waypoints(waypoints):
    out = bytearray()
    out.append(len(waypoints))
    for x, y in waypoints:
        out += int(x).to_bytes(2, "little", signed=True)
        out += int(y).to_bytes(2, "little", signed=True)
    return bytes(out)


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <port>")
        sys.exit(1)
    port = sys.argv[1]

    print(f"Opening {port} at {BAUD} baud...")
    with serial.Serial(port, BAUD, timeout=60) as ser:
        print("Waiting for grid header (4 bytes: rows, cols) from the base ESP32...")
        header = ser.read(4)
        if len(header) < 4:
            print("TIMED OUT waiting for the grid header.")
            return
        rows = int.from_bytes(header[0:2], "little")
        cols = int.from_bytes(header[2:4], "little")
        print(f"Grid is {rows}x{cols} ({rows * cols} cells)")

        cell_bytes = ser.read(rows * cols)
        if len(cell_bytes) < rows * cols:
            print(f"TIMED OUT -- only got {len(cell_bytes)} of {rows * cols} expected cell bytes.")
            return
        print(f"Received {len(cell_bytes)} cell bytes.")
        print(f"Row 0: {list(cell_bytes[:cols])}")
        print(f"Row {rows - 1}: {list(cell_bytes[(rows - 1) * cols: rows * cols])}")

        print(f"Sending canned path back: {FAKE_PATH}")
        ser.write(encode_waypoints(FAKE_PATH))
        print("Done.")


if __name__ == "__main__":
    main()
