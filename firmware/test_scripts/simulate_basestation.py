#!/usr/bin/env python3
"""
Stand-in for the real base-station path-planning computer -- lets you test
the BASE ESP32's grid-output/instructions-input path (and the whole LoRa
round trip) without waiting on the real A* implementation.

Connects to the BASE's native USB port (the same one used to flash it --
this bench-test firmware build runs its binary data protocol over `Serial`
with debug prints compiled out, see ENABLE_SERIAL_DEBUG in main.cpp), waits
for the decompressed grid the base forwards after receiving it over LoRa
from the rover, prints a summary, then sends back a canned waypoint list as
a stand-in for a real computed path.

Usage: python simulate_basestation.py <COM port or /dev/ttyUSBx>
"""
import sys
import time

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
    with serial.Serial(port, BAUD, timeout=90) as ser:
        time.sleep(2.5)  # let the ESP32's boot-time reset (and ROM boot log) finish
        ser.reset_input_buffer()  # discard the ROM/bootloader banner queued during the reset above
        ser.reset_output_buffer()

        round_num = 0
        try:
            while True:
                round_num += 1
                print(f"\n--- Round {round_num} ---")
                # See the matching comment in simulate_jetson.py -- discards any
                # stale bytes left over from a round whose response arrived
                # after we'd already stopped waiting on it.
                ser.reset_input_buffer()

                print("Waiting for grid header (4 bytes: rows, cols) from the base ESP32...")
                header = ser.read(4)
                if len(header) < 4:
                    print("TIMED OUT waiting for the grid header -- still waiting for the next round.")
                    continue
                rows = int.from_bytes(header[0:2], "little")
                cols = int.from_bytes(header[2:4], "little")
                print(f"Grid is {rows}x{cols} ({rows * cols} cells)")

                cell_bytes = ser.read(rows * cols)
                if len(cell_bytes) < rows * cols:
                    print(f"TIMED OUT -- only got {len(cell_bytes)} of {rows * cols} expected cell bytes -- "
                          "moving on to the next round.")
                    continue
                print(f"Received {len(cell_bytes)} cell bytes.")
                print(f"Row 0: {list(cell_bytes[:cols])}")
                print(f"Row {rows - 1}: {list(cell_bytes[(rows - 1) * cols: rows * cols])}")

                print(f"Sending canned path back: {FAKE_PATH}")
                ser.write(encode_waypoints(FAKE_PATH))
        except KeyboardInterrupt:
            print("\nStopped by user.")


if __name__ == "__main__":
    main()
