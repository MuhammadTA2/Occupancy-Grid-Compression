#!/usr/bin/env python3
"""
Stand-in for the real ZED-camera Jetson code -- lets you test the ROVER
ESP32's Serial2 grid-input path (and the whole LoRa round trip) without
waiting on the real ZED pipeline to be finished.

Connects to the ROVER's Serial2 link (via a USB-to-serial adapter wired to
GPIO16/17+GND, NOT the ESP32's USB programming port -- that's a separate
UART carrying only debug text), sends a 20x20 test grid, then waits for the
waypoint instructions the rover forwards back after its LoRa round trip
with the base station.

Usage: python simulate_jetson.py <COM port or /dev/ttyUSBx>
"""
import sys
import time

try:
    import serial
except ImportError:
    print("Missing dependency: pip install pyserial")
    sys.exit(1)

BAUD = 115200
GRID_ROWS = 20
GRID_COLS = 20


def build_test_grid():
    # Same bordered-square-with-a-gapped-wall pattern as the firmware's
    # buildTestGrid() used to generate internally -- easy to eyeball whether
    # the round trip preserved it correctly.
    grid = [0] * (GRID_ROWS * GRID_COLS)
    for c in range(GRID_COLS):
        grid[0 * GRID_COLS + c] = 1
        grid[(GRID_ROWS - 1) * GRID_COLS + c] = 1
    for r in range(GRID_ROWS):
        grid[r * GRID_COLS + 0] = 1
        grid[r * GRID_COLS + (GRID_COLS - 1)] = 1
    for r in range(3, 17):
        grid[r * GRID_COLS + 10] = 1
    for r in range(8, 12):
        grid[r * GRID_COLS + 10] = 0
    return bytes(grid)


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <port>")
        sys.exit(1)
    port = sys.argv[1]

    print(f"Opening {port} at {BAUD} baud...")
    with serial.Serial(port, BAUD, timeout=30) as ser:
        time.sleep(2)  # let things settle if opening the port reset anything

        grid_bytes = build_test_grid()
        print(f"Sending {len(grid_bytes)} grid bytes ({GRID_ROWS}x{GRID_COLS})...")
        ser.write(grid_bytes)

        print("Waiting for instructions back from the rover (after its LoRa round trip)...")
        count_byte = ser.read(1)
        if len(count_byte) < 1:
            print("TIMED OUT waiting for the waypoint count byte.")
            return
        count = count_byte[0]
        print(f"Waypoint count: {count}")

        waypoint_bytes = ser.read(count * 4)
        if len(waypoint_bytes) < count * 4:
            print(f"TIMED OUT -- only got {len(waypoint_bytes)} of {count * 4} expected bytes.")
            return

        for i in range(count):
            x = int.from_bytes(waypoint_bytes[i * 4:i * 4 + 2], "little", signed=True)
            y = int.from_bytes(waypoint_bytes[i * 4 + 2:i * 4 + 4], "little", signed=True)
            print(f"  waypoint {i}: ({x}, {y})")

        print("Done.")


if __name__ == "__main__":
    main()
