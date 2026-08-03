#!/usr/bin/env python3
"""
Stand-in for the real ZED-camera Jetson code -- lets you test the ROVER
ESP32's grid-input path (and the whole LoRa round trip) without waiting on
the real ZED pipeline to be finished.

Connects to the ROVER's native USB port (the same one used to flash it --
this bench-test firmware build runs its binary data protocol over `Serial`
with debug prints compiled out, see ENABLE_SERIAL_DEBUG in main.cpp), sends
a [rows:2][cols:2][cell bytes...] test grid (the rover now accepts any size,
not just a hardcoded 20x20), then waits for the waypoint instructions the
rover forwards back after its LoRa round trip with the base station.

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
    # Comfortably exceeds the rover's own 60s give-up budget for a round (see
    # INSTRUCTIONS_TIMEOUT_MS in main.cpp) -- otherwise this script could time
    # out and move on just before the rover was about to report back.
    with serial.Serial(port, BAUD, timeout=70) as ser:
        time.sleep(2.5)  # let the ESP32's boot-time reset (and ROM boot log) finish
        ser.reset_input_buffer()  # discard the ROM/bootloader banner queued during the reset above
        ser.reset_output_buffer()

        round_num = 0
        try:
            while True:
                round_num += 1
                print(f"\n--- Round {round_num} ---")
                # Discard anything still sitting in the input buffer from a
                # previous round's response that arrived after we'd already
                # given up on it -- otherwise those stale bytes get read as
                # this round's response instead, silently shifting every
                # later round's data by one.
                ser.reset_input_buffer()

                grid_bytes = build_test_grid()
                header = GRID_ROWS.to_bytes(2, "little") + GRID_COLS.to_bytes(2, "little")
                print(f"Sending grid header ({GRID_ROWS}x{GRID_COLS}) + {len(grid_bytes)} cell bytes...")
                ser.write(header + grid_bytes)

                print("Waiting for instructions back from the rover (after its LoRa round trip)...")
                count_byte = ser.read(1)
                if len(count_byte) < 1:
                    print("TIMED OUT waiting for the waypoint count byte -- moving on to the next round.")
                    continue
                count = count_byte[0]
                print(f"Waypoint count: {count}")
                if count == 0:
                    print("Rover reported no instructions for this round (base never confirmed the grid, "
                          "or the instructions leg timed out) -- moving on.")
                    continue

                waypoint_bytes = ser.read(count * 4)
                if len(waypoint_bytes) < count * 4:
                    print(f"TIMED OUT -- only got {len(waypoint_bytes)} of {count * 4} expected bytes -- "
                          "moving on to the next round.")
                    continue

                for i in range(count):
                    x = int.from_bytes(waypoint_bytes[i * 4:i * 4 + 2], "little", signed=True)
                    y = int.from_bytes(waypoint_bytes[i * 4 + 2:i * 4 + 4], "little", signed=True)
                    print(f"  waypoint {i}: ({x}, {y})")
        except KeyboardInterrupt:
            print("\nStopped by user.")


if __name__ == "__main__":
    main()
