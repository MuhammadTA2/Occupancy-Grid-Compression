#!/usr/bin/env python3
"""
Stand-in for the real ZED-camera Jetson code -- lets you test the ROVER
ESP32's grid-input path (and the whole LoRa round trip) without waiting on
the real ZED pipeline to be finished.

Connects to the ROVER's native USB port (the same one used to flash it --
this bench-test firmware build runs its binary data protocol over `Serial`
with debug prints compiled out, see ENABLE_SERIAL_DEBUG in main.cpp), sends
[cell bytes...][x:2][y:2] with no header, then waits for the waypoint
instructions the rover forwards back after its LoRa round trip with the base
station.

The framing has no header and no length field, so GRID_ROWS/GRID_COLS below
must match the firmware's constants exactly. They are checked nowhere at
runtime -- a mismatch just desyncs the leg silently.

Usage: python simulate_jetson.py <COM port or /dev/ttyUSBx> [--no-reset]
"""
import sys
import time

try:
    import serial
except ImportError:
    print("Missing dependency: pip install pyserial")
    sys.exit(1)

BAUD = 115200
GRID_ROWS = 70  # must match GRID_ROWS in firmware/src/main.cpp
GRID_COLS = 70  # must match GRID_COLS in firmware/src/main.cpp

# Deliberately asymmetric and signed, so an x/y swap or a sign error anywhere
# downstream shows up as an obviously wrong number rather than a plausible one.
ROVER_POSITION = (12, -34)


def build_test_grid():
    # Bordered square with a gapped interior wall -- easy to eyeball whether
    # the round trip preserved it correctly.
    grid = [0] * (GRID_ROWS * GRID_COLS)
    for c in range(GRID_COLS):
        grid[0 * GRID_COLS + c] = 1
        grid[(GRID_ROWS - 1) * GRID_COLS + c] = 1
    for r in range(GRID_ROWS):
        grid[r * GRID_COLS + 0] = 1
        grid[r * GRID_COLS + (GRID_COLS - 1)] = 1
    wall_col = GRID_COLS // 2
    for r in range(3, GRID_ROWS - 3):
        grid[r * GRID_COLS + wall_col] = 1
    for r in range(GRID_ROWS // 2 - 2, GRID_ROWS // 2 + 2):
        grid[r * GRID_COLS + wall_col] = 0
    return bytes(grid)


def encode_position(x, y):
    return int(x).to_bytes(2, "little", signed=True) + int(y).to_bytes(2, "little", signed=True)


def open_port(port, reset):
    """Opening with DTR/RTS asserted resets the ESP32. That's the default and
    usually what you want at the start of a session: these USB legs are
    header-less, so a reset is the *only* way to resynchronize one that has
    drifted. Pass --no-reset to attach to a board mid-round without
    disturbing it."""
    # Comfortably exceeds the rover's own 60s give-up budget for a round (see
    # INSTRUCTIONS_TIMEOUT_MS in main.cpp) -- otherwise this script could time
    # out and move on just before the rover was about to report back.
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = BAUD
    ser.timeout = 70
    if not reset:
        ser.dtr = False
        ser.rts = False
    ser.open()
    if reset:
        time.sleep(2.5)  # let the ESP32's boot-time reset (and ROM boot log) finish
        ser.reset_input_buffer()  # discard the ROM/bootloader banner queued during the reset above
        ser.reset_output_buffer()
    return ser


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args:
        print(f"Usage: {sys.argv[0]} <port> [--no-reset]")
        sys.exit(1)
    port = args[0]
    reset = "--no-reset" not in sys.argv

    print(f"Opening {port} at {BAUD} baud ({'resetting' if reset else 'not resetting'} the board)...")
    with open_port(port, reset) as ser:
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
                position_bytes = encode_position(*ROVER_POSITION)
                print(f"Sending {len(grid_bytes)} cell bytes ({GRID_ROWS}x{GRID_COLS}) "
                      f"+ position {ROVER_POSITION}...")
                ser.write(grid_bytes + position_bytes)

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
