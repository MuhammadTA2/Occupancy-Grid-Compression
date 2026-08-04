#!/usr/bin/env python3
"""
Stand-in for the real base-station path-planning computer -- lets you test
the BASE ESP32's grid+position-output/instructions-input path (and the whole
LoRa round trip) without waiting on the real A* implementation.

Connects to the BASE's native USB port (the same one used to flash it --
this bench-test firmware build runs its binary data protocol over `Serial`
with debug prints compiled out, see ENABLE_SERIAL_DEBUG in main.cpp), waits
for the decompressed grid + rover position the base forwards after receiving
it over LoRa from the rover, prints a summary, then sends back a canned
waypoint list as a stand-in for a real computed path.

Reads [cell bytes...][x:2][y:2] with no header. The framing has no length
field, so GRID_ROWS/GRID_COLS below must match the firmware's constants
exactly -- this is the same contract base/main_base.py implements via
map_dis.LOCAL_SIZE.

Usage: python simulate_basestation.py <COM port or /dev/ttyUSBx> [--no-reset]
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
CELLS = GRID_ROWS * GRID_COLS
FRAME_LEN = CELLS + 4

# Canned path -- stand-in for real A* output.
FAKE_PATH = [(2, 2), (5, 8), (10, 15), (18, 18)]


def encode_waypoints(waypoints):
    out = bytearray()
    out.append(len(waypoints))
    for x, y in waypoints:
        out += int(x).to_bytes(2, "little", signed=True)
        out += int(y).to_bytes(2, "little", signed=True)
    return bytes(out)


def open_port(port, reset):
    """Opening with DTR/RTS asserted resets the ESP32. That's the default and
    usually what you want at the start of a session: these USB legs are
    header-less, so a reset is the *only* way to resynchronize one that has
    drifted. Pass --no-reset to attach to a board mid-round without
    disturbing it -- otherwise merely starting this script can destroy the
    round you were trying to observe."""
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = BAUD
    ser.timeout = 90
    if not reset:
        ser.dtr = False
        ser.rts = False
    ser.open()
    if reset:
        time.sleep(2.5)  # let the ESP32's boot-time reset (and ROM boot log) finish
        ser.reset_input_buffer()  # discard the ROM/bootloader banner queued during the reset above
        ser.reset_output_buffer()
    return ser


def describe(cell_bytes, x, y):
    hist = {}
    for b in cell_bytes:
        hist[b] = hist.get(b, 0) + 1
    print(f"Cell value histogram: {dict(sorted(hist.items()))}")
    stray = set(hist) - {0, 1, 2}
    if stray:
        print(f"  ^^ values {sorted(stray)} are outside the valid set {{0,1,2}} -- this leg is "
              "desynced, or the two boards disagree on the grid size.")
    print(f"Row 0:              {list(cell_bytes[:GRID_COLS])}")
    print(f"Row {GRID_ROWS - 1}:             {list(cell_bytes[(GRID_ROWS - 1) * GRID_COLS:CELLS])}")
    print(f"Rover position:     ({x}, {y})")


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
                # See the matching comment in simulate_jetson.py -- discards any
                # stale bytes left over from a round whose response arrived
                # after we'd already stopped waiting on it.
                ser.reset_input_buffer()

                print(f"Waiting for {FRAME_LEN} bytes ({CELLS} cells + 4 position bytes) "
                      "from the base ESP32...")
                frame = ser.read(FRAME_LEN)
                if len(frame) < FRAME_LEN:
                    print(f"TIMED OUT -- only got {len(frame)} of {FRAME_LEN} expected bytes -- "
                          "moving on to the next round.")
                    continue

                # There is no header on this leg. If the first four bytes are
                # the grid dimensions, the board is running an older firmware
                # build and everything after is shifted by four.
                if frame[:4] == GRID_ROWS.to_bytes(2, "little") + GRID_COLS.to_bytes(2, "little"):
                    print(f"!! Frame starts with {GRID_ROWS},{GRID_COLS} little-endian -- that is a "
                          "[rows][cols] HEADER, which this firmware does not send. The board is "
                          "almost certainly running an older build. Reflash before trusting anything below.")

                cell_bytes = frame[:CELLS]
                x = int.from_bytes(frame[CELLS:CELLS + 2], "little", signed=True)
                y = int.from_bytes(frame[CELLS + 2:CELLS + 4], "little", signed=True)
                print(f"Received {len(frame)} bytes.")
                describe(cell_bytes, x, y)

                print(f"Sending canned path back: {FAKE_PATH}")
                ser.write(encode_waypoints(FAKE_PATH))
        except KeyboardInterrupt:
            print("\nStopped by user.")


if __name__ == "__main__":
    main()
