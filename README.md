# Telemetry Compression

An experimental telemetry compression system for transmitting occupancy grid maps
over bandwidth-constrained communication links.

Current features

- Occupancy grid representation
- Dynamic tiling
- Run-Length Encoding
- Grid reconstruction

Future work
- Huffman coding
- Delta encoding
- LoRa communication
- SDR support
- Adaptive compression
- Error correction
- Packet protocol
- Benchmark suite

The project is designed to investigate efficient telemetry systems for robotics,
autonomous exploration, and future space communication.

## Build

Requires CMake 3.16+ and a C++17 compiler.

```
cmake -S . -B build
cmake --build build
```

This produces `build/src/occupancy_grid`, which runs a demo: it builds a synthetic
100x100 occupancy grid, round-trips it through RLE encoding and through tiling, and
prints pass/fail results plus the compression ratio.
