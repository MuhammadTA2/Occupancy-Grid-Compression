# Telemetry Compression Project Specification

## Overview

This project explores efficient telemetry transmission for occupancy grid maps over
bandwidth-constrained communication links.

The project is inspired by robotic exploration and future space applications where
communications are limited by bandwidth, latency, energy, and reliability.

Although the current implementation runs on an ESP32, the software architecture
should remain independent of the communication hardware.

---

# Objectives

Primary goals:

- Learn data compression
- Learn telemetry systems
- Learn software architecture
- Learn embedded systems
- Learn wireless communications
- Build an impressive engineering portfolio project

---

# Long-Term Vision

Eventually this project should support:

- ESP32
- LoRa
- SDR
- Optical communication
- Satellite links
- Deep-space telemetry simulations

The software should make changing communication hardware require minimal code
changes.

---

# Current Occupancy Grid

Grid size:

100 x 100

Representation:

Flattened 1D vector.

Index calculation:

index = row * cols + column

Reason:

- contiguous memory
- cache friendly
- easier serialization
- lower overhead

---

# Grid Structure

Grid contains

- rows
- cols
- vector<int> data

---

# Tile Structure

Tiles currently contain

- rowStart
- colStart
- rows
- cols
- vector<int> data

Tiles represent subregions of the occupancy grid.

---

# Current Compression Pipeline

Occupancy Grid

↓

Split into tiles

↓

Run-Length Encoding

↓

Huffman Encoding

↓

Transmit

↓

Huffman Decode

↓

Run-Length Decode

↓

Reconstruct Tile

↓

Reconstruct Grid

---

# Planned Pipeline

Occupancy Grid

↓

Detect changed tiles

↓

Compress only changed tiles

↓

Packetize

↓

Transmit

↓

Receiver reconstructs changed tiles

↓

Rebuild global map

---

# Compression Modules

Current

- Run-Length Encoding
- Huffman Coding

Future experiments

- Delta encoding
- Arithmetic Coding
- LZ77
- LZ4
- Deflate
- Quadtrees
- Sparse encoding
- Predictive coding
- Wavelets
- Low-rank approximation

Compression algorithms should be interchangeable.

---

# Communication Layer

Current

ESP32

Future

LoRa

SDR

Optical

Laser

Satellite

The communication layer should not depend on the compression layer.

---

# Packet Layer

Eventually packets should include

Packet ID

Tile ID

Timestamp

Compressed payload

Checksum

CRC

Future:

Forward Error Correction

Acknowledgements

Retransmissions

Bandwidth estimation

Adaptive packet sizing

---

# Adaptive Features

Future goals include

Adaptive compression

Adaptive packet size

Adaptive bitrate

Adaptive transmission frequency

Bandwidth-aware compression

Energy-aware compression

---

# Software Architecture

Design principles

- modular
- low coupling
- high cohesion
- replaceable algorithms
- reusable components
- benchmark friendly
- unit testable

---

# Future Benchmarks

Compression ratio

Compression speed

Decompression speed

Memory usage

CPU usage

Energy usage

Latency

Bandwidth utilization

Packet loss tolerance

---

# Current Milestone

Implement

- Grid
- Tile
- RLE
- Huffman
- Reconstruction

Then move toward dynamic tile transmission.

---

# Coding Philosophy

Avoid hacks.

Prefer maintainable architecture over short-term solutions.

Always discuss engineering tradeoffs before implementing features.

When suggesting code:

- explain why
- explain tradeoffs
- preserve modularity
- avoid unnecessary dependencies
