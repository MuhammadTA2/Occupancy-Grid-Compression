# Engineering Decisions

## 2026-07-06 — Use a narrow-waist architecture

### Decision
The pipeline will convert map-specific data into generic `SymbolStream`s before entropy coding.

### Why
This lets occupancy grids, height maps, and future map layers use different preprocessing while reusing the same entropy coder, packetizer, and communication layer.

### Tradeoffs
Pros:
- Easier to add height maps later
- Keeps Huffman/Rice independent of map semantics
- Cleaner testing and benchmarking

Cons:
- More files and abstractions now
- Slightly slower progress on immediate RLE implementation

### Alternatives considered
- Keep RLE as the center of the project
- Hardcode occupancy grid compression first, generalize later

### Status
Accepted
