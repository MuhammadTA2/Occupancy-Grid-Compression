# Engineering Decisions

## 07/06/2026 — Use a narrow-waist architecture

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

## 07/06/2026 — Defer Map/Layer hierarchy

### Decision
`Grid`/`Tile` remain single-layer. We are not introducing a `Map` → `Layer` → `Tile` hierarchy yet, even though the long-term vision includes multiple map layers (occupancy, height, traversability, confidence).

### Why
`SymbolStream` is the narrow-waist boundary between map representation and everything downstream (entropy coders, packets, transport). Because RLE, Huffman, packetization, and transport only ever see `SymbolStream`, a future `Map`/`Layer` upgrade is fully containable inside the grid module and its `toSymbolStream`/`fromSymbolStream` conversions — nothing downstream has to change when it lands.

### Tradeoffs
Pros:
- Avoids designing heterogeneous-layer storage (variant/type-erasure/virtual dispatch) against only one real layer to validate it
- Avoids heap-backed type erasure and RTTI/vtable overhead on ESP32 before it's actually needed
- Keeps the current milestone-by-milestone refactor scoped and low-risk

Cons:
- When a second layer (e.g. height) is actually needed, `Grid`/`Tile` will require a real (bounded) migration rather than already being ready for it

### Alternatives considered
- Introduce `Map` → `Layer` → `Tile` now, modeled on ANYbotics' `grid_map` (named layers sharing common geometry) or `costmap_2d`'s layered costmap
- Use a homogeneously-typed layer container (all layers same concrete type, e.g. float) to sidestep heterogeneous-type storage, even now

### Future migration path (not scheduled)
Promote `Grid` → `Layer`; add `Map` as shared geometry (rows/cols/resolution/origin) plus a name → `Layer` registry; keep `SymbolStream` and everything downstream of it unchanged.

### Status
Accepted

## 07/06/2026 — Split `ICompressor` into `IPreprocessor` and `IEntropyCoder`

### Decision
Compression algorithms are modeled behind two interfaces instead of one: `IPreprocessor` (`SymbolStream → SymbolStream`, e.g. RLE) for structural transforms, and `IEntropyCoder` (`SymbolStream → bytes`) for statistical entropy coding (e.g. Huffman). An earlier plan modeled both as peer implementations of a single `ICompressor`.

### Why
RLE and Huffman are different architectural roles, not alternative implementations of the same role. RLE exploits structural redundancy (runs of repeated values) and rewrites a symbol stream as a shorter symbol stream; it does not produce a compressed bitstream on its own. Huffman exploits the statistical distribution of symbol frequencies to produce an actual compressed bitstream. Modeling both as one terminal `SymbolStream → bytes` interface makes them mutually exclusive alternatives, but `ProjectSpecs.md`'s own "Current Compression Pipeline" chains them (RLE then Huffman) — the single-interface design could not express the pipeline the project already documents.

### Tradeoffs
Pros:
- Matches the documented pipeline (RLE → Huffman) for the first time
- Two independent swap axes (preprocessors, entropy coders) instead of one, serving "replaceable algorithms" and "benchmark friendly" more thoroughly
- Future preprocessors (delta encoding, move-to-front, BWT) and future entropy coders (Rice, arithmetic/range coding) both have a natural home without faking being terminal compressors

Cons:
- Two interfaces to design and test instead of one
- Milestone 5 must end with a real composed RLE→Huffman pipeline (more code than comparing them side by side as originally planned)

### Alternatives considered
- Keep the single `ICompressor` interface, treat RLE and Huffman as selectable alternatives rather than composable stages

### Status
Accepted

## 07/06/2026 — Lessons learned: silent bugs from the Grid/RLE/Metrics module split

### What happened
Splitting the monolithic `rle.h`/`rle.cpp` into `grid.h`/`grid.cpp`, `rle.h`/`rle.cpp`, and `metrics.h`/`metrics.cpp` (Milestone 2) introduced two real bugs, neither of which produced a compiler error or warning:

1. `main.cpp`'s call to `compressionRatio` passed its two `size_t` arguments in the reverse order of the function's declared parameter list (`compressed, uncompressed`), silently inverting the printed ratio.
2. `metrics.cpp`'s implementation computed `original/compressed` as integer division (both operands were `size_t`) and only converted to `double` on return, truncating the result.

Both type-check fine — two same-typed arguments happily bind in either order, and integer division silently produces a `double`-compatible value on return, just the wrong one.

### Why the existing test suite didn't catch them
`test_compressionRatio` originally used 100 identical values, which RLE-encodes to a single run and produces an exact ratio (`400 bytes / 8 bytes = 50.0`). Integer division and floating-point division agree whenever the division happens to be exact, so the truncation bug passed the test while still being live in `main.cpp`. The argument-order bug wasn't caught at all, because no test exercises `main.cpp`'s own call sites — the test suite only calls the library API directly, in whatever order each test chooses.

### Why this matters going forward
- A test's input data should be chosen to break plausible wrong implementations, not just to exercise the happy path. A ratio/metric test in particular should avoid inputs whose correct answer happens to be a round number — that's exactly where integer-truncation bugs hide.
- Two adjacent same-typed parameters (`size_t compressed, size_t uncompressed`) are easy to pass in the wrong order at any call site, and nothing catches it automatically — not the compiler, not a same-shaped unit test.
- Unit tests validate library functions; they don't validate how callers actually invoke them. A bug can live entirely in a call site rather than in any function that has a test.

### Resulting change
Milestone 1's test suite was revised (after the fact) to use input data whose correct compression ratio is not a whole number, specifically so this class of bug can't hide behind a convenient round number again.

### Status
Recorded

## 07/20/2026 — Milestone 2 hardening: narrower cell/run types

### What happened
`Grid`/`Tile` data was changed from `std::vector<int>` to `std::vector<Cell>` (`Cell = uint8_t`), and `RLEbits` from `{int value; int count;}` to `{uint8_t value; uint16_t count;}` (with a guard forcing a new run before `count` wraps past 65535). This cuts grid/tile memory 4x, matters for the eventual ESP32 target, and required updating `main.cpp` and `tests/test_main.cpp` call sites to match — both still declared `vector<int>` and one used `sizeof(int)` for the "original size" byte count, which would have silently overstated the compression ratio by 4x once `Cell` shrank to 1 byte.

### Why
Occupancy values only need three states (free/obstacle/uncertain); a 4-byte `int` per cell is wasted memory on a target where memory is the scarce resource. This is a pure representation change — no new interfaces, no behavior change beyond byte width.

### Verification
Rebuilt and reverified: 24/24 tests pass, demo round-trips correctly (`main.cpp`'s printed ratio changed from `8.10373` to `4.05186`, which is expected — both the original and compressed operand byte counts shrank, not a regression).

### Status
Accepted

## 07/20/2026 — Packet prototype written early, not adopted as Milestone 6

### What happened
A `Packet` prototype (`include/packet.h`/`src/packet.cpp`) was written ahead of schedule: a packed `PacketHeader` + payload, an XOR checksum, and `packetizeTile`/`reassembleTile` for splitting a tile's serialized RLE stream into LoRa-radio-sized (~200 byte) chunks.

### Why it isn't being adopted as-is
It couples directly to `RLEbits` (`serializeRLE`/`deserializeRLE` hard-code RLE's wire format into the packet layer), which contradicts the narrow-waist decision recorded above — `Packet` was meant to depend only on the opaque bytes `IEntropyCoder::encode()` produces, not on a specific preprocessor's struct. It also implements LoRa-specific multi-packet chunking (`packetizeTile`/`reassembleTile`, the 255-byte SX127x payload limit), which this plan's "Explicitly deferred" list already scopes out for this pass.

### Resolution
- `include/packet.h`/`src/packet.cpp` are committed as-is, **not wired into the build** (not referenced in `src/CMakeLists.txt`), and explicitly not treated as Milestone 6 being complete.
- Both files carry a header comment marking them as a prototype/design reference, not final.
- Milestone 6 will rebuild `Packet` fresh against `IEntropyCoder`'s byte output once Milestones 3-5 exist. This prototype is expected to be replaced (not extended) at that point.
- The LoRa-specific chunking (`packetizeTile`/`reassembleTile`, radio payload-size limits) is split out as its own future/deferred item, separate from core `Packet` framing.

### Status
Recorded — superseded upon Milestone 6 implementation
