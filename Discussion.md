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

## 2026-07-06 — Defer Map/Layer hierarchy

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

## 2026-07-06 — Split `ICompressor` into `IPreprocessor` and `IEntropyCoder`

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
