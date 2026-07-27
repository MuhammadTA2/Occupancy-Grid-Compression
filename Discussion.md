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

## 07/21/2026 — Separate `StreamFormat` from `EntropyCoderType`; `StreamFormat` lives inside the entropy-coded blob

### Decision
Two independent metadata tags travel with compressed data, not one:
- `StreamFormat` (`Raw`, `RLE`, `Delta`, ...) describes how `SymbolStream::symbols` is structurally arranged after preprocessing. It says nothing about entropy coding.
- `EntropyCoderType` (`None`, `Huffman`, `Rice`, `Arithmetic`, ...) describes which entropy algorithm turned a `SymbolStream` into transmitted bytes. It says nothing about symbol meaning.

`SymbolStream` stays `{ StreamFormat format; std::vector<Cell> symbols; }` and is specifically the boundary type *between* preprocessing and entropy coding — it does not represent entropy-coded output. `IEntropyCoder::encode()` takes a `SymbolStream` and returns a plain `vector<uint8_t>` (never another `SymbolStream`); `decode()` reverses that. This reaffirms the original Milestone 5 sketch's asymmetric `encode`/`decode` shape, which a same-session detour (a composite `StreamFormat::RLE_Huffman` tag, explored but never committed) had drifted away from.

`EntropyCoderType` is carried in packet metadata, outside the entropy-coded payload — the receiver must know which decoder to run before any decoding can happen, so it cannot live inside the thing being decoded. `StreamFormat` is embedded inside the entropy-coded blob itself instead (part of what `IEntropyCoder::encode()` serializes and `decode()` recovers) — the receiver only needs it after entropy decoding already succeeded, to pick the inverse preprocessor, so it isn't a decode-time prerequisite the way `EntropyCoderType` is. Keeping `format` and `symbols` sourced from the same decode step (instead of splitting one logical `SymbolStream` across the packet header and the payload) also avoids a corruption class where a mismatched header could mislabel correctly-decoded bytes without tripping the payload checksum.

### Why
Conflating "how the symbols are structured" with "which entropy coder produced these bytes" into a single tag doesn't scale — every new preprocessor x entropy-coder combination would need its own enum value. Separating the two axes lets any preprocessor compose with any entropy coder without combinatorial tag growth, and matches the `IPreprocessor`/`IEntropyCoder` role split already accepted on 07/06/2026.

### Wire format correction
RLE's actual on-the-wire symbol layout is a fixed 3-byte record per run — `[value: 1 byte][count: 2 bytes, little-endian]` — not a naive alternating `[value, count, value, count, ...]` single-byte pairing. `count` must be 2 bytes because a single run can span an entire tile (up to ~10,000 cells for a 100x100 grid), which doesn't fit in 1 byte; this was already established in the 07/20/2026 hardening pass (`RLEbits.count` as `uint16_t`). Malformed-stream tests for RLE (Milestone 3) should check "byte length not a multiple of 3," not "odd element count."

### Packet metadata clarification
An entropy coder's codebook/frequency table (e.g. Huffman's) is per-payload content, not a fixed packet-header field — it stays embedded inside the entropy-coded blob (the coder's self-contained-blob requirement, per the original Milestone 5 note), the same place `StreamFormat` now lives. Packet metadata carries only fixed-shape fields: `EntropyCoderType`, protocol version, payload length, tile/layer id, checksum.

### Mechanism note (recommended, not yet locked in)
The `IPreprocessor`/`IEntropyCoder` split from 07/06/2026 is a role separation, not an implementation mandate — it doesn't require virtual base classes. Recommending tag-dispatch instead: `StreamFormat`/`EntropyCoderType` enums, one namespace per algorithm (`compressor::rle::encode`/`decode`, later `compressor::huffman::encode`/`decode`), and a small `switch`-based dispatcher (`compressor::encode`/`compressor::decode` for preprocessing, a separate one for entropy coding) rather than an `IPreprocessor`/`IEntropyCoder` class hierarchy. No vtables, no heap-allocated polymorphism — a better fit for the ESP32 target than the originally-planned ABC design. This changes the *mechanism* Milestones 4-5 use, not the role split itself.

### Status
Accepted (role separation, blob placement, wire format correction). Mechanism note: recommended default, pending confirmation.

## 07/22/2026 — `SymbolStream::symbols` is generic bytes, not `Cell`

### Decision
`SymbolStream::symbols` is `std::vector<uint8_t>`, not `std::vector<Cell>`. Reverts a same-week instruction to use the `Cell` alias wherever a value was `uint8_t`; that instruction was correct for `RLEbits::value` and the wire-serialization helpers inside `rle.cpp`, but not for `SymbolStream` itself.

### Why
`SymbolStream` is the narrow-waist type from the 07/06/2026 decision, specifically meant to be shared across every current and future map producer (occupancy grids now, height maps or other layers later) and consumed generically by any entropy coder/packetizer/transport. `Cell` is documented in `grid.h` as occupancy-specific (`0=free, 1=obstacle, 2=uncertain`). Typing `SymbolStream::symbols` as `vector<Cell>` would make the narrow-waist type quietly still speak occupancy-grid vocabulary — a future non-occupancy producer would have to hand it values that aren't occupancy states at all, under a type that says otherwise. Separately, and more concretely: after RLE preprocessing, `symbols` holds serialized run records (a value byte followed by two count bytes), not cell values — labeling that `Cell` is actively wrong, not just imprecise. `Cell` and "generic byte" coincide at `uint8_t` today, but that's a width coincidence, not a semantic match, and leaning on it would undercut the same reasoning that already justified giving `SymbolStream` its own distinct struct instead of a bare alias.

### What stays `Cell`
`Grid`/`Tile::data` — genuinely occupancy-specific, correctly named. `toSymbolStream`/`fromSymbolStream` are where the real `Cell ↔ uint8_t` conversion happens (a no-op copy today, since both are 1-byte), and that conversion existing at the type level is what actually enforces the boundary, not just a naming convention.

### Status
Accepted.

## 07/22/2026 — Replace Huffman milestone with Varint/LEB128 RLE count encoding

### Decision
Huffman entropy coding is deferred (not removed — see "Status" below). In its place, RLE's own serialization step is changed to encode each run's `count` as an unsigned LEB128 varint instead of a fixed 2-byte little-endian integer: `[value: 1 byte][count: LEB128, 1-3 bytes]`. `RLEbits::value` (the occupancy value, currently 0/1/2) stays a plain fixed 1 byte — LEB128 is not applied to it; its domain is tiny and fixed-width is already optimal.

A new standalone module, `compressor::varint` (`include/varint.h`), provides generic unsigned-integer encode/decode. `rle.cpp` calls into it for `count`; it carries no RLE-specific meaning and is available to any future preprocessor or packet field that wants compact integers.

### Why
Exact run counts don't repeat enough across a grid to justify Huffman's frequency-table/codebook overhead relative to payload size at this stage. Most run counts are expected to be small, so a variable-length integer format captures most of the win with far less complexity and no persistent per-payload codebook. This is a serialization-format change to `StreamFormat::RLE`, not an entropy-coding decision — see "EntropyCoderType unaffected" below.

### Supersedes: 07/21/2026 wire format correction
That entry documented RLE's wire record as a fixed 3 bytes (`[value:1][count:2]`). This is now replaced by a variable-length record (`[value:1][count: LEB128]`), self-terminating via the continuation bit — no separate record-count/length header is needed. Malformed-stream tests for RLE (Milestone 3, and the new Milestone 6) must be updated accordingly: a "not a multiple of 3" check no longer applies; instead, decode must detect truncated varints, excess continuation bytes, and out-of-range values (see below).

### Malformed-input handling
The varint decoder (and, by extension, RLE decode) must reject rather than silently misdecode:
- **Truncated input**: continuation bit set on the last available byte.
- **Too many continuation bytes**: bounded by the value domain in use — for RLE's `uint16_t` count, at most 3 bytes; a 3rd byte carrying more than its top 2 significant bits, or a 4th byte at all, is malformed.
- **Overflow**: a fully-decoded value that doesn't fit the target integer width (`uint16_t` for RLE counts).
- **Zero counts**: `rleEncode` never emits `count == 0` (a run is never empty by construction), so a decoded `count == 0` indicates a corrupted or adversarial stream and is rejected.
All rejections follow the existing no-exceptions convention: return an empty/`Unknown`-tagged result, not a throw.

### EntropyCoderType unaffected
LEB128 is a structural/serialization concern nested inside `StreamFormat::RLE`'s own encode/decode — it is not an entropy coder and does not get an `EntropyCoderType` value. Milestone 5's `EntropyCoderType::None` dispatch (`SymbolStream`'s own serialize/deserialize) is unchanged; it now simply wraps RLE's more compact byte stream. `EntropyCoderType::Huffman`/`Rice`/`Arithmetic` remain named-but-unimplemented future extensions, unaffected by this change.

### Relationship to `BitWriter`/`BitReader`
That utility (see 07/21/2026 discussion, was slated for the old Milestone 6) was purpose-built for entropy coders' bit-level packing. LEB128 is byte-oriented — every read/write is a whole byte — so it does not need `BitWriter`/`BitReader`. That utility moves to "explicitly deferred" alongside Huffman rather than being built now; it remains the right tool if Rice or Arithmetic coding are ever implemented.

### Status
Accepted (Varint replaces Huffman as the active milestone; Huffman itself is deferred, not deleted, and `EntropyCoderType::Huffman` stays in the enum as a marked future extension).
