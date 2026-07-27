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

## 07/27/2026 — Milestone 4 implemented as SoA + 2-bit packed values; corrects the 07/22/2026 entry

### Decision
Milestone 4 shipped with a different wire shape than the 07/22/2026 entry above describes. RLE's structural output is `RLERuns { values, counts }` (Structure-of-Arrays — all values, then all counts), not interleaved `[value][count]` records. Values are bit-packed via a new `FixedWidthCoder` (2 bits/value for occupancy, built on new `BitWriter`/`BitReader` classes), not left as plain 1-byte fields. Counts are LEB128-encoded via a new `VarintCoder`, itself a thin wrapper over `compressor::varint`. A new `rlecodec` module (`include/rle_codec.h`) combines both into one self-contained blob: `[run count N: varint][values: N * bitWidth bits, packed][counts: N LEB128 varints]`.

### Why
Interleaving values and counts per-run would force either padding every varint to a byte boundary before it (wasting bits) or a bit-cursor-aware varint decoder (breaking its byte-oriented independence). SoA keeps `BitWriter`/`FixedWidthCoder` and `VarintCoder` each in their own natural, fully independent mode.

### Corrections to 07/22/2026
Two claims from that entry do not hold under this design:
- **"No separate record-count/length header is needed"** — false once values are bit-packed. A bit-packed values section is no longer self-delimiting by EOF the way the old byte-per-run scheme was, so `rlecodec::encode` prepends an explicit leading run count `N` (itself a varint). That header also fixes the values section's byte length deterministically (`ceil(N * bitWidth / 8)`), so no delimiter is needed between the values and counts sections either.
- **"`BitWriter`/`BitReader` move to deferred alongside Huffman"** — reversed. They were built now (`include/bit_writer.h`, `src/bit_writer.cpp`) and are load-bearing for `FixedWidthCoder`'s 2-bit value packing, not deferred.

Also worth noting for anyone reading the two entries together: `RLEbits` (the old `{value, count}` struct) no longer exists; `rle.h`/`rle.cpp` were rewritten around `RLERuns`. And the enum that shipped is `CoderType` (`FixedWidth`, `Varint`, `Unknown`), not `EntropyCoderType` — currently a thin descriptive tag with no dispatch scaffolding (there's exactly one coder per role so far), meant to become load-bearing once packet metadata exists (Milestone 7).

### Status
Accepted, implemented, and verified — build and full test suite pass, including a full-pipeline round trip on empty, single-run, and worst-case-checkerboard grids.

## 07/27/2026 — Milestone 5: Rice/Golomb benchmark, and CoderType becomes real dispatch (not a label)

### Decision
Built `compressor::rice` (`include/rice_coder.h`) — Golomb-Rice coding for run counts, parameterized by `k` — and a benchmark (`src/benchmark.cpp`) comparing it against `VarintCoder` across synthetic datasets (uniform, sparse obstacles, random noise, worst-case checkerboard). Result: Rice beats Varint on every non-trivial dataset (20-60% smaller counts sections), because LEB128 has a hard floor of 8 bits per count while Rice can go well below that once `k` is tuned to the data.

Rather than picking one of Varint/Rice as a new fixed default, `CoderType` was turned into an actual dispatch mechanism: a new `compressor::countscoder` module (`include/counts_coder.h`) switches on `CoderType` to call the matching coder's encode/decode, plus a `chooseBest()` that tries all known counts coders (Varint, and a k-search over Rice) and returns whichever is smallest for that specific payload. `rlecodec`'s blob format gained a 2-byte header (`[CoderType][riceParam]`) ahead of the run-count `N`, so the blob is self-describing — `rlecodec::decode` no longer needs to be told which counts coder was used; it reads that back out of the blob itself. `rlecodec::encode` auto-selects via `chooseBest` by default, with an explicit-choice overload for callers that want to force a specific coder.

### Why
This corrects a real design mistake made in the previous entry (07/27/2026, Milestone 4): `CoderType` was built as "a thin descriptive tag with no dispatch scaffolding," on the reasoning that there was only one coder per role so far. That missed the actual point of splitting `IPreprocessor`/`IEntropyCoder` into separate roles in the first place (07/06/2026 entry) — the split exists precisely so multiple interchangeable entropy coders can be written once (Rice, Golomb, Huffman, Varint, ...) and selected per use case through `SymbolStream`'s narrow waist, not so one gets crowned "the" default and the others sit unused. Once there were two real counts coders (Varint, Rice), building the actual switch-based dispatch was the correct move, not optional polish.

### Status
Accepted, implemented, and verified — build and full test suite pass (including new tests for `countscoder`'s dispatch, `chooseBest`, and `rlecodec`'s embedded-CoderType round trip and malformed-CoderType rejection). Corrects this document's own 07/27/2026 Milestone 4 entry, which is now stale on the "CoderType is a thin tag" point.

## 07/27/2026 — Milestones 7-10: Packetizer, ITransport/LoopbackTransport, PacketReassembler, and splitcodec

### Decision
Built the packetizer (`include/packetizer.h`, new -- the earlier `packet.h`/`packet.cpp` prototype stays untouched as a historical reference per the 07/20/2026 entry below, not extended): fixed 9-byte header (version, messageId, streamId, fragmentIndex, totalFragments, payloadLength), CRC-16-CCITT over header+payload, `MAX_PACKET_SIZE`/`HEADER_SIZE`/`CRC_SIZE`/`MAX_PAYLOAD_SIZE` as named constants. `streamId` is opaque -- the packetizer has zero knowledge of what "values" or "counts" mean.

Built `ITransport` (`include/transport.h`) as an actual virtual interface with `LoopbackTransport` (in-memory FIFO queue) implementing it. This is deliberately not tag-dispatch: transport swapping happens per-packet, not per-symbol, so vtable overhead is negligible here, unlike the compression layer where it was rejected specifically to avoid ESP32 vtable cost.

Built `PacketReassembler` (`include/packet_reassembler.h`), keyed by `(messageId, streamId)`, tracking received fragments in a `map<fragmentIndex, payload>` so `tryGetCompleteStream()` reassembles strictly by fragment index regardless of arrival order. Rejects a fragment whose `totalFragments` disagrees with earlier fragments of the same key, or whose `fragmentIndex >= totalFragments`; accepts genuine duplicates idempotently. `missingFragments()` exposes what Milestone 11's ARQ needs.

Introduced a new module, `splitcodec` (`include/split_codec.h`), to actually realize "the values stream and counts stream fragment independently" (07/06/2026 / Milestone 7 roadmap intent): `rlecodec` (Milestone 4/5) produces one combined self-contained blob, which is the wrong shape to hand to the packetizer under two separate `streamId`s. `splitcodec::encode` instead returns two byte sequences: `valuesBytes` (raw `FixedWidthCoder` output, no header -- its length is deterministic once `N` and `valueBitWidth` are known) and `countsBytes` (self-describing: `[CoderType][riceParam][N: varint][counts]`). `N` deliberately lives only in `countsBytes`, not duplicated in both streams -- decode reads `countsBytes` first to learn `N`, which also gives it `valuesBytes`' expected deterministic length, so a length mismatch between the two streams is detectable as corruption rather than silently misdecoded.

Wired the full pipeline together in `src/main.cpp`: `Grid -> toSymbolStream -> rleEncode -> splitcodec::encode -> packetizer::fragment` (independently per stream) `-> LoopbackTransport -> PacketReassembler -> splitcodec::decode -> rleDecode -> fromSymbolStream -> Grid`. This is the first true end-to-end demonstration of the documented pipeline, not each piece tested in isolation.

### Why
`rlecodec`'s single-blob design was the right call when it was built (Milestone 4/5, before a packetizer existed) but doesn't compose with Milestone 7's fault-containment goal: fragmenting one combined blob under a single `streamId` means a single corrupted fragment can straddle the values/counts boundary and take down both, exactly what independent fragmentation was meant to prevent. `rlecodec` itself wasn't changed -- it remains a valid convenience API for callers that want one blob and don't need independent fragmentation (e.g. the Milestone 4/5 tests and demo block still use it as-is).

### Status
Accepted, implemented, and verified — build and full test suite pass, including packetizer fragmentation/serialization/CRC/malformed-input tests, `LoopbackTransport` FIFO tests, `PacketReassembler` out-of-order/duplicate/inconsistent-header/missing-fragment tests, `splitcodec` round-trip and disagreeing-stream-length tests, and the full loopback pipeline demo in `main.cpp`.

## 07/27/2026 — Milestone 11: ARQ over a lossy transport, plus a real bug caught by its own test

### Decision
Built `LossyTransport` (`include/lossy_transport.h`) -- a test double that drops packets at a configurable probability (fixed-seed RNG for reproducible tests). `send()` always returns `true`: a real sender has no synchronous way to learn a transmission was lost, so this doesn't fake one either. Built batched per-message ARQ (`include/arq.h`): `sendWithRetry()` fragments and sends a stream once, drains whatever arrived into the `PacketReassembler`, then repeats "resend exactly what `missingFragments()` still reports" for up to `maxRetries` additional rounds before giving up.

### Bug found and fixed
The first implementation broke under 100% packet loss: `sendWithRetry` would call `missingFragments()`, see an empty list, and `break` out of the retry loop after just 1 round instead of exhausting all `maxRetries` -- looking like a graceful give-up, but for the wrong reason. Root cause: `PacketReassembler::missingFragments()` can only report indices for a `(messageId, streamId)` it has learned `totalFragments` for, which only happens once at least one fragment has actually arrived. Under total loss, the reassembler never learns anything about the stream at all, so "missing" was empty not because nothing was missing, but because the reassembler didn't know the stream existed yet -- and the code mistook the former for the latter.

Fix: when `missingFragments()` comes back empty but the stream also isn't complete, `sendWithRetry` now falls back to its own authoritative fragment list (`allPackets`, built locally from the original `packetizer::fragment()` call) and resends everything, rather than trusting the reassembler's view when that view is empty by omission rather than by completion. Caught by `test_arq_sendWithRetry_givesUpAfterMaxRetries` (a `LossyTransport(1.0)` test) expecting `roundsUsed == 4` (1 initial send + 3 retries) and getting `1` instead -- exactly the kind of one-shot-not-N-round give-up this fix corrects.

### Status
Accepted, implemented, and verified — full test suite passes, including the bug-catching test above, a partial-loss recovery test (`LossyTransport(0.3)`, confirms eventual success and byte-exact reassembled data), and a `main.cpp` demo block showing the same grid pipeline recovering over a 30%-drop-rate transport.

## 07/27/2026 — Firmware: ARQ over the real rover<->base LoRa link

### Decision
`firmware/src/main.cpp`'s `arq::sendWithRetry` (host-side, single-process) doesn't translate directly to two physically separate radios -- "tell the sender what's missing" has to be an actual message sent back over the air, not something introspected locally. Added a third LoRa message type, `FRAGMENT_STATUS_MESSAGE_ID` (base -> rover), and a real two-way protocol:

- Base tracks a quiet period (`QUIET_PERIOD_MS` = 500ms of no new grid fragments arriving) and, once elapsed with the grid still incomplete, sends a status report naming which fragment indices it's still missing, per stream.
- Each stream's status is one of three states, not a plain missing-count: `STATUS_COMPLETE`, `STATUS_PARTIAL` (with indices), `STATUS_NOTHING_RECEIVED`. Collapsing the last two into "0 missing" would be ambiguous -- `PacketReassembler::missingFragments()` can't list indices for a stream it never learned `totalFragments` for (zero fragments arrived), so the rover would misread that as "confirmed complete."
- Rover retains its fragmented packet lists after the initial send (previously local variables, discarded once `loop()` moved on) so it can resend specific indices later, up to `MAX_RETRY_ROUNDS` = 5 rounds per stream. If no status report arrives at all within `REPORT_TIMEOUT_MS` = 2000ms, the rover assumes the report itself was lost and falls back to resending everything for that stream.
- Base sends one final `STATUS_COMPLETE`/`STATUS_COMPLETE` report the moment both streams finish, so the rover gets an unambiguous "done" signal instead of just eventually stopping retries.

### Why
Timeout/retry defaults (500ms quiet period, 5 retries, 2s report timeout) were picked as a reasonable starting point pending real airtime/loss measurements on the actual radios, not derived from hardware data -- expect to retune once real hardware testing happens.

### Verification
Firmware compiles clean for both `rover` and `base` PlatformIO environments. The protocol *logic* (not just that it compiles) was verified separately: ported the same algorithm into a host-side simulation under a transport that drops packets at a configurable rate, run across seeds at 0/20/40/60/80% loss. Realistic loss rates (0-40%) succeeded 100% of the time (60/60 runs, byte-exact decoded data); at unrealistically severe loss (60-80%) some runs failed, but every failure was a clean give-up after exhausting retries -- never a hang, never a false-positive success with corrupted data. Not yet verified against real hardware.

### Status
Accepted and implemented. Real-hardware verification (actual airtime, actual loss patterns, whether the timeout defaults hold up) is still outstanding.
