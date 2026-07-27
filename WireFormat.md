# Wire Format Specification

Implementation-independent description of every byte layout this project produces, as of Milestone 10. Anything not described here (a field, a length, an encoding) should be treated as unspecified/subject to change — this document is the source of truth for "what's actually on the wire," not the narrative design history in `Discussion.md`.

All multi-byte integer fields in this document are **little-endian** unless stated otherwise. All lengths are in bytes unless stated otherwise.

## 1. Pipeline overview

```
Grid (occupancy cells, 0/1/2)
  -> SymbolStream            (generic byte carrier + StreamFormat tag)
  -> RLERuns                 (structural run-length transform: values[], counts[])
  -> splitcodec               (two independently-encoded byte streams)
       -> valuesBytes         (FixedWidthCoder-packed values, no header)
       -> countsBytes         (self-describing: CoderType + N + encoded counts)
  -> packetizer::fragment      (each stream fragmented independently into Packets)
  -> ITransport                (LoopbackTransport today; LoRaTransport later)
  -> PacketReassembler          (reassembles each stream by fragmentIndex)
  -> splitcodec::decode         (recovers RLERuns)
  -> RLE rebuild -> SymbolStream -> Grid
```

`rlecodec` (below) is a second, simpler wire shape for callers that want one self-contained blob instead of two independently-fragmentable streams — it's what Milestone 4/5's tests and demo block use directly, without going through the packetizer.

## 2. SymbolStream

Not itself a wire format — an in-memory struct (`format: StreamFormat`, `symbols: byte sequence`). `StreamFormat` values: `Raw = 0`, `RLE = 1`, `Unknown = 2` (sentinel for malformed/unrecognized streams). Only `Raw` is produced by the current Grid <-> SymbolStream conversion; `RLE` is reserved for when a SymbolStream carrying already-RLE'd bytes needs to say so.

## 3. RLERuns (structural, not a byte format)

RLE's output is **Structure-of-Arrays**, not interleaved `(value, count)` pairs:

- `values`: one entry per run, each in `[0, 255]` (in practice occupancy's domain: 0/1/2, with `3` i.e. binary `11` reserved for future values/error detection at 2-bit width).
- `counts`: one entry per run, each in `[1, 65535]` (a run is never empty by construction; `0` in a decoded stream is malformed).

`values.length == counts.length == N` always.

## 4. Coders

### 4.1 FixedWidthCoder (values)

Packs each value into `bitWidth` bits, MSB-first within each byte, values packed back-to-back across byte boundaries, with zero-padding in the low bits of the final byte if `N * bitWidth` isn't a multiple of 8. Encoded length is always `ceil(N * bitWidth / 8)` bytes — deterministic from `N` and `bitWidth` alone, which is what lets `splitcodec` and `rlecodec` avoid a length prefix for this section. Occupancy's `bitWidth` is `2`.

### 4.2 VarintCoder (counts) — unsigned LEB128

Each integer is encoded as a sequence of bytes, 7 data bits each, low-order group first; the top bit of each byte is a continuation bit (`1` = more bytes follow, `0` = this is the last byte). A sequence of counts is just these back-to-back with no separators — the reader knows how many integers to pull out because `N` is known from context, not because the counts section is self-delimiting past that.

Malformed-input rules: a stream that ends before a continuation bit clears is truncated (reject). More than 5 bytes for one `uint32_t`-range value is reject (overflow). For a `uint16_t`-range value (RLE counts), a decoded value `> 0xFFFF` is reject.

### 4.3 Rice/Golomb coder (counts) — benchmark-validated alternative

Parameterized by `k` (0-255, in practice small, e.g. 0-12 in the current auto-search). Each value `v` splits into quotient `q = v >> k` and remainder `r = v & ((1<<k)-1)`. Encoded as `q` one-bits, then a terminating zero-bit, then `r` as exactly `k` bits. No byte alignment between values — this is a genuinely bit-packed format, decoded with a bit cursor, not a byte cursor.

Malformed-input rules: a unary prefix that never terminates before the stream runs out is reject; an unreasonably long unary prefix (over 100,000 in the current implementation) is treated as adversarial/corrupted and rejected rather than looped on.

### 4.4 CoderType

```
enum CoderType : uint8_t {
    FixedWidth = 0,
    Varint     = 1,
    Rice       = 2,
    Unknown    = 3,   // sentinel — any other byte value is also treated as Unknown/rejected
}
```

This is the actual dispatch tag a decoder switches on to know which coder produced a given byte sequence — not a passive label. Only `Varint` and `Rice` are valid values for the *counts* coder today (values always use `FixedWidth` — occupancy's alphabet is small and fixed, so there's nothing to gain from swapping that coder yet). A byte that doesn't match any defined `CoderType` is rejected by every consumer of this field (`rlecodec::decode`, `splitcodec::decode`), not treated as `Unknown` and passed through.

## 5. rlecodec — combined single-blob format

One self-contained byte sequence:

```
[CoderType countsCoderType : 1 byte]
[uint8_t   riceParam       : 1 byte]   (meaningful only if countsCoderType == Rice; present regardless)
[varint    N                        ]   (run count — see §4.2 for varint's own byte format)
[bytes     values                   ]   (FixedWidthCoder output, exactly ceil(N * valueBitWidth / 8) bytes)
[bytes     counts                   ]   (countsCoderType's encoding of N counts)
```

`valueBitWidth` is **not** part of this format — it's an out-of-band parameter both encoder and decoder must already agree on (occupancy's convention is `2`). Total length is not stored anywhere; it's implicitly `data.size()` when handed to `decode()`, and the counts section is assumed to run to the end of whatever buffer was provided.

## 6. splitcodec — two independent streams

Used when the two streams will be fragmented and transported separately (this is what the packetizer pipeline actually uses). `N` is carried only in `countsBytes` — `valuesBytes` has no header of its own.

**valuesBytes:**
```
[bytes values]   (FixedWidthCoder output, exactly ceil(N * valueBitWidth / 8) bytes)
```

**countsBytes:**
```
[CoderType countsCoderType : 1 byte]
[uint8_t   riceParam       : 1 byte]
[varint    N                        ]
[bytes     counts                   ]   (countsCoderType's encoding of N counts)
```

Decoding requires reading `countsBytes` first to learn `N`, which then gives the decoder `valuesBytes`' expected deterministic length (`ceil(N * valueBitWidth / 8)`). If `valuesBytes.size()` doesn't match that, the two streams disagree — rejected as corruption, not silently truncated or zero-padded.

## 7. Packet format (packetizer)

```
constexpr MAX_PACKET_SIZE  = 255;                                  // SX127x/LoRa-derived default, not universal
constexpr HEADER_SIZE      = 9;
constexpr CRC_SIZE         = 2;
constexpr MAX_PAYLOAD_SIZE = MAX_PACKET_SIZE - HEADER_SIZE - CRC_SIZE;  // 244
```

Wire layout of one serialized packet (total length = `HEADER_SIZE + payloadLength + CRC_SIZE`, **not** padded to `MAX_PACKET_SIZE`):

```
[uint8_t  version         : 1 byte]
[uint16_t messageId       : 2 bytes]
[uint8_t  streamId        : 1 byte]   (opaque — packetizer assigns no meaning to specific values)
[uint16_t fragmentIndex   : 2 bytes]  (0-based)
[uint16_t totalFragments  : 2 bytes]
[uint8_t  payloadLength   : 1 byte]   (<= MAX_PAYLOAD_SIZE; the last fragment of a stream may be shorter than others)
[bytes    payload         : payloadLength bytes]
[uint16_t crc16           : 2 bytes]  (CRC-16-CCITT, poly 0x1021, init 0xFFFF, computed over every byte above — header + payload)
```

An empty stream (0 bytes of data to fragment) still produces exactly **one** packet with `payloadLength = 0`, `totalFragments = 1` — this keeps "stream exists but is empty" distinguishable from "stream doesn't exist" for the reassembler.

Deliberately **not** in the header: the RLE run count `N` (it lives inside `countsBytes`/`rlecodec`'s payload, per §5-6) and any `CoderType`/compression-semantic field — the packetizer has zero awareness of what its payloads mean, only how to cut them into pieces and glue them back together by position.

`deserialize()` rejects (returns failure, never guesses): data shorter than `HEADER_SIZE + CRC_SIZE`; a CRC mismatch; or `payloadLength` disagreeing with the actual remaining data length.

## 8. Reassembly semantics (PacketReassembler)

Keyed by `(messageId, streamId)`. A stream is complete once fragments for every index in `[0, totalFragments)` have been received (order of arrival doesn't matter — reassembly concatenates strictly by `fragmentIndex`, never by arrival order). A fragment whose `totalFragments` disagrees with earlier fragments of the same key is rejected; a `fragmentIndex >= totalFragments` is rejected; a genuine duplicate (same index arriving twice) is accepted idempotently, not treated as an error.

`missingFragments(messageId, streamId)` returns the still-outstanding indices in ascending order — the input Milestone 11's ARQ round-trips against a sender to request retransmission of exactly those.

## 9. What's still open / not yet specified

- **ARQ (Milestone 11):** retry/timeout semantics, and how a "resend these fragment indices" request itself gets encoded and transported, are not yet designed.
- **Real transport limits (Milestone 12):** `MAX_PACKET_SIZE = 255` is an assumption inherited from typical LoRa/SX127x defaults, not measured against real hardware. Actual usable payload size, packet error rate, and airtime/duty-cycle constraints are unknown until real hardware is involved.
- **Multi-layer / non-occupancy streams (Milestone 14):** every format above assumes occupancy's specific alphabet (`bitWidth = 2`) is agreed out-of-band. A future layer with a different alphabet size would need that width to travel somewhere (today it doesn't travel at all — it's a hardcoded convention on both ends).
