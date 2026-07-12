# CRC Tables — Notes

## How a CRC table works

CRC treats a message as one big binary number and divides it by a fixed
"polynomial" constant using XOR instead of normal subtraction (division over
GF(2)). The remainder of that division is the CRC. Flip even one bit in
transit and the remainder almost certainly comes out different — that's what
makes it good at catching corruption.

Naive version processes one *bit* at a time (8 shift/XOR ops per byte):

```c
for (int i = 0; i < 8; i++) {
    crc = (crc & 0x80) ? (crc << 1) ^ POLY : (crc << 1);
}
```

The table trick: a byte only has 256 possible values, so precompute the result
of that 8-bit inner loop for every possible byte value once, at startup. Then
per-byte processing collapses to one lookup:

```c
uint8_t crc_table[256];

void build_crc_table() {
    for (int byte = 0; byte < 256; byte++) {
        uint8_t crc = byte;
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (crc << 1) ^ POLY : (crc << 1);
        }
        crc_table[byte] = crc;
    }
}

uint8_t compute_crc(uint8_t *data, int len) {
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc = crc_table[crc ^ data[i]];
    }
    return crc;
}
```

8 bit-level ops per byte → 1 table lookup + 1 XOR per byte. Same result,
much faster — the table just caches "what does the bit-loop do to byte X"
for all 256 possible X.

## Where WALL-E actually needs this: Mega → Jetson odometry link

`src/wall_e_bringup/src/mega_node.cpp`, `read_loop()` / `parse_odom()`
(around lines 127-179): syncs on the `0xAA 0xBB` header, then `memcpy`s 5
floats (x, y, theta, left/right velocity) straight out of the buffer.

**No checksum is validated.** The README documents bytes 14-21 of the
22-byte odometry packet as "padding/checksum," but the receiving code never
checks them.

**Why this matters:** a single bit flip during serial transmission (plausible
near motor EMI — exactly the environment this link runs in) produces a
garbage float published straight to `/odom`, which feeds directly into TF
and Nav2's costmap. Not a crash — silently wrong localization, with nothing
catching it, since the safety watchdog only checks for *timeout*, not
*garbage-but-present* data.

**Why CRC over a plain sum-checksum:** a plain sum can miss two-bit errors
that cancel out, a common failure mode with EMI-induced bit flips. CRC-8
catches those. Cost is negligible at this packet size/rate — a 256-byte
table and ~22 lookups per packet, trivial even on an Arduino Mega.

**Status:** not implemented. Would need matching changes on both ends —
Arduino Mega firmware (compute CRC-8 over the payload, append it) and
`mega_node.cpp` (`parse_odom`/`read_loop`: compute the same CRC-8 over the
received payload, reject the packet if it doesn't match the trailing byte).
Arduino-side firmware source hasn't been located/checked yet to confirm
what's currently in those "padding/checksum" bytes.
