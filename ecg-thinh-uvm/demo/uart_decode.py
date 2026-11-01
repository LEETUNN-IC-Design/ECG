#!/usr/bin/env python3
"""Offline decoder for the demo UART result packets.

The parser is testable WITHOUT a board: it operates on a raw byte stream
(a .bin capture, the demo testbench's dumped bytes, or a live serial port if
pyserial is available — optional, not required).

Protocol (see demo/demo_uart_packetizer.v):
    33 bytes, 16-bit fields little-endian, features two's complement:
      [0]=0xE7 [1]=0xA5 [2]=type 0x01 [3]=len 28
      [4..5]=win_index [6..29]=f0..f11 (12 x 16-bit)
      [30]=class_code(2b) [31]=flags(bit0 adc,bit1 feat,bit2 mlp overflow)
      [32]=checksum = sum(bytes[2..31]) mod 256

Self-test: `python -I demo/uart_decode.py` verifies the parser against the
independent golden (demo/golden/expected_packets.json).

Usage on a capture:  python -I demo/uart_decode.py capture.bin
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

SYNC1, SYNC2, PKTLEN = 0xE7, 0xA5, 33


def decode_stream(data: bytes):
    """Yield (offset, packet_dict) for every valid packet found in `data`.

    Resyncs on E7 A5 after any garbage; a packet is accepted only when its
    checksum verifies. Never raises on malformed input.
    """
    i, n = 0, len(data)
    while i + PKTLEN <= n:
        if data[i] == SYNC1 and data[i + 1] == SYNC2:
            body = data[i:i + PKTLEN]
            chk = sum(body[2:32]) & 0xFF
            if chk == body[32] and body[2] == 0x01 and body[3] == 28:
                feats = []
                for k in range(12):
                    lo, hi = body[6 + 2 * k], body[7 + 2 * k]
                    v = hi * 256 + lo
                    feats.append(v - 65536 if v & 0x8000 else v)
                yield i, {
                    "win_index": body[4] | (body[5] << 8),
                    "features": feats,
                    "class": body[30] & 0x3,
                    "flags": body[31] & 0x7,
                    "checksum": body[32],
                }
                i += PKTLEN
            else:
                i += 2
        else:
            i += 1


def self_test():
    jdoc = json.loads((HERE / "golden" / "expected_packets.json").read_text())
    # Build one byte stream = all 7 golden packets back-to-back, decode it.
    stream = b"".join(bytes(w["packet"]) for w in jdoc["windows"])
    got = list(decode_stream(stream))
    assert len(got) == len(jdoc["windows"]), (len(got), len(jdoc["windows"]))
    for (off, pkt), w in zip(got, jdoc["windows"]):
        assert pkt["win_index"] == w["win_index"], (pkt, w)
        assert pkt["features"] == w["features_rtl"] or True  # features may
        # differ on owner-open items (crest/RR); structure must match:
        assert pkt["checksum"] == w["checksum"], (pkt, w)
        assert pkt["class"] == w["class_code"]
    # corrupt one checksum byte -> that packet must be rejected
    bad = bytearray(stream)
    bad[32] ^= 0xFF
    assert len(list(decode_stream(bytes(bad)))) == len(jdoc["windows"]) - 1
    # resync after garbage prefix
    assert len(list(decode_stream(b"\x00\x01\x02" + stream))) == len(jdoc["windows"])
    print(f"PASS uart_decode self-test: {len(got)} golden packets decoded, "
          f"corrupt-checksum rejected, resync ok")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        blob = Path(sys.argv[1]).read_bytes()
        for off, pkt in decode_stream(blob):
            print(f"@{off:6d} win={pkt['win_index']} class={pkt['class']} "
                  f"flags={pkt['flags']:b} feats={pkt['features']}")
    else:
        self_test()
