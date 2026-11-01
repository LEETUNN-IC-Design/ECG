"""Tests for demo/ecg_convert.py::read_channel — WFDB 212 (signed 12-bit,
2 samples / 3 bytes) channel separation, order, sign extension, edges, and
the explicit data-loss policy (no silent truncation).

Fixtures are INDEPENDENT of the function under test: the byte literals below
were hand-derived from the 212 packing definition (ch0 = every 3rd-group
sample 0, ch1 = sample 1), NOT by running read_channel. Negative and edge
values (0, 2047, -1, -2048) are covered.

Run:  python -I -m pytest demo/test_ecg_convert.py   (or plain main below)
"""
import importlib.util
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("ecg_convert", HERE / "ecg_convert.py")
cv = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cv)
read_channel = cv.read_channel


# 6 sample-pairs (3-byte groups). ch0 = group sample 0, ch1 = group sample 1.
# Groups: a=+0/+255, +100/+1, +2047/+1, -1/-1, -50/0, -2048/-2048
FIX212 = bytes.fromhex("00F00F 641000 FF1700 FFFFFF CE0F00 000880".replace(" ", ""))
CH0_EXPECT = [0, 100, 2047, -1, -50, -2048]
CH1_EXPECT = [255, 1, 1, -1, 0, -2048]


def _write(tmp: Path, blob: bytes, name="t.dat") -> Path:
    p = tmp / name
    p.write_bytes(blob)
    return p


def test_hand_computed_212(tmp):
    p = _write(tmp, FIX212)
    assert read_channel(p, 0) == CH0_EXPECT, read_channel(p, 0)
    assert read_channel(p, 1) == CH1_EXPECT, read_channel(p, 1)
    print("PASS hand_computed_212 (neg + edges 0/2047/-1/-2048, both channels)")


def test_incomplete_group_rejected(tmp):
    # 3-byte group with one trailing byte: 3*k+1 -> data loss, must raise,
    # never silently drop the partial sample.
    p = _write(tmp, FIX212[:7], "short.dat")   # 7 bytes = 2 groups + 1 stray
    try:
        read_channel(p, 0)
    except ValueError:
        print("PASS incomplete_group_rejected (ValueError, no silent truncate)")
    else:
        raise AssertionError("incomplete 212 group must raise, not truncate")


def test_invalid_channel_raises(tmp):
    p = _write(tmp, FIX212)
    try:
        read_channel(p, 5)
    except IndexError:
        print("PASS invalid_channel_raises")
    else:
        raise AssertionError("channel >= n_channels must raise IndexError")


def test_single_channel(tmp):
    # One channel only: n_channels=1 -> every 3-byte group yields sample 0.
    p = _write(tmp, FIX212, "one.dat")
    assert read_channel(p, 0, n_channels=1) == CH0_EXPECT
    print("PASS single_channel (n_channels=1)")


# ---------------------------------------------------------------------------
# Real-data, decoder-INDEPENDENT integrity check (task item 2).
#
# This does NOT use read_channel's output as the expected value. It unpacks
# the WFDB 212 bit layout by hand (a second, self-contained implementation)
# and checks ONLY against external ground truth:
#   * file size == 650000 samples/ch x 2 ch x 12 bit / 8  (whole-record intgty)
#   * per-channel sample count == .hea nsamp (650000)
#   * ch0[0] == .hea MLII initial value (995)
# It also prints a full-channel checksum fingerprint for BOTH channels so a
# later official-WFDB diff can confirm byte-exactness (not asserted here,
# because no WFDB reference is installed in this environment).
# ---------------------------------------------------------------------------
REAL_DAT = HERE / "data" / "mitbih_100.dat"
REAL_HEA = HERE / "data" / "mitbih_100.hea"


def _unpack212_independent(raw: bytes):
    """Self-contained 212 unpack (independent of read_channel). ch0=group samp0,
    ch1=group samp1. Returns (ch0, ch1) as signed 12-bit lists."""
    c0, c1 = [], []
    append0, append1 = c0.append, c1.append
    for i in range(len(raw) // 3):
        b0 = raw[3 * i]
        b1 = raw[3 * i + 1]
        b2 = raw[3 * i + 2]
        s0 = b0 | ((b1 & 0x0F) << 8)
        s1 = ((b1 & 0xF0) >> 4) | (b2 << 4)
        append0(s0 - 4096 if s0 >= 2048 else s0)
        append1(s1 - 4096 if s1 >= 2048 else s1)
    return c0, c1


def test_real_data_independent_212(tmp):
    assert REAL_DAT.exists() and REAL_HEA.exists(), "real data/.hea not present"
    raw = REAL_DAT.open("rb").read()

    # .hea ground truth (external):  "100 2 360 650000" + per-channel lines.
    hea_lines = REAL_HEA.read_text().splitlines()
    hdr = hea_lines[0].split()
    nch = int(hdr[1])
    nsamp = int(hdr[3])
    ch_line = next(l for l in hea_lines if l.startswith(hdr[0] + ".dat"))
    f = ch_line.split()
    hea_gain_mlII = int(f[2])       # 200 units/mV
    hea_zero_mlII = int(f[4])       # 1024 = ADC zero (11 data bits)
    hea_initial_mlII = int(f[5])    # 995 = initial value

    # 1) whole-record integrity: file size == packed bytes for nsamp/ch.
    expect_bytes = nch * nsamp * 12 // 8
    assert len(raw) == expect_bytes, (len(raw), expect_bytes)

    # 2) sample count per channel == .hea nsamp.
    c0, c1 = _unpack212_independent(raw)
    assert len(c0) == nsamp and len(c1) == nsamp, (len(c0), nsamp)

    # 3) ch0[0] == .hea initial value (external, not decoder-produced).
    assert c0[0] == hea_initial_mlII, (c0[0], hea_initial_mlII)

    # 4) full-channel checksum fingerprints (record, not assert) for a later
    #    byte-exact WFDB cross-check. 64-bit Fletcher-style running sum.
    def checksum(ch):
        s = 0
        for x in ch:
            s = (s + x) & 0xFFFFFFFFFFFFFFFF
        return f"{s:016X}"
    print(f"PASS real_data_independent_212 "
          f"(size={len(raw)}={nch}x{nsamp}x12/8, ch0[0]={c0[0]}==init "
          f"{hea_initial_mlII}, ch1[0]={c1[0]})")
    print(f"      ch0 checksum(fingerprint)={checksum(c0)}  "
          f"ch1 checksum(fingerprint)={checksum(c1)}  "
          f"[for later WFDB byte-exact diff]")
    # NOTE (limit): ch1[0] ({c1[0]}) is NOT asserted against .hea V5 baseval
    # because no official WFDB reference is available to confirm it; ch0 (MLII)
    # is the pipeline's input, and it matches. Recorded as an open limit.


def main(tmpdir):
    t = Path(tmpdir)
    t.mkdir(parents=True, exist_ok=True)
    test_hand_computed_212(t)
    test_incomplete_group_rejected(t)
    test_invalid_channel_raises(t)
    test_single_channel(t)
    test_real_data_independent_212(t)
    print("ALL 212 converter tests PASS")


if __name__ == "__main__":
    import tempfile
    main(sys.argv[1] if len(sys.argv) > 1 else tempfile.mkdtemp())
