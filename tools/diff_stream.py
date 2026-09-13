#!/usr/bin/env python3
"""diff_stream.py — classify how a received byte stream differs from the golden.

A "first difference at byte N" report cannot tell apart the three things that
actually happen in a packing pipeline, and they have completely different
causes:

  dropped byte     a handshake was missed; everything after shifts by 8 bits
  duplicated byte  a beat was presented twice
  lost/extra bits  the bit-level accumulator mis-tracked its fill

This compares at bit level, finds where the streams diverge, and then searches
for the smallest edit that realigns them.

Usage: diff_stream.py <expected.txt|.bin> <got.txt|.bin>
Text files are one decimal byte per line.
"""

import sys


def load(path):
    raw = open(path, "rb").read()
    # decimal-per-line if it looks like text
    if all(c in b"0123456789\r\n \t" for c in raw[:200]):
        return bytes(int(t) for t in raw.split())
    return raw


def bits(b):
    return "".join(format(x, "08b") for x in b)


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 1
    exp, got = load(sys.argv[1]), load(sys.argv[2])
    print("expected %d bytes, got %d bytes (%+d)" % (len(exp), len(got), len(got) - len(exp)))

    n = min(len(exp), len(got))
    i = 0
    while i < n and exp[i] == got[i]:
        i += 1
    if i == n and len(exp) == len(got):
        print("identical")
        return 0
    print("first differing byte: %d (0x%X)" % (i, i))
    print("  expected %s" % " ".join("%02X" % x for x in exp[i:i + 12]))
    print("  got      %s" % " ".join("%02X" % x for x in got[i:i + 12]))

    eb, gb = bits(exp), bits(got)
    j = 0
    while j < min(len(eb), len(gb)) and eb[j] == gb[j]:
        j += 1
    print("first differing bit: %d (byte %d, bit %d within it)" % (j, j // 8, j % 8))

    # Try to realign: delete k bits from expected, or insert k, at the
    # divergence point, and see how far the streams then agree.
    best = None
    for k in range(1, 65):
        for label, a, b in (("expected has %d MORE bits" % k, eb[:j] + eb[j + k:], gb),
                            ("expected has %d FEWER bits" % k, eb, gb[:j] + gb[j + k:])):
            m = min(len(a), len(b))
            run = 0
            while j + run < m and a[j + run] == b[j + run]:
                run += 1
            if best is None or run > best[0]:
                best = (run, label, k)
    run, label, k = best
    print()
    if run > 512:
        print("REALIGNS: %s at the divergence point" % label)
        print("  after that edit the streams agree for %d more bits (%d bytes)"
              % (run, run // 8))
        if k % 8 == 0:
            print("  %d bits is exactly %d whole byte(s): a lost or extra BEAT"
                  % (k, k // 8))
        else:
            print("  %d bits is not a whole byte: a bit-level accumulator fault" % k)
    else:
        print("no single edit of 1..64 bits realigns the streams")
        print("  best was %s, agreeing for only %d more bits" % (label, run))
        print("  that points at corrupted content rather than a shift")
    return 0


if __name__ == "__main__":
    sys.exit(main())
