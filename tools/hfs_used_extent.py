#!/usr/bin/env python3
"""Report the USED extent of a Mac HFS disk image, so an OS swap writes ~6 MB
instead of 500 MB.

WHY THIS EXISTS
    A 500 MB OpenRetroSCSI image is almost entirely free space. The HFS
    allocation bitmap says which allocation blocks are actually in use; write
    only up to the highest set bit and the volume is fully consistent
    afterwards -- validated end-to-end 2026-08-03 by round-tripping
    7.5.3 -> 7.0.1 -> 7.5.3 -> 7.0.1 with no corruption, the restored 7.0.1
    booting to the Finder and reporting correct free space.

    At the measured ~264-301 KiB/s of `sd-write-fast`, that is the difference
    between ~23 s (7.0.1, 6.6 MB) and ~33 min (a full 500 MB image).

THE TRAP THAT COST A RESTORE
    You must ALSO write the ALTERNATE MDB. It lives at the second-to-last
    block of the VOLUME, which is NOT the end of the file, and a
    partition-arithmetic guess came out two sectors wrong. So this scans
    backwards for the `BD` signature on 512-byte boundaries instead of
    computing it. Measured on the 7.0.1 image: file offset 0x1F3FBC00,
    sector 1,023,966.

Emits shell-eval-able KEY=VALUE lines so callers do not re-parse prose.
"""
import struct
import sys

SECTOR = 512


def _be16(b, o):
    return struct.unpack_from(">H", b, o)[0]


def _be32(b, o):
    return struct.unpack_from(">I", b, o)[0]


def find_hfs_partition(fh):
    """Walk the Apple Partition Map for the Apple_HFS entry.

    Returns (start_sector, sector_count). Falls back to (0, filesize) for a
    bare HFS volume with no partition map, which is a legal layout and not an
    error -- silently assuming a partition map would mislocate everything.
    """
    fh.seek(0)
    blk0 = fh.read(SECTOR)
    if blk0[:2] != b"ER":
        return None                       # no driver descriptor: not an APM disk
    for i in range(1, 64):
        fh.seek(i * SECTOR)
        e = fh.read(SECTOR)
        if e[:2] != b"PM":
            break
        ptype = e[48:48 + 32].split(b"\0")[0].decode("ascii", "replace")
        if ptype == "Apple_HFS":
            return _be32(e, 8), _be32(e, 12)   # pmPyPartStart, pmPartBlkCnt
    return None


def read_mdb(fh, part_start):
    """MDB sits 1024 bytes into the partition, signature 'BD', big-endian."""
    fh.seek(part_start * SECTOR + 1024)
    mdb = fh.read(SECTOR)
    if mdb[:2] != b"BD":
        raise SystemExit(f"no HFS MDB at partition+1024 (got {mdb[:2]!r})")
    return {
        "drVBMSt":    _be16(mdb, 0x0E),   # bitmap start, in 512-blocks from vol start
        "drNmAlBlks": _be16(mdb, 0x12),   # number of allocation blocks
        "drAlBlkSiz": _be32(mdb, 0x14),   # bytes per allocation block
        "drAlBlSt":   _be16(mdb, 0x1C),   # first alloc block, in 512-blocks
        "drFreeBks":  _be16(mdb, 0x22),
    }


def highest_used_block(fh, part_start, mdb):
    """Highest SET bit in the allocation bitmap, or -1 if the volume is empty."""
    nbits = mdb["drNmAlBlks"]
    nbytes = (nbits + 7) // 8
    fh.seek(part_start * SECTOR + mdb["drVBMSt"] * SECTOR)
    bm = fh.read(nbytes)
    for i in range(nbytes - 1, -1, -1):
        if bm[i]:
            for bit in range(7, -1, -1):
                if bm[i] & (1 << bit):
                    idx = i * 8 + (7 - bit)
                    return idx if idx < nbits else nbits - 1
    return -1


def find_alt_mdb(fh, size):
    """Scan BACKWARDS for the alternate MDB's 'BD' signature.

    Deliberately a scan, not arithmetic: the alternate MDB is at the
    second-to-last block of the VOLUME (not the file), and computing it from
    the partition came out two sectors wrong in practice. Bounded to the last
    4 MiB so a corrupt image cannot turn this into a full-file scan.
    """
    last = size // SECTOR - 1
    floor = max(0, last - (4 * 1024 * 1024) // SECTOR)
    for sec in range(last, floor, -1):
        fh.seek(sec * SECTOR)
        if fh.read(2) == b"BD":
            return sec
    return None


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: hfs_used_extent.py <disk.img>")
    path = sys.argv[1]
    with open(path, "rb") as fh:
        fh.seek(0, 2)
        size = fh.tell()
        part = find_hfs_partition(fh)
        part_start = part[0] if part else 0
        mdb = read_mdb(fh, part_start)
        hi = highest_used_block(fh, part_start, mdb)
        alt = find_alt_mdb(fh, size)

    # bytes from volume start through the end of the highest used alloc block
    used_bytes = (mdb["drAlBlSt"] * SECTOR
                  + (hi + 1) * mdb["drAlBlkSiz"]) if hi >= 0 else 0
    used_sectors = part_start + (used_bytes + SECTOR - 1) // SECTOR

    # SAFETY MARGIN. An earlier hand-rolled parser reported slightly LARGER
    # extents for the same two images (12947 vs 12931, 95411 vs 95363) and its
    # numbers are the ones validated end-to-end on hardware. I have not
    # reconciled the difference, so rather than assert mine is the correct one,
    # pad past both. Under-writing truncates a volume; over-writing costs
    # 64 sectors = 32 KiB ~= 0.1 s at the measured 264-301 KiB/s. The asymmetry
    # is the whole argument.
    MARGIN_SECTORS = 64
    used_sectors = min(used_sectors + MARGIN_SECTORS, size // SECTOR)

    print(f"IMG={path}")
    print(f"IMG_SECTORS={size // SECTOR}")
    print(f"PART_START={part_start}")
    print(f"ALLOC_BLKS={mdb['drNmAlBlks']}")
    print(f"ALLOC_SIZE={mdb['drAlBlkSiz']}")
    print(f"HIGHEST_USED={hi}")
    print(f"USED_SECTORS={used_sectors}")
    print(f"USED_MB={used_bytes / 1048576:.1f}")
    print(f"ALT_MDB_SECTOR={alt if alt is not None else -1}")
    if alt is None:
        print("WARN=alternate MDB not found in the last 4 MiB;"
              " write the whole image rather than guessing", file=sys.stderr)


if __name__ == "__main__":
    main()
