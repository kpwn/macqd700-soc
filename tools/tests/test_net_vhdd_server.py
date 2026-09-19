#!/usr/bin/env python3
"""Unit tests for tools/net_vhdd_server.py.

No sockets and no FPGA: the request handler is driven directly with byte
strings.  Every test vector is built here from the wire-format spec in
rtl/board/net_block_framer.sv -- literal magic bytes, explicit big-endian
int.to_bytes() -- and never by calling the daemon's own encoder.  A bug in the
daemon's packing therefore cannot pass its own test.
"""

from __future__ import annotations

import os
import struct
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import net_vhdd_server as server  # noqa: E402


BLOCK = 512

# Spelled out from the spec, deliberately not imported from the daemon.
MAGIC_BYTES = b"\x4e\x42\x48\x44"       # 0x4e424844, "NBHD"
OP_READ = 0x00
OP_WRITE = 0x01
REQUEST_HEADER_BYTES = 13
REPLY_HEADER_BYTES = 7
IPV4_MTU = 1500


def make_request(op: int, lba: int, count: int, tag: int,
                 payload: bytes = b"", magic: bytes = MAGIC_BYTES) -> bytes:
    """Build a request UDP payload byte-for-byte from the format spec."""
    assert len(magic) == 4
    datagram = (
        magic
        + bytes((op,))
        + lba.to_bytes(4, "big")
        + count.to_bytes(2, "big")
        + tag.to_bytes(2, "big")
        + payload
    )
    assert len(datagram) == REQUEST_HEADER_BYTES + len(payload)
    return datagram


def parse_reply(reply: bytes) -> tuple[bytes, int, int, bytes]:
    """Decode a reply UDP payload independently of the daemon."""
    magic, tag, status = struct.unpack("!4sHB", reply[:REPLY_HEADER_BYTES])
    return magic, tag, status, reply[REPLY_HEADER_BYTES:]


class NetVhddServerTests(unittest.TestCase):
    IMAGE_BLOCKS = 8

    def setUp(self) -> None:
        self.contents = bytes(
            (index * 7 + 13) % 256 for index in range(self.IMAGE_BLOCKS * BLOCK)
        )
        self.image = tempfile.TemporaryFile(mode="w+b")
        self.image.write(self.contents)
        self.image.flush()
        self.image.seek(0)
        self.addCleanup(self.image.close)

    def handle(self, datagram: bytes, *, read_only: bool = False) -> bytes:
        return server.handle_request(datagram, self.image, read_only=read_only)

    def image_bytes(self) -> bytes:
        self.image.seek(0)
        return self.image.read()

    # ── happy paths ────────────────────────────────────────────────────────
    def test_read_returns_requested_blocks_and_echoes_tag(self) -> None:
        reply = self.handle(make_request(OP_READ, 3, 2, 0xA17E))

        magic, tag, status, payload = parse_reply(reply)
        self.assertEqual(magic, MAGIC_BYTES)
        self.assertEqual(tag, 0xA17E)
        self.assertEqual(status, 0x00)
        self.assertEqual(payload, self.contents[3 * BLOCK:5 * BLOCK])

    def test_single_block_read_at_the_last_block(self) -> None:
        last = self.IMAGE_BLOCKS - 1
        reply = self.handle(make_request(OP_READ, last, 1, 0x0001))

        _magic, tag, status, payload = parse_reply(reply)
        self.assertEqual((tag, status), (0x0001, 0x00))
        self.assertEqual(payload, self.contents[last * BLOCK:])

    def test_two_block_read_reply_fits_an_unfragmented_datagram(self) -> None:
        reply = self.handle(make_request(OP_READ, 0, 2, 0x0002))

        self.assertEqual(parse_reply(reply)[2], 0x00)
        # Ethernet+IPv4+UDP: the RTL rejects fragments, so the whole IPv4
        # datagram (20 + 8 + reply) must stay inside the 1500-byte MTU.
        self.assertLessEqual(20 + 8 + len(reply), IPV4_MTU)

    def test_write_lands_in_the_backing_file(self) -> None:
        payload = bytes((0xC0 ^ (i & 0xFF)) for i in range(2 * BLOCK))
        reply = self.handle(make_request(OP_WRITE, 2, 2, 0x1234, payload))

        self.assertEqual(parse_reply(reply), (MAGIC_BYTES, 0x1234, 0x00, b""))
        after = self.image_bytes()
        self.assertEqual(after[2 * BLOCK:4 * BLOCK], payload)
        # Nothing outside the written range moved.
        self.assertEqual(after[:2 * BLOCK], self.contents[:2 * BLOCK])
        self.assertEqual(after[4 * BLOCK:], self.contents[4 * BLOCK:])
        self.assertEqual(len(after), len(self.contents))

    def test_write_then_read_back_round_trips(self) -> None:
        payload = b"\xde\xad\xbe\xef" * (BLOCK // 4)
        self.handle(make_request(OP_WRITE, 5, 1, 0x0007, payload))
        reply = self.handle(make_request(OP_READ, 5, 1, 0x0008))

        _magic, tag, status, read_back = parse_reply(reply)
        self.assertEqual((tag, status), (0x0008, 0x00))
        self.assertEqual(read_back, payload)

    # ── refusals: every one replies, none is dropped ───────────────────────
    def test_bad_magic_is_rejected(self) -> None:
        reply = self.handle(
            make_request(OP_READ, 0, 1, 0x1001, magic=b"XXXX")
        )

        magic, tag, status, payload = parse_reply(reply)
        self.assertEqual(magic, MAGIC_BYTES)
        self.assertEqual(tag, 0x1001)
        self.assertEqual(status, server.STATUS_BAD_MAGIC)
        self.assertNotEqual(status, 0x00)
        self.assertEqual(payload, b"")

    def test_unknown_op_is_rejected(self) -> None:
        reply = self.handle(make_request(0x7F, 0, 1, 0x1002))

        self.assertEqual(
            parse_reply(reply),
            (MAGIC_BYTES, 0x1002, server.STATUS_UNKNOWN_OP, b""),
        )

    def test_out_of_range_read_lba_is_rejected(self) -> None:
        reply = self.handle(make_request(OP_READ, self.IMAGE_BLOCKS, 1, 0x1003))

        self.assertEqual(
            parse_reply(reply),
            (MAGIC_BYTES, 0x1003, server.STATUS_OUT_OF_RANGE, b""),
        )

    def test_read_straddling_the_end_of_the_image_is_rejected(self) -> None:
        reply = self.handle(
            make_request(OP_READ, self.IMAGE_BLOCKS - 1, 2, 0x1004)
        )

        self.assertEqual(parse_reply(reply)[2], server.STATUS_OUT_OF_RANGE)

    def test_out_of_range_write_does_not_extend_the_image(self) -> None:
        payload = b"Z" * BLOCK
        reply = self.handle(
            make_request(OP_WRITE, self.IMAGE_BLOCKS, 1, 0x1005, payload)
        )

        self.assertEqual(parse_reply(reply)[2], server.STATUS_OUT_OF_RANGE)
        self.assertEqual(self.image_bytes(), self.contents)

    def test_read_only_refuses_writes_with_a_distinct_status(self) -> None:
        payload = b"R" * BLOCK
        reply = self.handle(
            make_request(OP_WRITE, 0, 1, 0x1006, payload), read_only=True
        )

        magic, tag, status, body = parse_reply(reply)
        self.assertEqual((magic, tag, body), (MAGIC_BYTES, 0x1006, b""))
        self.assertEqual(status, server.STATUS_READ_ONLY)
        self.assertNotIn(status, {
            0x00,
            server.STATUS_BAD_MAGIC,
            server.STATUS_UNKNOWN_OP,
            server.STATUS_OUT_OF_RANGE,
            server.STATUS_BAD_LENGTH,
            server.STATUS_TOO_MANY_BLOCKS,
        })
        self.assertEqual(self.image_bytes(), self.contents)

    def test_read_only_still_serves_reads(self) -> None:
        reply = self.handle(make_request(OP_READ, 1, 1, 0x1007), read_only=True)

        _magic, tag, status, payload = parse_reply(reply)
        self.assertEqual((tag, status), (0x1007, 0x00))
        self.assertEqual(payload, self.contents[BLOCK:2 * BLOCK])

    def test_write_of_three_blocks_is_rejected(self) -> None:
        # Fully well-formed apart from its width: the payload really is three
        # blocks long, so this cannot be mistaken for a length mismatch.
        payload = b"\x5a" * (3 * BLOCK)
        reply = self.handle(make_request(OP_WRITE, 0, 3, 0x1008, payload))

        self.assertEqual(
            parse_reply(reply),
            (MAGIC_BYTES, 0x1008, server.STATUS_TOO_MANY_BLOCKS, b""),
        )
        self.assertEqual(self.image_bytes(), self.contents)

    def test_read_of_three_blocks_is_rejected_too(self) -> None:
        # A three-block reply would exceed the MTU and be fragmented, and the
        # RTL receive path drops fragments.
        reply = self.handle(make_request(OP_READ, 0, 3, 0x1009))

        self.assertEqual(parse_reply(reply)[2], server.STATUS_TOO_MANY_BLOCKS)

    def test_zero_block_count_is_rejected(self) -> None:
        reply = self.handle(make_request(OP_READ, 0, 0, 0x100A))

        self.assertNotEqual(parse_reply(reply)[2], 0x00)

    def test_write_payload_shorter_than_block_count_is_rejected(self) -> None:
        reply = self.handle(
            make_request(OP_WRITE, 0, 2, 0x100B, b"\x11" * (2 * BLOCK - 1))
        )

        self.assertEqual(
            parse_reply(reply),
            (MAGIC_BYTES, 0x100B, server.STATUS_BAD_LENGTH, b""),
        )
        self.assertEqual(self.image_bytes(), self.contents)

    def test_read_request_with_trailing_bytes_is_rejected(self) -> None:
        reply = self.handle(make_request(OP_READ, 0, 1, 0x100C, b"junk"))

        self.assertEqual(parse_reply(reply)[2], server.STATUS_BAD_LENGTH)

    def test_truncated_header_still_gets_a_reply(self) -> None:
        reply = self.handle(make_request(OP_READ, 0, 1, 0x100D)[:9])

        magic, _tag, status, payload = parse_reply(reply)
        self.assertEqual(magic, MAGIC_BYTES)
        self.assertEqual(status, server.STATUS_BAD_LENGTH)
        self.assertEqual(payload, b"")

    def test_empty_datagram_still_gets_a_reply(self) -> None:
        reply = self.handle(b"")

        self.assertEqual(len(reply), REPLY_HEADER_BYTES)
        self.assertNotEqual(parse_reply(reply)[2], 0x00)

    def test_every_refusal_status_is_distinct(self) -> None:
        codes = [
            server.STATUS_SUCCESS,
            server.STATUS_BAD_MAGIC,
            server.STATUS_UNKNOWN_OP,
            server.STATUS_OUT_OF_RANGE,
            server.STATUS_READ_ONLY,
            server.STATUS_BAD_LENGTH,
            server.STATUS_TOO_MANY_BLOCKS,
            server.STATUS_IO_ERROR,
        ]
        self.assertEqual(len(codes), len(set(codes)))
        self.assertEqual(server.STATUS_SUCCESS, 0x00)
        self.assertTrue(all(0 < code <= 0xFF for code in codes[1:]))


class ConstantsMatchTheRtlTests(unittest.TestCase):
    """Guard the constants the daemon shares with net_block_framer.sv."""

    def test_max_blocks_per_datagram_is_two(self) -> None:
        self.assertEqual(server.MAX_BLOCKS_PER_DATAGRAM, 2)

    def test_header_sizes_match_the_spec(self) -> None:
        self.assertEqual(server.REQUEST_HEADER_BYTES, REQUEST_HEADER_BYTES)
        self.assertEqual(server.REPLY_HEADER_BYTES, REPLY_HEADER_BYTES)
        self.assertEqual(server.BLOCK_BYTES, BLOCK)
        self.assertEqual(server.MAGIC, MAGIC_BYTES)

    def test_mtu_budget_matches_the_rtl_localparams(self) -> None:
        self.assertEqual(server.MAX_REPLY_PAYLOAD_BYTES,
                         IPV4_MTU - 20 - 8 - REPLY_HEADER_BYTES)
        self.assertEqual(server.MAX_WRITE_PAYLOAD_BYTES,
                         IPV4_MTU - 20 - 8 - REQUEST_HEADER_BYTES)


class ArpDocumentationTests(unittest.TestCase):
    """The static-neighbour warning must survive refactors of the help text."""

    NEIGH = "ip neigh replace"

    def test_module_docstring_documents_the_missing_arp(self) -> None:
        doc = server.__doc__ or ""
        self.assertIn(self.NEIGH, doc)
        self.assertIn("ARP", doc)

    def test_help_output_documents_the_missing_arp(self) -> None:
        help_text = server.build_argument_parser().format_help()
        self.assertIn(self.NEIGH, help_text)
        self.assertIn("nud permanent", help_text)
        self.assertIn("ARP", help_text)


class CliTests(unittest.TestCase):
    def test_read_only_flag_parses(self) -> None:
        args = server.build_argument_parser().parse_args(
            ["disk.img", "--read-only", "--port", "1234"]
        )
        self.assertTrue(args.read_only)
        self.assertEqual(args.port, 1234)
        self.assertEqual(args.image, "disk.img")

    def test_defaults_are_sensible(self) -> None:
        args = server.build_argument_parser().parse_args(["disk.img"])
        self.assertFalse(args.read_only)
        self.assertEqual(args.host, server.DEFAULT_HOST)
        self.assertEqual(args.port, server.DEFAULT_PORT)


class ImageSizeTests(unittest.TestCase):
    def test_size_does_not_disturb_the_file_position(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, "disk.img")
            with open(path, "wb") as handle:
                handle.write(b"\x00" * (4 * BLOCK))
            with open(path, "rb") as handle:
                handle.seek(123)
                self.assertEqual(server.image_size(handle), 4 * BLOCK)
                self.assertEqual(handle.tell(), 123)


if __name__ == "__main__":
    unittest.main()
