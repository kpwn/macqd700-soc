import io
import unittest

from tools.hfs_used_extent import highest_used_block


class HighestUsedBlockTest(unittest.TestCase):
    def test_last_set_bit_within_final_byte(self):
        bitmap = b"\xff\xf8"  # allocation blocks 0..12 used
        image = io.BytesIO(b"\0" * 512 + bitmap)
        mdb = {"drNmAlBlks": 13, "drVBMSt": 1}
        self.assertEqual(highest_used_block(image, 0, mdb), 12)

    def test_unused_padding_bits_do_not_extend_volume(self):
        bitmap = b"\x80\x07"  # block 0 and out-of-range padding bits
        image = io.BytesIO(b"\0" * 512 + bitmap)
        mdb = {"drNmAlBlks": 13, "drVBMSt": 1}
        self.assertEqual(highest_used_block(image, 0, mdb), 0)


if __name__ == "__main__":
    unittest.main()
