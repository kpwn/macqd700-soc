"""Restricted KU5P configuration-packet support for ADB release patch maps.

No third-party packages or FPGA tools. This is intentionally not a general
bitstream editor: compressed, encrypted and multi-FDRI streams are
rejected. A map is valid only for the SHA-256-bound blank image it accompanies.
"""
from array import array
import hashlib
import sys

SYNC = bytes.fromhex("aa995566")
FORMAT = "macqd700-adb-python-v1"
FRAME_WORDS = 93
FIRMWARE_BITS = 512 * 12


def _crc_table():
    table = []
    for value in range(256):
        for _ in range(8):
            value = (value >> 1) ^ (0x82F63B78 if value & 1 else 0)
        table.append(value)
    return table


CRC_TABLE = _crc_table()
# Five configuration-register address bits follow each 32-bit data word.
CRC_ADDRESS = []
for _value in range(32):
    for _ in range(5):
        _value = (_value >> 1) ^ (0x82F63B78 if _value & 1 else 0)
    CRC_ADDRESS.append(_value)


class ConfigImage:
    def __init__(self, data):
        try:
            start = data.index(SYNC) + 4
        except ValueError:
            raise ValueError("missing configuration sync word") from None
        if (len(data) - start) % 4:
            raise ValueError("truncated configuration word")
        self.header = data[:start]
        self.words = array("I")
        self.words.frombytes(data[start:])
        if sys.byteorder == "little":
            self.words.byteswap()
        self.packets = []
        self.crc_indices = []
        self.fdri = None
        index, previous = 0, None
        seen_id = False
        while index < len(self.words):
            header = self.words[index]
            kind, op = header >> 29, (header >> 27) & 3
            if header == 0xFFFFFFFF:
                index += 1
                previous = None
                continue
            if header == 0x20000000:
                index += 1
                previous = None
                continue
            if op != 2:
                raise ValueError("unsupported configuration packet operation")
            if kind == 1:
                if header & 0x07FC1800:
                    raise ValueError("unsupported configuration register/header")
                register, count = (header >> 13) & 31, header & 0x7FF
                previous = register if count == 0 else None
            elif kind == 2 and previous == 2:
                register, count = 2, header & 0x07FFFFFF
                previous = None
            else:
                raise ValueError("unexpected configuration packet type")
            begin, end = index + 1, index + 1 + count
            if end > len(self.words):
                raise ValueError("truncated configuration packet")
            if register in (10, 11):
                raise ValueError("compressed/encrypted images need a Python release bundle")
            if count and register != 2 and count != 1:
                raise ValueError("unsupported multiword register write")
            if register == 12 and count:
                if self.words[begin] != 0x04A62093:
                    raise ValueError("only the KU5P device is supported")
                seen_id = True
            if register == 5 and count and self.words[begin] & 0x40:
                raise ValueError("encrypted configuration is unsupported")
            if register == 24 and count and self.words[begin] & 0x1000:
                raise ValueError("compressed images need a Python release bundle")
            if register == 0 and count:
                self.crc_indices.append(begin)
            if register == 2 and count:
                if self.fdri is not None or count % FRAME_WORDS:
                    raise ValueError("expected one complete, uncompressed FDRI payload")
                self.fdri = (begin, end)
            self.packets.append((register, begin, end))
            index = end
        if not seen_id or self.fdri is None or not self.crc_indices:
            raise ValueError("incomplete KU5P configuration")

    def crc(self, repair=False):
        """Validate or regenerate packet CRCs; never disable CRC checking.

        CRC-32C, LSB first, no final inversion. Feed data then the five-bit
        register address. RCRC and CRC writes restart the accumulator.
        """
        crc, words, table, addresses = 0, self.words, CRC_TABLE, CRC_ADDRESS
        for register, begin, end in self.packets:
            for index in range(begin, end):
                value = words[index]
                if register == 0:
                    if repair:
                        words[index] = crc
                    elif value != crc:
                        raise ValueError(f"configuration CRC mismatch at word {index}")
                    crc = 0
                elif register == 4 and value == 7:  # CMD.RCRC
                    crc = 0
                else:
                    crc ^= value
                    crc = (crc >> 8) ^ table[crc & 255]
                    crc = (crc >> 8) ^ table[crc & 255]
                    crc = (crc >> 8) ^ table[crc & 255]
                    crc = (crc >> 8) ^ table[crc & 255]
                    crc = (crc >> 5) ^ addresses[(crc ^ register) & 31]

    def bytes(self):
        words = array("I", self.words)
        if sys.byteorder == "little":
            words.byteswap()
        return self.header + words.tobytes()


def validate_map(data, mapping):
    if not isinstance(mapping, dict) or mapping.get("format") != FORMAT:
        raise ValueError("unsupported Python patch map")
    if hashlib.sha256(data).hexdigest() != mapping.get("bit_sha256"):
        raise ValueError("bitstream hash mismatch; use the matching release bundle")
    image = ConfigImage(data)
    locations = mapping.get("bit_locations")
    if (not isinstance(locations, list) or len(locations) != FIRMWARE_BITS
            or any(type(bit) is not int for bit in locations)
            or len(set(locations)) != FIRMWARE_BITS):
        raise ValueError("patch map must contain 6144 unique integer bit locations")
    begin, end = image.fdri
    for location in locations:
        word, bit = divmod(location, 32)
        if not begin <= word < end:
            raise ValueError("patch location outside FDRI")
        if (word - begin) % FRAME_WORDS in (45, 46, 47):
            raise ValueError("patch location overlaps frame ECC/reserved words")
        if image.words[word] & (1 << bit):
            raise ValueError("PIC patch location is not blank")
    image.crc()
    return image


def insert_firmware(data, mapping, firmware):
    # Import the common validator; it checks all upper bits rather than masks.
    from prepare_adb_firmware import convert
    convert(firmware)
    image = validate_map(data, mapping)
    for logical, location in enumerate(mapping["bit_locations"]):
        address, bit = divmod(logical, 12)
        if firmware[2 * address + bit // 8] & (1 << (bit % 8)):
            word, shift = divmod(location, 32)
            image.words[word] |= 1 << shift
    # Only BRAM content bits in a validated map are changed. The export gate
    # requires frame ECC/reserved words to remain byte-identical to the vendor
    # references. Unsupported mappings that need ECC changes are NOT shipped.
    image.crc(repair=True)
    return image.bytes()
