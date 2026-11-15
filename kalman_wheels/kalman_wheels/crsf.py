SYNC_BYTES = (0xC8, 0xEE, 0xEA)
FRAME_LINK_STATISTICS = 0x14
FRAME_RC_CHANNELS_PACKED = 0x16
CHANNEL_MIN, CHANNEL_MID, CHANNEL_MAX = 172, 992, 1811


def crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def unpack_channels(payload: bytes) -> list:
    bits = int.from_bytes(payload[:22], "little")
    return [(bits >> (11 * i)) & 0x7FF for i in range(16)]


def normalize_channel(v: int) -> float:
    # 820 steps below mid, 819 above: scale each half so MIN/MID/MAX -> -1/0/1 exactly
    span = (CHANNEL_MAX - CHANNEL_MID) if v >= CHANNEL_MID else (CHANNEL_MID - CHANNEL_MIN)
    return max(-1.0, min(1.0, (v - CHANNEL_MID) / span))


class CrsfParser:
    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes) -> list:
        """Append bytes; return a list of complete (type, payload) frames."""
        self.buf.extend(data)
        frames = []
        while True:
            while self.buf and self.buf[0] not in SYNC_BYTES:
                del self.buf[0]
            if len(self.buf) < 2:
                break
            length = self.buf[1]
            if not 2 <= length <= 62:          # CRSF max frame is 64 bytes
                del self.buf[0]
                continue
            if len(self.buf) < length + 2:
                break                           # incomplete, wait for more bytes
            body = bytes(self.buf[2:length + 1])
            if crc8_dvb_s2(body) != self.buf[length + 1]:
                self.crc_errors += 1
                del self.buf[0]                 # resync one byte later
                continue
            frames.append((body[0], body[1:]))
            del self.buf[:length + 2]
        return frames