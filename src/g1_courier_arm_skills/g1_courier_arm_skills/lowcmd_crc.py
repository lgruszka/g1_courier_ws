"""CRC32 for unitree_hg/msg/LowCmd, mirrors motor_crc_hg.cpp."""
from __future__ import annotations

import struct


class LowCmdCrc:
    _POLYNOMIAL = 0x04C11DB7
    _INIT = 0xFFFFFFFF

    def Crc(self, msg) -> int:
        return self._crc32_core(self._to_words(msg))

    def _to_words(self, msg):
        raw = bytearray()
        raw.extend(struct.pack('<BB2x', int(msg.mode_pr), int(msg.mode_machine)))
        for motor in msg.motor_cmd:
            raw.extend(struct.pack(
                '<B3x5fI',
                int(motor.mode),
                float(motor.q), float(motor.dq), float(motor.tau),
                float(motor.kp), float(motor.kd),
                int(motor.reserve),
            ))
        reserve = list(msg.reserve)
        if len(reserve) < 4:
            reserve = reserve + [0] * (4 - len(reserve))
        raw.extend(struct.pack('<4I', *(int(x) for x in reserve[:4])))
        return struct.unpack('<250I', bytes(raw[:1000]))

    def _crc32_core(self, words) -> int:
        crc = self._INIT
        for data in words:
            xbit = 1 << 31
            for _ in range(32):
                if crc & 0x80000000:
                    crc = ((crc << 1) & 0xFFFFFFFF) ^ self._POLYNOMIAL
                else:
                    crc = (crc << 1) & 0xFFFFFFFF
                if data & xbit:
                    crc ^= self._POLYNOMIAL
                xbit >>= 1
        return crc & 0xFFFFFFFF
