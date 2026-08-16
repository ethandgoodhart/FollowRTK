#!/usr/bin/env python3
"""Pi GPS-bridge config must not disable RTCM, and must use UART1 keys."""

import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cartlib import config
from cartlib.gps import _CFG_UART1INPROT_RTCM3X, _CFG_USBINPROT_RTCM3X
from cartlib.gps import _configure_receiver


class FakeSer:
    def __init__(self):
        self.written = []
        self.in_waiting = 0

    def write(self, data):
        self.written.append(bytes(data))
        return len(data)

    def read(self, n):
        return b""


class TestGpsBridgeConfig(unittest.TestCase):
    def test_bridge_valset_enables_uart1_rtcm_not_usb(self):
        ser = FakeSer()
        with mock.patch("cartlib.gps.time.sleep"):
            ok = _configure_receiver(ser, uart_bridge=True)
        self.assertTrue(ok)
        self.assertEqual(len(ser.written), 1)
        packet = ser.written[0]
        self.assertIn(_CFG_UART1INPROT_RTCM3X.to_bytes(4, "little"), packet)
        self.assertNotIn(_CFG_USBINPROT_RTCM3X.to_bytes(4, "little"), packet)

    def test_usb_valset_still_enables_usb_rtcm(self):
        ser = FakeSer()
        with mock.patch("cartlib.gps.time.sleep"):
            ok = _configure_receiver(ser, uart_bridge=False)
        self.assertTrue(ok)
        packet = ser.written[0]
        self.assertIn(_CFG_USBINPROT_RTCM3X.to_bytes(4, "little"), packet)

    def test_bridge_open_skips_ubx_config_and_keeps_corrections(self):
        from cartlib.gps import GpsReceiver

        gps = GpsReceiver(port="/dev/gps-bridge")
        class OkSer:
            is_open = True
            in_waiting = 0
            dtr = False
            rts = False
            def write(self, data):
                raise AssertionError("bridge must not send UBX config")
        with mock.patch.object(config, "is_gps_bridge", return_value=True), \
             mock.patch("cartlib.gps.serial.Serial", return_value=OkSer()), \
             mock.patch.object(config, "find_gps_port", return_value="/dev/gps-bridge"):
            gps._open_serial_locked()
        self.assertFalse(gps._config_pending)
        self.assertTrue(gps.corrections_supported)

    def test_write_corrections_chunks_and_releases(self):
        from cartlib.gps import GpsReceiver, _RTCM_CHUNK

        gps = GpsReceiver(port="/dev/gps-bridge")
        writes = []
        class OkSer:
            is_open = True
            in_waiting = 0
            def write(self, data):
                writes.append(bytes(data))
                return len(data)
        gps._ser = OkSer()
        gps._open_serial_locked = lambda: None  # already open
        payload = b"\xaa" * (_RTCM_CHUNK * 2 + 10)
        gps.write_corrections(payload)
        self.assertEqual(len(writes), 3)
        self.assertEqual(sum(len(w) for w in writes), len(payload))

    def test_is_gps_bridge_by_symlink_name(self):
        self.assertTrue(config.is_gps_bridge("/dev/gps-bridge"))
        self.assertFalse(config.is_gps_bridge(
            "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00"))


if __name__ == "__main__":
    unittest.main()
