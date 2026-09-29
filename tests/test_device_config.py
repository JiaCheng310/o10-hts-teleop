"""Device configuration must never silently fall back to a different physical hand."""
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'teleop/driver'))
from quest_usb_bridge import validate_usb_identity
from hand_service import HandService


class DeviceIdentityTests(unittest.TestCase):
    def test_new_serial_is_supported_without_source_edit(self):
        with patch.dict(os.environ, {'O10_RIGHT_SERIAL': 'CUSTOM_RIGHT'}, clear=True):
            validate_usb_identity('/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_CUSTOM_RIGHT-if00', 'right')

    def test_wrong_port_and_unconfigured_side_are_rejected(self):
        with patch.dict(os.environ, {'O10_RIGHT_SERIAL': 'CUSTOM_RIGHT'}, clear=True):
            for port in ['/dev/ttyACM0', '/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_WRONG-if00']:
                with self.assertRaises(RuntimeError):
                    validate_usb_identity(port, 'right')
            with self.assertRaises(RuntimeError):
                validate_usb_identity('/dev/ttyACM0', 'left')

    def test_hardware_without_serial_fails_before_retargeter_or_sender(self):
        with patch.dict(os.environ, {}, clear=True), patch('hand_service.O10Retargeter') as rt, patch('hand_service.DirectSender') as sender:
            with self.assertRaises(ValueError):
                HandService(hardware=True, side='right')
            rt.assert_not_called()
            sender.assert_not_called()
