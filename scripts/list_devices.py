"""Print USB identifiers without opening a robot serial port."""
from pathlib import Path
import re
import shutil
import subprocess

print('Quest / Android devices (the first column is QUEST_SERIAL):', flush=True)
if shutil.which('adb'):
    subprocess.run(['adb', 'devices', '-l'], check=True, timeout=15)
else:
    print('adb missing: sudo apt install adb')

print('\nO10 serial devices:')
found = False
for path in sorted(Path('/dev/serial/by-id').glob('*')):
    match = re.fullmatch(r'usb-STMicroelectronics_STM32_Virtual_ComPort_(.+)-if\d+', path.name)
    if match:
        found = True
        print(f'  Serial: {match[1]}\n  Port:   {path}')
if not found:
    print('  No STM32 Virtual ComPort found. Check power and the USB data cable.')
print('\nUSB enumeration cannot determine left/right. Assign the serial to the physical hand you connected.')
