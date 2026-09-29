"""Launch the optional recorder without making robot availability depend on it."""
import json
from pathlib import Path
import subprocess
import sys
import time
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parent


def state():
    with urlopen('http://127.0.0.1:8044/state', timeout=1) as reply:
        result = json.load(reply)
    if result.get('source') != 'quest-recording':
        raise RuntimeError('8044 被其他服务占用')
    return result


def ensure():
    try:
        return state()
    except OSError:
        pass
    (ROOT / 'output').mkdir(exist_ok=True)
    with (ROOT / 'output/quest_recording.log').open('ab') as log:
        child = subprocess.Popen([sys.executable, '-u', str(ROOT / 'quest_recording.py')],
                                 cwd=ROOT, stdin=subprocess.DEVNULL, stdout=log,
                                 stderr=subprocess.STDOUT, start_new_session=True)
    for _ in range(40):
        if child.poll() is not None:
            raise RuntimeError('录屏服务启动失败，查看 output/quest_recording.log')
        try:
            return state()
        except OSError:
            time.sleep(.1)
    raise RuntimeError('录屏服务启动超时')


if __name__ == '__main__':
    print(json.dumps(ensure(), ensure_ascii=False))
