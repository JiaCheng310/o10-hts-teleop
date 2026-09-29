"""Start/reuse the persistent arm owner separately from disposable HTS processes."""
import json
from pathlib import Path
import socket
import subprocess
import sys
import time
from urllib.request import Request, urlopen
from x5_arm import DEFAULT_POSITION_SCALE, RelativePose, BEHAVIOR_REVISION

ROOT = Path(__file__).resolve().parent
URL = 'http://127.0.0.1:8043'


def request(path='/state', data=None):
    req = Request(URL+path, data=None if data is None else json.dumps(data).encode(),
                  headers={'Content-Type': 'application/json'})
    with urlopen(req, timeout=.5) as reply:
        return json.load(reply)


def existing(mode, profile='fast', scale=DEFAULT_POSITION_SCALE, ik_backend='unitree'):
    RelativePose(scale)
    try:
        with socket.create_connection(('127.0.0.1', 8043), timeout=.3):
            pass
    except ConnectionRefusedError:
        return None
    state = request()
    if (state.get('source') != 'hts-x5' or state.get('version') != 1
            or state.get('mode') != mode or state.get('side') != 'right'):
        raise RuntimeError('8043 已被其他服务/模式占用，请先停止预览或正常释放旧机械臂控制器')
    hz, preview = (120., .02) if profile == 'fast' else (30., .05)
    if state.get('control_hz') != hz or state.get('preview_time_s') != preview:
        raise RuntimeError('已运行的机械臂控制器参数不同；先暂停并正常释放，再切换 profile')
    if abs(state.get('position_scale', float('inf'))-scale) > 1e-9:
        raise RuntimeError(f'机械臂仍在使用平移系数 {state.get("position_scale")}，请求为 {scale}；'
                           '没有重启或退出它。继续原配置可传 --arm-scale '+str(state.get('position_scale'))+
                           '；更换系数须在支撑好机械臂后正常释放再启动。')
    if state.get('ik_backend', 'sdk') != ik_backend:
        raise RuntimeError('已有机械臂控制器使用旧 IK；未退出或重启它。支撑好后正常释放并重启加载新 IK。')
    if state.get('behavior_revision') != BEHAVIOR_REVISION:
        raise RuntimeError('磁盘上的 IK 已更新，但保持中的控制器仍是旧版本；没有停止它。'
                           '托住或固定机械臂后运行 ./release_hts_arm.sh，再正常启动以加载新版。')
    return state


def check(mode, profile='fast', scale=DEFAULT_POSITION_SCALE, ik_backend='unitree'):
    from x5_arm import Kinematics, hardware_preflight
    kin = Kinematics(ik_backend)  # Model-only FK/IK validation; never opens CAN.
    warm = kin.prewarm()
    state = existing(mode, profile, scale, ik_backend)
    if state is None and mode == 'hardware':
        hardware_preflight()
    print(f'X5 逆解环境正常；预热 {warm["elapsed_ms"]:.2f} ms；模式：{mode}', flush=True)


def ensure(mode, halt, profile='fast', scale=DEFAULT_POSITION_SCALE, ik_backend='unitree'):
    if halt.is_set():
        raise InterruptedError('已取消机械臂启动')
    state = existing(mode, profile, scale, ik_backend)
    if state is None:
        args = [sys.executable, '-u', str(ROOT/'x5_service.py'), '--profile', profile, '--scale', str(scale), '--ik-backend', ik_backend]
        if mode == 'hardware':
            args.append('--hardware')
        (ROOT/'output').mkdir(exist_ok=True)
        with (ROOT/'output/hts_x5.log').open('ab') as log:
            child = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=log,
                                     stderr=subprocess.STDOUT, start_new_session=True, cwd=ROOT)
        deadline = time.monotonic()+12
        while time.monotonic() < deadline:
            if child.poll() is not None:
                raise RuntimeError('X5 服务启动失败，请查看 output/hts_x5.log')
            try:
                state = existing(mode, profile, scale, ik_backend)
            except (OSError, ValueError):
                state = None
            if state is not None:
                break
            # Do not terminate/kill the physical controller on cancellation.
            time.sleep(.1)
        if state is None:
            raise RuntimeError('X5 启动未就绪；没有强杀控制器，请查看 output/hts_x5.log')
    request('/control', {'source': 'hts-x5', 'action': 'pause'})
    if halt.is_set():
        raise InterruptedError('已停止启动；X5 保持位置')
    print(f'X5 已保持当前位置，平移系数 {scale:g}，旋转 1:1；收到跟踪后在状态页对齐启用。', flush=True)


def pause():
    request('/control', {'source': 'hts-x5', 'action': 'pause'})
