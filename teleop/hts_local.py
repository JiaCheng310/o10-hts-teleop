"""Run Quest HTS and the selected O10 entirely on ARX; default to RIGHT."""
import argparse
import contextlib
import fcntl
import grp
import json
import os
from pathlib import Path
import shutil
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time
import pwd
from urllib.request import Request, urlopen
from x5_arm import DEFAULT_POSITION_SCALE

ROOT = Path(__file__).resolve().parent
OUTPUT = ROOT / 'output'
PIDFILE = OUTPUT / 'hts_local.pid.json'
QUEST = os.environ.get('QUEST_SERIAL', '')
SIDE = 'right'
LABEL = '右手'
SERIALS = {side: os.environ.get('O10_'+side.upper()+'_SERIAL', '') for side in ('left','right')}
HAND = SERIALS[SIDE]
PORT = Path('/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_' + HAND + '-if00')
PACKAGE = 'com.wengmister.handtrackingstreamer'
ACTIVITY = PACKAGE + '/com.unity3d.player.UnityPlayerGameActivity'


def configure(side):
    global SIDE, LABEL, HAND, PORT
    SIDE = side
    LABEL = '右手' if side == 'right' else '左手'
    HAND = SERIALS[side]
    PORT = Path('/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_' + HAND + '-if00')


def refresh_serial_group():
    """Refresh an already-granted dialout membership inherited from an old terminal."""
    group = grp.getgrnam('dialout')
    if group.gr_gid in {os.getgid(), os.getegid(), *os.getgroups()}:
        return
    user = pwd.getpwuid(os.getuid())
    if user.pw_name not in group.gr_mem:
        return  # Do not grant new permissions; preflight reports missing access.
    sg = shutil.which('sg')
    if sg is None:
        raise RuntimeError('当前终端尚未获得 dialout 权限，请重新登录 ARX 后再启动。')
    command = shlex.join([sys.executable, str(ROOT / 'hts_local.py'), *sys.argv[1:]])
    print('当前终端的串口组权限未刷新，正在应用已授予的 dialout 权限。', flush=True)
    os.execv(sg, [sg, 'dialout', '-c', command])


def adb(*args, check=True):
    return subprocess.run(['adb', '-s', QUEST, *args], capture_output=True,
                          text=True, timeout=8, check=check)


def request(port, path='/state', data=None):
    req = Request(f'http://127.0.0.1:{port}{path}',
                  data=None if data is None else json.dumps(data).encode(),
                  headers={'Content-Type': 'application/json'})
    with urlopen(req, timeout=1) as reply:
        return json.load(reply)


def starttime(pid):
    return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19]


def stop():
    if not PIDFILE.exists():
        print('ARX HTS 已停止。')
        return
    state = json.loads(PIDFILE.read_text())
    pid = state['pid']
    try:
        cmd = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
        owned = starttime(pid) == state['starttime'] and os.fsencode(ROOT / 'hts_local.py') in cmd
    except FileNotFoundError:
        print('ARX HTS 已停止（旧 PID 记录）。')
        return
    if not owned:
        raise RuntimeError('PID 已被其他进程使用，没有终止它。')
    os.kill(pid, signal.SIGTERM)
    for _ in range(300):
        if not PIDFILE.exists():
            print('ARX HTS 和手驱动已停止。')
            return
        time.sleep(.1)
    raise RuntimeError('停止尚未完成，请查看启动终端和 output 日志。')


def preflight(hand_required=True):
    # Importing these modules opens neither the serial port nor the arm.
    import aiohttp, numpy, scipy, nlopt, omnihand  # noqa: F401
    from mapping import O10Retargeter  # noqa: F401
    print('环境依赖正常：' + sys.executable, flush=True)
    problems = []
    if not QUEST:
        raise RuntimeError('Set QUEST_SERIAL in config/devices.env first.')
    if hand_required and not HAND:
        raise RuntimeError('Set O10_'+SIDE.upper()+'_SERIAL in config/devices.env first.')
    if hand_required and not PORT.exists():
        problems.append('请将 O10 ' + LABEL + ' ' + HAND + ' 的 USB 接到 ARX，并保持供电。')
    elif hand_required and not os.access(PORT, os.R_OK | os.W_OK):
        problems.append(LABEL+'串口无读写权限；检查 ARX 用户的 dialout 权限。')
    if not shutil.which('adb'):
        problems.append('缺少 adb。')
    else:
        result = adb('get-state', check=False)
        if result.returncode or result.stdout.strip() != 'device':
            problems.append('请将 Quest 3 的 USB 接到 ARX，并在头显内允许这台 ARX 的 USB 调试。')
        else:
            package = adb('shell', 'pm', 'path', PACKAGE, check=False)
            if package.returncode or 'package:' not in package.stdout:
                problems.append('Quest 未安装 Hand Tracking Streamer；APK 在 ../hand-tracking-streamer/。')
    if problems:
        raise RuntimeError('\n'.join(problems))
    print('Quest 3 和指定的 O10 '+LABEL+'已就绪。' if hand_required else
          'Quest 3 已就绪；手指仅预览，不连接 O10 串口。', flush=True)


def check_free_ports(hand_required=True):
    for port in ((8000, 8042, 8890) if hand_required else (8000, 8042)):
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                sock.bind(('127.0.0.1', port))
            except OSError as error:
                raise RuntimeError(f'端口 {port} 已被占用；没有启动第二套控制程序。') from error


def run(arm_mode=None, arm_profile='fast', hand_preview=False, arm_scale=DEFAULT_POSITION_SCALE, ik_backend='unitree'):
    OUTPUT.mkdir(exist_ok=True)
    with (OUTPUT / 'hts_local.lock').open('w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('ARX HTS 已在运行。停止请用 ./stop_hts_right.sh')
        if hand_preview:
            preflight(hand_required=False)
            check_free_ports(hand_required=False)
        else:
            preflight()
            check_free_ports()
        if arm_mode:
            import arm_launch
            arm_launch.check(arm_mode, arm_profile, arm_scale, ik_backend)
        halt = threading.Event()
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, lambda *_: halt.set())
        children = []
        reverse = False
        receiver = None
        PIDFILE.write_text(json.dumps({'pid': os.getpid(), 'starttime': starttime(os.getpid())}))

        def launch(args, logname, env=None):
            with (OUTPUT / logname).open('ab') as log:
                child = subprocess.Popen([sys.executable, '-u', *args], cwd=ROOT, env=env,
                                         stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                         start_new_session=True)
            children.append(child)
            return child

        def wait_ready(child, port, valid):
            deadline = time.monotonic() + 12
            while not halt.is_set() and time.monotonic() < deadline:
                if child.poll() is not None:
                    raise RuntimeError(f'服务启动失败（退出码 {child.returncode}），请查看 output 日志。')
                try:
                    state = request(port)
                except (OSError, ValueError):
                    halt.wait(.1)
                    continue
                if not valid(state):
                    raise RuntimeError(f'{port} 返回了不匹配的设备或服务身份。')
                return
            if halt.is_set():
                raise InterruptedError('已停止启动。')
            raise RuntimeError(f'{port} 启动超时，请查看 output 日志。')

        try:
            if not hand_preview:
                bridge = launch([str(ROOT / 'driver/quest_usb_bridge.py'), '--hand', SIDE, '--port', str(PORT)], 'hts_usb_'+SIDE+'.log')
                wait_ready(bridge, 8890, lambda s: s.get('hand') == SIDE and s.get('usb_port') == str(PORT)
                           and s.get('protocol') == 2 and len(s.get('angles', [])) == 10)
            env = dict(os.environ, O10_BRIDGE_URL='http://127.0.0.1:8890', O10_EXPECTED_SERIAL=HAND)
            receiver_args = [str(ROOT / 'hts_service.py'), '--side', SIDE]
            if not hand_preview:
                receiver_args.append('--hardware')
            if arm_mode:
                arm_launch.ensure(arm_mode, halt, arm_profile, arm_scale, ik_backend)
                receiver_args += ['--arm-url', arm_launch.URL]
            receiver = launch(receiver_args, 'hts_hardware.log', env)
            wait_ready(receiver, 8042, lambda s: s.get('source') == 'hts' and s.get('host') == 'arx'
                       and s.get('mode') == ('preview' if hand_preview else 'hardware')
                       and s.get('side') == SIDE and s.get('hardware_usb_serial') == HAND)
            if halt.is_set():
                return
            # Recorder failure must never stop an otherwise healthy robot controller.
            try:
                import recording_launch
                recording_launch.ensure()
            except Exception as error:
                print('录屏服务暂不可用（遥操继续）：'+str(error), flush=True)
            adb('reverse', 'tcp:8000', 'tcp:8000')
            reverse = True
            adb('shell', 'am', 'start', '-n', ACTIVITY)
            print('ARX 本机遥操已就绪。状态：http://127.0.0.1:8042', flush=True)
            print('头显内选 TCP - wired、'+SIDE.title()+'，打开 Debug Info 和 Show Landmarks，再点 Start Streaming。', flush=True)
            if arm_mode:
                print('机械臂模式：还需开启 Head Pose；平视前方，在状态页点击「对齐并启用机械臂」。', flush=True)
                print('Ctrl+C 停止跟随后，X5 控制器继续保持位置；支撑好后用 ./release_hts_arm.sh 退出。', flush=True)
            else:
                print('按 Ctrl+C 或运行 ./stop_hts_'+SIDE+'.sh 停止。', flush=True)
            while not halt.wait(.5):
                if any(child.poll() is not None for child in children):
                    raise RuntimeError('接收服务或驱动退出，已停止整套遥操。请查看 output 日志。')
                if not hand_preview and not PORT.exists():
                    raise RuntimeError(LABEL+' USB 已断开，遥操已停止。重新接好后再运行启动命令。')
        finally:
            if arm_mode:
                try:
                    arm_launch.pause()
                    print('X5 控制器继续保持位置；释放前不能启动 U-Arm 主从。', flush=True)
                except Exception as error:
                    print('机械臂暂停请求未确认（输入超时也会停止跟随）：'+str(error), flush=True)
            if receiver is not None and receiver.poll() is None:
                with contextlib.suppress(Exception):
                    request(8042, '/control', {'source': 'hts', 'action': 'pause'})
            # Stop the receiver before releasing the physical driver.
            for child in reversed(children):
                if child.poll() is None:
                    child.terminate()
                    try:
                        child.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait(timeout=2)
            if reverse:
                for args in [('reverse', '--remove', 'tcp:8000'), ('shell', 'am', 'force-stop', PACKAGE)]:
                    with contextlib.suppress(Exception):
                        adb(*args, check=False)
            PIDFILE.unlink(missing_ok=True)
            print('遥操和手驱动已停止。', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('left','right'), default='right')
    parser.add_argument('--arm', choices=('hardware','preview'), help='Also control X5 from the HTS wrist pose')
    parser.add_argument('--arm-profile', choices=('fast','reference'), default='fast')
    parser.add_argument('--ik-backend', choices=('unitree','sdk'), default='unitree')
    parser.add_argument('--arm-scale', type=float, default=DEFAULT_POSITION_SCALE,
                        help='Translation gain (default 1.0); wrist orientation remains 1:1')
    parser.add_argument('--hand-preview', action='store_true', help='Use DexPilot preview; do not connect physical O10')
    parser.add_argument('action', choices=('run', 'check', 'stop'), nargs='?', default='run')
    args = parser.parse_args()
    configure(args.side)
    os.environ['O10_EXPECTED_SERIAL'] = HAND
    try:
        if args.action != 'stop' and not args.hand_preview:
            refresh_serial_group()
        if args.arm and args.side != 'right':
            raise RuntimeError('当前 X5 集成仅支持右手')
        if args.action == 'run':
            run(args.arm, args.arm_profile, args.hand_preview, args.arm_scale, args.ik_backend)
        elif args.action == 'check':
            preflight(hand_required=not args.hand_preview)
            if args.arm:
                import arm_launch
                arm_launch.check(args.arm, args.arm_profile, args.arm_scale, args.ik_backend)
        else:
            stop()
    except (RuntimeError, OSError, subprocess.SubprocessError, InterruptedError) as error:
        print(str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
