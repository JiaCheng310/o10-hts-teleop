"""Persistent X5 controller: stopping HTS pauses motion, never kills motor support."""
import argparse
import asyncio
import contextlib
import fcntl
import json
import signal
import socket
import time
from urllib.request import Request, urlopen

from aiohttp import web
import numpy as np
from x5_arm import ArmTeleop, HardwareArm, Kinematics, PreviewArm, ROOT, TargetRejected, RelativePose, DEFAULT_POSITION_SCALE


async def worker(arm):
    cache_key, cached_solution = None, None
    cached_rejection = None
    cached_fraction = 1.
    cached_diagnostics = None
    cached_motion_terminal = False
    pending_motion = False
    while True:
        # New poses wake immediately. The timeout checks tracking loss and lets
        # velocity-limited moves approach their last fresh target between frames.
        with contextlib.suppress(asyncio.TimeoutError):
            await asyncio.wait_for(arm.event.wait(), timeout=1/arm.hz if pending_motion or arm.homing else .02)
        arm.event.clear()
        try:
            if arm.enabled:
                await asyncio.sleep(max(0., arm.last_send+1/arm.hz-time.monotonic()))
            pending_motion = False
            if arm.homing:
                cache_key, cached_solution, cached_rejection = None, None, None
                arm.step_home()
                continue
            try:
                work = arm.prepare()
            except TargetRejected as error:
                if arm.reject_target(None, error):
                    print('X5_TARGET_HOLD', str(error), flush=True)
                continue
            if work is not None:
                if arm.kin.backend == 'unitree':
                    # The feasible box depends on the LAST SENT command and
                    # velocity, even for the same HTS frame. Never reuse the
                    # old pose-only cache for a new control tick.
                    key = work[0], work[4]
                    if key == cache_key and cached_motion_terminal:
                        continue  # prepare() still checks tracking age every tick.
                    if key == cache_key and cached_rejection is not None:
                        arm.reject_target(work, cached_rejection)
                        continue
                    cached_rejection = None
                    cached_motion_terminal = False
                    motion = arm.motion_context(work)
                    tick = time.monotonic()
                    try:
                        solution, _, fraction = await asyncio.to_thread(
                            arm.kin.solve_step, work[2], work[3], motion=motion)
                    except TargetRejected as error:
                        cache_key, cached_rejection = key, error
                        if arm.reject_target(work, error):
                            print('X5_TARGET_HOLD', str(error), flush=True)
                        continue
                    diagnostics = dict(arm.kin.last_solution)
                    arm.ik_ms = (time.monotonic()-tick)*1000
                    arm.finish(work, solution, motion=motion)
                    cache_key = key
                    cached_motion_terminal = fraction == 1.
                    if arm.enabled:
                        arm.last_ik_fraction, arm.ik_solution = fraction, diagnostics
                        if diagnostics.get('status') == 'limited':
                            arm.reason = ('已接近当前可达姿态，继续接收目标；位置误差 '
                                f"{diagnostics['position_error_mm']:.1f} mm，姿态误差 "
                                f"{diagnostics['rotation_error_deg']:.1f}°")
                    pending_motion = arm.enabled and fraction < 1.
                    continue
                key = work[0], work[4]
                if key != cache_key:
                    tick = time.monotonic()
                    cached_rejection = None
                    try:
                        cached_solution, _, cached_fraction = await asyncio.to_thread(arm.kin.solve_step, work[2], work[3])
                        cached_diagnostics = None if arm.kin.last_solution is None else dict(arm.kin.last_solution)
                    except TargetRejected as error:
                        cached_solution, cached_rejection = None, error
                    arm.ik_ms = (time.monotonic()-tick)*1000
                    cache_key = key
                elif cached_rejection is None and np.max(np.abs(cached_solution-arm.command)) < 1e-7:
                    if cached_fraction < 1.:
                        cache_key = None  # Reached a substep: continue toward the same fresh goal.
                        pending_motion = True
                    continue
                if cached_rejection is not None:
                    if arm.reject_target(work, cached_rejection):
                        print('X5_TARGET_HOLD', str(cached_rejection), flush=True)
                    continue
                arm.finish(work, cached_solution)
                if arm.enabled:
                    arm.last_ik_fraction = cached_fraction
                    arm.ik_solution = cached_diagnostics
                    if cached_diagnostics and cached_diagnostics.get('status') == 'limited':
                        arm.reason = ('已接近当前可达姿态，继续接收目标；位置误差 '
                            f"{cached_diagnostics['position_error_mm']:.1f} mm，姿态误差 "
                            f"{cached_diagnostics['rotation_error_deg']:.1f}°")
                pending_motion = arm.enabled and (cached_fraction < 1. or np.max(np.abs(cached_solution-arm.command)) >= 1e-7)
        except Exception as error:
            arm.pause(str(error))
            arm.error = str(error)
            print('X5_HOLD', str(error), flush=True)


def application(arm, release_event=None):
    app = web.Application(client_max_size=8192)

    async def state(_):
        return web.json_response(arm.snapshot(), headers={'Cache-Control': 'no-store'})

    async def metrics(_):
        return web.json_response(dict(samples=list(arm.latency_samples)),
                                 headers={'Cache-Control': 'no-store'})

    async def frame(request):
        try:
            arm.feed(await request.json())
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            return web.json_response({'error': str(error)}, status=400)
        return web.json_response(arm.snapshot())

    async def control(request):
        data = await request.json()
        if data.get('source') != 'hts-x5':
            return web.json_response({'error': 'HTS X5 control required'}, status=409)
        try:
            if data.get('action') == 'align':
                if data.get('session') != arm.session:
                    raise ValueError('HTS 会话已变化，请等待新数据')
                arm.align()
            elif data.get('action') == 'pause':
                if not (data.get('tracking_only') is True and arm.homing):
                    arm.pause('已暂停，保持当前位置；继续前请重新对齐')
            elif data.get('action') == 'home':
                arm.start_home()
            elif data.get('action') == 'scale':
                scale = RelativePose(float(data['scale'])).scale
                if arm.enabled or arm.homing:
                    raise ValueError('先暂停机械臂，再调整平移系数')
                arm.mapping.scale = scale
                arm.mapping.anchor = None
                arm.reason = f'平移系数已设为 {scale:g}，请重新对齐'
            elif data.get('action') == 'release':
                if data.get('arm_supported') is not True or release_event is None:
                    raise ValueError('先支撑机械臂，再使用 release 命令退出控制器')
                arm.pause('退出控制器')
                release_event.set()
            else:
                raise ValueError('Unknown arm action')
        except (ValueError, RuntimeError, KeyError, TypeError) as error:
            if data.get('action') == 'home':
                arm.pause('回零未完成：'+str(error))
                arm.error = str(error)
            return web.json_response({'error': str(error)}, status=409)
        return web.json_response(arm.snapshot())

    app.router.add_get('/state', state)
    app.router.add_get('/metrics', metrics)
    app.router.add_post('/frame', frame)
    app.router.add_post('/control', control)

    async def startup(a):
        a['worker'] = asyncio.create_task(worker(arm))

    async def cleanup(a):
        arm.pause('服务停止')
        a['worker'].cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await a['worker']

    app.on_startup.append(startup)
    app.on_cleanup.append(cleanup)
    return app


async def serve(args):
    RelativePose(args.scale)  # Validate before any hardware driver is constructed.
    lock = open('/tmp/hts-x5-arm.lock' if args.hardware else f'/tmp/hts-x5-preview-{args.port}.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    # Reserve the HTTP port BEFORE any SDK constructor can touch motors.
    sock = socket.socket()
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('127.0.0.1', args.port))
    sock.listen(32)
    sock.setblocking(False)
    kin = Kinematics(args.ik_backend)
    # Like EVA's prewarm, this publishes no action. Use the same worker path as
    # live IK so the first tracked frame does not create the thread/solver path.
    prewarm = await asyncio.to_thread(kin.prewarm)
    print('X5_IK_PREWARM', json.dumps(prewarm), flush=True)
    hz, preview = (120., .02) if args.profile == 'fast' else (30., .05)
    output = HardwareArm(kin, args.interface, preview) if args.hardware else PreviewArm(args.preview_q)
    arm = ArmTeleop(kin, output, scale=args.scale, hz=hz, preview_time_s=preview)
    released = asyncio.Event()
    loop = asyncio.get_running_loop()

    def handle_signal(sig):
        arm.pause(f'收到 {sig.name}，保持当前位置')
        if not args.hardware:
            released.set()
        else:
            print('X5 保持位置。支撑机械臂后用 ./release_hts_arm.sh 退出控制器。', flush=True)

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        loop.add_signal_handler(sig, handle_signal, sig)
    runner = web.AppRunner(application(arm, released), shutdown_timeout=1)
    try:
        await runner.setup()
        await web.SockSite(runner, sock).start()
        print('X5_READY', json.dumps(arm.snapshot(), ensure_ascii=False), flush=True)
        if args.hardware and not kin.mount.get('payload_verified'):
            print('当前为 X5 无原夹爪模型；O10+转接件负载参数尚未标定。', flush=True)
        await released.wait()
    finally:
        await runner.cleanup()
        # Only an explicit supported-arm release reaches normal hardware exit.
        # The SDK destructor transitions X5 to passive; never auto-home here.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', nargs='?', choices=('run', 'pause', 'release'), default='run')
    parser.add_argument('--hardware', action='store_true')
    parser.add_argument('--port', type=int, default=8043)
    parser.add_argument('--interface', default='can1')
    parser.add_argument('--profile', choices=('fast', 'reference'), default='fast')
    parser.add_argument('--ik-backend', choices=('unitree', 'sdk'), default='unitree')
    parser.add_argument('--scale', type=float, default=DEFAULT_POSITION_SCALE)
    parser.add_argument('--preview-q', type=float, nargs=6, default=[0., .5, .8, 0., .1, 0.])
    args = parser.parse_args()
    if args.action != 'run':
        if args.action == 'release':
            if input('已托住/固定机械臂后输入 RELEASE（随后电机转为被动）：').strip() != 'RELEASE':
                return 1
        data = dict(source='hts-x5', action=args.action, arm_supported=args.action == 'release')
        req = Request(f'http://127.0.0.1:{args.port}/control', data=json.dumps(data).encode(),
                      headers={'Content-Type': 'application/json'})
        with urlopen(req, timeout=2) as reply:
            print(json.load(reply)['status'])
        return 0
    asyncio.run(serve(args))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
