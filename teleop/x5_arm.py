"""HTS relative wrist control; no CAN connection until HardwareArm is constructed."""
import asyncio
from collections import deque
import fcntl
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

ROOT = Path(__file__).resolve().parent
XR_TO_ROBOT = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
STALE_S = .2
DEFAULT_POSITION_SCALE = 1.0
BEHAVIOR_REVISION = 'unitree-motion-v5'
DEFAULT_POLICY = dict(relative_translation_m=None, relative_rotation_rad=None,
                      ik_joint_step_rad=.45, ik_backtrack_factors=[1., .5, .25, .125, .0625],
                      ik_budget_ms=6.)


class TargetRejected(ValueError):
    """Valid tracking, but this target cannot currently be commanded."""


def rotation(value):
    value = np.asarray(value, float)
    if (value.shape != (3, 3) or not np.isfinite(value).all()
            or not np.allclose(value.T @ value, np.eye(3), atol=1e-5)
            or abs(np.linalg.det(value)-1) > 1e-5):
        raise ValueError('Invalid rotation matrix')
    return value


def vector(value, n):
    value = np.asarray(value, float)
    if value.shape != (n,) or not np.isfinite(value).all():
        raise ValueError('Invalid pose/joint vector')
    return value


def angle(a, b):
    return float(R.from_matrix(a @ b.T).magnitude())


class Kinematics:
    def __init__(self, backend="unitree"):
        if backend not in ("unitree", "sdk"):
            raise ValueError("Unknown IK backend")
        self.backend = backend
        self.last_solution = None
        import arx5_interface as sdk
        self.config = c = sdk.RobotConfigFactory.get_instance().get_config('X5')
        c.gripper_motor_type = sdk.MotorType.NONE
        c.eef_link_name = 'link6'
        c.urdf_path = str(Path(sdk.__file__).parent / 'models/X5_no_gripper.urdf')
        mount_path = ROOT / 'mount.json'
        self.mount = json.loads(mount_path.read_text()) if mount_path.exists() else {}
        policy_path = ROOT / 'arm_control.json'
        policy = json.loads(policy_path.read_text()) if policy_path.exists() else {}
        if not isinstance(policy, dict) or set(policy)-set(DEFAULT_POLICY):
            raise ValueError('Unknown arm_control.json parameters')
        self.policy = {**DEFAULT_POLICY, **policy}
        for name in ('relative_translation_m', 'relative_rotation_rad'):
            value = self.policy[name]
            if value is not None and (not np.isfinite(value) or value <= 0):
                raise ValueError(name+' must be positive or null')
        for name in ('ik_joint_step_rad', 'ik_budget_ms'):
            value = self.policy[name]
            if not np.isfinite(value) or value <= 0:
                raise ValueError(name+' must be positive')
        factors = self.policy['ik_backtrack_factors']
        if (not isinstance(factors, list) or not 1 <= len(factors) <= 8
                or factors[0] != 1 or not all(np.isfinite(f) and 0 < f <= 1 for f in factors)
                or any(a <= b for a, b in zip(factors, factors[1:]))):
            raise ValueError('ik_backtrack_factors must decrease from 1, with at most 8 entries')
        # An explicitly calibrated payload model can replace the arm-only model.
        if self.mount.get('payload_verified'):
            payload = Path(self.mount['payload_urdf'])
            payload = payload if payload.is_absolute() else ROOT / payload
            if not payload.is_file():
                raise ValueError('Verified payload URDF missing')
            c.urdf_path = str(payload)
        self.low, self.high = np.array(c.joint_pos_min), np.array(c.joint_pos_max)
        self.solver = sdk.Arx5Solver(c.urdf_path, 6, self.low, self.high,
                                    c.base_link_name, c.eef_link_name, c.gravity_vector)
        self.prewarm_result = None
        self.optimizer = None
        self.model_check = None
        if backend == 'unitree':
            from unitree_ik import UnitreeIK
            self.optimizer = UnitreeIK(c.urdf_path, self.low, self.high, ROOT/'unitree_ik.json')
            self.model_check = self.optimizer.verify_model(self.fk)

    def prewarm(self):
        """Exercise FK/IK using model-only poses before opening a motor driver."""
        if self.prewarm_result is not None:
            return dict(self.prewarm_result)
        started = time.monotonic()
        seed = np.clip(np.array([0., .5, .8, 0., .1, 0.]), self.low, self.high)
        probe = np.clip(seed + .001, self.low, self.high)
        target = self.fk(probe)
        solved = self.ik(target, seed)
        actual = self.fk(solved)
        self.prewarm_result = dict(ready=True, backend=self.backend, model_check=self.model_check,
            elapsed_ms=(time.monotonic()-started)*1000,
            position_residual_m=float(np.linalg.norm(actual[:3]-target[:3])))
        return dict(self.prewarm_result)

    def fk(self, q):
        return vector(self.solver.forward_kinematics(vector(q, 6)), 6)

    def equivalent_joints(self, q, seed):
        """Map revolute IK output into real limits, without changing its FK pose.

        Equivalent endpoint angles do NOT imply a traversable short rotation.
        The local step guard below still rejects a long move across a stop.
        """
        q, seed = vector(q, 6), vector(seed, 6)
        period = 2*np.pi
        lower_turn = np.ceil((self.low-q-1e-6)/period)
        upper_turn = np.floor((self.high-q+1e-6)/period)
        bad = lower_turn > upper_turn
        if np.any(bad):
            joints = ', '.join('J'+str(i+1) for i in np.flatnonzero(bad))
            raise TargetRejected('逆解超出关节范围：'+joints)
        turns = np.clip(np.rint((seed-q)/period), lower_turn, upper_turn)
        return np.clip(q+period*turns, self.low, self.high)

    def ik(self, target, seed):
        target, seed = vector(target, 6), vector(seed, 6)
        if self.backend == 'unitree':
            q, actual, _ = self.solve_step(target, seed)
            if np.linalg.norm(actual[:3]-target[:3]) > .008 or angle(
                    R.from_euler('xyz', actual[3:]).as_matrix(),
                    R.from_euler('xyz', target[3:]).as_matrix()) > .08:
                raise TargetRejected('Constrained IK needs further steps')
            return q
        code, q = self.solver.inverse_kinematics(target, seed)
        if code != 0:
            raise TargetRejected('逆解未收敛: ' + self.solver.get_ik_status_name(code))
        q = self.equivalent_joints(q, seed)
        actual = self.fk(q)
        if np.linalg.norm(actual[:3]-target[:3]) > .008:
            raise TargetRejected('逆解位置误差超过 8 mm')
        if angle(R.from_euler('xyz', actual[3:]).as_matrix(),
                 R.from_euler('xyz', target[3:]).as_matrix()) > .08:
            raise TargetRejected('逆解姿态误差超过 0.08 rad')
        if np.max(np.abs(q-seed)) > self.policy['ik_joint_step_rad']:
            # This bounds distance to the current command; a distant continuous
            # target can trip it too. It is not proof of an IK branch switch.
            raise TargetRejected('逆解目标超过当前局部求解步幅')
        return q

    def solve_step(self, target, seed, *, motion=None):
        """Try the full goal, then reachable SE(3) substeps; never relax joint limits."""
        target, seed = vector(target, 6), vector(seed, 6)
        if self.backend == 'unitree':
            q, continuation, info = self.optimizer.solve(target, seed, motion=motion)
            actual = self.fk(q)  # Cross-check every result with the independent SDK FK.
            p, r, _ = self.optimizer.geometry(q)
            if np.linalg.norm(actual[:3]-p) > 1e-6 or angle(
                    R.from_euler('xyz', actual[3:]).as_matrix(), r) > 1e-6:
                raise RuntimeError('IK model mismatch')
            self.last_solution = info
            return q, actual, continuation
        self.last_solution = dict(backend='sdk', status='legacy')
        start = time.monotonic()
        current = self.fk(seed)
        r0 = R.from_euler('xyz', current[3:]).as_matrix()
        delta_r = R.from_matrix(R.from_euler('xyz', target[3:]).as_matrix() @ r0.T).as_rotvec()
        last_error = None
        for fraction in self.policy['ik_backtrack_factors']:
            if last_error is not None and (time.monotonic()-start)*1000 > self.policy['ik_budget_ms']:
                break
            intermediate = target if fraction == 1 else np.r_[
                current[:3]+fraction*(target[:3]-current[:3]),
                R.from_matrix(R.from_rotvec(fraction*delta_r).as_matrix() @ r0).as_euler('xyz')]
            try:
                q = self.ik(intermediate, seed)
                return q, intermediate.copy(), float(fraction)
            except TargetRejected as error:
                last_error = error
        raise TargetRejected(str(last_error))


class RelativePose:
    def __init__(self, scale=1., max_translation=None, max_rotation=None):
        if not np.isfinite(scale) or not 0 < scale <= 2:
            raise ValueError('Position scale must be in (0, 2]')
        self.scale, self.anchor = scale, None
        self.max_translation, self.max_rotation = max_translation, max_rotation

    def align(self, p, wrist_r, head_r, flange):
        forward = XR_TO_ROBOT @ head_r @ np.array([0., 0., -1.])
        if np.linalg.norm(forward[:2]) < .2:
            raise ValueError('对齐时请平视前方')
        basis = R.from_euler('z', -np.arctan2(forward[1], forward[0])).as_matrix() @ XR_TO_ROBOT
        self.anchor = (p.copy(), wrist_r.copy(), flange.copy(), basis)

    def target(self, p, wrist_r):
        if self.anchor is None:
            raise ValueError('请先对齐')
        p0, r0, flange0, basis = self.anchor
        delta = self.scale * basis @ (p-p0)
        if self.max_translation is not None and np.linalg.norm(delta) > self.max_translation:
            raise TargetRejected('超出配置的相对位置范围')
        delta_r = basis @ wrist_r @ r0.T @ basis.T
        if self.max_rotation is not None and R.from_matrix(delta_r).magnitude() > self.max_rotation:
            raise TargetRejected('超出配置的相对旋转范围')
        result_r = delta_r @ R.from_euler('xyz', flange0[3:]).as_matrix()
        return np.r_[flange0[:3]+delta, R.from_matrix(result_r).as_euler('xyz')]


class PreviewArm:
    hardware = False

    def __init__(self, q):
        self.q = vector(q, 6).copy()

    def read(self):
        return self.q.copy()

    def write(self, q):
        self.q = vector(q, 6).copy()

    def hold(self):
        pass


def can_receivers(interface, root=Path('/proc/net/can')):
    """EVA's startup check: also detect CAN owners outside our advisory lock."""
    if not root.is_dir():
        raise RuntimeError('无法检查 CAN 接收者：/proc/net/can 不存在')
    receivers = []
    for path in sorted(root.glob('rcvlist_*')):
        for line in path.read_text().splitlines():
            fields = line.split()
            if fields and fields[0] in (interface, 'any'):
                receivers.append(dict(interface=fields[0], table=path.name))
    return receivers


def hardware_preflight(interface='can1'):
    if interface != 'can1':
        raise ValueError('This deployment is configured for X5 can1')
    path = Path('/sys/class/net') / interface
    if not path.is_dir() or not (int((path/'flags').read_text(), 16) & 1):
        raise RuntimeError('can1 未启动，请先按 uarm_x5 的 CAN 配置接好适配器')
    # The reference shell holds this lock. Also catch a direct Python launch.
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            cmd = (proc/'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if any(Path(arg.decode(errors='replace')).name == 'uarm_x5_official_teleop.py' for arg in cmd):
            raise RuntimeError('U-Arm 主从仍在运行；先在原终端按其正常流程退出，不能并发控制 X5')
    with open('/tmp/uarm_x5_teleop.lock', 'a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError('X5 已被另一套主从/机械臂控制程序占用') from error
    if can_receivers(interface):
        raise RuntimeError('can1 已有 CAN 接收者（EVA、其他控制程序或监视器）；'
                           '请先正常退出占用程序，不能并发控制 X5')


class HardwareArm:
    hardware = True

    def __init__(self, kin, interface='can1', preview_time_s=.02):
        hardware_preflight(interface)
        self.lock = open('/tmp/uarm_x5_teleop.lock', 'a')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        import arx5_interface as sdk
        self.sdk, self.kin = sdk, kin
        cfg = sdk.ControllerConfigFactory.get_instance().get_config('joint_controller', 6)
        cfg.background_send_recv = True
        cfg.gravity_compensation = True
        cfg.shutdown_to_passive = True
        cfg.interpolation_method = 'linear'
        cfg.default_preview_time = preview_time_s
        cfg.over_current_cnt_max = 20
        cfg.default_gripper_kp = cfg.default_gripper_kd = 0.
        self.cfg = cfg
        self.controller = sdk.Arx5JointController(kin.config, cfg, interface)
        self.gains_enabled = False
        # Current-position command BEFORE restoring Kp. Never home on startup.
        self.write(self.read())
        self.controller.set_gain(sdk.Gain(cfg.default_kp.copy(), cfg.default_kd.copy(), 0., 0.))
        self.gains_enabled = True

    def read(self):
        state = self.controller.get_joint_state()
        q = vector(state.pos(), 6).copy()
        if np.allclose(q, 0) or np.any(q < self.kin.low-.1) or np.any(q > self.kin.high+.1):
            raise RuntimeError('Invalid X5 joint readback')
        # This is SDK-loop freshness, not a per-motor CAN receipt timestamp.
        stamp, now = float(state.timestamp), float(self.controller.get_timestamp())
        if not np.isfinite(stamp) or now-stamp > .2 or stamp > now+.02:
            raise RuntimeError('X5 feedback stale')
        if self.gains_enabled:
            # kp() is a non-owning SDK NumPy view. Keep Gain alive until copied;
            # controller.get_gain().kp() reads freed memory after the temporary dies.
            gain = self.controller.get_gain()
            kp = np.asarray(gain.kp()).copy()
            if not np.isfinite(kp).all() or np.any(kp <= 0):
                raise RuntimeError('X5 SDK 已进入阻尼/保护状态，停止跟随')
        return q

    def write(self, q):
        q = vector(q, 6)
        if np.any(q < self.kin.low) or np.any(q > self.kin.high):
            raise ValueError('Command exceeds joint limits')
        cmd = self.sdk.JointState(6)
        cmd.pos()[:] = q
        cmd.vel()[:] = cmd.torque()[:] = 0.
        cmd.gripper_pos = cmd.gripper_vel = cmd.gripper_torque = 0.
        self.controller.set_joint_cmd(cmd)

    def hold(self):
        # Keep position gains and the controller alive; stop at measured joints.
        # Do not leave an SDK interpolation target pulling towards the old pose.
        self.write(self.read())


class ArmTeleop:
    def __init__(self, kin, output, scale=1., hz=120., max_speed_deg_s=60., preview_time_s=.02):
        if not 1 <= hz <= 120 or not 0 < max_speed_deg_s <= 60:
            raise ValueError('Invalid arm control rate/speed')
        self.kin, self.output = kin, output
        self.mapping = RelativePose(scale, kin.policy['relative_translation_m'],
                                    kin.policy['relative_rotation_rad'])
        self.hz, self.speed = hz, np.deg2rad(max_speed_deg_s)
        self.preview_time_s = preview_time_s
        self.epoch, self.enabled, self.latest = 0, False, None
        self.last_sequence = -1
        self.session = None
        self.reason = '等待右手与 Head Pose，然后对齐启用机械臂'
        self.q = output.read()
        self.command = self.q.copy()
        self.command_velocity = np.zeros(6)
        self.target = self.kin.fk(self.q)
        self.last_send, self.ik_ms, self.sent = time.monotonic(), None, 0
        self.error = None
        self.blocked, self.rejected, self.last_rejection = False, 0, None
        self.last_ik_fraction = None
        self.ik_solution = None
        self.last_frame_sequence, self.pose_to_command_ms = None, None
        self.event = asyncio.Event()
        self.latency_samples = deque(maxlen=512)
        self.home = dict(active=False, state='idle', progress=0., max_speed_deg_s=60.)
        self._home_start = None

    @property
    def homing(self):
        return self.home['active']

    def start_home(self, now=None):
        """Finite joint-space zero move using the existing motor owner only."""
        if self.homing:
            return  # Double-clicks never restart the trajectory.
        self.pause('准备回零')
        q = self.output.read()
        if np.any(q < self.kin.low) or np.any(q > self.kin.high):
            raise ValueError('当前实测关节在范围外，不能自动回零')
        if np.any(self.kin.low > 0) or np.any(self.kin.high < 0):
            raise ValueError('零位不在关节范围内')
        self.output.write(q)  # Cancel any old interpolation before the finite move.
        self.q, self.command = q.copy(), q.copy()
        self.target = self.kin.fk(q)
        self._home_start = q.copy()
        # Four-second trapezoid, with 0.5 s acceleration/deceleration ramps.
        # Even the X5's 3.5 rad maximum excursion fits below 60 deg/s.
        duration, ramp = 4., .5
        peak_speed = float(np.max(np.abs(q)))/(duration-ramp)
        if peak_speed > self.speed+1e-9:
            raise ValueError('当前关节位置无法在 5 秒内按速度上限回零')
        started = time.monotonic() if now is None else now
        self.home = dict(active=True, state='running', progress=0., elapsed_s=0.,
                         started_at=started, duration_s=duration, ramp_s=ramp,
                         settle_s=.5, deadline_s=5., max_speed_deg_s=float(np.rad2deg(self.speed)),
                         planned_peak_speed_deg_s=float(np.rad2deg(peak_speed)),
                         max_abs_readback_deg=float(np.rad2deg(np.max(np.abs(q)))))
        self.error = None
        self.reason = '回零中，预计 5 秒内完成，可随时暂停'
        self.last_send = started
        self.event.set()

    def step_home(self, now=None):
        if not self.homing:
            return
        now = time.monotonic() if now is None else now
        if now-self.home['started_at'] > self.home['deadline_s']:
            raise ValueError('回零超过 5 秒，已停止；未确认到达零位')
        elapsed = now-self.last_send
        if elapsed < 1/self.hz-1e-9:
            return
        # Do not accelerate to catch up after a blocked event loop.
        dt = min(max(.02, 1/self.hz), elapsed)
        self.q = self.output.read()
        progress_s = self.home['elapsed_s']+dt
        duration, ramp = self.home['duration_s'], self.home['ramp_s']
        t = min(duration, progress_s)
        u = t/duration
        if t < ramp:
            blend = t*t/(2*ramp*(duration-ramp))
        elif t <= duration-ramp:
            blend = (t-ramp/2)/(duration-ramp)
        else:
            blend = 1-(duration-t)**2/(2*ramp*(duration-ramp))
        command = self._home_start*(1-blend)
        if np.max(np.abs(command-self.q)) > .2:
            raise ValueError('回零跟随误差超过 0.2 rad，请检查负载/关节状态')
        self.output.write(command)
        self.command, self.target, self.last_send = command, self.kin.fk(command), now
        self.sent += 1
        self.home.update(progress=u, elapsed_s=progress_s,
                         max_abs_readback_deg=float(np.rad2deg(np.max(np.abs(self.q)))))
        self.reason = f'回零中 {u*100:.0f}%，可随时暂停'
        if progress_s >= self.home['duration_s']+self.home['settle_s']:
            residual = self.home['max_abs_readback_deg']
            self.home.update(active=False, state='complete' if residual <= 1. else 'near_zero')
            # Keep commanding zero on completion. pause() would instead hold
            # measured load deflection and turn that residual into a new target.
            self.reason = (f'回零完成，实测最大偏差 {residual:.2f}°；继续遥操请重新对齐' if residual <= 1.
                           else f'已到零位附近，实测最大偏差 {residual:.2f}°，未达到 1°校验；保持零位目标')
            self.event.set()

    def pause(self, reason):
        self.epoch += 1
        self.command_velocity = np.zeros(6)
        active, self.enabled = self.enabled or self.homing, False
        if self.homing:
            self.home.update(active=False, state='cancelled')
        self.blocked = False
        self.mapping.anchor = None
        self.reason = reason
        self.event.set()
        if active:
            try:
                self.output.hold()
                self.q = self.output.read()
                self.command = self.q.copy()
            except Exception as error:
                self.error = 'Hold/readback failed: ' + str(error)

    def reject_target(self, work, error):
        """Hold once without losing the anchor or permitting a paused session."""
        frame = self.latest
        if frame is None:
            return False
        epoch, captured, sequence = (self.epoch, frame['captured'], frame['sequence']) if work is None else (work[0], work[1], work[4])
        if not self.enabled or epoch != self.epoch:
            return False  # A manual pause/new session invalidates in-flight IK.
        now = time.monotonic()
        if now-captured > STALE_S or now-frame['captured'] > STALE_S:
            self.pause('右手跟踪超时，请重新对齐')
            return False
        entering = not self.blocked
        if entering:
            # Cancel interpolation toward the old target. Preserve the relative
            # anchor: returning to reachable space resumes the SAME mapping.
            # Driver/readback errors propagate to the worker's hard-pause path.
            self.output.hold()
        self.q = self.output.read()  # Continue fault checks while a target is blocked.
        if entering:
            self.command = self.q.copy()
            self.command_velocity = np.zeros(6)
            self.last_send = now
        self.blocked = True
        self.last_ik_fraction = None
        self.reason = '目标暂不可达，保持位置；回到可达范围后自动继续：' + str(error)
        previous = self.last_rejection
        if previous is None or (previous['epoch'], previous['sequence']) != (epoch, sequence):
            self.rejected += 1
            self.last_rejection = dict(epoch=epoch, sequence=sequence, reason=str(error),
                captured=captured, rejected_at=now, latest_wrist_position=frame['p'].tolist(),
                target_xyz_rpy=None if work is None else work[2].tolist(),
                seed_rad=None if work is None else work[3].tolist())
        return entering

    def feed(self, data, now=None):
        if self.homing:
            # Homing is explicitly requested by the operator, independent of
            # Quest visibility. A stream cannot steer or restart this move.
            return
        now = time.monotonic() if now is None else now
        try:
            if data.get('side') != 'right':
                raise ValueError('X5 input must be RIGHT')
            captured = float(data['captured'])
            if not np.isfinite(captured) or not 0 <= now-captured <= STALE_S:
                raise ValueError('Stale/future wrist pose')
            p, r = vector(data['position'], 3), rotation(data['rotation'])
            session, sequence = str(data['session']), int(data['sequence'])
            if not session or len(session) > 100 or sequence < 0:
                raise ValueError('Invalid frame identity')
            if session != self.session:
                self.pause('新 HTS 会话，请重新对齐')
                self.session, self.last_sequence, self.latest = session, -1, None
            if sequence <= self.last_sequence:
                raise ValueError('Repeated wrist frame')
            self.last_sequence = sequence
            head_r, head_time = None, None
            if data.get('head_rotation') is not None:
                head_r, head_time = rotation(data['head_rotation']), float(data['head_captured'])
                if not np.isfinite(head_time) or not 0 <= now-head_time <= STALE_S:
                    head_r = None
            if self.enabled and self.latest is not None:
                old = self.latest
                if captured-old['captured'] > STALE_S:
                    self.pause('跟踪中断，请重新对齐')
                elif np.linalg.norm(p-old['p']) > .10 or angle(r, old['r']) > .60:
                    self.pause('检测到位姿跳变/重定位，请重新对齐')
            self.latest = dict(p=p.copy(), r=r.copy(), captured=captured,
                               head_r=head_r, head_time=head_time, sequence=sequence, received=now)
            self.event.set()
        except (ValueError, KeyError, TypeError, OverflowError):
            self.pause('无效或过期的 HTS 位姿，请重新对齐')
            raise

    def align(self, now=None):
        if self.homing:
            raise ValueError('正在回零；请先暂停或等待完成，再对齐')
        now = time.monotonic() if now is None else now
        frame = self.latest
        if frame is None or not 0 <= now-frame['captured'] <= STALE_S:
            raise ValueError('没有新鲜右手数据，请先 Start Streaming')
        if frame['head_r'] is None or not 0 <= now-frame['head_time'] <= STALE_S:
            raise ValueError('请在 HTS 开启 Head Pose，并保持右手可见')
        self.pause('正在对齐')
        self.q = self.output.read()
        self.command = self.q.copy()
        self.target = self.kin.fk(self.q)
        self.mapping.align(frame['p'], frame['r'], frame['head_r'], self.target)
        self.last_send = now
        self.enabled, self.error, self.reason = True, None, '机械臂跟随中'
        self.event.set()

    def prepare(self):
        if not self.enabled:
            return None
        frame = self.latest
        if frame is None or time.monotonic()-frame['captured'] > STALE_S:
            self.pause('右手跟踪超时，请重新对齐')
            return None
        target = self.mapping.target(frame['p'], frame['r'])
        return (self.epoch, frame['captured'], target, self.command.copy(), frame['sequence'],
                frame['received'], time.monotonic())

    def motion_context(self, work):
        # Use the timestamp recorded by prepare, not an unsent IK target. The
        # final send occurs later, so this duration is conservative for speed.
        dt = min(max(.02, 1/self.hz), max(1e-6, work[6]-self.last_send))
        previous = self.command_velocity.copy()
        if work[6]-self.last_send > .05:
            previous[:] = 0.  # No catch-up after a delayed event loop.
        return dict(dt=dt, max_velocity=self.speed, previous_velocity=previous)

    def finish(self, work, solution, *, motion=None):
        epoch, captured, target, seed, sequence, received, prepared = work
        now = time.monotonic()
        if not self.enabled or self.epoch != epoch:
            return
        if now-captured > STALE_S:
            self.pause('逆解结果已过期，请重新对齐')
            return
        q = vector(solution, 6)
        if np.any(q < self.kin.low) or np.any(q > self.kin.high):
            raise ValueError('IK output outside limits')
        self.q = self.output.read()
        # Actual elapsed time preserves 60 deg/s when native input is slower than
        # the control ceiling. Cap catch-up at 20 ms (33 ms in reference mode).
        dt = min(max(.02, 1/self.hz), max(0., now-self.last_send))
        delta = q-self.command
        if motion is not None:
            if not np.allclose(seed, self.command, atol=1e-10, rtol=0):
                return  # A command-dependent solution cannot be cached/reused.
            if np.max(np.abs(delta)) > self.speed*dt+1e-8:
                raise ValueError('Motion IK violated the command speed bound')
            command = q.copy()  # Already feasible: no post-IK direction scaling.
        elif self.kin.backend == 'unitree':
            # Preserve the optimizer's joint direction when applying the speed
            # cap, rather than saturating each joint independently.
            fraction = min(1., self.speed*dt/max(float(np.max(np.abs(delta))), 1e-15))
            command = self.command + delta*fraction
        else:
            command = self.command + np.clip(delta, -self.speed*dt, self.speed*dt)
        if np.max(np.abs(command-self.q)) > .2:
            raise ValueError('X5 跟随误差超过 0.2 rad，请检查负载/关节状态')
        self.output.write(command)
        self.command_velocity = (command-self.command)/max(dt, 1e-6)
        self.blocked = False
        self.reason = '机械臂跟随中'
        self.command, self.target, self.last_send = command, target, now
        if sequence != self.last_frame_sequence:
            self.latency_samples.append(dict(sequence=sequence, session=self.session,
                command_time=time.monotonic(), captured=captured, ik_ms=self.ik_ms,
                arm_received=received, prepared=prepared))
        self.last_frame_sequence = sequence
        self.pose_to_command_ms = (time.monotonic()-captured)*1000
        self.sent += 1

    def snapshot(self):
        return dict(source='hts-x5', version=1, side='right',
                    behavior_revision=BEHAVIOR_REVISION,
                    mode='hardware' if self.output.hardware else 'preview', interface='can1',
                    enabled=self.enabled, status=self.reason, error=self.error, sent=self.sent,
                    home=dict(self.home),
                    blocked=self.blocked, rejected=self.rejected, last_rejection=self.last_rejection,
                    ik_target_fraction=self.last_ik_fraction, target_policy=self.kin.policy,
                    ik_backend=self.kin.backend, ik_solution=self.ik_solution,
                    ik_options=None if self.kin.optimizer is None else self.kin.optimizer.options,
                    motion_constraints_in_solver=self.kin.backend == 'unitree',
                    command_velocity_rad_s=self.command_velocity.tolist(),
                    control_hz=self.hz, max_speed_deg_s=float(np.rad2deg(self.speed)),
                    preview_time_s=self.preview_time_s,
                    ik_prewarm=self.kin.prewarm_result,
                    last_frame_sequence=self.last_frame_sequence,
                    pose_to_command_ms=self.pose_to_command_ms,
                    position_scale=self.mapping.scale, reference='X5 link6 (flange)',
                    payload_verified=bool(self.kin.mount.get('payload_verified')),
                    tcp_verified=bool(self.kin.mount.get('tcp_verified')),
                    urdf=self.kin.config.urdf_path, q_rad=self.q.tolist(),
                    command_rad=self.command.tolist(), target_xyz_rpy=self.target.tolist(),
                    ik_ms=self.ik_ms, session=self.session,
                    age_ms=None if self.latest is None else (time.monotonic()-self.latest['captured'])*1000)
