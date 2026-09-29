"""Unitree-style constrained pose/continuity IK, adapted to the six-joint X5.

Uses Pinocchio analytical Jacobians and SciPy TRF instead of CasADi/IPOPT.
The pose-only solver guides distant goals. The live solver optimizes one
command step with joint, speed and acceleration bounds, a rotation-log error,
command continuity and outward-motion penalties near stops. No input moving
average and no bias toward the folded joint-zero posture.
Only model operations occur here; this module never opens a motor driver.
"""
import json
from pathlib import Path
import time
import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

DEFAULTS = dict(position_weight=50., rotation_weight=1., continuity_weight=1.,
                max_nfev=24, budget_ms=6., joint_step_rad=.45,
                position_tolerance_m=.0002, rotation_tolerance_rad=.001,
                motion_continuity_weight=.3, acceleration_weight=.5,
                max_acceleration_deg_s2=1200., limit_margin_rad=.15,
                limit_avoidance_weight=.2, motion_position_weight=200.,
                guidance_position_m=.03, guidance_rotation_rad=.2,
                guidance_step_weight=20.)

class BudgetReached(Exception):
    pass

class UnitreeIK:
    def __init__(self, urdf, low, high, config_path=None):
        self.options = dict(DEFAULTS)
        if config_path and Path(config_path).exists():
            overrides = json.loads(Path(config_path).read_text())
            if not isinstance(overrides, dict) or set(overrides)-set(DEFAULTS):
                raise ValueError('Unknown unitree_ik.json parameters')
            self.options.update(overrides)
        for key, value in self.options.items():
            if not isinstance(value, (float, int)) or not np.isfinite(value) or value <= 0:
                raise ValueError('Invalid IK option: '+key)
        if int(self.options['max_nfev']) != self.options['max_nfev'] or self.options['max_nfev'] > 100:
            raise ValueError('max_nfev must be an integer <= 100')
        self.low, self.high = np.array(low), np.array(high)
        self.model = pin.buildModelFromUrdf(str(urdf))
        if self.model.nq != 6 or self.model.nv != 6 or list(self.model.names)[1:] != [f'joint{i}' for i in range(1,7)]:
            raise ValueError('Expected X5 six-revolute-joint model')
        self.frame = self.model.getFrameId('link6')
        if self.frame >= self.model.nframes:
            raise ValueError('X5 link6 missing')
        self.data = self.model.createData()
        self._cached_q = None
        self.weights = np.sqrt([self.options['position_weight']]*3+[self.options['rotation_weight']]*3)
        self.continuity = np.sqrt(self.options['continuity_weight'])

    def geometry(self, q):
        if self._cached_q is None or not np.array_equal(q, self._cached_q):
            pin.computeJointJacobians(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            tf = self.data.oMf[self.frame]
            self._p, self._r = tf.translation.copy(), tf.rotation.copy()
            self._j = pin.getFrameJacobian(self.model, self.data, self.frame, pin.LOCAL_WORLD_ALIGNED).copy()
            self._cached_q = q.copy()
        return self._p, self._r, self._j

    def pose_error_jacobian(self, q, target_position, target_rotation):
        p, r, jac = self.geometry(q)
        error_rotation = r @ target_rotation.T
        error = np.r_[p-target_position, pin.log3(error_rotation)]
        derivative = np.vstack([jac[:3], pin.Jlog3(error_rotation.T) @ jac[3:]])
        return error, derivative

    def verify_model(self, sdk_fk):
        rng = np.random.default_rng(21)
        maximum_position, maximum_rotation = 0., 0.
        for q in [np.zeros(6), *rng.uniform(self.low, self.high, (12,6))]:
            actual = sdk_fk(q)
            p, r, _ = self.geometry(q)
            maximum_position = max(maximum_position, float(np.linalg.norm(p-actual[:3])))
            maximum_rotation = max(maximum_rotation, float(Rotation.from_matrix(r @ Rotation.from_euler('xyz', actual[3:]).as_matrix().T).magnitude()))
        if maximum_position > 1e-6 or maximum_rotation > 1e-6:
            raise ValueError('Pinocchio model differs from SDK FK')
        return dict(position_m=maximum_position, rotation_rad=maximum_rotation, samples=13)

    def solve(self, target, seed, *, motion=None):
        if motion is not None:
            return self._solve_motion(target, seed, motion)
        return self._solve_pose(target, seed)

    def _solve_motion(self, target, seed, motion):
        """One feasible command step, with joint/speed/acceleration constraints.

        The continuity reference is the last SENT command and its measured send
        interval, never an unsent distant IK solution. No input moving average.
        Near a stop, discrete braking bounds avoid arriving too fast to stop on
        the next tick. An inconsistent incoming velocity relaxes acceleration
        only; joint and speed bounds remain hard and the event is reported.
        """
        started = time.perf_counter()
        target, seed = np.asarray(target, float), np.asarray(seed, float)
        dt = float(motion['dt'])
        vmax = np.broadcast_to(np.asarray(motion['max_velocity'], float), (6,)).copy()
        previous = np.asarray(motion['previous_velocity'], float)
        if (target.shape != (6,) or seed.shape != (6,) or previous.shape != (6,)
                or not all(np.isfinite(x).all() for x in (target, seed, previous, vmax))
                or not np.isfinite(dt) or not 0 < dt <= .05 or np.any(vmax <= 0)):
            raise ValueError('Invalid motion-constrained IK input')
        if np.any(seed < self.low-1e-6) or np.any(seed > self.high+1e-6):
            raise ValueError('IK seed outside actual SDK joint limits')
        seed = np.clip(seed, self.low, self.high)
        amax = np.deg2rad(self.options['max_acceleration_deg_s2'])

        def brake_speed(distance):
            # v*dt + v^2/(2*a) <= distance (conservative discrete stop).
            return np.maximum(0., np.sqrt((amax*dt)**2+2*amax*distance)-amax*dt)

        lo_v = -np.minimum(vmax, brake_speed(seed-self.low))
        hi_v = np.minimum(vmax, brake_speed(self.high-seed))
        limited_lo = np.maximum(lo_v, previous-amax*dt)
        limited_hi = np.minimum(hi_v, previous+amax*dt)
        conflict = limited_lo > limited_hi
        limited_lo = np.where(conflict, lo_v, limited_lo)
        limited_hi = np.where(conflict, hi_v, limited_hi)
        lower = np.maximum(self.low, seed+limited_lo*dt)
        upper = np.minimum(self.high, seed+limited_hi*dt)
        # Normal operation keeps a positive box width, including at a joint stop.
        if np.any(upper-lower <= 1e-14):
            raise ValueError('Empty motion-constrained IK box')
        predicted = np.clip(seed+previous*dt, lower, upper)
        target_rotation = Rotation.from_euler('xyz', target[3:]).as_matrix()
        initial_error, _ = self.pose_error_jacobian(seed, target[:3], target_rotation)
        initial_pose_cost = float(np.sum((initial_error*self.weights)**2))
        if (np.linalg.norm(initial_error[:3]) <= self.options['position_tolerance_m']
                and np.linalg.norm(initial_error[3:]) <= self.options['rotation_tolerance_rad']
                and np.max(np.abs(previous)) < min(1e-4, amax*dt)):
            q, fraction, meta = self._solve_pose(target, seed)
            meta.update(implementation='pinocchio-scipy-trf-motion', velocity_constrained=True,
                motion_dt_s=dt, max_velocity_rad_s=float(np.max(vmax)),
                velocity_rad_s=np.zeros(6).tolist(), finite_pose_guidance=False,
                braking_override_joints=[], max_acceleration_deg_s2=self.options['max_acceleration_deg_s2'])
            return q, fraction, meta
        control_position, control_rotation = target[:3], target_rotation
        joint_guide = seed
        guided = (np.linalg.norm(initial_error[:3]) > self.options['guidance_position_m']
                  or np.linalg.norm(initial_error[3:]) > self.options['guidance_rotation_rad'])
        if guided:
            # Keep the existing finite-pose search for distant/unreachable goals:
            # a purely infinitesimal pose descent can get stuck at a singular
            # extension. Its output guides a SECOND, motion-constrained solve;
            # it is never sent or clipped into a motor command directly.
            guide, _, _ = self._solve_pose(target, seed)
            delta = guide-seed
            fraction = min(1., float(np.min(vmax))*dt/max(float(np.max(np.abs(delta))), 1e-15))
            joint_guide = seed+delta*fraction
            control_position, control_rotation, _ = self.geometry(joint_guide)
        motion_weights = np.sqrt([self.options['motion_position_weight']]*3+
                                 [self.options['rotation_weight']]*3)
        time_scale = (1/120.)/dt
        continuity = np.sqrt(self.options['motion_continuity_weight'])*time_scale
        acceleration = np.sqrt(self.options['acceleration_weight'])*time_scale**2
        margin = self.options['limit_margin_rad']
        proximity_lo = np.clip(1-(seed-self.low)/margin, 0., 1.)
        proximity_hi = np.clip(1-(self.high-seed)/margin, 0., 1.)
        limit_weight = np.sqrt(self.options['limit_avoidance_weight'])
        guide_weight = np.sqrt(self.options['guidance_step_weight']) if guided else 0.
        deadline = started+self.options['budget_ms']/1000
        evaluations, exceeded = 0, False

        def evaluate(q):
            error, jac = self.pose_error_jacobian(q, control_position, control_rotation)
            delta = q-seed
            residual = np.r_[motion_weights*error, continuity*delta,
                acceleration*(delta-previous*dt),
                guide_weight*(q-joint_guide),
                limit_weight*proximity_lo*np.minimum(delta, 0.),
                limit_weight*proximity_hi*np.maximum(delta, 0.)]
            derivative = np.vstack([motion_weights[:, None]*jac, continuity*np.eye(6),
                acceleration*np.eye(6), guide_weight*np.eye(6),
                np.diag(limit_weight*proximity_lo*(delta < 0)),
                np.diag(limit_weight*proximity_hi*(delta > 0))])
            return residual, derivative

        # This fallback is itself feasible, including when seed is outside the
        # acceleration box. Never return an infeasible zero-speed stop on timeout.
        best_q = np.clip(seed, lower, upper)
        best_cost = float(np.sum(evaluate(best_q)[0]**2))

        def residual(q):
            nonlocal evaluations, best_q, best_cost
            if time.perf_counter() > deadline:
                raise BudgetReached()
            value, _ = evaluate(q)
            evaluations += 1
            cost = float(value @ value)
            if cost < best_cost:
                best_q, best_cost = q.copy(), cost
            return value

        def jacobian(q):
            if time.perf_counter() > deadline:
                raise BudgetReached()
            return evaluate(q)[1]

        try:
            result = least_squares(residual, predicted, jac=jacobian, bounds=(lower, upper),
                method='trf', max_nfev=int(self.options['max_nfev']),
                ftol=1e-6, xtol=1e-9, gtol=1e-9)
            result_status = str(result.status)
        except BudgetReached:
            exceeded, result_status = True, 'time_budget'
        q = np.clip(best_q, lower, upper)
        error, _ = self.pose_error_jacobian(q, target[:3], target_rotation)
        velocity = (q-seed)/dt
        position_error, rotation_error = np.linalg.norm(error[:3]), np.linalg.norm(error[3:])
        reached = (position_error <= self.options['position_tolerance_m']
                   and rotation_error <= self.options['rotation_tolerance_rad'])
        moving = np.max(np.abs(velocity)) > 1e-4
        status = 'reached' if reached else ('approaching' if moving else 'limited')
        metadata = dict(backend='unitree-constrained', implementation='pinocchio-scipy-trf-motion',
            status=status, goal_reached=bool(reached), position_error_mm=float(position_error*1000),
            rotation_error_deg=float(np.rad2deg(rotation_error)), evaluations=evaluations,
            solve_ms=(time.perf_counter()-started)*1000, budget_exhausted=exceeded,
            optimizer_status=result_status, cost_before=initial_pose_cost,
            cost_after=float(np.sum((error*self.weights)**2)),
            max_joint_step_rad=float(np.max(np.abs(q-seed))), input_filter=False,
            velocity_constrained=True, motion_dt_s=dt,
            finite_pose_guidance=bool(guided),
            max_velocity_rad_s=float(np.max(vmax)),
            velocity_rad_s=velocity.tolist(),
            max_acceleration_deg_s2=self.options['max_acceleration_deg_s2'],
            braking_override_joints=(np.flatnonzero(conflict)+1).tolist(),
            min_limit_margin_deg=float(np.rad2deg(np.min(np.minimum(q-self.low, self.high-q)))))
        return q, (.5 if moving else 1.), metadata

    def _solve_pose(self, target, seed):
        target, seed = np.asarray(target, float), np.asarray(seed, float)
        if target.shape != (6,) or seed.shape != (6,) or not np.isfinite(target).all() or not np.isfinite(seed).all():
            raise ValueError('Invalid IK target/seed')
        if np.any(seed < self.low-1e-6) or np.any(seed > self.high+1e-6):
            raise ValueError('IK seed outside actual SDK joint limits')
        seed = np.clip(seed, self.low, self.high)
        target_rotation = Rotation.from_euler('xyz', target[3:]).as_matrix()
        initial_error, _ = self.pose_error_jacobian(seed, target[:3], target_rotation)
        initial_cost = float(np.sum((initial_error*self.weights)**2))
        best_q, best_cost = seed.copy(), initial_cost
        evaluations = 0
        started = time.perf_counter()
        deadline = started+self.options['budget_ms']/1000
        exceeded = False
        def residual(q):
            nonlocal best_q, best_cost, evaluations
            if time.perf_counter() > deadline:
                raise BudgetReached()
            error, _ = self.pose_error_jacobian(q, target[:3], target_rotation)
            result = np.r_[self.weights*error, self.continuity*(q-seed)]
            cost = float(result @ result)
            evaluations += 1
            if cost < best_cost:
                best_q, best_cost = q.copy(), cost
            return result
        def jacobian(q):
            if time.perf_counter() > deadline:
                raise BudgetReached()
            _, jac = self.pose_error_jacobian(q, target[:3], target_rotation)
            return np.vstack([self.weights[:,None]*jac, self.continuity*np.eye(6)])
        step = self.options['joint_step_rad']
        lower, upper = np.maximum(self.low, seed-step), np.minimum(self.high, seed+step)
        result_status = 'within_tolerance'
        if (np.linalg.norm(initial_error[:3]) > self.options['position_tolerance_m']
                or np.linalg.norm(initial_error[3:]) > self.options['rotation_tolerance_rad']):
            try:
                result = least_squares(residual, seed, jac=jacobian, bounds=(lower, upper),
                    method='trf', max_nfev=int(self.options['max_nfev']),
                    ftol=1e-6, xtol=1e-7, gtol=1e-7)
                result_status = str(result.status)
            except BudgetReached:
                exceeded, result_status = True, 'time_budget'
        q = np.clip(best_q, lower, upper)
        error, _ = self.pose_error_jacobian(q, target[:3], target_rotation)
        cost = float(np.sum((error*self.weights)**2))
        if not np.isfinite(q).all() or cost > initial_cost+1e-10:
            raise ValueError('Constrained IK produced invalid/non-improving output')
        position_error, rotation_error = float(np.linalg.norm(error[:3])), float(np.linalg.norm(error[3:]))
        reached = (position_error <= self.options['position_tolerance_m']
                   and rotation_error <= self.options['rotation_tolerance_rad'])
        improvement = initial_cost-cost
        moving = np.max(np.abs(q-seed)) > 1e-5 and improvement > max(1e-10, initial_cost*1e-7)
        status = 'reached' if reached else ('approaching' if moving else 'limited')
        metadata = dict(backend='unitree-constrained', implementation='pinocchio-scipy-trf',
            status=status, goal_reached=bool(reached), position_error_mm=position_error*1000,
            rotation_error_deg=float(np.rad2deg(rotation_error)), evaluations=evaluations,
            solve_ms=(time.perf_counter()-started)*1000, budget_exhausted=exceeded,
            optimizer_status=result_status, cost_before=initial_cost, cost_after=cost,
            max_joint_step_rad=float(np.max(np.abs(q-seed))), input_filter=False)
        # A terminal best approximation is cached, without snapping the motor
        # target back to measured joints on every rejection/acceptance boundary.
        return q, (.5 if moving and not reached else 1.), metadata
