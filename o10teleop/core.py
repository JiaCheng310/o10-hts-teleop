from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from dex_retargeting.constants import RobotName, RetargetingType, HandType, get_default_config_path
from dex_retargeting.retargeting_config import RetargetingConfig

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "vendor/omnihand_description-omnihandT2_1/assets"
SUFFIXES = ["thumb_roll", "thumb_abad", "thumb_mcp", "index_abad", "index_pip",
            "middle_pip", "ring_abad", "ring_pip", "pinky_abad", "pinky_pip"]
FINGERS = ["thumb", "index", "middle", "ring", "pinky"]


def sdk_joint_names(hand):
    return [f"{'R' if hand == 'right' else 'L'}_{s}_joint" for s in SUFFIXES]



def prepare_urdf(hand):
    """Resolve ROS package:// mesh paths without editing the vendor originals."""
    tree = ET.parse(ASSETS / f"urdf/omnihand_{hand}.urdf")
    for mesh in tree.findall(".//mesh"):
        filename = mesh.attrib["filename"]
        if filename.startswith("package://omnihand_description/"):
            path = ASSETS / filename.removeprefix("package://omnihand_description/")
            if not path.is_file():
                raise FileNotFoundError(path)
            mesh.set("filename", str(path))
    output = ROOT / "output" / f"resolved_{hand}.urdf"
    output.parent.mkdir(exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return output


def palm_coordinates(points, hand):
    """21 metric MediaPipe WORLD landmarks -> fixed O10 palm axes.

    +z: wrist to fingers; +y: toward index for R, toward pinky for L;
    +x: cross(+y, +z). Use a mirrored camera frame for handedness detection.
    """
    p = np.asarray(points, dtype=float)
    if p.shape != (21, 3) or not np.isfinite(p).all():
        raise ValueError("Expected finite (21, 3) world landmarks in meters")
    z = p[9] - p[0]
    y = (p[5] - p[17]) * (1 if hand == "right" else -1)
    if np.linalg.norm(z) < 1e-5:
        raise ValueError("Degenerate palm length")
    z /= np.linalg.norm(z)
    y -= z * np.dot(y, z)
    if np.linalg.norm(y) < 1e-5:
        raise ValueError("Degenerate palm width")
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return (p - p[0]) @ np.column_stack([x, y, z])


class O10Retargeter:
    def __init__(self, hand="right", method="vector", scale=1.0, alpha=0.35, extension_assist=False, fist_assist=False):
        if hand not in ("right", "left") or method not in ("vector", "dexpilot"):
            raise ValueError("Invalid hand or method")
        if not np.isfinite(scale) or scale <= 0 or not 0 < alpha <= 1:
            raise ValueError("Scale must be positive; alpha must be in (0,1]")
        self.hand, self.alpha = hand, alpha
        self.extension_assist = extension_assist
        self.fist_assist = fist_assist
        self.fist_mode = False
        cfg = RetargetingConfig.load_from_file(
            get_default_config_path(RobotName.omnihand_o10, RetargetingType[method], HandType[hand]),
            override={"urdf_path": str(prepare_urdf(hand)), "scaling_factor": scale,
                      "polynomial_coupling_path": str(ASSETS / f"MJCF/omnihand_{hand}.xml")},
        )
        self.seq = cfg.build()
        self.robot = self.seq.optimizer.robot
        self.names = sdk_joint_names(hand)
        self.active_idx = np.array([self.seq.joint_names.index(n) for n in self.names])
        self.adaptor = self.seq.optimizer.adaptor
        self.limits = self.seq.joint_limits.copy()
        # Respect both URDF and MJCF active bounds (some differ by rounding).
        mj = ET.parse(ASSETS / f"MJCF/omnihand_{hand}.xml")
        for joint in mj.findall(".//worldbody//joint"):
            if joint.get("name") in self.names:
                i = self.names.index(joint.get("name"))
                lo, hi = np.fromstring(joint.get("range"), sep=" ")
                self.limits[i] = [max(lo, self.limits[i, 0]), min(hi, self.limits[i, 1])]
        # Coupled joints must also remain inside their URDF limits. The vendor
        # polynomials are monotone on these active ranges; verify before bisection.
        for k, i in enumerate(self.adaptor.idx_target2source):
            lo, hi = self.limits[i]
            c = self.adaptor.coefficients[k]
            f = lambda x: np.polynomial.polynomial.polyval(x, c)
            derivative = np.polynomial.polynomial.polyder(c)
            if np.min(np.polynomial.polynomial.polyval(np.linspace(lo, hi, 200), derivative)) <= 0:
                raise ValueError("Coupling is not monotone on joint range")
            qlo, qhi = self.robot.joint_limits[self.adaptor.idx_pin2mimic[k]]
            for bound, lower in [(qlo, True), (qhi, False)]:
                if (lower and f(lo) < bound) or (not lower and f(hi) > bound):
                    a, b = lo, hi
                    for _ in range(55):
                        mid = (a + b) / 2
                        if f(mid) < bound:
                            a = mid
                        else:
                            b = mid
                    if lower:
                        lo = b
                    else:
                        hi = a
            if lo >= hi:
                raise ValueError("No feasible active joint interval")
            self.limits[i] = [lo, hi]
        self.seq.joint_limits[:] = self.limits
        self.seq.optimizer.set_joint_limit(self.limits, epsilon=0)
        self.filtered = None
        self.prefix = "R" if hand == "right" else "L"
        self.tip_ids = [self.robot.get_link_index(f"{self.prefix}_{f}_tip") for f in FINGERS]

    def expand(self, active):
        q = np.zeros(self.robot.dof)
        q[self.active_idx] = active
        return self.adaptor.forward_qpos(q)

    def retarget(self, landmarks_in_palm):
        p = np.asarray(landmarks_in_palm, dtype=float)
        if p.shape != (21, 3) or not np.isfinite(p).all():
            raise ValueError("Expected finite (21,3) landmarks")
        indices = self.seq.optimizer.target_link_human_indices
        ref = p[indices[1]] - p[indices[0]]
        opt = self.seq.optimizer
        if self.fist_assist and hasattr(opt, "projected"):
            curls = finger_curls(p)
            self.fist_mode = bool(np.min(curls) > (55 if self.fist_mode else 80))
        thresholds = None
        if self.fist_mode:
            # A closed fist is not a simultaneous multi-finger precision pinch.
            thresholds = opt.project_dist, opt.escape_dist
            opt.projected[:] = False
            opt.project_dist = opt.escape_dist = -1.
        try:
            q = self.seq.retarget(ref)
        finally:
            if thresholds is not None:
                opt.project_dist, opt.escape_dist = thresholds
        active = np.clip(q[self.active_idx], *self.limits.T)
        if self.extension_assist:
            active[[4, 5, 7, 9]] *= 1 - extension_weights(p)
        self.filtered = active if self.filtered is None else self.alpha * active + (1-self.alpha) * self.filtered
        self.filtered = np.clip(self.filtered, *self.limits.T)
        # Filter only independent joints, then re-evaluate nonlinear coupling.
        return self.expand(self.filtered), self.filtered.copy()

    def reference_from_active(self, active):
        """Synthetic reachable target for an offline kinematics demo, not a camera test."""
        q = self.expand(active)
        self.robot.compute_forward_kinematics(q)
        p = np.zeros((21, 3))
        for idx, link in zip([4, 8, 12, 16, 20], self.tip_ids):
            p[idx] = self.robot.get_link_pose(link)[:3, 3]
        bases = [f"{self.prefix}_thumb_mcp_link"] + [f"{self.prefix}_{f}_pip_link" for f in FINGERS[1:]]
        for idx, name in zip([2, 5, 9, 13, 17], bases):
            p[idx] = self.robot.get_link_pose(self.robot.get_link_index(name))[:3, 3]
        return p


def extension_weights(points):
    """Optional O10 open-finger prior; do not override thumb opposition.

    Uses internal bone angles, not absolute fingertip position. Smoothly fades
    between 30 and 45 degrees; disabled for thumb-tip distances <= 60 mm.
    """
    p = np.asarray(points, dtype=float)
    weights = []
    for base in (5, 9, 13, 17):
        bones = np.diff(p[[0, base, base+1, base+2, base+3]], axis=0)
        lengths = np.linalg.norm(bones, axis=1)
        if np.min(lengths) < 1e-5:
            weights.append(0.)
            continue
        bones /= lengths[:, None]
        bends = np.degrees(np.arccos(np.clip(np.sum(bones[:-1]*bones[1:], axis=1), -1, 1)))
        straight = np.clip((45 - np.max(bends)) / 15, 0, 1)
        gap = np.linalg.norm(p[base+3] - p[4])
        away_from_pinch = np.clip((gap - .060) / .020, 0, 1)
        weights.append(float(straight * away_from_pinch))
    return np.array(weights)


def finger_curls(points):
    """PIP-to-tip bend angle for each long finger; degenerate data cannot enable fist mode."""
    p = np.asarray(points)
    curls = []
    for b in (5, 9, 13, 17):
        a, c = p[b+1]-p[b], p[b+3]-p[b+1]
        denom = np.linalg.norm(a)*np.linalg.norm(c)
        curls.append(0. if denom < 1e-8 else float(np.degrees(np.arccos(np.clip(a@c/denom, -1, 1)))))
    return np.array(curls)
