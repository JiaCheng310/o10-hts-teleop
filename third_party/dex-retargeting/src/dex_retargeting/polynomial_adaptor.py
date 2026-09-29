import xml.etree.ElementTree as ET

import numpy as np

from dex_retargeting.kinematics_adaptor import MimicJointKinematicAdaptor

class PolynomialJointKinematicAdaptor(MimicJointKinematicAdaptor):
    """Use the vendor MJCF coupling inside FK AND the optimizer Jacobian."""

    def __init__(self, robot, active_names, mjcf_path):
        equations = ET.parse(mjcf_path).findall("./equality/joint")
        if not equations:
            raise ValueError("MJCF must contain direct equality/joint couplings")
        self.coefficients = np.array([
            np.fromstring(e.attrib["polycoef"], sep=" ") for e in equations
        ])
        if self.coefficients.shape != (len(equations), 5) or not np.isfinite(self.coefficients).all():
            raise ValueError("Each coupling needs five finite polynomial coefficients")
        super().__init__(robot, active_names,
                         [e.attrib["joint2"] for e in equations],
                         [e.attrib["joint1"] for e in equations],
                         [1.0] * len(equations), [0.0] * len(equations))
        self.slopes = np.ones(len(equations))

    def forward_qpos(self, qpos):
        x = qpos[self.idx_pin2source]
        c = self.coefficients
        qpos[self.idx_pin2mimic] = np.sum(c * x[:, None] ** np.arange(5), axis=1)
        self.slopes = np.sum(c[:, 1:] * np.arange(1, 5) * x[:, None] ** np.arange(4), axis=1)
        return qpos

    def backward_jacobian(self, jacobian):
        result = jacobian[..., self.idx_pin2target].copy()
        for i, target in enumerate(self.idx_target2source):
            result[..., target] += jacobian[..., self.idx_pin2mimic[i]] * self.slopes[i]
        return result
