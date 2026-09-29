import tempfile
import unittest
from pathlib import Path

import numpy as np

from dex_retargeting.polynomial_adaptor import PolynomialJointKinematicAdaptor


class Robot:
    dof_joint_names = ["active", "passive"]

    def get_joint_index(self, name):
        return self.dof_joint_names.index(name)


class PolynomialAdaptorTests(unittest.TestCase):
    def test_polynomial_and_chain_rule(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "coupling.xml"
            path.write_text('<mujoco><equality><joint joint1="passive" '
                            'joint2="active" polycoef="0 1 2 0 0"/>'
                            '</equality></mujoco>')
            adaptor = PolynomialJointKinematicAdaptor(Robot(), ["active"], path)
            q = adaptor.forward_qpos(np.array([0.3, 0.0]))
            np.testing.assert_allclose(q, [0.3, 0.48])
            jacobian = np.array([[2.0, 3.0], [4.0, 5.0]])
            np.testing.assert_allclose(adaptor.backward_jacobian(jacobian),
                                       [[2 + 3*2.2], [4 + 5*2.2]])

    def test_empty_equations_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "coupling.xml"
            path.write_text('<mujoco/>')
            with self.assertRaises(ValueError):
                PolynomialJointKinematicAdaptor(Robot(), ["active"], path)
