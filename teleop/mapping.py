"""Quest WebXR -> palm-relative DexPilot and relative X5 end-effector targets.
Axis convention references Unitree TeleVuer; no stale-pose fallback is used.
"""
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from o10teleop.core import O10Retargeter,palm_coordinates

NAMES=['wrist']
for finger in ['thumb','index-finger','middle-finger','ring-finger','pinky-finger']:
    parts=['metacarpal','phalanx-proximal','phalanx-distal','tip'] if finger=='thumb' else ['phalanx-proximal','phalanx-intermediate','phalanx-distal','tip']
    NAMES.extend(finger+'-'+p for p in parts)
XR_TO_ROBOT=np.array([[0.,0.,-1.],[-1.,0.,0.],[0.,1.,0.]])

def rotation(q):
    q=np.asarray(q,float)
    if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>.05:raise ValueError('Invalid quaternion')
    return Rotation.from_quat(q).as_matrix()

def validate_frame(frame,side='right'):
    if not frame.get('valid') or frame.get('side')!=side:raise ValueError('Selected hand not tracked')
    p=np.asarray(frame['points'],float)
    if p.shape!=(21,3) or not np.isfinite(p).all():raise ValueError('Invalid 21-joint frame')
    lengths=np.linalg.norm(p[[5,9,17]]-p[0],axis=1)
    if np.any(lengths<.025) or np.any(lengths>.2):raise ValueError('Invalid hand geometry or units')
    wrist_r=rotation(frame['wrist_q']);head_r=rotation(frame['head_q'])
    return p,wrist_r,head_r

class RelativeMapping:
    def __init__(self,scale=.7):self.scale=scale;self.anchor=None
    def align(self,p,wrist_r,head_r,eef):
        forward=XR_TO_ROBOT@head_r@np.array([0.,0.,-1.])
        if np.linalg.norm(forward[:2])<.2:raise ValueError('Look approximately forward while aligning')
        yaw=np.arctan2(forward[1],forward[0])
        basis=Rotation.from_euler('z',-yaw).as_matrix()@XR_TO_ROBOT
        self.anchor=(p[0].copy(),wrist_r.copy(),np.asarray(eef).copy(),basis)
    def target(self,p,wrist_r):
        if self.anchor is None:raise ValueError('Align first')
        wrist0,r0,eef0,basis=self.anchor
        delta=basis@(p[0]-wrist0)*self.scale
        dr=basis@wrist_r@r0.T@basis.T
        if np.linalg.norm(delta)>.4:raise ValueError('Wrist outside relative workspace; realign')
        target_r=dr@Rotation.from_euler('xyz',eef0[3:]).as_matrix()
        return np.r_[eef0[:3]+delta,Rotation.from_matrix(target_r).as_euler('xyz')]

class X5Kinematics:
    def __init__(self):
        import arx5_interface as a
        self.config=a.RobotConfigFactory.get_instance().get_config('X5')
        c=self.config
        c.gripper_motor_type=a.MotorType.NONE
        c.eef_link_name='link6'
        c.urdf_path=str(__import__('pathlib').Path(a.__file__).parent/'models/X5_no_gripper.urdf')
        self.solver=a.Arx5Solver(c.urdf_path,c.joint_dof,c.joint_pos_min,c.joint_pos_max,c.base_link_name,c.eef_link_name,c.gravity_vector)
    def fk(self,q):return np.asarray(self.solver.forward_kinematics(q))
    def ik(self,target,seed):
        status,q=self.solver.inverse_kinematics(target,seed)
        q=np.asarray(q)
        if status!=0 or not np.isfinite(q).all():raise ValueError('IK: '+self.solver.get_ik_status_name(status))
        if np.any(q<self.config.joint_pos_min) or np.any(q>self.config.joint_pos_max):raise ValueError('IK exceeds joint limits')
        actual=self.fk(q)
        if np.linalg.norm(actual[:3]-target[:3])>.008:raise ValueError('IK position residual too large')
        angle=Rotation.from_matrix(Rotation.from_euler('xyz',actual[3:]).as_matrix()@Rotation.from_euler('xyz',target[3:]).as_matrix().T).magnitude()
        if angle>.08:raise ValueError('IK rotation residual too large')
        return q
