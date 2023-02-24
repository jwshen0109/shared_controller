# %%
from concurrent.futures.process import _process_worker
from typing_extensions import Self
from scipy.spatial.transform import Rotation as R
import numpy as np
# get marker pose in world coordinate
# cm
pos_world = np.array(
    [6.335306167602539, 126.66913604736328, -94.1477279663086])
rot_world = (0.0057526035234332085, -0.024932270869612694,
             0.006081796251237392, 0.9996541142463684)


class RotationU:
    def __init__(self, rot):
        self.rot = R.from_quat(rot)

    def apply(self, vec):
        return self.rot.apply(vec)

    def up(self):
        return self.rot.apply([0.0, 1.0, 0.0])

    def down(self):
        return self.rot.apply([0.0, -1.0, 0.0])

    def left(self):
        return self.rot.apply([1.0, 0.0, 0.0])

    def right(self):
        return self.rot.apply([-1.0, 0.0, 0.0])

    def forward(self):
        return self.rot.apply([0.0, 0.0, 1.0])

    def backward(self):
        return self.rot.apply([0.0, 0.0, -1.0])

    def transform_to(self, other: Self) -> Self:
        return RotationU((other.rot * self.rot.inv()).as_quat())


rot = RotationU(rot_world)


# %%
# calculate ee pose in world coordinate
# get direction of marker
flange = pos_world + rot.backward() * 7.5
print(flange)
ee_world = flange + rot.down() * 15.8
print(ee_world)
# get ee pose in franka coordinate
## unit: m
pos_franka = np.array(
    [0.43564125732280606, 0.07948217113629183, 0.3456035869545224])
rot_franka = RotationU(
    [0.89783186, - 0.43754856, - 0.03649256, 0.03342909])

# the rotation looks good
print(rot.down(), rot_franka.backward())

# Calculate Transform matrix between ee poses

trans_rot = rot.transform_to(rot_franka)
print(rot.apply([0.0, 1.0, 0.0]))
print(trans_rot.apply([0.0, 1.0, 0.0]))
print(rot_franka.apply([0.0, 1.0, 0.0]))

trans_pos = pos_franka * 100 - pos_world
print(trans_pos, pos_franka, pos_world)
# there is 0.01 error, is this ok?
# They are in the different coordinate

# get pose of flange for validation

# %%
# project ee_world to head coordinate
# we know the position of ee_world and two_eyes
# we also know the position of the center of eyes
# so we can calculate the distance between ee and center(x, y, z)
# we need to know the following things:
# 1. the position of center in CT
# 2. the ratio between real head and CT
# 3. the size of head in CT
# 4. the size of head in real
# 5. the size of bounding box in real

# then we have virtual position of ee
# finally we can query.
head_rot = (-0.0008391742594540119, 0.04098034277558327,
            -0.0026718301232904196, 0.9991559982299805)
head_markers = [(45.07046890258789, 90.56038665771484, -107.35630798339844),
                (41.192543029785156, 87.69705200195312, -103.25264739990234),
                (46.634925842285156, 84.35108947753906, -85.29367065429688),
                (49.353092193603516, 87.91131591796875, -103.81647491455078)]
head_rot = RotationU(head_rot)
# I should care about the position of flange.
head_pos = np.array(
    [45.276649475097656, 87.72486114501953, -103.41819763183594])
flange_in_head_pos = flange - head_pos
print(flange, head_pos, flange_in_head_pos)

flange_in_head_rot = head_rot.transform_to(rot)
print(rot.up())
print(head_rot.apply(flange_in_head_rot.up()))

# %%
# BBX in real
# XYZ
bbx = np.array([17.3, 17.0, 33.1]) / \
    np.array([241.0, 234.7, 260.8]) * np.array([510, 510, 263])
print(bbx)
# center in CT (-0.5, 0.5)
center = np.array([0.0, 355, (263 - 100.7)]) / \
    np.array([510, 510, 263]) - np.array([0, 0.5, 0.5])
print(center)
# flange wrt center
# flange_in_head_pos
# flange_relativ
relative_pos = flange_in_head_pos / bbx - center  # - np.array([0.5, 0.5, 0.5])
print(relative_pos)
# relative to the center of eyes
# print(flange_in_head_pos)

# %%
from scipy.spatial.transform import Rotation as R
rot = R.from_euler("xyz", [-90, -90, 0], degrees=True)

print(rot.apply([1, 0, 0]))  # [0, 1, 0] 001 X -> Z
print(rot.apply([0, 1, 0]))  # [0, 0, 1] 100 Y -> X
print(rot.apply([0, 0, 1]))  # [1, 0, 0] 010 Z -> Y

print(rot.inv().apply([1, 0, 0]))
print(rot.inv().apply([0, 1, 0]))
print(rot.inv().apply([0, 0, 1]))

# %%
