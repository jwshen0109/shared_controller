"""
Transformer: transform coordinate between world coordinate and franka coordinate.

unit is cm;

World Coordinate:
X: left right
Y: up down
Z: forward backward
"""
from optitrack import OptiTrackClient
from scipy.spatial.transform import Rotation as R
import numpy as np
from typing_extensions import Self
# world_to_franka
RT = R.from_euler("xyz", [-90, -90, 0], degrees=True)
# franka_to_world
RT2 = RT.inv()


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


class Transformer:
    def __init__(self, pos_franka=None, rot_franka=None) -> None:
        self.client = OptiTrackClient()
        # pos and rot of ee in franka coordinate
        self.pos_franka = pos_franka
        self.rot_franka = RotationU(rot_franka)
        # get marker pose in world coordinate
        # robot marker
        self.pos1, rot1, self.markers1 = self.client.get_pose(0)
        print(f"pos1: {self.pos1}")
        # head marker (2)left_eye (4)right_eye
        self.pos2, rot2, self.markers2 = self.client.get_pose(1)
        self.rot1 = RotationU(rot1)
        self.rot2 = RotationU(rot2)
        # get the position of flange
        self.flange = self.pos1 + self.rot1.backward() * 7.5 + self.rot1.down() * 0.14
        self.ee = self.pos1 + self.rot1.backward() * 7.5 + self.rot1.up() * 21.8
        print(f"ee_world: {self.ee}")
        print(f"flange_world: {self.flange}")
        # get direction of marker

        # base in world
        # Translation, this should be related to rotation as well.
        # world: [XYZ], franka: [Y, Z, X]
        # franka: [XYZ], world: [ZXY]
        self.base_world_pos = self.ee - \
            np.array([self.pos_franka[1], self.pos_franka[2],
                     self.pos_franka[0]]) * 100 * np.array([1, 1, 1])
        # print(f"base_world: {self.base_world_pos}")
        # get the rotation of base in world
        # franka_rot is the end
        # rot1 is the rot of ee in world
        # RT * self.rot_franka.rot.inv() * RT.inv() * self.rot1.rot
        # original one
        # self.base_world_rot = self.rot_franka.rot.inv() * self.rot1.rot
        # Rotation
        # Base_R * Local_R(known) = World_R(known)
        # get base
        # Base_R = World_R * Local_R.inv()
        # world to franka
        # Local_R = Base_R.inv() * World_R
        # franka to world
        # World_R = Base_R * Local_R
        self.base_world_rot = self.rot1.rot * self.rot_franka.rot.inv()
        # self.base_world_rot = RT.inv() * self.rot_franka.rot.inv() * RT * self.rot1.rot
        # print(f"base_rot: {self.base_world_rot.as_euler('xyz', degrees=True)}")
        # print(
        #     f"franka_rot: {self.rot_franka.rot.as_euler('xyz', degrees=True)}")

        # base in world

        # self.franka_to_world_rot = self.rot1.transform_to(self.rot_franka)
        # self.franka_to_world_pos = self.ee - self.pos_franka * 100

        # self.world_to_franka_rot = self.rot_franka.transform_to(self.rot1)
        # self.world_to_franka_pos = self.pos_franka * 100 - self.ee

        # get ee pose in franka coordinate

        # Calculate Transform matrix between ee poses

        # get head marker pos

        # project ee to head coordinate

        # visualize these markers.

    def set_franka_coords(self, pos_franka, rot_franka):
        self.pos_franka = pos_franka
        self.rot_franka = RotationU(rot_franka)

    def get_ee_in_world(self):
        return self.ee, self.rot1.rot.as_quat()

    def get_flange_in_franka(self):
        return self.world_to_franka(self.flange, self.rot1.rot.as_quat())

    def franka_to_world(self, pos, rot):
        """Transform pos and rot in franka coordinate to world coordinate

        Args:
            pos: float3 pos in franka coordinate
            rot: float4 rot(quat) in franka coordinate
        Returns:
            new_pos: float3 pos in world coordinate
            new_rot: float4 rot(quat) in world coordinate
        """
        # pos from franka use m as unit, bnt the unit is cm in world
        new_pos = np.array(
            [self.base_world_pos[2], self.base_world_pos[0], self.base_world_pos[1]])
        new_pos = new_pos + pos * 100
        new_pos = np.array([new_pos[1], new_pos[2], new_pos[0]])
        new_rot = self.base_world_rot * R.from_quat(rot)
        return new_pos, new_rot.as_quat()

    def world_to_franka(self, pos, rot):
        """Transform pos and rot in world coordinate to franka coordinate
        WORKING

        Args:
            pos: float3 pos in world coordinate
            rot: float4 rot(quat) in world coordinate
        Returns:
            new_pos: float3 pos in franka coordinate
            new_rot: float4 rot(quat) in franka coordinate
        """
        new_pos = pos - self.base_world_pos
        new_pos = np.array([new_pos[2], new_pos[0], new_pos[1]]) / 100
        # new_pos = (self.world_to_franka_pos + pos) / 100
        new_rot = self.base_world_rot.inv() * R.from_quat(rot)
        return new_pos, new_rot.as_quat()

    def world_to_franka_points(self, poses):
        res = []
        for pos in poses:
            # print(f"marker: {pos}, base: {self.base_world_pos}")
            new_pos = pos - self.base_world_pos
            new_pos = np.array([new_pos[2], new_pos[0], new_pos[1]]) / 100
            # new_pos = new_pos / 100
            res.append(new_pos)
        return res

    def world_to_head(self, sampling_count=10):
        """Transform pos and rot in world coordinate to head local coordinate

        Args:
            pos: float3 pos in world coordinate
            rot: float4 rot(quat) in world coordinate
            sampling_count: int sampling points count
        Returns:
            new_pos: float3 pos in head local coordinate
            new_rot: float4 rot(quat) in head local coordinate

        This transform the ee_world into head, from end back to flange1
        """

        pos_in_head = self.ee - self.pos2
        rot_in_head = self.rot2.rot.inv() * self.rot1.rot
        step = 21.8 / sampling_count
        points = np.array(
            [pos_in_head + step * i * rot_in_head.apply([0.0, -1.0, 0.0]) for i in range(sampling_count + 1)])
        print(points)
        # return points
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
        relative_pos = points / bbx - \
            center  # - np.array([0.5, 0.5, 0.5])
        print(relative_pos)
        return relative_pos

    def head_to_relative(self, pos, rot):
        """Transform pos and rot in world coordinate to head local coordinate

        Args: 
            pos: float3 pos in world coordinate
            rot: float4 rot(quat) in world coordinate
        Returns:
            new_pos: float3 pos in head local coordinate
            new_rot: float4 rot(quat) in head local coordinate
        """
        pass


# Should I do validation of the work?
