
"""

Some data containers to make my life easier

"""

from dataclasses import dataclass
from collections import namedtuple

servo_ids = namedtuple("servo_ids", ["coxa_servo", "femur_servo", "tibia_servo"])
body_offsets = namedtuple("body_offsets", ["x_offset", "y_offset", "rotation_offset"])
processed_js_info = namedtuple("processed_js_info",    ["js1_mag", "js1_rot", "js2_mag", "js2_rot"])


@dataclass
class body_ik_values:
    body_ikx: float = 0.0
    body_iky: float = 0.0
    body_ikz: float = 0.0

    def __str__(self):
        return "{} {} {}".format(self.body_ikx, self.body_iky, self.body_ikz)


@dataclass
class leg_ik_angles:
    coxa_angle: float=0.0
    femur_angle: float=0.0
    tibia_angle: float=0.0

    def __str__(self):
        return "{} {} {}".format(self.coxa_angle, self.femur_angle, self.tibia_angle)


@dataclass
class gait_values:
    x_gait: float=0.0
    y_gait: float=0.0
    z_gait: float=0.0

    def __str__(self):
        return "{} {} {}".format(self.x_gait, self.y_gait, self.z_gait)


# These classes are identical but they store information that are of different reference points
# leg foot positions is the foot position in the leg reference frame meaning the information processed
# is local to each foot. The body foot positions is the foot position in the global body reference plane
# as if someone is looking down on the hexapod from the top.
@dataclass
class leg_foot_positions:

    x_pos: float=0.0
    y_pos: float=0.0
    z_pos: float=0.0

    def __str__(self):
        return "{} {} {}".format(self.x_pos, self.y_pos, self.z_pos)


@dataclass
class body_foot_positions:

    x_pos: float=0.0
    y_pos: float=0.0
    z_pos: float=0.0

    def __str__(self):
        return "{} {} {}".format(self.x_pos, self.y_pos, self.z_pos)


@dataclass
class rotation_translation_values:

    x_rot: float = 0.0
    y_rot: float = 0.0
    z_rot: float = 0.0
    x_trans: float = 0.0
    y_trans: float = 0.0
    z_trans: float = 0.0

    def __str__(self):
        return "{} {} {} {} {} {}".format(self.x_rot, self.y_rot, self.z_rot, self.x_trans, self.y_trans, self.z_trans)


@dataclass
class initial_feet_positions:
    x: float=0.0
    y: float=0.0
    z: float=0.0

    def __str__(self):
        return "{} {} {}".format(self.x, self.y, self.z)


@dataclass
class servo_values:
    coxa_steps: float=0.0
    femur_steps:float=0.0
    tibia_steps:float=0.0

    def __str__(self):
        return "{} {} {}".format(self.coxa_steps, self.femur_steps, self.tibia_steps)