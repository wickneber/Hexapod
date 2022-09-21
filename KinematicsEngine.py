import Leg
import numpy as np
from numpy import cos, sin
from collections import namedtuple
from helpers import *
from HexapodContainers import *
from HexapodConstants import *


'''

This class is the main interface for calculating the inverse kinematics for the hexapod robot.

'''


class KinematicsEngine:

    def __init__(self):

        # Hexapod contstants
        self.leg_servo_ids = \
            [
                servo_ids(1, 3, 5), servo_ids(2, 4, 6), servo_ids(7, 9, 11),
                servo_ids(8, 10, 12), servo_ids(13, 15, 17), servo_ids(14, 16, 18)
            ]

        self.hexapod_body_offsets = \
            [
                body_offsets(-60, 120, 120), body_offsets(60, 120, 60), body_offsets(-60, -120, 240),
                body_offsets(60, -120, 300), body_offsets(-100, 0, 180), body_offsets(100, 0, 0)
            ]

        self.initial_feet_positions = []

        # containers for ik and gait calculations
        self.body_ik_calculations = [body_ik_values()] * NUMLEGS
        self.leg_ik_calculations = [leg_ik_angles()] * NUMLEGS
        self.leg_reference_foot_positions = [leg_foot_positions()] * NUMLEGS
        self.gait_calculations = [gait_values()] * NUMLEGS

        self.initial_feet_positions = calculate_initial_feet_positions()

    def update_gait(self, gait_calculations: [gait_values]) -> None:
        """
        Updates the internal gait values
        :param gait_calculations: The new gait calculations
        :return: None
        """

        self.gait_calculations = gait_calculations

    def body_ik(self, rotation_translation_vals: rotation_translation_values = rotation_translation_values())\
            -> [(float, float, float)]:
        """
        Calculates the body inverse kinematics given the rotation and translation
        :param gait_calculations: The gait calculations calculated by the gait engine
        :param rotation_translation_vals: The data structure containing all the rotation and translation values
        :return: A list of the leg positions after body calculations for each leg
        """
        rot_x = np.radians(rotation_translation_vals.x_rot)
        rot_y = np.radians(rotation_translation_vals.y_rot)
        rot_z = np.radians(rotation_translation_vals.z_rot)

        for leg_number in range(NUMLEGS):

            # first do rotation, then translation, then add gait positions and return value in body coordinate
            total_x = self.initial_feet_positions[leg_number].x + self.hexapod_body_offsets[leg_number].x_offset
            total_y = self.initial_feet_positions[leg_number].y + self.hexapod_body_offsets[leg_number].y_offset
            total_z = self.initial_feet_positions[leg_number].z

            # rotation
            body_ikx = total_x * cos(rot_y) * cos(rot_z) - total_y * cos(rot_y) * sin(rot_z) + total_z * sin(rot_y)

            body_iky = (-sin(rot_x) * sin(rot_y) * sin(rot_z) + cos(rot_x) * cos(rot_z)) * total_y + \
                       (sin(rot_x) * sin(rot_y) * cos(rot_z) + sin(rot_z) * cos(rot_x)) * total_x - \
                       total_z * sin(rot_x) * cos(rot_y)

            body_ikz = -total_x * cos(rot_x) * sin(rot_y) * cos(rot_z) + total_x * sin(rot_x) * sin(rot_z) + \
                       total_y * cos(rot_x) * sin(rot_y) * sin(rot_z) + total_y * sin(rot_x) * cos(rot_z) + \
                       total_z * cos(rot_x) * cos(rot_y)

            body_ikx -= total_x
            body_iky -= total_y
            body_ikz -= total_z

            # translation + gait
            body_ikx += self.initial_feet_positions[leg_number].x - \
                        rotation_translation_vals.x_trans + self.gait_calculations[leg_number].x_gait

            body_iky += self.initial_feet_positions[leg_number].y - \
                        rotation_translation_vals.y_trans + self.gait_calculations[leg_number].y_gait

            body_ikz += self.initial_feet_positions[leg_number].z - \
                        rotation_translation_vals.z_trans + self.gait_calculations[leg_number].z_gait

            self.body_ik_calculations[leg_number] = body_ik_values(body_ikx, body_iky, body_ikz)

        return self.body_ik_calculations

    def leg_ik(self) -> []:

        """
        Calculates the leg inverse kinematics
        :return: The calculated leg angles
        """

        for leg_number in range(NUMLEGS):

            x = self.leg_reference_foot_positions[leg_number].x_pos
            y = self.leg_reference_foot_positions[leg_number].y_pos
            z = self.leg_reference_foot_positions[leg_number].z_pos

            leg_length = np.sqrt(np.power(x, 2) + np.power(y, 2))
            l1 = leg_length - COXA
            r1 = np.sqrt(np.power(z, 2) + np.power(l1, 2))

            alpha = np.arccos(np.clip((np.power(TIBIA, 2) - np.power(FEMUR, 2) - np.power(r1, 2)) /
                                      (-2 * FEMUR * r1), -1.0, 1.0))

            gamma = np.arctan2(l1, z)

            beta = np.arccos(np.clip((np.power(r1, 2) - np.power(TIBIA, 2) - np.power(FEMUR, 2)) /
                                     (-2 * TIBIA * FEMUR), -1.0, 1.0))

            coxa_angle = float(np.degrees(np.arctan2(y, x)))
            femur_angle = 90 - np.degrees(alpha + gamma)
            tibia_angle = 90 - np.degrees(beta) + np.degrees(
                END_EFFECTOR_OFFSET)  # np.degrees(0.72)  # the .72 is the end effector offset from the tibia

            # flip the angle if the leg is on the right side of the hexapod
            if leg_number % 2 == 1:
                femur_angle *= -1
                tibia_angle *= -1

            self.leg_ik_calculations[leg_number] = leg_ik_angles(coxa_angle, femur_angle, tibia_angle)

        return self.leg_ik_calculations

    def do_ik(self, rotation_translation_vals=None) -> []:
        """
        Main ik loop, calculates the IK values, this calculates the ik for every leg at once.
        If single leg control is desired, the individual use of body_ik and leg_ik is recommended.
        :param rotation_translation_vals: The rotation and translation values taken from the user
        :return: The calculated leg angles after doing all the inverse kinematics
        """
        # calculate the body ik values in the body reference frame given the body rotation and translation
        if rotation_translation_vals is None:
            rotation_translation_vals = rotation_translation_values()
        self.body_ik(rotation_translation_vals)

        # then rotate the leg positions calculated by the body ik to the leg reference plane for leg ik calculations
        for leg_number in range(NUMLEGS):
            """            
            print('Do ik body: {}'.format(self.body_ik_calculations[leg_number]))
            print("rotation offset: {}".format(self.hexapod_body_offsets[leg_number].rotation_offset))
            print("Z rotated: {}".format(rotate_z(
                self.body_ik_calculations[leg_number].body_ikx,
                self.body_ik_calculations[leg_number].body_iky,
                self.body_ik_calculations[leg_number].body_ikz,
                -self.hexapod_body_offsets[leg_number].rotation_offset)
            ))
            """

            self.leg_reference_foot_positions[leg_number] = leg_foot_positions(

                *rotate_z(
                    self.body_ik_calculations[leg_number].body_ikx,
                    self.body_ik_calculations[leg_number].body_iky,
                    self.body_ik_calculations[leg_number].body_ikz,
                    -self.hexapod_body_offsets[leg_number].rotation_offset
                )

            )

        # leg reference foot positions contains the foot positions rotated from the body reference plane
        # to the leg reference plane

        # calculate the local foot inverse kinematics
        self.leg_ik()
        return self.leg_ik_calculations


if __name__ == "__main__":

    engine = KinematicsEngine()
    rot_translation = rotation_translation_values(0, 0, 0, 0, 0, 0)
    return_val = engine.do_ik(rotation_translation_vals=rot_translation)

    for val in return_val:
        print(val)
