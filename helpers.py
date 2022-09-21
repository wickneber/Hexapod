import time

import numpy as np
from HexapodContainers import *
from HexapodConstants import *


def remap_values(x: float, old_min: float, old_max: float, new_min: float, new_max: float) -> float:
    """
    Remaps a given value to a new range
    :param x: The value to remap
    :param old_min: The old minimum
    :param old_max: The old maximum
    :param new_min: The new minimum
    :param new_max: The new maximum
    :return: The remapped value
    """
    x = np.clip(x, old_min, old_max)
    old_range = old_max - old_min
    new_range = new_max - new_min
    scaled_value = new_range / old_range
    scaled_value *= (x - old_min)
    scaled_value += new_min
    scaled_value = np.clip(scaled_value, new_min, new_max)
    return float(scaled_value)


def remap_leg_angles(ik_angles: leg_ik_angles, old_min: float, old_max: float, new_min: float, new_max: float) \
        -> leg_ik_angles:
    """
    Given leg ik angles, remap them to a new range.
    :param ik_angles: The ik angles to remap
    :return: The newly remapped ik angles
    """

    remapped_coxa = remap_values(ik_angles.coxa_angle, old_min, old_max, new_min, new_max)
    remapped_femur = remap_values(ik_angles.femur_angle, old_min, old_max, new_min, new_max)
    remapped_tibia = remap_values(ik_angles.tibia_angle, old_min, old_max, new_min, new_max)

    return leg_ik_angles(remapped_coxa, remapped_femur, remapped_tibia)


def batch_remap_leg_angles(ik_angles: [leg_ik_angles], old_min: float, old_max: float, new_min: float, new_max: float) -> [leg_ik_angles]:
    """
    Remaps the given list of ik angles and returns the newly remapped values
    :param ik_angles: The list of ik angles to remap
    :param old_min: The old min of the previous range
    :param old_max: The old max of the previous range
    :param new_min: The new min of the new range
    :param new_max: The new max of the new range
    :return: A list of leg ik angles
    """

    to_return = []

    for ik_calculation in ik_angles:
        to_return.append(remap_leg_angles(ik_calculation, old_min, old_max, new_min, new_max))

    return to_return


def rotate_z(x: float, y: float, z:float, angle_of_rotation: float) -> (float, float, float):
    """
    Given a value that is a scalar, rotate the value around th z axis at a degree of angle of rotation
    :param x: The x value to rotate
    :param y: The y value to rotate
    :param z: The z value to rotate
    :param angle_of_rotation: The angle of rotation in degrees
    :return: The rotated result
    """
    rotation = np.radians(angle_of_rotation)

    new_x = x * np.cos(rotation) - y * np.sin(rotation)
    new_y = x * np.sin(rotation) + y * np.cos(rotation)

    return new_x, new_y, z


def process_joystick_information(js1_reading: [float, float], js2_reading: [float, float]) \
        -> (float, float, float, float):

    """
    Computes the rotation and magnitude of the given joystick readings
    :param js1_reading: First joystick's reading
    :param js2_reading: Second joystick's reading
    :return: A namedtuple containing the processed joystick information
    """

    js1_mag = np.linalg.norm(js1_reading)
    js1_rot = np.arctan2(js1_reading[1], js1_reading[0])

    js2_mag = np.linalg.norm(js2_reading)
    js2_rot = np.arctan2(js2_reading[1], js2_reading[0])

    return processed_js_info(js1_mag, js1_rot, js2_mag, js2_rot)


def convert_to_servo_angles(leg_angles: leg_ik_angles) -> servo_values:
    """
    Given the calculated ik angles to convert to, convert the degree offsets to actual servo values
    :param leg_angles: The calculated ik angles to convert
    :return: The converted servo values
    """
    coxa_steps  = int(leg_angles.coxa_angle  / DEGREES_PER_STEPS)
    femur_steps = int(leg_angles.femur_angle / DEGREES_PER_STEPS)
    tibia_steps = int(leg_angles.tibia_angle / DEGREES_PER_STEPS)
    return servo_values(coxa_steps, femur_steps, tibia_steps)


def batch_convert_to_servo_angles(legs_to_convert: [leg_ik_angles]) -> [servo_values]:
    """
    Converts a list of leg ik angles to servo values
    :param legs_to_convert: The list of ik angles to convert
    :return: A list of servo values
    """
    converted_servo_values = []
    for ik_angle in legs_to_convert:
        converted_servo_values.append(convert_to_servo_angles(ik_angle))

    return converted_servo_values


def construct_write_dict(leg_servo_steps: [servo_values]) -> {int: int}:
    """
    Given the calculated servo steps, map the values to each servo and return a dict containing all the information
    :param leg_servo_steps: The calculated servo steps
    :return: A dictionary containing all the mapped values
    """

    write_dict = {}
    for leg_number in range(NUMLEGS):
        servo_numbers = SERVO_MAPPING[leg_number]
        write_dict[servo_numbers.coxa_servo] = leg_servo_steps[leg_number].coxa_steps
        write_dict[servo_numbers.femur_servo] = leg_servo_steps[leg_number].femur_steps
        write_dict[servo_numbers.tibia_servo] = leg_servo_steps[leg_number].tibia_steps
    return write_dict

'''
def construct_write_dict(leg_servos: [servo_ids], leg_servo_steps: [leg_servo_values]) -> {int: int}:

    write_dict = {}
    for leg_number in range(6):

        servo_numbers = leg_servos[leg_number]
        write_dict[servo_numbers[0]] = leg_servo_steps[leg_number].coxa_steps
        write_dict[servo_numbers[1]] = leg_servo_steps[leg_number].femur_steps
        write_dict[servo_numbers[2]] = leg_servo_steps[leg_number].tibia_steps

    return write_dict

def construct_write_dict(calculated_servo_values: [servo_values]) ->
    write_dict = {}
    for leg_number in range(NUMLEGS):
        servo_numbers = leg_servos[leg_number]
        write_dict[servo_numbers[0]] = leg_servo_steps[leg_number].coxa_steps
        write_dict[servo_numbers[1]] = leg_servo_steps[leg_number].femur_steps
        write_dict[servo_numbers[2]] = leg_servo_steps[leg_number].tibia_steps

    return write_dict
'''


def is_substantial_input(user_input: processed_js_info) -> bool:
    """
    Checks to see if the input from the controller is not zero
    :param user_input: The user input to check
    :return: A boolean value determining whether no not the controller input has any input that is non zero
    """

    return not user_input.js1_mag == user_input.js1_rot == user_input.js2_mag == user_input.js2_rot == 0.0


def calculate_initial_feet_positions() -> [initial_feet_positions]:

    '''
    Rotate the initial feet positions to the global body reference frame.
    :return: A list of initial leg positions for each leg
    '''
    initial_leg_positions = []
    for rotation_angle in [120, 60, 240, 300, 180, 0]:
        initial_leg_positions.append(
            initial_feet_positions(*rotate_z(INIT_FEET_X, INIT_FEET_Y, INIT_FEET_Z, rotation_angle))
        )

    return initial_leg_positions


def get_time_ms() -> float:
    """
    Gets the current time since epoch in milliseconds
    :return: The current time in milliseconds
    """
    return time.perf_counter_ns() // 1_000_000

