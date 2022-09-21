import sys
import time

import numpy as np
from numpy import cos, sin
from collections import namedtuple
from HexapodController import HexapodController
from Ax12a import AX12AController

"""
A mixed collection of testing functions aimed at figuring out the best way to construct the movement algorithm.
"""

# In mm
INIT_FEET_X = 160
INIT_FEET_Y = 0
INIT_FEET_Z = 80

STRIDE_HEIGHT = 20
STEP_SIZE     = 30

COXA  = 53
FEMUR = 66
TIBIA = 132

TRIPOD_GAIT_SEQUENCE = [1, 2, 1, 2, 2, 1]

# Both in ms
UPDATE_PERIOD = 20
STEP_DURATION = 300

# variable for step height calculation
STEP_CONST = .5
SCALE_FACTOR = 2*STEP_CONST

INITTED = True

body_offsets         = namedtuple("body_offsets",         ["x_offset", "y_offset", "rotation_offset"])
init_leg_position    = namedtuple("init_leg_position",    ["x", "y", "z"])
leg_ik_values        = namedtuple("leg_ik_values",        ["x", "y", "z"])
gait_values          = namedtuple("gait_values",          ["x", "y", "z"])
rotation_translation = namedtuple("rotation_translation", ["rot_x", "rot_y", "rot_z", "trans_x", "trans_y", "trans_z"])
hexapod_body         = namedtuple("hexapod_body",         ["body_offsets", "init_leg_pos"])
body_ik_calculations = namedtuple("body_ik_calculations", ["body_ikx", "body_iky", "body_ikz"])
leg_ik_calculations  = namedtuple("leg_ik_calculations",  ["coxa_angle", "femur_angle", "tibia_angle"])
leg_servo_values     = namedtuple("servo_values",         ["coxa_steps", "femur_steps", "tibia_steps"])
servo_ids            = namedtuple("servo_ids",            ["coxa_servo", "femur_servo", "tibia_servo"])
remap_range          = namedtuple("remap_range",          ["from_min", "from_max", "to_min", "to_max"])
gait_timing          = namedtuple("gait_timing",          ["tick", "gait_sequence", "step_size", "max_ticks"])
processed_js_info    = namedtuple("processed_js_info",    ["js1_mag", "js1_rot", "js2_mag", "js2_rot"])
current_foot_pos     = namedtuple("current_foot_pos",     ["x", "y", "z"])

current_leg_positions = [current_foot_pos(0, 0, 0)] * 6 # initial global foot positions


def remap_values(x: float, old_min: float, old_max: float, new_min: float, new_max: float) -> float:

    x = np.clip(x, old_min, old_max)
    old_range = old_max - old_min
    new_range = new_max - new_min
    scaled_value = new_range / old_range
    scaled_value *= (x - old_min)
    scaled_value += new_min
    scaled_value = np.clip(scaled_value, new_min, new_max)
    return float(scaled_value)


def rotate_z(x: float, y: float, z: float, rotation: float) -> (float, float, float):
    rotation = np.radians(rotation)

    new_x = x * np.cos(rotation) - y * np.sin(rotation)
    new_y = x * np.sin(rotation) + y * np.cos(rotation)

    return new_x, new_y, z


def body_ik(hexapod_body_vals: hexapod_body, rotation_translation_vals: rotation_translation, gait: gait_values) -> [(float, float, float)]:

    initial_leg_positions = hexapod_body_vals.init_leg_pos
    hexapod_body_offsets = hexapod_body_vals.body_offsets
    
    rot_x = np.radians(rotation_translation_vals.rot_x)
    rot_y = np.radians(rotation_translation_vals.rot_y)
    rot_z = np.radians(rotation_translation_vals.rot_z)

    body_ik = [body_ik_calculations(0, 0, 0)] * 6

    for leg_number in range(6):
        # first do rotation, then translation, then add gait positions and return value in body coordinate

        total_x = initial_leg_positions[leg_number].x + hexapod_body_offsets[leg_number].x_offset
        total_y = initial_leg_positions[leg_number].y + hexapod_body_offsets[leg_number].y_offset
        total_z = initial_leg_positions[leg_number].z

        # rotation
        body_ikx = total_x * cos(rot_y) * cos(rot_z) - total_y * cos(rot_y) * sin(rot_z) + total_z * sin(rot_y)
        
        body_iky = (-sin(rot_x) * sin(rot_y) * sin(rot_z) + cos(rot_x) * cos(rot_z)) * total_y + \
               (sin(rot_x) * sin(rot_y) * cos(rot_z) + sin(rot_z) * cos(rot_x)) * total_x - \
               total_z * sin(rot_x) * cos(rot_y)
        
        body_ikz = -total_x * cos(rot_x) * sin(rot_y) * cos(rot_z) + total_x * sin(rot_x) * sin(rot_z) + \
               total_y * cos(rot_x) * sin(rot_y) * sin(rot_z) + total_y * sin(rot_x) * cos(rot_z) + total_z * cos(rot_x) * cos(rot_y)

        body_ikx -= total_x
        body_iky -= total_y
        body_ikz -= total_z

        # translation + gait
        body_ikx += initial_leg_positions[leg_number].x - rotation_translation_vals.trans_x + gait[leg_number].x
        body_iky += initial_leg_positions[leg_number].y - rotation_translation_vals.trans_y + gait[leg_number].y
        body_ikz += initial_leg_positions[leg_number].z - rotation_translation_vals.trans_z + gait[leg_number].z

        body_ik[leg_number] = body_ik_calculations(body_ikx, body_iky, body_ikz)

    #print("BODY IK CALCULATIONS : {}".format(body_ik))
    return body_ik


def leg_ik(leg_positions: namedtuple) -> [namedtuple]:

    total_leg_calculations = [leg_ik_calculations(0, 0, 0)] * 6

    for leg_number in range(6):

        x = leg_positions[leg_number].x
        y = leg_positions[leg_number].y
        z = leg_positions[leg_number].z

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
        tibia_angle = 90 - np.degrees(beta) + np.degrees(.60)#np.degrees(0.72)  # the .72 is the end effector offset from the tibia

        # flip the angle if the leg is on the right side of the hexapod
        if leg_number % 2 == 1:
            femur_angle *= -1
            tibia_angle *= -1

        total_leg_calculations[leg_number] = leg_ik_calculations(coxa_angle, femur_angle, tibia_angle)

    return total_leg_calculations


def do_ik(hexapod_body: [body_offsets], rotation_translation_values: [rotation_translation], gait: [gait_values]) -> [leg_ik_calculations]:

    #print('\nHexapod body: {}\n rotation trans vals: {}\n gait: {}\n'.format(hexapod_body, rotation_translation_values, gait))

    body_ik_calculations = body_ik(hexapod_body, rotation_translation_values, gait)
    #print("BODY IK: {}".format(body_ik_calculations))
    hexapod_body_offsets = hexapod_body.body_offsets

    leg_values = [leg_ik_values(0, 0, 0)] * 6

    # rotate body calculations to leg reference frame
    for leg_number in range(6):
        leg_values[leg_number] = leg_ik_values(*rotate_z(*body_ik_calculations[leg_number],
                                                          -hexapod_body_offsets[leg_number].rotation_offset))

    print("DO IK CALC: {}".format(leg_values))

    return leg_ik(leg_values)


def remap_leg_calculations(leg_calculations: [leg_ik_calculations], new_remap_range: remap_range) -> [leg_ik_calculations]:

    remapped_calculations = [leg_ik_calculations(0, 0, 0)] * 6

    from_min = new_remap_range.from_min
    from_max = new_remap_range.from_max
    to_min   = new_remap_range.to_min
    to_max   = new_remap_range.to_max

    for leg_number in range(6):

        coxa_angle  = remap_values(leg_calculations[leg_number].coxa_angle,  from_min, from_max, to_min, to_max)
        femur_angle = remap_values(leg_calculations[leg_number].femur_angle, from_min, from_max, to_min, to_max)
        tibia_angle = remap_values(leg_calculations[leg_number].tibia_angle, from_min, from_max, to_min, to_max)

        remapped_calculations[leg_number] = leg_ik_calculations(coxa_angle, femur_angle, tibia_angle)

    return remapped_calculations


def convert_to_servo_angles(leg_angles: [leg_ik_calculations]) -> [leg_servo_values]:

    converted_servo_values = [leg_servo_values(0, 0, 0)] * 6
    for leg_number in range(6):
        coxa_steps  = int(leg_angles[leg_number].coxa_angle  / 0.293)
        femur_steps = int(leg_angles[leg_number].femur_angle / 0.293)
        tibia_steps = int(leg_angles[leg_number].tibia_angle / 0.293)
        converted_servo_values[leg_number] = leg_servo_values(coxa_steps, femur_steps, tibia_steps)

    return converted_servo_values


def construct_write_dict(leg_servos: [servo_ids], leg_servo_steps: [leg_servo_values]) -> {int: int}:

    write_dict = {}
    for leg_number in range(6):

        servo_numbers = leg_servos[leg_number]
        write_dict[servo_numbers[0]] = leg_servo_steps[leg_number].coxa_steps
        write_dict[servo_numbers[1]] = leg_servo_steps[leg_number].femur_steps
        write_dict[servo_numbers[2]] = leg_servo_steps[leg_number].tibia_steps

    return write_dict


def calculate_joystick_rotation(joystick_reading: [float, float]) -> float:
    return np.arctan2(joystick_reading[1], joystick_reading[0])


def process_joystick_information(js1_reading: [float, float], js2_reading: [float, float]) -> processed_js_info:

    js1_mag = np.linalg.norm(js1_reading)
    js1_rot = np.arctan2(js1_reading[1], js1_reading[0])

    js2_mag = np.linalg.norm(js2_reading)
    js2_rot = np.arctan2(js2_reading[1], js2_reading[0])

    return processed_js_info(js1_mag, js1_rot, js2_mag, js2_rot)

"""
def gait_generator(joystick_reading: processed_js_info, timing: gait_timing) -> [gait_values]:

    # if the most recent input is not substantial...
    if np.clip(max(joystick_reading.js1_mag, joystick_reading.js2_mag), 0, 1) < 0.15:

        # make the last step reset all the values back to the initial leg positions.
        pass

    # the most recent input is substantial...
    else:
        # continue on with the leg movements"""


def tripod_gait1(joystick_reading: processed_js_info, hexapod_constants: hexapod_body, timing: gait_timing):


    # just going to do one leg calculation for now
    calculated_gait = [gait_values(0, 0, 0)]*6
    step_size = timing.step_size

    # joystick 1 rotation is used for walking translation
    joystick1_rotation = joystick_reading.js1_rot

    # joystick 2 rotation is used for rotation of the entire body
    joystick2_rotation = joystick_reading.js2_rot + np.radians(np.pi)

    max_mag = np.clip(max(joystick_reading.js1_mag, joystick_reading.js2_mag), 0, 1)

    #print("Max mag: {}".format(max_mag))

    # we need to get the body rotation for the movement
    # so we can call body fk because its just that call but that means that we need to get the arguments needed to pass
    # to body fk. OOOORRRR we can just violate the stuff because this code will really not be resuable so that my life
    # can be a bit easier.

    rotated_value = rotate_z(step_size, 0, 0, float(np.degrees(joystick1_rotation)))

    tick_ratio = timing.tick/(timing.max_ticks-1)

    tick, max_ticks = timing.tick, timing.max_ticks-1

    new_gait_sequence = timing.gait_sequence
    interpolating = True

    print("Rotated value: {}".format(rotated_value))
    print("Tick ratio: {}/{}".format(timing.tick, timing.max_ticks-1))
    #print("cos(pi*({}/{})) = {}".format(timing.tick, timing.max_ticks-1, np.cos(np.pi*tick_ratio)))

    #rotated_value = np.array([0, STEP_SIZE, 0])

    for leg_num in range(6):

        total_x = hexapod_constants.body_offsets[leg_num].x_offset + hexapod_constants.init_leg_pos[leg_num].x
        total_y = hexapod_constants.body_offsets[leg_num].y_offset + hexapod_constants.init_leg_pos[leg_num].y

        body_rot_x = 0  # total_x * np.cos() + total_y * np.sin() - total_x
        body_rot_y = 0  # total_x * np.sin() + total_y * np.cos() - total_y

        # forward reach

        if not INITTED:
            #first time the hexapod has moved since being initialized
            pass

        if new_gait_sequence[leg_num] == 1:

            #x_swing = -ampx * np.cos(np.pi*tick_ratio)
            #y_swing = -ampy * np.cos(np.pi*tick_ratio)

            print("x cos: {}".format(np.cos(np.pi*tick_ratio)))

            #PROBLEM
            x_swing = -rotated_value[0] * np.cos(np.pi*tick_ratio) + body_rot_x
            y_swing = -rotated_value[1] * np.cos(np.pi*tick_ratio) + body_rot_y

            print(" x swing: {}\n y_swing: {}\n".format(x_swing, y_swing))

            #x_swing = -rotated_value[0] * np.pi*tick_ratio
            #y_swing = -rotated_value[1] * np.pi*tick_ratio

            # Half ellipse step trajectory
            z_swing = -1*((STRIDE_HEIGHT/STEP_CONST) * np.sqrt(abs(STEP_CONST**2 - (tick_ratio*SCALE_FACTOR - STEP_CONST)**2)))
            calculated_gait[leg_num] = gait_values(x_swing, y_swing, z_swing)

            #print("Calculated gait forward leg# {} reach : {}".format(leg_num, calculated_gait[leg_num]))

            if tick == max_ticks:
                new_gait_sequence[leg_num] = 2
                interpolating = False
                #print("1->2\n")
                print("=============================================================")

        # backward reach
        elif new_gait_sequence[leg_num] == 2:

            #x_swing = foot_pos[leg_num].x - 2*(rotated_value[0]) * tick_ratio
            #y_swing = foot_pos[leg_num].y - 2*(rotated_value[1]) * tick_ratio

            #x_swing = current_leg_positions[leg_num]
            #y_swing =

            x_swing = rotated_value[0] - 2 * rotated_value[0] * tick_ratio
            y_swing = rotated_value[1] - 2 * rotated_value[1] * tick_ratio
            z_swing = 0 # np.sin(np.pi*tick_ratio)

            calculated_gait[leg_num] = gait_values(x_swing, y_swing, z_swing)

            # if the step has been fully completed
            if tick == max_ticks:
                new_gait_sequence[leg_num] = 1
                interpolating = False
                print("=============================================================")
                #print("2->1\n")

    print(new_gait_sequence)
    print(calculated_gait)
    return calculated_gait, new_gait_sequence, interpolating


def tripod_gait(joystick_reading: processed_js_info, hexapod_constants: hexapod_body, timing: gait_timing):

    # just going to do one leg calculation for now
    calculated_gait = [gait_values(0, 0, 0)]*6
    step_size = timing.step_size

    # joystick 1 rotation is used for walking translation
    joystick1_rotation = joystick_reading.js1_rot

    # joystick 2 rotation is used for rotation of the entire body
    joystick2_rotation = joystick_reading.js2_rot + np.radians(np.pi)

    max_mag = np.clip(max(joystick_reading.js1_mag, joystick_reading.js2_mag), 0, 1)

    #print("Max mag: {}".format(max_mag))

    # we need to get the body rotation for the movement
    # so we can call body fk because its just that call but that means that we need to get the arguments needed to pass
    # to body fk. OOOORRRR we can just violate the stuff because this code will really not be resuable so that my life
    # can be a bit easier.

    rotated_value = rotate_z(step_size, 0, 0, float(np.degrees(joystick1_rotation)))

    tick_ratio = timing.tick/(timing.max_ticks-1)

    tick, max_ticks = timing.tick, timing.max_ticks-1

    new_gait_sequence = timing.gait_sequence
    interpolating = True

    print("Rotated value: {}".format(rotated_value))
    print("Tick ratio: {}/{}".format(timing.tick, timing.max_ticks-1))
    #print("cos(pi*({}/{})) = {}".format(timing.tick, timing.max_ticks-1, np.cos(np.pi*tick_ratio)))

    #rotated_value = np.array([0, STEP_SIZE, 0])

    for leg_num in range(6):

        total_x = hexapod_constants.body_offsets[leg_num].x_offset + hexapod_constants.init_leg_pos[leg_num].x
        total_y = hexapod_constants.body_offsets[leg_num].y_offset + hexapod_constants.init_leg_pos[leg_num].y

        body_rot_x = 0  # total_x * np.cos() + total_y * np.sin() - total_x
        body_rot_y = 0  # total_x * np.sin() + total_y * np.cos() - total_y

        # forward reach
        if new_gait_sequence[leg_num] == 1:

            #x_swing = -ampx * np.cos(np.pi*tick_ratio)
            #y_swing = -ampy * np.cos(np.pi*tick_ratio)

            print("x cos: {}".format(np.cos(np.pi*tick_ratio)))

            #PROBLEM
            x_swing = -rotated_value[0] * np.cos(np.pi*tick_ratio) + body_rot_x
            y_swing = -rotated_value[1] * np.cos(np.pi*tick_ratio) + body_rot_y

            print(" x swing: {}\n y_swing: {}\n".format(x_swing, y_swing))

            #x_swing = -rotated_value[0] * np.pi*tick_ratio
            #y_swing = -rotated_value[1] * np.pi*tick_ratio

            # Half ellipse step trajectory
            z_swing = -1*((STRIDE_HEIGHT/STEP_CONST) * np.sqrt(abs(STEP_CONST**2 - (tick_ratio*SCALE_FACTOR - STEP_CONST)**2)))
            calculated_gait[leg_num] = gait_values(x_swing, y_swing, z_swing)

            #print("Calculated gait forward leg# {} reach : {}".format(leg_num, calculated_gait[leg_num]))

            if tick == max_ticks:
                new_gait_sequence[leg_num] = 2
                interpolating = False
                #print("1->2\n")
                print("=============================================================")

        # backward reach
        elif new_gait_sequence[leg_num] == 2:

            #x_swing = foot_pos[leg_num].x - 2*(rotated_value[0]) * tick_ratio
            #y_swing = foot_pos[leg_num].y - 2*(rotated_value[1]) * tick_ratio

            #x_swing = current_leg_positions[leg_num]
            #y_swing =

            x_swing = rotated_value[0] - 2 * rotated_value[0] * tick_ratio
            y_swing = rotated_value[1] - 2 * rotated_value[1] * tick_ratio
            z_swing = 0 # np.sin(np.pi*tick_ratio)

            calculated_gait[leg_num] = gait_values(x_swing, y_swing, z_swing)

            # if the step has been fully completed
            if tick == max_ticks:
                new_gait_sequence[leg_num] = 1
                interpolating = False
                print("=============================================================")
                #print("2->1\n")

    print(new_gait_sequence)
    print(calculated_gait)
    return calculated_gait, new_gait_sequence, interpolating


def time_to_update(current_time: float, previous_time: float) -> bool:

    return (current_time - previous_time)*1000 > UPDATE_PERIOD


def update_gait_sequence(timing: gait_timing, new_gait_sequence: [int]) -> gait_timing:

    new_tick = timing.tick+1
    new_tick = new_tick % timing.max_ticks

    return gait_timing(new_tick, new_gait_sequence, timing.step_size, timing.max_ticks)


def initialize_feet_positions() -> [init_leg_position]:

    initial_positions = []

    #for rotation_angle in [135, 45, 225, 315, 180, 0]:
    #    initial_positions.append(init_leg_position(*rotate_z(INIT_FEET_X, INIT_FEET_Y, INIT_FEET_Z, rotation_angle)))

    for rotation_angle in [120, 60, 240, 300, 180, 0]:
       initial_positions.append(init_leg_position(*rotate_z(INIT_FEET_X, INIT_FEET_Y, INIT_FEET_Z, rotation_angle)))

    return initial_positions


def update_max_ticks(timing: gait_timing, new_max_ticks: float):

    new_timing = gait_timing(timing.tick, timing.gait_sequence, timing.step_size, new_max_ticks)
    return new_timing


def test_ik():

    servo_controller = AX12AController("Com3")
    controller_state = {"x": 0}
    leg_servo_ids = \
        [
            servo_ids(1, 3, 5), servo_ids(2, 4, 6), servo_ids(7, 9, 11),
            servo_ids(8, 10, 12), servo_ids(13, 15, 17), servo_ids(14, 16, 18)
        ]

    hexapod_body_offsets = \
        [
            body_offsets(-60, 120, 120), body_offsets(60, 120, 60), body_offsets(-60, -120, 240),
            body_offsets(60, -120, 300), body_offsets(-100, 0, 180), body_offsets(100, 0, 0)
        ]

    heaxpod_init_feet_pos = initialize_feet_positions()

    hexapod_body_values = hexapod_body(hexapod_body_offsets, heaxpod_init_feet_pos)
    hexapod_gait = [gait_values(0, 0, 0)] * 6
    gamepad = HexapodController()

    leg_ik_remap_range = remap_range(-90, 90, 0, 300)

    rotation_mode = False

    prev_time = 0

    while controller_state['x'] == 0:

        if time_to_update(time.perf_counter(), prev_time):

            controller_state.update(gamepad.get_controller_state())

            joystick_info = process_joystick_information(controller_state['joystick1'],
                                                             controller_state['joystick2'])

            joystick1_magnitude = joystick_info.js1_mag
            joystick2_magnitude = joystick_info.js2_mag

            if controller_state['circle'] == 1:
                rotation_mode = not rotation_mode
                time.sleep(0.3)

            elif rotation_mode:
                rotx = controller_state['joystick2'][0]
                roty = controller_state['joystick2'][1]
                rotz = controller_state['joystick1'][0]

                rotx = remap_values(rotx, -1.0, 1.0, -15, 15)
                roty = remap_values(roty, -1.0, 1.0, -15, 215)
                rotz = remap_values(rotz, -1.0, 1.0, -20, 20)

                rotation_trans_values = rotation_translation(rotx, roty, rotz, 0, 0, 0)

            else:
                trans_x = controller_state['joystick1'][0]
                trans_y = controller_state['joystick1'][1]
                trans_z = controller_state['joystick2'][1]

                trans_x = remap_values(trans_x, -1.0, 1.0, -40, 40)
                trans_y = remap_values(trans_y, -1.0, 1.0, -40, 40)
                trans_z = remap_values(trans_z, -1.0, 1.0, -40, 40)

                rotation_trans_values = rotation_translation(0, 0, 0, trans_x, -trans_y, trans_z)

                leg_ik_angles = do_ik(hexapod_body_values, rotation_trans_values, hexapod_gait)

                # remap the leg angles from -90, 90 to 0 300
                remapped_leg_angles = remap_leg_calculations(leg_ik_angles, leg_ik_remap_range)

                # convert values to servo steps
                servo_steps = convert_to_servo_angles(remapped_leg_angles)

                print(servo_steps)

                write_dict = construct_write_dict(leg_servo_ids, servo_steps)
                # time.sleep(1)
                servo_controller.sync_write_goal_position(write_dict)

            prev_time = time.perf_counter()


def test_gait():
    servo_controller = AX12AController("Com3")
    controller_state = {"x": 0}

    leg_servo_ids = \
        [
            servo_ids(1, 3, 5), servo_ids(2, 4, 6), servo_ids(7, 9, 11),
            servo_ids(8, 10, 12), servo_ids(13, 15, 17), servo_ids(14, 16, 18)
        ]

    hexapod_body_offsets = \
        [
            body_offsets(-60, 120, 120), body_offsets(60, 120, 60), body_offsets(-60, -120, 240),
            body_offsets(60, -120, 300), body_offsets(-100, 0, 180), body_offsets(100, 0, 0)
        ]

    hexapod_init_feet_pos = initialize_feet_positions()

    hexapod_body_values = hexapod_body(hexapod_body_offsets, hexapod_init_feet_pos)
    hexapod_gait = [gait_values(0, 0, 0)] * 6
    scanned = servo_controller.scan(18)

    print(servo_controller.get_voltage())
    gamepad = HexapodController()

    # initial gait sequence for the hexapod
    # 1 = forward reach, 2 = backwards push, 3 = initial leg position
    gait_sequence = TRIPOD_GAIT_SEQUENCE

    gait_mode = True
    rotation_mode = False

    leg_ik_remap_range = remap_range(-90, 90, 0, 300)
    prev_time = time.perf_counter()

    curr_foot_pos = [current_foot_pos(0, 0, 0)] * 6

    interpolating = False

    timing = gait_timing(0, gait_sequence, STEP_SIZE, int(STEP_DURATION / UPDATE_PERIOD))

    joystick1_magnitude = 0
    joystick2_magnitude = 0

    max_step = 0

    while controller_state['x'] == 0:

        if time_to_update(time.perf_counter(), prev_time):

            controller_state.update(gamepad.get_controller_state())

            if gait_sequence == TRIPOD_GAIT_SEQUENCE and not interpolating:
                joystick_info = process_joystick_information(controller_state['joystick1'],
                                                             controller_state['joystick2'])

                joystick1_magnitude = joystick_info.js1_mag
                joystick2_magnitude = joystick_info.js2_mag

            max_mag = np.clip(max(joystick1_magnitude, joystick2_magnitude), 0, 1)

            if max_mag > 0.15:

                timing = update_max_ticks(timing,
                                          int((np.clip(STEP_DURATION + STEP_DURATION * (1 - max_mag),
                                                       100, STEP_DURATION * 2) / UPDATE_PERIOD)))

                if controller_state['circle'] == 1:
                    rotation_mode = not rotation_mode
                    time.sleep(0.3)

                if gait_mode:
                    hexapod_gait, gait_sequence, interpolating = tripod_gait(joystick_info, hexapod_body_values, timing)
                    rotation_trans_values = rotation_translation(0, 0, 0, 0, 0, 0)
                    #print("Hexapod gait: {}".format(hexapod_gait))

                    # need to add a delay after each step
                    prev_time = time.perf_counter()
                    while interpolating == False:
                        # lets wait for some time
                        if (time.perf_counter() - prev_time) * 1000 > 100:
                            # reset the previous time to account for the delay
                            prev_time = time.perf_counter()
                            break

                elif rotation_mode:

                    rotx = controller_state['joystick2'][0]
                    roty = controller_state['joystick2'][1]
                    rotz = controller_state['joystick1'][0]

                    rotx = remap_values(rotx, -1.0, 1.0, -15, 15)
                    roty = remap_values(roty, -1.0, 1.0, -15, 15)
                    rotz = remap_values(rotz, -1.0, 1.0, -20, 20)

                    rotation_trans_values = rotation_translation(rotx, roty, rotz, 0, 0, 0)

                else:
                    trans_x = controller_state['joystick1'][0]
                    trans_y = controller_state['joystick1'][1]
                    trans_z = controller_state['joystick2'][1]

                    trans_x = remap_values(trans_x, -1.0, 1.0, -40, 40)
                    trans_y = remap_values(trans_y, -1.0, 1.0, -40, 40)
                    trans_z = remap_values(trans_z, -1.0, 1.0, -40, 40)

                    rotation_trans_values = rotation_translation(0, 0, 0, trans_x, -trans_y, trans_z)

                leg_ik_angles = do_ik(hexapod_body_values, rotation_trans_values, hexapod_gait)
                print("Ik angles: {}".format(leg_ik_angles))
                #print("Hexapod gait: {}".format(hexapod_gait))

                # remap the leg angles from -90, 90 to 0 300
                remapped_leg_angles = remap_leg_calculations(leg_ik_angles, leg_ik_remap_range)

                # convert values to servo steps
                servo_steps = convert_to_servo_angles(remapped_leg_angles)

                print(servo_steps)

                write_dict = construct_write_dict(leg_servo_ids, servo_steps)
                # time.sleep(1)
                servo_controller.sync_write_goal_position(write_dict)

                timing = update_gait_sequence(timing, gait_sequence)

            # if timing.tick == timing.max_ticks-1:
            #    break

            # print("Timing: {}".format(timing))
            prev_time = time.perf_counter()

    # servo_controller.set_torque(scanned, servo_controller.TORQUE_OFF)


if __name__ == "__main__":
    calculated_gait = [gait_values(0, 0, 0)]*6
    leg_servo_ids = \
        [
            servo_ids(1, 3, 5), servo_ids(2, 4, 6), servo_ids(7, 9, 11),
            servo_ids(8, 10, 12), servo_ids(13, 15, 17), servo_ids(14, 16, 18)
        ]

    hexapod_body_offsets = \
        [
            body_offsets(-60, 120, 120), body_offsets(60, 120, 60), body_offsets(-60, -120, 240),
            body_offsets(60, -120, 300), body_offsets(-100, 0, 180), body_offsets(100, 0, 0)
        ]

    hexapod_init_feet_pos = initialize_feet_positions()

    hexapod_body_values = hexapod_body(hexapod_body_offsets, hexapod_init_feet_pos)
    rot_trans = rotation_translation(0, 0, 0, 0, 0, 0)
    do_ik(hexapod_body_values, rot_trans, calculated_gait)
    #test_ik()