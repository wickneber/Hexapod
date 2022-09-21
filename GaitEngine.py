'''
This is the main gait engine where all the gait values are calculated. This interfaces with the kinematics engine
to get actual leg angles. Gait engine and Kinematics Engine will be used by a controller class which serves
as the main interface into controlling the heaxpod.

                -------------
                |   USER    |
                -------------
                      ^
                      |
                      v
                -----------------
                |   Controller  |
                -----------------
                    /     ^
                   /       \
                  v         \
             ------------    ------------------
            |Gait engine| ->|Kinematics engine|
            -------------   -------------------


'''

import numpy as np
from HexapodContainers import *
from HexapodConstants import *
from helpers import is_substantial_input, rotate_z, processed_js_info, calculate_initial_feet_positions, get_time_ms


class GaitEngine:

    def __init__(self):

        self.init = True  # assumes that the hexapod is in the ready position and is ready for gait calculations.
        self.current_gait_state = [gait_values()] * NUMLEGS  # just the internal most recent gait calculation.
        self.moving_legs = [0] * NUMLEGS  # shows which legs are moving and which ones are not
        self.tick = 1  # used to determine how far the engine is to completing one movement
        self.max_ticks = MAX_TICKS  # the number of ticks it takes to perform a full cycle
        self.gait_sequence = TRIPOD_GAIT_SEQUENCE # gait sequence high level movement engine
        # get the robot initial leg positions, used for calculating the body rotation during movement
        self.initial_feet_positions = calculate_initial_feet_positions()

        # used to measure when it is time to increment tick and move forward
        self.prev_time = get_time_ms()

        self.interpolating = False
        self.rotated_step = None

    def re_center_hexapod(self) -> None:
        """
        Re centers the hexapod feet
        :return: None
        """

        for leg in range(NUMLEGS):
            # check to see if the current leg is already within the bounding box of init, if it isnt,
            # take the necessary steps to move the leg back a half stride length back to the initial
            # foot position.
            within_bounding_box = self.check_leg_drift(leg)

            # reach init
            if self.gait_sequence[leg] == REACH and within_bounding_box:
                pass

            # pull init
            if self.gait_sequence[leg] == PULL and within_bounding_box:
                pass

        self.init = True

    def tripod_gait(self, user_input: processed_js_info) -> None:

        """
        Basic tripod gait
        :param user_input: Input taken from the user to control the direction that the hexapod should move.
        :return: None
        """

        # check to see if the user has given input
        new_input = is_substantial_input(user_input)
        print('User input : {}'.format(user_input))
        print(is_substantial_input(user_input))
        # if the system is not interpolating and there is new input, then we accept the new joystick information
        if not self.interpolating and new_input:
            self.rotated_step = rotate_z(STEP_SIZE, 0, 0, user_input.js1_rot)

        # each call to tripod gait means there will be interpolating so we will set the interpolating to true every time
        self.interpolating = True

        for leg in range(NUMLEGS):
            # check to see if there is any substantial input
            if new_input:
                # init reach
                if self.gait_sequence[leg] == REACH and self.init:
                    self.init_reach(leg)

                # init pull
                elif self.gait_sequence[leg] == PULL and self.init:
                    self.init_pull(leg)

                # normal reach
                elif self.gait_sequence[leg] == REACH and not self.init:
                    self.reach(leg)

                # normal pull
                elif self.gait_sequence[leg] == PULL and not self.init:
                    self.pull(leg)

            else:
                # check to see if the current leg is already within the bounding box of init, if it isnt,
                # take the necessary steps to move the leg back a half stride length back to the initial
                # foot position.

                # reach init
                if self.gait_sequence[leg] == REACH:
                    pass

                # pull init
                if self.gait_sequence[leg] == PULL:
                    pass

            self.update_moving_legs(leg)  # do i even need to do this?

        # makes sure that a full movement has been made before ticks are reset and interpolating is reset
        if not self.is_interpolating():
            self.reset_ticks()
            self.interpolating = False

        #  if there is a new input that means the legs have moved from their init states to a full state
        #  if there is not a new input that means that the legs have moved from a full state back to a init state
        self.init = True if not new_input else False
        print("tick : {}".format(self.tick))
        print("interpolating: {}".format(self.interpolating))
        self.increment_tick()

    def is_interpolating(self) -> bool:
        """
        Checks the current tick with respect to the max ticks to see if a full movement has been made
        :return: Whether or not a full movement has been achieved
        """
        return self.tick <= self.max_ticks

    def init_reach(self, leg: int) -> None:
        """
        First state, hexapod is in its ready state reaching half the stride length to the target position
        :param leg: The leg that needs to perform the init reach
        :return: None
        """
        #  because it is init step to reach, we take a half step towards the target

        print("LEG : {}, STATE : Init Reach".format(leg))
        self.update_moving_legs(leg)

        half_stepx = self.rotated_step[0] / 2
        half_stepy = self.rotated_step[1] / 2

        tick_ratio = 1 / self.max_ticks

        current_x = self.current_gait_state[leg].x_gait
        current_y = self.current_gait_state[leg].y_gait

        new_state = gait_values()
        new_state.x_gait = current_x + (half_stepx * tick_ratio)
        new_state.y_gait = current_y + (half_stepy * tick_ratio)
        new_state.z_gait = -1 * (STRIDE_HEIGHT * np.sqrt())
        new_state.z_gait = -1*((STRIDE_HEIGHT/STEP_CONST) *
                               np.sqrt(abs(STEP_CONST**2 - (tick_ratio * self.tick * SCALE_FACTOR - STEP_CONST)**2)))

        self.current_gait_state[leg] = new_state

        #  check to see if a full motion has been completed
        if self.full_motion_completed():
            # change the state of the leg to pull after an init reach has been completed
            self.gait_sequence[leg] = PULL

    def init_pull(self, leg: int) -> None:
        """
        Second state, hexapod is in its ready state pulling half its stride length to the target position
        :param leg: The leg that needs to perform the init pull
        :return: None
        """

        print("LEG: {}, STATE : Init pull".format(leg))
        self.update_moving_legs(leg)

        half_stepx = self.rotated_step[0] / 2
        half_stepy = self.rotated_step[1] / 2

        tick_ratio = 1 / self.max_ticks

        current_x = self.current_gait_state[leg].x_gait
        current_y = self.current_gait_state[leg].y_gait

        new_state = gait_values()
        new_state.x_gait = current_x - (half_stepx * tick_ratio)
        new_state.y_gait = current_y - (half_stepy * tick_ratio)
        new_state.z_gait = 0

        self.current_gait_state[leg] = new_state

        if self.full_motion_completed():
            # change the gait state to reach after an init pull has been completed
            self.gait_sequence[leg] = REACH

    def reach(self, leg: int) -> None:
        """
        Third state, the hexapod has performed a pull to its target position and needs to raise the leg and
        move it one whole step size towards its new reach target which is CURRENT_POSITION + STEP_SIZE (after rotation)
        :param leg: The leg that needs to perform the reach
        :return: None
        """
        print(" LEG : {} STATE: Reach".format(leg))

        tick_ratio = 1 / self.max_ticks

        current_x = self.current_gait_state[leg].x_gait
        current_y = self.current_gait_state[leg].y_gait

        new_state = gait_values()
        new_state.x_gait = current_x + (self.rotated_step[0] * tick_ratio)
        new_state.y_gait = current_y + (self.rotated_step[1] * tick_ratio)
        new_state.z_gait = -1*((STRIDE_HEIGHT/STEP_CONST) *
                               np.sqrt(abs(STEP_CONST**2 - (tick_ratio*SCALE_FACTOR - STEP_CONST)**2)))

        self.current_gait_state[leg] = new_state

        if self.full_motion_completed():
            # change the gait sequence to pull after a full reach has been completed
            self.gait_sequence[leg] = PULL

    def pull(self, leg: int) -> None:
        """
        Fourth state, the hexapod has performed a reach to its target position and needs to now pull itself backwards
        :param leg: The leg that needs to perform the pull
        :return: None
        """
        print("LEG : {} STATE : Pull".format(leg))

        self.update_moving_legs(leg)

        tick_ratio = 1 / self.max_ticks

        current_x = self.current_gait_state[leg].x_gait
        current_y = self.current_gait_state[leg].y_gait

        new_state = gait_values()
        new_state.x_gait = current_x - (self.rotated_step[0] * tick_ratio)
        new_state.y_gait = current_y - (self.rotated_step[1] * tick_ratio)
        new_state.z_gait = 0

        self.current_gait_state[leg] = new_state

        if self.full_motion_completed():
            # change the sait state to reach after a full pull has been completed
            self.gait_sequence[leg] = REACH

    def pull_init(self, leg: int) -> None:
        """
        Fifth state, the hexapod has performed a reach to its target position and the user has not given any more input.
        The hexapod now needs to move from its pull position half a step size back to the initial position
        :param leg: A list of the legs that need to perform the pull init
        :return: None
        """
        print("LEG : {}, STATE : Pull init".format(leg))

        '''
        This is going to be different because we are going to incorporate error into it, 
        the error will be from its current position to the zero position. I am using this because
        it is a more 
        '''

        tick_ratio = 1 / self.max_ticks

        half_stride_x = self.rotated_step[0] / 2
        half_stride_y = self.rotated_step[1] / 2

        current_x = self.current_gait_state[leg].x_gait
        current_y = self.current_gait_state[leg].y_gait

        new_state = gait_values()
        new_state.x_gait = current_x - (half_stride_x * tick_ratio)
        new_state.y_gait = current_y - (half_stride_y * tick_ratio)
        new_state.z_gait = 0

        if self.full_motion_completed():
            # change the gait state to reach after a pull init has been completed
            self.gait_sequence[leg] = REACH

    def reach_init(self, leg: int) -> None:
        """
        Sixth state, the hexapod has performed a pull to its target position and the use has not given any more input.
        The hexapod not needs to move from its reach position half a step size back to the initial position
        :param leg: A list of the legs that need to perform the reach init
        :return: None
        """

        print("LEG : {}, STATE : Reach Init".format(leg))

        tick_ratio = 1 / self.max_ticks

        half_stride_x = self.rotated_step[0] / 2
        half_stride_y = self.rotated_step[1] / 2

        current_x = self.current_gait_state[leg].x_gait
        current_y = self.current_gait_state[leg].y_gait

        new_state = gait_values()
        new_state.x_gait = current_x - (half_stride_x * tick_ratio)
        new_state.y_gait = current_y - (half_stride_y * tick_ratio)

        # tick ratio is slightly different here because I am using 1 / max_ticks for the reach
        new_state.z_gait = -1*((STRIDE_HEIGHT/STEP_CONST) *
                              np.sqrt(abs(STEP_CONST**2 - (tick_ratio * self.tick * SCALE_FACTOR - STEP_CONST)**2)))

        if self.full_motion_completed():
            # change the let state to pull after a reach init has been completed
            self.gait_sequence[leg] = PULL

    def increment_tick(self) -> None:
        """
        Increments the tick counter once
        :return: None
        """
        self.tick += 1

    def update_moving_legs(self, leg) -> None:
        """
        Check to see if the leg in question has completed a full movement and update the moving legs data structure
        :param leg: The leg to check
        :return: None
        """

        # if the leg has made a full movement, then set the leg to not moving
        if self.tick >= self.max_ticks:
            self.moving_legs[leg] = 0

        else:  # if the leg has not made a full movement, set the leg to moving
            self.moving_legs[leg] = 1

    def full_motion_completed(self) -> bool:
        """
        Checks to see if a full stride has been made.
        :return: Whether or not a full stride has been taken
        """
        return self.tick == self.max_ticks

    def reset_ticks(self) -> None:
        """
        Resets the internal tick counter
        :return: None
        """
        self.tick = 1

    def get_gait_values(self) -> [gait_values]:
        """
        Returns the current gait values
        :return: A list of the current gait values
        """
        return self.current_gait_state

    def is_moving(self) -> bool:
        """
        Checks the movement vector to see if the robot is in the middle of a movement
        :return: Whether or not any leg is moving
        """
        return 1 in self.moving_legs

    def is_time_to_update(self) -> bool:
        """
        Checks the time to see if the gait handler is up for another update
        :return: Whether or not it is time to update
        """
        # if it is time to tick forward
        current_time = get_time_ms()
        if current_time - self.prev_time > UPDATE_PERIOD:
            self.prev_time = current_time
            return True
        return False

    def __str__(self):
        to_return = ""
        for gait in self.current_gait_state:
            to_return += " ({}, {}, {}) ".format(gait.x_gait, gait.y_gait, gait.z_gait)
        return to_return


if __name__ == "__main__":
    # some basic testing
    engine = GaitEngine()
    for i in range(30):
        print(engine.get_gait_values())
        engine.tripod_gait(processed_js_info(1.0, 0, 0, 0))
        #print(engine.get_gait_values())
    #engine.tripod_gait(processed_js_info(0, 45, 0, 0))

    print("Engine interpolating: {}".format(engine.is_interpolating()))
    print(engine.get_gait_values())
    print(engine)
    print(engine.tick)