import pygame
import numpy as np
import sys

BUFF_SIZE = 25 # for the running average

"""

Gamepad interface that supports Xbox and Playstation controllers, mainly geared towards Playstation simply because I
have a playstation controller.
 
"""


class HexapodController:
    BUTTON_NAMES = ['x', 'circle', 'square', 'triangle', 'share', 'ps_button', 'options', 'l3', 'r3',
                    'l1', 'r1', 'dpad_up', 'dpad_down', 'dpad_left', 'dpad_right', 'trackpad', 'mic_button',
                    'left_joystick_x', 'left_joystick_y', 'right_joystick_x', 'right_joystick_y']
    # for deadzone
    EPS = 0.00001

    def __init__(self, deadzone: float = 0.1):
        pygame.init()

        # get the first instance of a connected controller
        try:
            self.joystick = pygame.joystick.Joystick(0)

        except pygame.error as error:
            print("An error occurred when looking for the controller...")
            print("Error from pygame: {}".format(error))
            sys.exit(-1)

        self.joystick.init()

        self.joystick1x_average = [0]*BUFF_SIZE
        self.joystick1y_average = [0]*BUFF_SIZE
        self.joystick2x_average = [0]*BUFF_SIZE
        self.joystick2y_average = [0]*BUFF_SIZE

        self.joystick_average_ind = 0

        self.controller_state = {}
        self.num_buttons = self.joystick.get_numbuttons()
        self.num_axes = self.joystick.get_numaxes()
        self.deadzone = deadzone

        print("Successfully connected to {}.".format(self.joystick.get_name()))

    def get_controller_state(self) -> {}:
        """
        Gets the current controller state and returns it to the user
        :return: The current controller state
        """
        self._update_joystick_information()
        self._update_button_information()
        return self.controller_state

    def _update_joystick_information(self) -> None:
        """
        Obtains the joystick information of the gamepad and calculates the deadzone for every joystick
        :return: None
        """
        pygame.event.get()  # get the most recent event
        joystick_trigger_reading = np.array([self.joystick.get_axis(axis) for axis in range(self.num_axes)])

        joystick_trigger_reading = [np.round(reading, 2) for reading in joystick_trigger_reading]

        # add read values into the buffer for average
        self.joystick1x_average[self.joystick_average_ind] = joystick_trigger_reading[0]
        self.joystick1y_average[self.joystick_average_ind] = -joystick_trigger_reading[1]

        self.joystick2x_average[self.joystick_average_ind] = joystick_trigger_reading[2]
        self.joystick2y_average[self.joystick_average_ind] = -joystick_trigger_reading[3]

        self.joystick_average_ind += 1
        self.joystick_average_ind = self.joystick_average_ind % BUFF_SIZE

        self.controller_state['joystick1'] = self._calculate_deadzone([np.mean(self.joystick1x_average),
                                                                       np.mean(self.joystick1y_average)])

        self.controller_state['joystick2'] = self._calculate_deadzone([np.mean(self.joystick2x_average),
                                                                       np.mean(self.joystick2y_average)])
        self.controller_state['l2'] = joystick_trigger_reading[4]
        self.controller_state['r2'] = joystick_trigger_reading[5]

    def _update_button_information(self) -> None:
        """
        Gets the current gamepad button readings and updates the internal state of the class.
        :return: None
        """
        pygame.event.get()
        buttons = [self.joystick.get_button(button) for button in range(self.num_buttons)]
        for button_name, button_state in zip(HexapodController.BUTTON_NAMES, buttons):
            self.controller_state[button_name] = button_state

    def _calculate_deadzone(self, raw_joystick_reading) -> [float, float]:
        """
        Calculates the deadzone of the current joystick readings.
        :param raw_joystick_reading: The joystick readings that are to be deadzoned
        :return: The deadzoned joystick values
        """
        if np.linalg.norm(raw_joystick_reading) > self.deadzone:
            return raw_joystick_reading
        return np.array([0.0, 0.0])
