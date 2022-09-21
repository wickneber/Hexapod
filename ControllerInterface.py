from abc import abstractmethod, ABC

import pygame
import numpy as np
from RobotControllers.Errors.ControllerError import WrongControllerError

'''
Ps4/5 controller input class
'''


class ControllerError(Exception):
    pass


class ControllerInterface(ABC):
    '''
    This is the base class for any future controller interfaces to inherit from.
    All it does is gets the raw information and calculates the deadzone values
    of the joysticks and returns it.
    '''

    PERIPHERAL_TYPE = ["Buttons", "Joysticks"]

    def __init__(self, deadzone_weight: float = 1.0):
        pygame.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.name = self.joystick.get_name()
        self.num_axes = self.joystick.get_numaxes()
        self.num_buttons = self.joystick.get_numbuttons()
        self.deadzone_weight = deadzone_weight
        self.controller_values = {}

        print("Successfully connected to: {}.".format(self.name))

    @abstractmethod
    def get_joystick_values(self) -> [float, float, float, float, float, float]:
        pass

    @abstractmethod
    def calculate_deadzone(self, to_calculate:float) -> float:
        pass

    @abstractmethod
    def get_button_states(self) -> []:
        pass

    @abstractmethod
    def get_controller_state(self) -> []:
        pass


class PlaystationControllerInterface(ControllerInterface):

    BUTTON_NAMES= ['x', 'circle', 'square', 'triangle', 'share', 'ps_button', 'options', 'l3', 'r3',
                    'l1', 'r1', 'dpad_up', 'dpad_down', 'dpad_left', 'dpad_right', 'trackpad', 'mic_button',
                   'left_joystick_x', 'left_joystick_y', 'right_joystick_x', 'right_joystick_y']

    def __init__(self, deadzone_weight: float = 1.0):

        # initialize the controller values
        super().__init__(deadzone_weight)

        if self.name not in ["PS5 Controller", "PS4 Controller"]:
            raise WrongControllerError("Error, please connect a PS5 or PS4 controller or use a different controller"
                                       "interface.")
        for name in PlaystationControllerInterface.BUTTON_NAMES:
            self.controller_values[name] = 0

    def get_joystick_values(self) -> [float, float, float, float, float, float]:
        '''
        Reads the joystick values and returns them with deadzone calculated as well.
        :return: The read joystick states after a deadzone calculation has been made
        '''
        pygame.event.get()  # get the most recent event
        joystick_trigger_reading = [self.joystick.get_axis(axis) for axis in range(self.num_axes)]
        for js in range(len(joystick_trigger_reading) - 2):
            joystick_trigger_reading[js] = self.calculate_deadzone(joystick_trigger_reading[js])

        joystick_trigger_reading = [np.round(reading, 2) for reading in joystick_trigger_reading]

        return joystick_trigger_reading

    def calculate_deadzone(self, to_calculate: float) -> float:
        """
        Calculate the deadzone of a value using this formula:
        http://www.mimirgames.com/articles/games/joystick-input-and-using-deadbands/
        :param to_calculate: The value to calculate the deadzone
        :return: The deadzone calculation
        """
        scaled_db = (to_calculate - (np.abs(to_calculate) / to_calculate) * 0.1) / 0.9
        cubic_scaled = self.deadzone_weight * to_calculate ** 3 + (1 - self.deadzone_weight) * to_calculate
        cubic_scaled_deadband = cubic_scaled - (np.abs(to_calculate) / to_calculate) * \
                                (self.deadzone_weight * scaled_db ** 3 + (1 - self.deadzone_weight) * scaled_db)
        cubic_scaled_deadband /= (1 - (self.deadzone_weight * scaled_db ** 3 + (1 - self.deadzone_weight) * scaled_db))
        return cubic_scaled_deadband

    def get_button_states(self) -> []:
        """
        Get the button states from the controller
        :return: The read button states
        """
        pygame.event.get()
        return [self.joystick.get_button(button) for button in range(self.num_buttons)]

    def get_controller_state(self) -> []:
        """
        Gets the current state of the controller
        :return: The entire state of the controller
        """
        joystick_values = self.get_joystick_values()
        button_states = self.get_button_states()
        controller_state = button_states + joystick_values

        for button_name, button_state in zip(PlaystationControllerInterface.BUTTON_NAMES, controller_state):
            self.controller_values[button_name] = button_state

        return self.controller_values

    def __str__(self) -> str:
        return "Js name: {}, num_axes: {}, num_buttons: {}".format(
            self.name, self.num_axes, self.num_buttons)

