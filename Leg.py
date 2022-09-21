
import numpy as np

"""
Leg class for hexapod robot on the PhantomX III robot chassis 
"""

BODY_CENTER_X_OFF = 60
BODY_CENTER_Y_OFF = 120
X_CENTER_DISP = 100 # This is for the middle legs
ZOFFSET = 75

class Leg:

    def __init__(self, leg_number: int, servo_ids: [int, int, int], initial_feet_positions: [float, float, float]):
        self.leg_id = leg_number
        self.servo_ids = servo_ids

        # Offset values with the frame of reference at the center of the robot
        self.body_offset_angle = 0
        self.body_offset_dist = 0
        self.body_offset_x = 0
        self.body_offset_y = 0

        self.init_feet_x = initial_feet_positions[0]
        self.init_feet_y = initial_feet_positions[1]
        self.init_feet_z = initial_feet_positions[2]

        # the angle that the leg is displaced from the x axis
        self.rotation = 0

        # actual values to write to the servo
        self.servo_values = [0, 0, 0]

        self._init_leg()

    def set_servo_values(self, new_servo_values) -> None:
        self.servo_valuse = new_servo_values

    def _init_leg(self) -> None:
        """
        Given the leg number, set the offset values accordingly
        :return: None
        """
        # all of these measurements are in millimeters
        match self.leg_id:
            case 1:
                self.rotation = np.radians(120)
                self.body_offset_x = -60
                self.body_offset_y = 120

            case 2:
                self.rotation = np.radians(60)
                self.body_offset_x = 60
                self.body_offset_y = 120

            case 3:
                self.rotation = np.radians(240)
                self.body_offset_x = -60
                self.body_offset_y = -120

            case 4:
                self.rotation = np.radians(300)
                self.body_offset_x = 60
                self.body_offset_y = -120

            case 5:
                self.rotation = np.radians(0)
                self.body_offset_x = -100
                self.body_offset_y = 0
                self.init_feet_y = 0

            case 6:
                self.rotation = np.radians(180)
                self.body_offset_x = 100
                self.body_offset_y = 0
                self.init_feet_y = 0

        self.body_offset_dist = np.arctan2(self.body_offset_y, self.body_offset_x)
        self.body_offset_angle = np.sqrt(self.body_offset_y**2 + self.body_offset_x**2)
        #self._rotate_init_leg_pos()

    def _rotate_init_leg_pos(self) -> None:
        """
        After initialization is done, the initial feet positions need to be calculated with respect to the
        body frame.
        :return: None
        """
        new_pos_x = self.init_feet_x * np.cos(self.rotation) - self.init_feet_y * np.sin(self.rotation)
        new_pos_y = self.init_feet_x * np.sin(self.rotation) + self.init_feet_y * np.cos(self.rotation)

        self.init_feet_x = new_pos_x
        self.init_feet_y = new_pos_y

        #self.init_feet_x *= -1
        #self.init_feet_y *= -1

    def __str__(self):
        return "Leg id : {}, Leg servos : {}, init x position : {}, init y position : {}, init z position : {}"\
            .format(self.leg_id, self.servo_ids, self.init_feet_x, self.init_feet_y, self.init_feet_z)