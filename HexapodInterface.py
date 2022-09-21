
'''
This class is the main interface that controls the hexapod. It uses both the kinematics engine and the gait engine
to form an interface where a lot of the nitty gritty is abstracted.
'''

from HexapodConstants import *
from GaitEngine import GaitEngine
from KinematicsEngine import KinematicsEngine


class HexapodInterface:

    def __init__(self):
        self.gait_mode = TRIPOD_GAIT_ID
        self.gait_engine = GaitEngine()
        self.kinematics_engine = KinematicsEngine()

    def select_gait(self, gait_number: int) -> None:
        """
        Just selects the gait for the gait engine.
        :param gait_number: The integer corresponding to the gait to use
        :return: None
        """
        # TODO



