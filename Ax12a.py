import dynamixel_sdk as ds

'''
Wrapper class to control the Dynamixel AX12a using python
'''


class AX12AController:
    PROTOCOL = 1.0

    # EEPROM Control Table ADDRESSES:
    MODEL_NUMBER_ADDR = 0
    FIRMWARE_VERSION_ADDR = 2
    ID_ADDR = 3
    BAUD_RATE_ADDR = 4
    RETURN_DELAY_TIME_ADDR = 5
    CW_ANGLE_LIMIT_ADDR = 6
    CCW_ANGLE_LIMIT_ADDR = 8
    TEMPERATURE_LIMIT_ADDR = 11
    MIN_VOLTAGE_ADDR = 12
    MAX_VOLTAGE_ADDR = 13
    MAX_TORQUE_ADDR = 14
    STATUS_RETURN_ADDR = 16
    ALARM_LED_ADDR = 17
    SHUTDOWN_INFO_ADDR = 18

    # RAM ADDRESSES
    TORQUE_ENABLE_ADDR = 24
    LED_ADDR = 25
    CW_COMPLIANCE_MARGIN_ADDR = 26
    CCW_COMPLIANCE_MARGIN_ADDR = 27
    CW_COMPLIANCE_SLOPE_ADDR = 28
    CCW_COMPLIANCE_SLOPE_ADRR = 29
    GOAL_POSITION_ADDR = 30
    MOVING_SPEED_ADDR = 32
    TORQUE_LIMIT_ADDR = 34
    PRESENT_POSITION_ADDR = 36
    PRESENT_SPEED_ADDR = 38
    PRESENT_LOAD_ADDR = 40
    PRESENT_VOLTAGE_ADDR = 42
    PRESENT_TEMP_ADDR = 43
    INSTRUCTION_REGISTERED_ADDR = 44
    MOVING_STATUS_ADDR = 46
    LOCK_EEPROM_ADDR = 47
    MIN_CURRENT_THRESHOLD_ADDR = 48

    # Some misc stuff for readability
    TORQUE_ON = 1
    TORQUE_OFF = 0
    LED_ON = 1
    LED_OFF = 0
    GOAL_POSITION_SIZE = 2  # size in bytes

    def __init__(self, com_port: str, baudrate: int = 1000000):
        self.comPort = com_port
        self.baudrate = baudrate

        self.portHandler = ds.PortHandler(com_port)
        self.packetHandler = ds.PacketHandler(AX12AController.PROTOCOL)

        if self.portHandler is None:
            raise ValueError("Failed to open port at: {}".format(self.comPort))

        if not self.portHandler.setBaudRate(baudrate):
            raise ValueError("Failed to set baud rate...")

        if not self.portHandler.openPort():
            raise ValueError(
                "Port given : {}, does not seem to be responding...\n Please check the com port and try again."
            )
        self.goalPositionGroupSyncWrite = ds.GroupSyncWrite(self.portHandler, self.packetHandler,
                                                            AX12AController.GOAL_POSITION_ADDR,
                                                            AX12AController.GOAL_POSITION_SIZE)

        self.most_recent_error_message = 0
        self.most_recent_comm_message = 0

    def set_torque(self, servo_ids: [int], torque_value: int) -> bool:
        """
        Turns on the torque for the servos specified
        :param torque_value: A number that is either 0 or 1
        :param servo_ids: A list of the ids to turn the torque on
        :return: A boolean value signifying whether or not the operation was successful
        """
        if torque_value not in [0, 1]:
            raise ValueError("Set torque value must be either 0 or 1.")

        for servo_id in servo_ids:
            comm_result, error = self.packetHandler.write1ByteTxRx(self.portHandler, servo_id,
                                                                   AX12AController.TORQUE_ENABLE_ADDR, torque_value)
            if comm_result != ds.COMM_SUCCESS:
                print("An issue setting the torque on servo {} has been encountered".format(servo_id))
                print(self.packetHandler.getTxRxResult(comm_result))
                return False

            if error != 0:
                print("An error has occurred turning on the torque for servo {}".format(servo_id))
                print(self.packetHandler.getRxPacketError(error))
                return False

        return True

    def set_led(self, servo_ids: [int], led_value: int) -> bool:
        """
        Turns on or off the led of a set of servo id numbers
        :param servo_ids: List of servo id numbers
        :param led_value: The value between 0 or 1 to set the led register to
        :return: A boolean value indicating whether or not the operation was successful
        """
        if led_value not in [0, 1]:
            raise ValueError("Led value given to set is not 0 or 1.")

        for servo_id in servo_ids:
            self.most_recent_comm_message, self.most_recent_error_message = self.packetHandler.write1ByteTxRx \
                    (
                    self.portHandler, servo_id,
                    AX12AController.LED_ADDR, led_value
                )

            if self.most_recent_comm_message != ds.COMM_SUCCESS:
                print("There was an issue setting the led on servo {}".format(servo_id))
                print(self.packetHandler.getTxRxResult(self.most_recent_comm_message))
                return False

            if self.most_recent_error_message != 0:
                print("There was an issue with setting the led on servo {}".format(servo_id))
                print(self.packetHandler.getRxPacketError(self.most_recent_error_message))
                return False

        return True

    def set_goal_position(self, servo_id: int, goal_position: int) -> bool:
        """
        Sets the goal position for a specific servo. This is meant to only write to one servo.
        Use sync_write_goal_position if you want to write to multiple servos at once.
        :param servo_id: The servo to write to
        :param goal_position: The value to set the servo
        :return: A boolean indicator indicating whether or not an error has occurred
        """
        self.most_recent_comm_message, self.most_recent_error_message = self.packetHandler.write2ByteTxRx(
            self.portHandler, servo_id,
            AX12AController.GOAL_POSITION_ADDR,
            goal_position)
        return self.most_recent_comm_message == ds.COMM_SUCCESS and self.most_recent_error_message == 0

    def scan(self, servo_range: int = 253) -> [int]:
        """
        Scans a for all servos within the specified range
        :servo_range: The id range to scan
        :return: A list of all discovered servos
        """
        discovered_servos = []
        for servo_id in range(1, servo_range + 1):

            _, self.most_recent_comm_message, self.most_recent_error_message = self.packetHandler.ping \
                (self.portHandler, servo_id)

            if self.most_recent_comm_message != ds.COMM_SUCCESS:
                print("[{}] {}".format(servo_id,
                                       self.packetHandler.getTxRxResult(self.most_recent_comm_message)))
            if self.most_recent_error_message != 0:
                print(self.packetHandler.getRxPacketError(self.most_recent_error_message))

            if self.most_recent_comm_message == ds.COMM_SUCCESS and self.most_recent_error_message == 0:
                discovered_servos.append(servo_id)

        return discovered_servos

    def get_current_position(self, servo_id) -> int:
        """
        Reads the current postion of the servo
        :param servo_id: The servo to read from
        :return: The current position of the servo
        """
        present_position, self.most_recent_comm_message, self.most_recent_error_message = \
            self.packetHandler.read2ByteTxRx(self.portHandler,
                                             servo_id,
                                             AX12AController.PRESENT_POSITION_ADDR
                                             )

        if self.most_recent_comm_message != ds.COMM_SUCCESS:
            print("[{}] read error : {}".format(servo_id, self.packetHandler.getTxRxResult(
                self.most_recent_comm_message)))

        if self.most_recent_error_message != 0:
            print("[{}] read error : {}".format(servo_id, self.packetHandler.getRxPacketError(
                self.most_recent_error_message)))

        return present_position

    def sync_write_goal_position(self, to_write) -> bool:
        """
        Add the value to the write buffer for sync writing
        :param to_write: The value to add to the sync write buffer
        :return: A status code indicating whether or not the addition to the buffer was successful
        """

        status = True
        for servo_id, goal_position in to_write.items():
            param_goal_position = [ds.DXL_LOBYTE(goal_position), ds.DXL_HIBYTE(goal_position)]
            status = self.goalPositionGroupSyncWrite.addParam(servo_id, param_goal_position)

        self.goalPositionGroupSyncWrite.txPacket()
        self.goalPositionGroupSyncWrite.clearParam()
        return status

    def read_error_message(self) -> str:
        """
        Reads the most recent error message stored and returns the translated error code
        :return: The error code decoded into a string
        """
        return self.packetHandler.getRxPacketError(self.most_recent_error_message)

    def read_comm_message(self) -> str:
        """
        Reads the most recent communication servo message from the most recent operation
        :return: The most recent stored communication message
        """
        return self.packetHandler.getTxRxResult(self.most_recent_comm_message)

    def get_voltage(self, servo_id:int = 1):
        """
        Reads the voltage from a given servo
        :param servo_id: The servo number to read from
        :return: The voltage value from the desired servo
        """
        voltage, self.most_recent_comm_message, self.most_recent_error_message = \
            self.packetHandler.read1ByteTxRx(self.portHandler, servo_id, AX12AController.PRESENT_VOLTAGE_ADDR)
        return voltage

    def read_register(self, servo_id, register_number) -> int:
        """
        Reads given servo's register and returns read information
        :param servo_id: The servo to read from
        :param register_number: The register number to read
        :return: The information read
        """
        # TODO

    def __del__(self):
        if self.portHandler is not None and self.portHandler.is_open is True:
            self.portHandler.closePort()
