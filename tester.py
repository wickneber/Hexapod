
import KinematicsEngine
import GaitEngine
from Ax12a import AX12AController
from HexapodContainers import processed_js_info
from helpers import batch_remap_leg_angles, batch_convert_to_servo_angles, construct_write_dict

"""
Test bench file to test all the stuff in motion
"""



if __name__ == "__main__":
    #servo_controller = AX12AController("Com3")
    ik_engine = KinematicsEngine.KinematicsEngine()
    movement_engine = GaitEngine.GaitEngine()
    movement_engine.tick = 0
    movement_engine.gait_step(processed_js_info(0, 20, 0, 0))
    gait_values = movement_engine.get_gait_values()
    print(gait_values)
    ik_engine.update_gait(gait_values)

    angles = ik_engine.do_ik()
    converted = batch_remap_leg_angles(angles, -90, 90, 0, 300)
    converted = batch_convert_to_servo_angles(converted)

    converted = construct_write_dict(converted)
    print(converted)
    """
    for leg_angle in angles:
        converted.append(convert_to_servo_angles(remap_leg_angles(leg_angle, -90, 90, 0, 300)))

    """



