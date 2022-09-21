'''
Im testing out the CCD algorithm to hopefully fix the issue of computation time while also making it
easier to apply it to different arm configurations.
'''

import numpy as np

COXA = 55
FEMUR = 70
TIBIA = 133

def get_joint_positions(theta1, theta2, theta3):
    tibia = 133
    theta1, theta2, theta3 = np.radians(theta1), np.radians(theta2), np.radians(theta3)
    theta3 += 0.75962
    h01 = np.array([[np.cos(theta1), 0, np.sin(theta1), COXA * np.cos(theta1)],
                    [np.sin(theta1), 0, -np.cos(theta1), COXA * np.sin(theta1)],
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]], dtype=np.float16)

    h12 = np.array([[np.cos(theta2), -np.sin(theta2), 0, FEMUR * np.cos(theta2)],
                    [np.sin(theta2), np.cos(theta2), 0, FEMUR * np.sin(theta2)],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]], dtype=np.float16)

    h23 = np.array([[np.cos(theta3), 0, np.sin(theta3), tibia * np.cos(theta3)],
                    [np.sin(theta3), 0, -np.cos(theta3), tibia * np.sin(theta3)],
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]], dtype=np.float16)

    displacement = [[0],[0],[0],[1]]
    h02 = np.dot(h01, h12)
    h03 = np.dot(h02, h23)

    d01 = np.dot(h01, displacement).flatten()
    d02 = np.dot(h02, displacement).flatten()
    d03 = np.dot(h03, displacement).flatten()

    return d01, d02, d03


def get_angle_between_vectors(vec1, vec2):
    vec1_unit_vec = vec1/np.linalg.norm(vec1)
    vec2_unit_vec = vec2/np.linalg.norm(vec2)

    dot_prod = np.dot(vec1_unit_vec, vec2_unit_vec)
    angle = np.arccos(dot_prod)
    cross_prod = np.cross(vec1_unit_vec, vec2_unit_vec)
    print("Cross product: {}".format(cross_prod))
    direction_of_rotation = 1 if cross_prod[1] > 0 else -1

    '''
    NOTE: You need to get the cross product of the vectors with the axis of rotation set to 0
    so for the femur and tibia joints, this means setting the y value to 0 and for the coxa, 
    setting the z value to 0. If the resultant cross product is positive, rotate it clockwise
    if it is negative, rotate it counterclockwise.
    '''

    return angle, direction_of_rotation

if __name__ == "__main__":

    theta1 = 0
    theta2 = 0
    theta3 = 0

    joints = [i[:3] for i in get_joint_positions(theta1, theta2, theta3)]
    goal_position = np.array([140, 0, -50])
    goal_angles_radians = []
    end_effector_position = joints[-1]

    print("Goal position {}\n".format(goal_position))
    print("Difference {}\n".format(goal_position - end_effector_position))

    error = 1000
    tolerance = 5
    iterations = 0
    max_iterations = 10
    while error > tolerance and iterations < max_iterations:
        print("End effector position: {}".format(end_effector_position))

        print("Joints {}".format(joints))
        for i in range(1, -1, -1):
            joints = [i[:3] for i in get_joint_positions(theta1, theta2, theta3)]
            end_effector_position = joints[-1]
            print(" New end effector position: {}".format(end_effector_position))
            print(" I {}".format(i))
            e_i = end_effector_position - joints[i]
            t_i = goal_position - joints[i]

            print(" Current joint position: {}\n".format(joints[i]))
            print(" e_i: {}\n".format(e_i))
            print(" t_i: {}\n".format(t_i))

            angle, rotation_direction = get_angle_between_vectors(t_i, e_i)
            print(" Angle between the two: {}".format(np.degrees(angle)))
            print(" Axis of rotation: {}".format(rotation_direction))

            if np.degrees(angle) > 20:
                angle = np.radians(20)

            if i == 0:
                theta2 += np.degrees(angle)*rotation_direction

            if i == 1:
                theta3 += np.degrees(angle)*rotation_direction

            if theta3 > 90:
                theta3 = 90

            if theta3 < -90:
                theta3 = 0

            if theta2 > 90:
                theta2 = 90

            if theta2 < -90:
                theta2 = 0

        print("End effector position after one pass: {}".format(end_effector_position))
        print("Theta1: {}, Theta2: {}, Theta3: {}".format(theta1, theta2, theta3))
        iterations += 1
        error = np.linalg.norm(goal_position - end_effector_position)
        print("Error {}".format(error))