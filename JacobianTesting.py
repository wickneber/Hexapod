import sys

import numpy as np


'''
Im trying to find alternative solutions to the analytical solution because the analytical
solution is a pain in my butt. This, along with the CCD testing, will be tested to find out
which solution I like better which I will then transfer to my ik solution. Making the solution
numerical also makes it so that I can transfer these methods across a wide range of robots 
that require inverse kinematics.
'''

COXA = 55
FEMUR = 75
TIBIA = 133

import time
from sympy.physics.mechanics import dynamicsymbols
import sympy as sm
import timeit


def get_homogeneous_transformation_matrix_sym(theta1, theta2, theta3):
    tibia1 = 75
    tibia2 = 110
    tibia = 133

    h01 = sm.matrices.Matrix(([[sm.cos(theta1), 0, sm.sin(theta1), COXA * sm.cos(theta1)],
                    [sm.sin(theta1), 0, -sm.cos(theta1), COXA * sm.sin(theta1)],
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]]))

    h12 = sm.matrices.Matrix(([[sm.cos(theta2), -sm.sin(theta2), 0, FEMUR * sm.cos(theta2)],
                    [sm.sin(theta2), sm.cos(theta2), 0, FEMUR * sm.sin(theta2)],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]]))

    h23 = sm.matrices.Matrix(([[sm.cos(theta3), 0, sm.sin(theta3), tibia * sm.cos(theta3)],
                    [sm.sin(theta3), 0, -sm.cos(theta3), tibia * sm.sin(theta3)],
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]]))

    h02 = h01*h12
    h03 = h02*h23
    return h01, h02, h03


class KinematicsEngine:

    def __init__(self):
        self.h01 = np.zeros(shape=(4, 4))
        self.h02 = np.zeros(shape=(4, 4))
        self.h03 = np.zeros(shape=(4, 4))
        self.h12 = np.zeros(shape=(4, 4))
        self.h23 = np.zeros(shape=(4, 4))
        print(self.h01)

    def calculate_homogeneous_transformation_matrix(self, theta1, theta2, theta3) -> None:
        tibia1 = 75
        tibia2 = 110
        tibia = 133
        theta1, theta2, theta3 = np.radians(theta1), np.radians(theta2), np.radians(theta3)

        self.h01[0, 0] = np.cos(theta1)
        self.h01[0, 1] = 0
        self.h01[0, 2] = np.sin(theta1)
        self.h01[0, 3] = COXA*np.cos(theta1)
        self.h01[1, 0] = np.sin(theta1)
        self.h01[1, 1] = 0
        self.h01[1, 2] = -np.cos(theta1)
        self.h01[1, 3] = COXA*np.sin(theta1)
        self.h01[2, 0] = 0
        self.h01[2, 1] = 1
        self.h01[2, 2] = 0
        self.h01[2, 3] = 0

        self.h12[0, 0] = np.cos(theta2)
        self.h12[0, 1] = -np.sin(theta2)
        self.h12[0, 2] = 0
        self.h12[0, 3] = FEMUR*np.cos(theta2)
        self.h12[1, 0] = np.sin(theta2)
        self.h12[1, 1] = np.cos(theta2)
        self.h12[1, 2] = 0
        self.h12[1, 3] = FEMUR*np.sin(theta2)
        self.h12[2, 0] = 0
        self.h12[2, 1] = 0
        self.h12[2, 2] = 1
        self.h12[2, 3] = 0
        self.h12[3, 0] = 0
        self.h12[3, 1] = 0
        self.h12[3, 2] = 0
        self.h12[3, 3] = 1

        self.h23[0, 0] = np.cos(theta3)
        self.h23[0, 1] = 0
        self.h23[0, 2] = np.sin(theta3)
        self.h23[0, 3] = tibia * np.cos(theta3)
        self.h23[1, 0] = np.sin(theta3)
        self.h23[1, 1] = 0
        self.h23[1, 2] = -np.cos(theta3)
        self.h23[1, 3] = tibia * np.cos(theta3)
        self.h23[2, 0] = 0
        self.h23[2, 1] = 1
        self.h23[2, 2] = 0
        self.h23[2, 3] = 0
        self.h23[3, 0] = 0
        self.h23[3, 1] = 0
        self.h23[3, 2] = 0
        self.h23[3, 3] = 1

        self.h02 = np.dot(self.h01, self.h12)
        self.h03 = np.dot(self.h02, self.h23)

    def get_jacobian(self, theta1, theta2, theta3):

        # we begin with getting the rotation components for the jacobian
        z_axis = np.array([[0], [0], [1], [0]])
        r00 = np.array([[0], [0], [1]])
        r01 = np.dot(self.h01, z_axis)[0:3]
        r02 = np.dot(self.h02, z_axis)[0:3]

        displacement_location = np.array([[0], [0], [0], [1]])

        # now we get the displacement
        current_position = np.dot(self.h03, displacement_location)[0:3]
        d02 = np.dot(self.h02, displacement_location)[0:3]
        d01 = np.dot(self.h01, displacement_location)[0:3]

        # TODO, dude youre hard coding it right now, you might as well just get all the
        # TODO, values at their states and calculate the jacobian from that
        # TODO, worry about the modularity when you are working on the other robots



def get_homogenous_transformation_matrix(theta1, theta2, theta3):
    tibia1 = 75
    tibia2 = 110
    tibia = 133
    theta1, theta2, theta3 = np.radians(theta1), np.radians(theta2), np.radians(theta3)
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

    h02 = np.dot(h01, h12)
    h03 = np.dot(h02, h23)
    return h01, h02, h03


def get_displacement_component(transformation_matrix):
    displacement_location = np.array([0,0,0,1])
    return np.dot(transformation_matrix, displacement_location)[0:3][np.newaxis].T


def get_rotation_component(transformation_matrix):
    return np.dot(transformation_matrix, [0, 0, 1, 0])[0:3][np.newaxis].T

def calculate_pseudoinverse(jacobian):
    inner = np.dot(jacobian.T, jacobian)
    inner = np.linalg.inv(inner)
    return np.dot(inner, jacobian.T)


def get_joint_positions(theta1, theta2, theta3):
    tibia1 = 75
    tibia2 = 110
    tibia = 133
    p3 = np.array([[0], [0], [-tibia2], [1]])
    theta1, theta2, theta3 = np.radians(theta1), np.radians(theta2), np.radians(theta3)
    h01 = np.array([[np.cos(theta1), 0, np.sin(theta1), COXA * np.cos(theta1)],
                    [np.sin(theta1), 0, -np.cos(theta1), COXA * np.sin(theta1)],
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]], dtype=np.float16)

    h12 = np.array([[np.cos(theta2), -np.sin(theta2), 0, FEMUR * np.cos(theta2)],
                    [np.sin(theta2), np.cos(theta2), 0, FEMUR * np.sin(theta2)],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]], dtype=np.float16)

    h23 = np.array([[np.cos(theta3), 0, np.sin(theta3), tibia1 * np.cos(theta3)],
                    [np.sin(theta3), 0, -np.cos(theta3), tibia1 * np.sin(theta3)],
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]], dtype=np.float16)

    displacement = [[0], [0], [0], [1]]
    joint0 = np.array([0, 0, 0, 1])
    joint1 = np.dot(h01, displacement).flatten()
    joint2 = np.dot(np.dot(h01, h12), displacement).flatten()
    h03 = np.dot(np.dot(h01, h12), h23)
    joint3 = np.dot(h03, p3).flatten()
    return joint0, joint1, joint2, joint3


def convert_to_steps(degrees):
    steps = degrees*(1/0.29)
    print("Degrees {} => {} Steps".format(degrees, steps))

# sources: https://www.youtube.com/watch?v=SefTCXrpL8U&t=772s
# https://mathweb.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf
# https://www.youtube.com/watch?v=VhUA0jf7tI8&t=79s


def get_jacobian(theta1, theta2, theta3):

    tolerance = 1000
    goal_position = np.array([[90], [0], [-50], [0], [0], [0]], dtype=np.float32)
    iter = 0
    z_component = np.array([[0], [0], [1]])
    while tolerance > .5:
        transformation_matrices = [i for i in get_homogenous_transformation_matrix(theta1, theta2, theta3)]

        # to construct a Jacobian matrix, first we need to get the rotation and displacement
        # components from the homogeneous transformation matrix
        rotations = [get_rotation_component(i)for i in transformation_matrices]
        displacement = [get_displacement_component(i) for i in transformation_matrices]

        current_position = displacement[-1]
        displacements_from_base = [current_position]

        # now we need to get the delta
        for i in range(len(displacement)-1):
            displacements_from_base.append(current_position - displacement[i])

        # now lets get the rotational components
        # TODO It seems that the problem stems from this location
        rotation_component = np.cross(z_component, current_position, axis=0)
        for i in range(len(rotations)-1):
            rotation_component = np.hstack((rotation_component, np.cross(rotations[i], displacements_from_base[i], axis=0)))

        # now lets get the linear component
        linear_component = z_component
        for i in range(len(rotations)-1):
            linear_component = np.hstack((linear_component, rotations[i]))

        jacobian = np.vstack((rotation_component, linear_component))

        pseudoinverse = np.linalg.pinv(jacobian)
        error = goal_position - np.vstack((current_position, [[0], [0], [0]]))
        delta_theta = np.dot(pseudoinverse, error)

        theta1 += delta_theta[0]
        theta2 += delta_theta[1]
        theta3 += delta_theta[2]
        tolerance = np.linalg.norm(error)


def print_matrix(to_print):
    for row in to_print:
        print(row)


def simulate_rotation_multiplication():
    rotx, roty, rotz, totalx, totaly, totalz, x, y, z = dynamicsymbols('rotx roty rotz totalx totaly totalz x y z')
    x_rotation_matrix = sm.Matrix([[1, 0, 0],
                                   [0, sm.cos(rotx), -sm.sin(rotx)],
                                   [0, sm.sin(rotx), sm.cos(rotx)]])

    y_rotation_matrix = sm.Matrix([[sm.cos(roty), 0, sm.sin(roty)],
                                   [0, 1, 0],
                                   [-sm.sin(roty), 0, sm.cos(roty)]])

    z_rotation_matrix = sm.Matrix([[sm.cos(rotz), -sm.sin(rotz), 0],
                                   [sm.sin(rotz), sm.cos(rotz), 0],
                                   [0, 0, 1]])

    total_rotation = x_rotation_matrix * y_rotation_matrix * z_rotation_matrix

    xyz = sm.Matrix([[totalx], [totaly], [totalz]])

    multiplied_values = total_rotation*xyz

    x = multiplied_values[0]
    y = multiplied_values[1]
    z = multiplied_values[2]

    print("X : {}".format(x))
    print("Y : {}".format(y))
    print("Z : {}".format(z))

if __name__ == "__main__":

    simulate_rotation_multiplication()
    sys.exit(0)
    theta1, theta2, theta3, coxa, femur, tibia = dynamicsymbols('theta1 theta2 theta3 coxa femur tibia')

    transformation_matrix = get_homogeneous_transformation_matrix_sym(theta1, theta2, theta3)

    print(transformation_matrix)
    print("="*50)
    a = transformation_matrix[-1]

    px = transformation_matrix[-1][0, -1]
    py = transformation_matrix[-1][1, -1]
    pz = transformation_matrix[-1][2, -1]

    a11 = sm.diff(px, theta1)
    a12 = sm.diff(px, theta2)
    a13 = sm.diff(px, theta3)
    a21 = sm.diff(py, theta1)
    a22 = sm.diff(py, theta2)
    a23 = sm.diff(py, theta3)
    a31 = sm.diff(pz, theta1)
    a32 = sm.diff(pz, theta2)
    a33 = sm.diff(pz, theta3)

    jacobian = sm.matrices.Matrix([[a11, a12, a13], [a21, a22, a23], [a31, a32, a33]])
    jacobian_transpose = jacobian.T

    end_effector_position = sm.matrices.Matrix([[px], [py], [pz]])
    goal_position = sm.matrices.Matrix([[90], [0], [0]])
    error = goal_position - end_effector_position
    print(jacobian_transpose)

    delta_theta = jacobian_transpose*error
    print("Delta theta: \n{}".format(delta_theta.shape))
    for i in delta_theta:
        print(i)
        


    '''
    engine = KinematicsEngine()
    begin = time.time()
    engine.calculate_homogeneous_transformation_matrix(0, 0, 0)
    engine.get_jacobian(0, 0, 0)
    end = time.time()
    print("Kinematics Engine time : {}".format(end - begin))
    '''
    #print(get_homogenous_transformation_matrix(0, 90, -90))
    #get_jacobian(0, -90, 90)
    #print(timeit.timeit('get_jacobian(0, -90, 90)', number=1, setup="from __main__ import get_jacobian"))
    #average_times = []
    #avg_time = 100
    #convert_to_steps(-90)

    '''
    for i in range(avg_time):
        begin = time.time()
        get_jacobian(0, -90, 90)
        end = time.time()
        average_times.append(end-begin)

    print("Average time to compute jacobian : {}ns".format(sum(average_times)/avg_time))
'''