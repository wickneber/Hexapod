
from HexapodContainers import servo_ids

SERVO_MAPPING = [servo_ids(1, 3, 5), servo_ids(2, 4, 6), servo_ids(7, 9, 11), servo_ids(8, 10, 12),
                 servo_ids(13, 15, 17), servo_ids(14, 16, 18)]

DEGREES_PER_STEPS = 0.293

NUMLEGS = 6
END_EFFECTOR_OFFSET = 0.6
STEPS_PER_DEGREE = 0.2929

# In mm
INIT_FEET_X = 160
INIT_FEET_Y = 0
INIT_FEET_Z = 80

STRIDE_HEIGHT = 30
STEP_SIZE     = 30



COXA  = 53
FEMUR = 66
TIBIA = 132

TRIPOD_GAIT_SEQUENCE = [1, 2, 1, 2, 2, 1]

# Both in ms
UPDATE_PERIOD = 30
STEP_DURATION = 300
MAX_TICKS = int(STEP_DURATION/UPDATE_PERIOD)

# variable for step height calculation
STEP_CONST = .5
SCALE_FACTOR = 2*STEP_CONST

INITTED = True

# names for states in the gait controller
REACH = 1
PULL  = 2

TRIPOD_GAIT_ID = 0