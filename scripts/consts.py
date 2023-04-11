import enum
import logging

# enum for choosing between controller types
class ControllerType(enum.Enum):
    P = 0
    MPC = 1


MAGNETIC_FIELD_FREQ = 2
# this can be much much smaller, but, beacuse we do not use magnetic field frequency, 
# we cannot control the speed, we can only control the direction of the speed
# therefore, we need to accept a bigger accepted distance for now
ACCEPTED_DISTANCE = 10.0
NOISE = 0.1
CONTROLLER_TYPE = ControllerType.MPC

# MPC consts
# the coefficients for the cost function
MPC_W_U = 1
MPC_W_P = 10000
MAX_F = 10
MPC_PREDICTION_HORIZON = 5
MPC_U_LIMIT = 10.0
MPC_MINIMUM_SPEED = 0.1
MPC_SOLVER_MAX_ITER = 1000
TIME_STEP = 1.0

STEP_SIZE = 0.1

# Environment variables
ROBOTS_START_X = [2, 50, 58, 15, 25, 35, 45, 55, 65, 75]
ROBOTS_START_Y = [2, 50, 20, 100, 75, 35, 45, 55, 65, 75]
ROBOTS_GOAL_X = [150, 98, 160, 20, 50, 10, 40, 30, 60, 100]
ROBOTS_GOAL_Y = [100, 102, 35, 70, 5, 50, 30, 20, 10, 60]
ENV_MIN_X = 0
ENV_MIN_Y = 0
ENV_WIDTH = 200
ENV_HEIGHT = 200
OBSTACLES = [
    ([0.5, 0.5], [0.5, 1.5], [1.5, 1.5], [1.5, 0.5]),
    ([1.0, 0.0], [4.0, 0.0], [4.0, -5.0]),
    ([-2.0, 6.0], [-2.0, 2.0], [2.0, 4.0], [4.0, 6.0], [5.0, 7.0])
]
SPEEDS_FOR_FREQ = [
    [
        1, 2, 1, 1
    ],
    [
        0.5, 1.5, 2, 1
    ],
    [
        2, 0.4, 0.2, 1
    ],
    [
        0.7, 1, 2, 1.5
    ]
]

# RRT consts
RRT_STEP_SIZE = 4
RRT_REWIRE_DISTANCE = RRT_STEP_SIZE
RRT_MAX_ITER = 10000

LOGGER_LEVEL = logging.WARNING

NUMBER_OF_AGENTS = 4
