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
ACCEPTED_DISTANCE = 0.1
NOISE = 0.0
CONTROLLER_TYPE = ControllerType.MPC

# MPC consts
# the coefficients for the cost function
MPC_W_U = 1
MPC_W_P = 10
MAX_F = 10
MPC_PREDICTION_HORIZON = 5
MPC_U_LIMIT = 10.0
MPC_MINIMUM_SPEED = 0.01
MPC_SOLVER_MAX_ITER = 1000
TIME_STEP = 1.0

STEP_SIZE = 0.1

# Environment variables
ROBOTS_START_X = [-2.5]
ROBOTS_START_Y = [7.5]
ROBOTS_GOAL_X = [7.5]
ROBOTS_GOAL_Y = [-7.5]
ENV_MIN_X = -10
ENV_MIN_Y = -10
ENV_WIDTH = 20
ENV_HEIGHT = 20
OBSTACLES = [
    ([0.5, 0.5], [0.5, 1.5], [1.5, 1.5], [1.5, 0.5]),
    ([1.0, 0.0], [4.0, 0.0], [4.0, -5.0]),
    ([-2.0, 6.0], [-2.0, 2.0], [2.0, 4.0], [4.0, 6.0], [5.0, 7.0])
]

# RRT consts
RRT_STEP_SIZE = 0.2
RRT_REWIRE_DISTANCE = 5.0
RRT_MAX_ITER = 5000

LOGGER_LEVEL = logging.WARNING

NUMBER_OF_AGENTS = 1
