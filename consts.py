import enum

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
MPC_PREDICTION_HORIZON = 10
MPC_U_LIMIT = 0.1
MPC_SOLVER_MAX_ITER = 1000

STEP_SIZE = 0.1

