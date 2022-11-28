from scipy.optimize import minimize
import numpy as np

from motion_planner.utils import get_distance

from typing import List, Tuple
from shapely.geometry import LineString, Point

# To be set by the user each time
# it is assumed that the first point of the path is the next goal point
PATH = []
INIT_POSITION = (0.0, 0.0)
W_u = 0.1
W_p = 1
PREDICTION_HORIZON = 5

class MPC():
    """
    This class is used to run the MPC algorithm.
    Parameters:
        path: List[Tuple[float, float]] - the path to follow
        curr_position: Tuple[float, float] - the initial position of the robot
        w_u: float - the weight of the control input
        w_p: float - the weight of the position error
        prediction_horizon: int - the number of steps to predict
    """
    def __init__(
        self,
        path: List[Tuple[float, float]],
        curr_position: Tuple[float, float],
        w_u: float,
        w_p: float,
        prediction_horizon: int,
        ):
        self.path = path
        self.curr_position = curr_position
        self.w_u = w_u
        self.w_p = w_p
        self.prediction_horizon = prediction_horizon

    def set_position(self, position: Tuple[float, float]):
        """
        Sets the current position of the robot.
        Parameters:
            position: Tuple[float, float] - the current position of the robot
        """
        self.curr_position = position

        



def d(u: List[float]):
    """
    Calculates the cost according to the distance between the path and the agent positions
    Input:
        u: control input
    """
    
    # lets make it go to the first point of the path
    curr_point = np.array(INIT_POSITION)
    cost = 0
    i = 0
    aim = PATH[0]
    calculated_path = []
    while i < PREDICTION_HORIZON:
        calculated_path.append(curr_point)
        cost += get_distance(curr_point[0], curr_point[1], aim[0], aim[1])
        curr_point[0] += u[2*i]
        curr_point[1] += u[2*i+1]
        i += 1

    return cost




def to_be_optimized(u):
    """
    The function to be optimized
    Input:
        u: control input
    """
    # TODO: np.sum is wrong here!
    return W_u * np.sum(u) + W_p * d(u)


def test():
    global PATH
    global INIT_POSITION
    PATH = [np.array([0, 0]), np.array([1, 1]), np.array([2, 0])]
    INIT_POSITION = (0.0, 0.0)
    print("Should be 0,0")
    d(None)

    PATH = [np.array([1, 1]), np.array([2, 0]), np.array([3, 1])]
    INIT_POSITION = (0.0, 0.0)
    print("Should be 1,1")
    d(None)

def constraint(u, i):
    return 0.1 - np.sqrt(u[2*i]**2 + u[2*i+1]**2)


def run_once(path, init_position):
    # this is bad, change it!
    global PATH, INIT_POSITION
    PATH = path
    INIT_POSITION = init_position

    U_limit = 0.1
    # make PREDICTION_HORIZON control inputs, each of which is a 2D vector
    u0 = [0.0]*PREDICTION_HORIZON*2
    #u0 = [0.0, -0.1, 0.0, -0.1, 0.0, -0.1, 0.0, 0.0, 0.0, 0.0]
    bounds = [(-U_limit, U_limit)]*PREDICTION_HORIZON*2
    cons = (
        {'type': 'ineq', 'fun': lambda u: constraint(u, 0)},
        {'type': 'ineq', 'fun': lambda u: constraint(u, 1)},
        {'type': 'ineq', 'fun': lambda u: constraint(u, 2)},
        {'type': 'ineq', 'fun': lambda u: constraint(u, 3)},
        {'type': 'ineq', 'fun': lambda u: constraint(u, 4)},
    )
    a = minimize(to_be_optimized, u0, method = "SLSQP", constraints = cons, options={'maxiter': 1000}, bounds=bounds)
    #print(a)

    #u = a.x.reshape((PREDICTION_HORIZON, 2))
    u = a.x
    cost = d(u)

    return cost, u

if __name__ == "__main__":
    # PATH = [np.array([0, 0]), np.array([1, 0]), np.array([2, 0])]
    # INIT_POSITION = (0.5, 0.2)
    # d(None)
    
    U_limit = 0.1
    PATH = [np.array([0, 0]), np.array([1, 0]), np.array([2, 0])]
    INIT_POSITION = (0.3, -0.3)

    """
    random_count = 1
    min_cost = np.inf
    best_u = None
    for i in range(random_count):
        u_start = np.random.uniform(-U_limit, U_limit, (PREDICTION_HORIZON, 2))
        temp_cost, temp_u = run_once(u_start, PATH, INIT_POSITION)
        if temp_cost < min_cost:
            min_cost = temp_cost
            best_u = temp_u
    """

    min_cost, best_u = run_once(PATH, INIT_POSITION)

    print("the best u is: ", best_u)
    print("the best cost is: ", min_cost)

    #best_u = [0.0, -0.1, 0.0, -0.1, 0.0, -0.1, 0.0, 0.0, 0.0, 0.0]
    # WRITE THE PATH!
    print("PRINTING PATH")
    curr_position = np.array(INIT_POSITION)
    print(curr_position)
    for i in range(PREDICTION_HORIZON):
        curr_position[0] += best_u[2*i]
        curr_position[1] += best_u[2*i+1]
        print(curr_position)
        




