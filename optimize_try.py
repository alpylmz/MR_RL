from scipy.optimize import minimize
import numpy as np

from motion_planner.utils import get_distance

from typing import List, Tuple
from shapely.geometry import LineString, Point

# To be set by the user each time
# it is assumed that the first point of the path is the next goal point
PATH = []
INIT_POSITION = (0.0, 0.0)
STEP_SIZE = 0.1
W_u = 0.1
W_p = 1
PREDICTION_HORIZON = 5



def d(u: List[Tuple[float, float]]):
    """
    Calculates the cost according to the distance between the path and the agent positions
    Input:
        u: control input
    """

    p1 = PATH[0]
    p2 = PATH[1]
    p3 = INIT_POSITION
    line = LineString([p1, p2])
    point = Point(p3)
    closest_point = line.interpolate(line.project(point))
    
    # if the closest_point is not PATH[0], then skip the first PATH point
    if abs(closest_point.x - PATH[0][0]) < 0.0001 or abs(closest_point.y - PATH[0][1]) < 0.0001:
        aim_list = [[closest_point.x, closest_point.y]] + PATH[1:]
    else:
        aim_list = [[closest_point.x, closest_point.y]] + PATH


    # now, we need to calculate a trajectory
    # for that purpose we need to separate aim_list into STEP_SIZE parts
    # and add the points to the trajectory

    trajectory = []
    for i in range(len(aim_list) - 1):
        p1 = aim_list[i]
        p2 = aim_list[i + 1]
        distance = get_distance(p1[0], p1[1], p2[0], p2[1])
        steps = int(distance / STEP_SIZE)
        for j in range(steps):
            trajectory.append(p1 + j * (p2 - p1) / steps)

    #print(trajectory)

    curr_point = np.array(INIT_POSITION)
    cost = 0
    i = 0
    trajectory = trajectory[:PREDICTION_HORIZON]
    while i < len(trajectory):
        point = trajectory[i]
        cost += get_distance(curr_point[0], curr_point[1], point[0], point[1])
        curr_point += u[i]

        i += 1


    return cost

def dd(u: List[Tuple[float, float]]):
    """
    Calculates the cost according to the distance between the path and the agent positions
    Input:
        u: control input
    """

    p1 = PATH[0]
    p2 = PATH[1]
    p3 = INIT_POSITION
    line = LineString([p1, p2])
    point = Point(p3)
    closest_point = line.interpolate(line.project(point))
    
    # if the closest_point is not PATH[0], then skip the first PATH point
    if abs(closest_point.x - PATH[0][0]) < 0.0001 or abs(closest_point.y - PATH[0][1]) < 0.0001:
        aim_list = [[closest_point.x, closest_point.y]] + PATH[1:]
    else:
        aim_list = [[closest_point.x, closest_point.y]] + PATH


    # now, we need to calculate a trajectory
    # for that purpose we need to separate aim_list into STEP_SIZE parts
    # and add the points to the trajectory

    trajectory = []
    for i in range(len(aim_list) - 1):
        p1 = aim_list[i]
        p2 = aim_list[i + 1]
        distance = get_distance(p1[0], p1[1], p2[0], p2[1])
        steps = int(distance / STEP_SIZE)
        for j in range(steps):
            trajectory.append(p1 + j * (p2 - p1) / steps)

    #print(trajectory)

    curr_point = np.array(INIT_POSITION)
    cost = 0
    i = 0
    trajectory = trajectory[:PREDICTION_HORIZON]
    while i < len(trajectory):
        point = trajectory[i]
        print('first cost', cost)
        cost += get_distance(curr_point[0], curr_point[1], point[0], point[1])
        curr_point += u[i]

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

def MPC(U_limit: float, path_to_traverse: List[np.array], init_position: np.array):
    """
    The main function of the MPC
    Input:
        U_limit: the limit of the control input
        path_to_traverse: the path to traverse
        init_position: the initial position of the agent
    Output:
    """
    global PATH, INIT_POSITION

    U_limit = 0.1
    PATH = [np.array([0, 0]), np.array([1, 0]), np.array([2, 0])]
    INIT_POSITION = (-1.0, 0.0)

    cons = (
        {
            'type': 'ineq',
            'fun': lambda u: u[0] + U_limit
        },
        {
            'type': 'ineq',
            'fun': lambda u: -u[0] + U_limit
        },
        {
            'type': 'ineq',
            'fun': lambda u: u[1] + U_limit
        },
        {
            'type': 'ineq',
            'fun': lambda u: -u[1] + U_limit
        },
        {
            'type': 'ineq',
            'fun': lambda u: u[2] + U_limit
        },
        {
            'type': 'ineq',
            'fun': lambda u: -u[2] + U_limit
        },
        {
            'type': 'ineq',
            'fun': lambda u: u[3] + U_limit
        },
        {
            'type': 'ineq',
            'fun': lambda u: -u[3] + U_limit
        },
        {
            'type': 'ineq',
            'fun': lambda u: u[4] + U_limit
        },
        {
            'type': 'ineq',
            'fun': lambda u: -u[4] + U_limit
        },
        {
            'type': 'ineq',
            'fun': lambda u: u[5] + U_limit
        },
        {
            'type': 'ineq',
            'fun': lambda u: -u[5] + U_limit
        },
        {
            'type': 'ineq',
            'fun': lambda u: u[6] + U_limit
        },
        {
            'type': 'ineq',
            'fun': lambda u: -u[6] + U_limit
        },
        {
            'type': 'ineq',
            'fun': lambda u: u[7] + U_limit
        },
        {
            'type': 'ineq',
            'fun': lambda u: -u[7] + U_limit
        },
        {
            'type': 'ineq',
            'fun': lambda u: u[8] + U_limit
        },
        {
            'type': 'ineq',
            'fun': lambda u: -u[8] + U_limit
        },
        {
            'type': 'ineq',
            'fun': lambda u: u[9] + U_limit
        },
        {
            'type': 'ineq',
            'fun': lambda u: -u[9] + U_limit
        }
        
    )

    # make PREDICTION_HORIZON control inputs, each of which is a 2D vector
    u0 = np.zeros((PREDICTION_HORIZON, 2))
    a = minimize(to_be_optimized, u0, constraints=cons,method = "SLSQP", options={'maxiter': 1000})

    u = a.x.reshape((PREDICTION_HORIZON, 2))

    curr_position = np.array(INIT_POSITION)
    for i in range(PREDICTION_HORIZON):
        curr_position += u[i]

    return u



if __name__ == "__main__":
    # PATH = [np.array([0, 0]), np.array([1, 0]), np.array([2, 0])]
    # INIT_POSITION = (0.5, 0.2)
    # d(None)
    
    U_limit = 0.1
    PATH = [np.array([0, 0]), np.array([1, 0]), np.array([2, 0])]
    INIT_POSITION = (0.0, 0.3)

    """
    cons = ({
        'type': 'ineq',
        'fun': lambda u: u[0] - 0.1
    })
    for i in range(PREDICTION_HORIZON):
        cons += ({'type': 'ineq', 'fun': lambda u: u[i] + U_limit},
                 {'type': 'ineq', 'fun': lambda u: -u[i] + U_limit})
    """
    

    # make PREDICTION_HORIZON control inputs, each of which is a 2D vector
    u0 = np.array([0]*PREDICTION_HORIZON*2)
    bounds = [(-U_limit, U_limit)]*PREDICTION_HORIZON*2
    print(u0, bounds)
    a = minimize(to_be_optimized, u0, method = "SLSQP", options={'maxiter': 1000}, bounds=bounds)
    print(a)

    u = a.x.reshape((PREDICTION_HORIZON, 2))
    print(u)

    # WRITE THE PATH!
    print("PRINTING PATH")
    curr_position = np.array(INIT_POSITION)
    print(curr_position)
    for i in range(PREDICTION_HORIZON):
        curr_position += u[i]
        print(curr_position)

    print("the cost of the path is: ", dd(u))
    



