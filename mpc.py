from gekko import GEKKO

import numpy as np
from consts import *

from motion_planner.utils import get_distance

from typing import List, Tuple
from shapely.geometry import LineString, Point

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
        a0: float,
        ):
        self.curr_position = curr_position
        print("given path: ", path)
        self.path = self.discretize_path(path, curr_position)[:MPC_PREDICTION_HORIZON - 1]
        self.a0 = a0

    def discretize_path(self, path: List[Tuple[float, float]], curr_pos: Tuple[float, float], len_of_path = 0):
        """
        This function is used to discretize the path.
        """
        # First of all, find the closest point from the current position to the line that is defined by path[0] and path[1]
        # but, if there is only one point in the path, make the line from the current position to the path[0]
        if len(path) == 0:
            return []
        elif len(path) == 1:
            line = LineString([curr_pos, path[0]])
        else:
            line = LineString([path[0], path[1]])
        closest_point = line.interpolate(line.project(Point(curr_pos)))

        temp_path = []
        # If the closest point is not on the line, that means that the robot is not on the path
        if not line.contains(closest_point):
            # here we need to discretize the path from the current position to path[0]
            curr_point = np.array(curr_pos)
            next_point = np.array(path[0])

        else:
            # here we need to discretize the path from the closest position to path[1]
            curr_point = np.array(closest_point)
            next_point = np.array(path[1])

        # add the points from curr point to next point step by step
        while get_distance(curr_point[0], curr_point[1], next_point[0], next_point[1]) > STEP_SIZE:
            curr_point += STEP_SIZE * (next_point - curr_point) / get_distance(curr_point[0], curr_point[1], next_point[0], next_point[1])
            temp_path.append(curr_point.copy())
        
        if (len(temp_path) + len_of_path) < MPC_PREDICTION_HORIZON:
            temp_path += self.discretize_path(path[1:], temp_path[-1], len_of_path=len_of_path + len(temp_path))

        return temp_path

    def path_cost(self, u: np.array):
        """
        Calculates the cost of the path.
        Parameters:
            u: np.array - the control inputs
        Returns:
            cost: float - the cost of the path
        """
        # lets make it go to the first point of the path
        curr_point = np.array(self.curr_position)
        cost = 0
        i = 0
        aim = self.path[0]
        calculated_path = []
        while i < MPC_PREDICTION_HORIZON:
            calculated_path.append(curr_point)
            cost += get_distance(curr_point[0], curr_point[1], aim[0], aim[1])**2
            curr_point[0] += u[2*i]
            curr_point[1] += u[2*i+1]
            i += 1

        return cost

    def objective_function(self, u):
        """
        The function to be optimized
        Input:
            u: control input
        """
        return MPC_W_U * np.sum(u) + MPC_W_P * self.path_cost(u)

    def constraint(self, u, i):
        """
        This function is used to generate the constraints for the optimization.
        ||u_x, u_y|| <= 0.1
        Input:
            u: control input
            i: the index of the constraint
        """
        return MPC_U_LIMIT - np.sqrt(u[2*i]**2 + u[2*i+1]**2)

    def run(self):
        prediction_horizon = min(MPC_PREDICTION_HORIZON, len(self.path))
        
        m = GEKKO(remote = False)
        a0 = self.a0

        # Define the variables
        # control input
        f = [m.Var(lb = 0.0, ub = MAX_F) for _ in range(prediction_horizon)]
        alpha = [m.Var(lb = -3.14, ub = 3.14) for _ in range(prediction_horizon)]

        """
        u = []
        for i in range(prediction_horizon):
            u.append(a0 * f[i] * np.cos(alpha[i]))
            u.append(a0 * f[i] * np.sin(alpha[i]))
        """

        # position
        q = []
        for i in range(prediction_horizon):
            q += [
                m.Var(lb = ENV_MIN_X, ub = ENV_MIN_X + ENV_WIDTH), 
                m.Var(lb = ENV_MIN_Y, ub = ENV_MIN_Y + ENV_HEIGHT)
                ] 

        # constraints
        # constraint on the input magnitude
        
        for i in range(prediction_horizon):
            m.Equation(
                a0 * f[i] <= MPC_U_LIMIT
            )
            m.Equation(
                a0 * f[i] >= -MPC_U_LIMIT
            )


        # constraint on the next positions
        for i in range(prediction_horizon):
            m.Equation(q[2*i] == q[2*i-2] + (a0 * f[i] * m.cos(alpha[i]) * TIME_STEP))
            m.Equation(q[2*i+1] == q[2*i-1] + (a0 * f[i] * m.sin(alpha[i]) * TIME_STEP))

        
        # objective function
        m.Obj(
            # MPC_W_U * np.sum([a0 * f[i] for i in range(prediction_horizon)])
            # + 
            MPC_W_P * np.sum((np.array(np.reshape(q, (prediction_horizon, 2))) - np.array(self.path))**2)
        )

        m.solve()
        print('the used path is: ', self.path)
        print('lets check the f')
        print(f[0].value[0])
        
        return np.array([
            f[i].value[0] for i in range(prediction_horizon)
            ]), \
            np.array([
            alpha[i].value[0] for i in range(prediction_horizon)
            ])


if __name__ == "__main__":
    
    path = [np.array([0, 0]), np.array([1, 0]), np.array([2, 0])]
    INIT_POSITION = (0.3, -0.2)

    mpc = MPC(path, INIT_POSITION)
    min_cost, best_u = mpc.run()

    print("the best u is: ", best_u)
    print("the best cost is: ", min_cost)

    # WRITE THE PATH!
    print("PRINTING PATH")
    curr_position = np.array(INIT_POSITION)
    print(curr_position)
    for i in range(5):
        curr_position[0] += best_u[2*i]
        curr_position[1] += best_u[2*i+1]
        print(curr_position)
        




