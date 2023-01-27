from gekko import GEKKO
from MR_logger import mr_logger as log

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
        log.debug(f"input path: {path}")
        self.path = self.discretize_path(path, curr_position)[:MPC_PREDICTION_HORIZON]
        log.debug(f"discretized path: {self.path}")
        self.a0 = a0

    def discretize_path(
        self, 
        path: List[Tuple[float, float]],
        curr_pos: Tuple[float, float], 
        ) -> List[Tuple[float, float]]:
        
        if len(path) == 0:
            log.warn("The path returned empty from the discretize_path function!")
            return []
        
        return_path = []
        path_index = 0
        curr_point = np.array(curr_pos)
        while len(return_path) < MPC_PREDICTION_HORIZON and path_index < len(path):
            next_point = np.array(path[path_index])
            while get_distance(curr_point[0], curr_point[1], next_point[0], next_point[1]) > STEP_SIZE:
                curr_point += STEP_SIZE * (next_point - curr_point) / get_distance(curr_point[0], curr_point[1], next_point[0], next_point[1])
                return_path.append(curr_point.copy())
            return_path.append(next_point.copy())
            path_index += 1
            
        return return_path[:MPC_PREDICTION_HORIZON+1]

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

        q = [(
            m.Const(value = self.curr_position[0]),
            m.Const(value = self.curr_position[1])
            )]
            
            
        for _ in range(prediction_horizon - 1):
            q.append(
                (
                    m.Var(lb = ENV_MIN_X, ub = ENV_MIN_X + ENV_WIDTH),
                    m.Var(lb = ENV_MIN_Y, ub = ENV_MIN_Y + ENV_HEIGHT) 
                ))

        for i in range(1, prediction_horizon):
            m.Equation(
                q[i][0] == q[i - 1][0] + f[i - 1] * a0 * m.cos(alpha[i - 1])
            )
            m.Equation(
                q[i][1] == q[i - 1][1] + f[i - 1] * a0 * m.sin(alpha[i - 1])
            )

        # lets put a minimum velocity to force the optimizer to find a good solution
        for i in range(prediction_horizon):
            m.Equation(f[i] * a0 >= MPC_MINIMUM_SPEED)

        # objective function
        m.Obj(
            MPC_W_U * np.sum([(a0 * f[i])**2 for i in range(prediction_horizon)])
            + 
            MPC_W_P * np.sum((np.array(np.reshape(q, (prediction_horizon, 2))) - np.array(self.path))**2)
        )

        m.options.SOLVER = 3
        try:
            m.solve(disp=False)
        except:
            log.error("The optimization failed!")
            log.error("The path is: " + str(self.path) + " and the current position is: " + str(self.curr_position))
            raise Exception("The optimization failed!")
        log.debug(f'the used path is: {self.path}')
        log.debug(f'lets check the f {f[0].value[0]}')
        log.debug('lets check the positions')
        log.debug(f"{self.curr_position[0]}, {self.curr_position[1]}")
        for i in range(1, prediction_horizon):
            log.debug(f"{q[i][0].value[0]}, {q[i][1].value[0]}")
        
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
        




