from scipy.optimize import minimize
import numpy as np

from motion_planner.utils import get_distance

from typing import List, Tuple
from shapely.geometry import LineString, Point

STEP_SIZE = 0.1

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
        w_u: float = 1,
        w_p: float = 1,
        prediction_horizon: int = 5,
        ):
        self.path = path
        self.curr_position = curr_position
        self.w_u = w_u
        self.w_p = w_p
        self.prediction_horizon = prediction_horizon
        self.path = self.discretize_path()

    def discretize_path(self):
        """
        This function is used to discretize the path.
        """
        # First of all, find the closest point from the current position to the line that is defined by path[0] and path[1]
        # but, if there is only one point in the path, make the line from the current position to the path[0]
        p = Point(self.curr_position)
        if len(self.path) == 1:
            line = LineString([self.curr_position, self.path[0]])
        else:
            line = LineString([self.path[0], self.path[1]])
        closest_point = line.interpolate(line.project(p))

        temp_path = []
        # If the closest point is not on the line, that means that the robot is not on the path
        if not line.contains(closest_point):
            # here we need to discretize the path from the current position to path[0]
            # increase the point to the path[0] by step size
            # and add the point to the path
            curr_point = np.array(self.curr_position)
            next_point = np.array(self.path[0])

        else:
            # here we need to discretize the path from the closest position to path[1]
            curr_point = np.array(closest_point)
            next_point = np.array(self.path[1])

        while get_distance(curr_point[0], curr_point[1], next_point[0], next_point[1]) > STEP_SIZE:
            curr_point += STEP_SIZE * (next_point - curr_point) / get_distance(curr_point[0], curr_point[1], next_point[0], next_point[1])
            temp_path.append(curr_point.copy())

        return temp_path

    def set_position(self, position: Tuple[float, float]):
        """
        Sets the current position of the robot.
        Parameters:
            position: Tuple[float, float] - the current position of the robot
        """
        self.curr_position = position

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
        while i < self.prediction_horizon:
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
        # TODO: np.sum is wrong here!
        return self.w_u * np.sum(u) + self.w_p * self.path_cost(u)

    def constraint(self, u, i):
        """
        This function is used to generate the constraints for the optimization.
        ||u_x, u_y|| <= 0.1
        Input:
            u: control input
            i: the index of the constraint
        """
        return 0.1 - np.sqrt(u[2*i]**2 + u[2*i+1]**2)

    def run(self, curr_pose: Tuple[float, float] = None):
        if curr_pose:
            self.curr_position = curr_pose

        U_limit = 0.1
        u0 = [0.0]*self.prediction_horizon*2
        bounds = [(-U_limit, U_limit)]*self.prediction_horizon*2
        cons = (
            {'type': 'ineq', 'fun': lambda u: self.constraint(u, 0)},
            {'type': 'ineq', 'fun': lambda u: self.constraint(u, 1)},
            {'type': 'ineq', 'fun': lambda u: self.constraint(u, 2)},
            {'type': 'ineq', 'fun': lambda u: self.constraint(u, 3)},
            {'type': 'ineq', 'fun': lambda u: self.constraint(u, 4)},
        )
        a = minimize(
            self.objective_function, 
            u0, 
            method = "SLSQP", 
            constraints = cons, 
            options = {'maxiter': 1000}, 
            bounds = bounds
            )
        
        u = a.x
        cost = self.path_cost(u)

        return cost, u

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
        




