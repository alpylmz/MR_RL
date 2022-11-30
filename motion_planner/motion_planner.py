import random
import math
from typing import List, Tuple
from tqdm import tqdm
import matplotlib.pyplot as plt
from matplotlib import patches

from .utils import get_distance, intersect


class Node:
    """
    A node in the RRT tree.
    """
    def __init__(self, x, y, parent, cost):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = cost

    def __repr__(self):
        return f"Node({self.x}, {self.y}, {self.parent})"

    def change_parent(self, new_parent):
        self.parent = new_parent

    def get_parent(self):
        return self.parent

    def set_parent(self, parent):
        self.parent = parent

    def get_position(self):
        return self.x, self.y

class RRT:
    """
    A class for the RRT* algorithm.
    """

    def __init__(
                self, 
                start_x: float, 
                start_y: float, 
                goal_x: float, 
                goal_y: float,
                step_size: float,
                rewire_distance: float,
                max_iter: int,
                env_min_x: float,
                env_min_y: float,
                env_width: float,
                env_height: float,
                obstacles: List[Tuple[float, float, float, float]],
                ):
        """
        Initializes the RRT* algorithm.
        Inputs:
            start_x: The x coordinate of the start point.
            start_y: The y coordinate of the start point.
            goal_x: The x coordinate of the goal point.
            goal_y: The y coordinate of the goal point.
            step_size: The distance between each step
            max_iter: The maximum number of iterations
            env_min_x: The minimum x coordinate of the environment
            env_min_y: The minimum y coordinate of the environment
            env_width: The width of the environment
            env_height: The height of the environment
            obstacles: A list of obstacles. Currently only rectangles are supported.
                    The tuple is [min_x, min_y, width, height]
        """
        self.nodes = [Node(start_x, start_y, None, 0)]
        self.start_x = start_x
        self.start_y = start_y
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.step_size = step_size
        self.rewire_distance = rewire_distance
        self.max_iter = max_iter
        self.env_min_x = env_min_x
        self.env_min_y = env_min_y
        self.env_width = env_width
        self.env_height = env_height
        self.obstacles = obstacles

    def get_nearest_node(self, x, y):
        """
        Returns the nearest node to the given point.
        """
        min_distance = float("inf")
        nearest_node = None
        for node in self.nodes:
            distance = get_distance(node.x, node.y, x, y)
            if distance < min_distance:
                min_distance = distance
                nearest_node = node
        return nearest_node

    def get_random_point(self):
        """
        Returns a random point in the environment.
        """
        x = random.uniform(self.env_min_x, self.env_min_x + self.env_width)
        y = random.uniform(self.env_min_y, self.env_min_y + self.env_height)
        return x, y

    def get_new_point(self, nearest_node, x, y):
        """
        Returns a new point that is a step_size distance away from the nearest node.
        """
        theta = math.atan2(y - nearest_node.y, x - nearest_node.x)
        new_x = nearest_node.x + self.step_size * math.cos(theta)
        new_y = nearest_node.y + self.step_size * math.sin(theta)
        return new_x, new_y

    def intersects_rectangle(self, x1, y1, x2, y2):
        """
        Returns true if the line segment between the two points intersects an rectangle.
        """
        for obstacle in self.obstacles:
            min_x = obstacle[0]
            min_y = obstacle[1]
            max_x = obstacle[0] + obstacle[2]
            max_y = obstacle[1] + obstacle[3]
            if  intersect((x1, y1), (x2, y2), (min_x, min_y), (max_x, min_y)) or \
                intersect((x1, y1), (x2, y2), (max_x, min_y), (max_x, max_y)) or \
                intersect((x1, y1), (x2, y2), (max_x, max_y), (min_x, max_y)) or \
                intersect((x1, y1), (x2, y2), (min_x, max_y), (min_x, min_y)):
                return True
        return False
    
    def get_path(self, node):
        """
        Returns the path from the start to the given node.
        """
        path = []
        while node is not None:
            path.append(node.get_position())
            node = node.get_parent()
        return path[::-1]

    def RRTStar(self, plot = True, rewire = True, repeat = True, plot_only_path = True):
        """
        Runs the RRT* algorithm.
        """
        iteration_count = self.max_iter
        for _ in tqdm(range(iteration_count)):
            x, y = self.get_random_point()
            nearest_node = self.get_nearest_node(x, y)
            new_x, new_y = self.get_new_point(nearest_node, x, y)
            if self.intersects_rectangle(nearest_node.x, nearest_node.y, new_x, new_y):
                iteration_count += 1 # If the new point intersects an obstacle, try again
                continue
            new_node = Node(new_x, new_y, nearest_node, nearest_node.cost + get_distance(nearest_node.x, nearest_node.y, new_x, new_y))
            self.nodes.append(new_node)
            if get_distance(new_x, new_y, self.goal_x, self.goal_y) < self.step_size:
                path = self.get_path(new_node)
                if plot:
                    self.plot(path, plot_only_path)
                return path
            # Rewire
            if rewire:
                for node in self.nodes:
                    if node == new_node:
                        continue
                    if get_distance(node.x, node.y, new_x, new_y) < self.rewire_distance and \
                        node.cost > new_node.cost + get_distance(node.x, node.y, new_x, new_y):
                        if self.intersects_rectangle(new_x, new_y, node.x, node.y):
                            continue
                        node.set_parent(new_node)
                        node.cost = new_node.cost + get_distance(node.x, node.y, new_x, new_y)      

        if repeat:
            # delete all nodes except first one
            self.nodes = self.nodes[:1]
            return self.RRTStar()
        if plot:
            self.plot(None)
        return None

    def plot(self, path, plot_only_path = True):
        """
        Plots the environment and the path.
        """
        fig, ax = plt.subplots()
        ax.set_xlim(self.env_min_x, self.env_min_x + self.env_width)
        ax.set_ylim(self.env_min_y, self.env_min_y + self.env_height)

        # plot start and goal
        ax.scatter(self.start_x, self.start_y, c='g', s=50)
        ax.scatter(self.goal_x, self.goal_y, c='r', s=50)

        # print all nodes
        if not plot_only_path:
            for node in self.nodes:
                if node.parent is not None:
                    ax.plot([node.x, node.parent.x], [node.y, node.parent.y], c='b')

        for obstacle in self.obstacles:
            # draw red rectangle
            ax.add_patch(patches.Rectangle((obstacle[0], obstacle[1]), obstacle[2], obstacle[3], color='r'))
        if path is not None:
            x, y = zip(*path)
            ax.plot(x, y, color="black")
            ax.scatter(self.start_x, self.start_y, color="green")
            ax.scatter(self.goal_x, self.goal_y, color="green")
            plt.show()
        else:
            print("No path found")
            plt.show()

def main():
    obstacles = [
        (-5, -5, 5, 5), 
        (5, 5, 5, 5),
        (0.1, 0.1, 4.8, 4.8)]
    start_x = -2.5
    start_y = 7.5
    goal_x = 7.5
    goal_y = -7.5
    step_size = 0.2
    max_iter = 2000
    env_min_x = -10
    env_min_y = -10
    env_width = 20
    env_height = 20
    rrt = RRT(
        start_x, 
        start_y, 
        goal_x, 
        goal_y, 
        step_size, 
        max_iter, 
        env_min_x, 
        env_min_y, 
        env_width,
        env_height, 
        obstacles)
    
    path = rrt.RRTStar(rewire = True, repeat = True, plot_only_path = True)
    print(path)




if __name__ == "__main__":
    main()