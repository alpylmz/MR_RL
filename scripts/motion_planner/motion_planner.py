import numpy as np
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
                image_frame: np.ndarray = None
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
        self.image_frame = image_frame
          
    def get_nearest_node(self, x, y):
        """
        Returns the nearest node to the given point.
        """
        min_distance = float("inf")
        nearest_node = None
        for node in self.nodes:
            fake_distance = (node.x - x)**2 + (node.y - y)**2
            #distance = get_distance(node.x, node.y, x, y)
            if fake_distance < min_distance:
                min_distance = fake_distance
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

    def add_node(self, node):
        """
        Adds a node to the tree.
        """
        self.nodes.append(node)
        #self.quad_tree.insert(node, (node.x, node.y, node.x, node.y))

    def get_nearest_node_fast(self, x, y):
        """
        Returns the nearest node to the given point.
        """
        pass


        
    def intersects_polygon(self, x1, y1, x2, y2):
        """
        Returns true if the line segment between the two points intersects an polygon.
        """
        # Check the line segment coordinates in the image frame
        # if the line segment's coordinates' values are all 0, then it does not intersect with any polygon

        start_position = (int(x1), int(y1))
        end_position = (int(x2), int(y2))

        distance_between_points = np.sqrt((end_position[0] - start_position[0])**2 + (end_position[1] - start_position[1])**2)

        # Get the coordinates of the line segment
        x, y = np.linspace(start_position[0], end_position[0], int(distance_between_points*10)), np.linspace(start_position[1], end_position[1], int(distance_between_points * 10))
        x, y = x.astype(int), y.astype(int)
        points = np.vstack((x, y)).T

        # Check if the line segment intersects with any polygon
        for point in points:
            if (point[0] < 0) or (point[0] >= self.image_frame.shape[0]) or (point[1] < 0) or (point[1] >= self.image_frame.shape[1]):
                return True
            if (self.image_frame[point[0]][point[1]] != [0, 0, 0]).all():
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
        for i in tqdm(range(iteration_count)):
            #if (i % 1000) == 0:
            #    self.plot(None, plot_only_path = False)
            x, y = self.get_random_point()
            nearest_node = self.get_nearest_node(x, y)
            new_x, new_y = self.get_new_point(nearest_node, x, y)
            if self.intersects_polygon(nearest_node.x, nearest_node.y, new_x, new_y):
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
                parent_node = new_node.get_parent().get_parent()
                # TODO I may also check if the parent node is close enough to the new node, it actually changes a whole lot!
                if parent_node is None:
                    continue
                if self.intersects_polygon(parent_node.x, parent_node.y, new_x, new_y):
                    continue
                else:
                    new_node.set_parent(parent_node)
                    new_node.cost = parent_node.cost + get_distance(parent_node.x, parent_node.y, new_x, new_y)
                    # delete intermediate node only if it is not a parent of another node! TODO
                    #self.nodes.remove(new_node.get_parent())
                """
                for node in self.nodes:
                    if node == new_node:
                        continue
                    # TODO: you forgot to delete the intermediate node!!!! 
                    if get_distance(node.x, node.y, new_x, new_y) < self.rewire_distance and \
                        node.cost > new_node.cost + get_distance(node.x, node.y, new_x, new_y):
                        if self.intersects_polygon(new_x, new_y, node.x, node.y):
                            continue
                        node.set_parent(new_node)
                        node.cost = new_node.cost + get_distance(node.x, node.y, new_x, new_y)  
                        continue   
                """
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
            print("Plotting all nodes")
            for node in self.nodes:
                if node.parent is not None:
                    ax.plot([node.x, node.parent.x], [node.y, node.parent.y], c='b')

        for obstacle in self.obstacles:
            # draw polygon
            ax.add_patch(patches.Polygon(obstacle, closed=True, fill=True, color='gray'))
        if path is not None:
            x, y = zip(*path)
            ax.plot(x, y, color="black")
            ax.scatter(self.start_x, self.start_y, color="green")
            ax.scatter(self.goal_x, self.goal_y, color="green")
            plt.show()
        else:
            #print("No path found")
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