import numpy as np
from const import *
import random

def generate_random_frequencies():
    while True:
        frequencies = []
        # generate a random N*N frequency matrix
        for i in range(NUMBER_OF_AGENTS):
            lst = []
            for j in range(NUMBER_OF_AGENTS):
                lst.append(random.randint(1, 20))
            frequencies.append(lst)

        # calculate determinant to make sure that the frequencies are linearly independent
        det = np.linalg.det(frequencies)
        if det == 0:
            continue
        break
    return frequencies

def generate_random_positions():
    while True:
        positions = []
        for i in range(NUMBER_OF_AGENTS):
            positions.append([random.randint(0, 400), random.randint(0, 400)])

        # check the distances between the agents, make sure they are not too close
        too_close = False
        for i in range(NUMBER_OF_AGENTS):
            for j in range(i + 1, NUMBER_OF_AGENTS):
                if np.linalg.norm(np.array(positions[i]) - np.array(positions[j])) < 1:
                    too_close = True
                    break
            if too_close:
                break
        if too_close:
            continue
        return positions

