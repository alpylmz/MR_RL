from kmeansinfo import KmeansInfo
import numpy as np

def pick_closest_center(
    blue_matrix: np.ndarray,
    yellow_matrix: np.ndarray,
    background_matrix0: np.ndarray,
    background_matrix1: np.ndarray,
    red_matrix0: np.ndarray,
    #red_matrix1: np.ndarray
    ) -> np.ndarray:
    """
    Pick the closest center to the color.
    I dont want to use yellow's red center currently!
    """

    distance_to_blue_matrix = np.linalg.norm(blue_matrix , axis=2)
    distance_to_yellow_matrix = np.linalg.norm(yellow_matrix , axis=2)
    distance_to_background0_matrix = np.linalg.norm(background_matrix0 , axis=2)
    distance_to_background1_matrix = np.linalg.norm(background_matrix1 , axis=2)
    distance_to_red0_matrix = np.linalg.norm(red_matrix0 , axis=2)
    #distance_to_red1_matrix = np.linalg.norm(red_matrix1 , axis=2)

    min_distance_matrix = np.ndarray((len(distance_to_blue_matrix), len(distance_to_blue_matrix[0])))
    result_matrix = np.ndarray((len(distance_to_blue_matrix), len(distance_to_blue_matrix[0]), 3), dtype=np.uint8)
    np.minimum(distance_to_blue_matrix, distance_to_yellow_matrix, out=min_distance_matrix)
    np.minimum(min_distance_matrix, distance_to_background0_matrix, out=min_distance_matrix)
    np.minimum(min_distance_matrix, distance_to_background1_matrix, out=min_distance_matrix)
    np.minimum(min_distance_matrix, distance_to_red0_matrix, out=min_distance_matrix)
    #np.minimum(min_distance_matrix, distance_to_red1_matrix, out=min_distance_matrix)

    height_of_matrix = len(distance_to_blue_matrix)
    width_of_matrix = len(distance_to_blue_matrix[0])
    for i in range(height_of_matrix):
        for j in range(width_of_matrix):
            if distance_to_blue_matrix[i][j] == min_distance_matrix[i][j]:
                result_matrix[i][j] = [255, 0, 0]
            elif distance_to_yellow_matrix[i][j] == min_distance_matrix[i][j]:
                result_matrix[i][j] = [0, 255, 255]
            elif distance_to_background0_matrix[i][j] == min_distance_matrix[i][j] or \
                distance_to_background1_matrix[i][j] == min_distance_matrix[i][j]:
                result_matrix[i][j] = [0, 0, 0]
            elif distance_to_red0_matrix[i][j] == min_distance_matrix[i][j]:
                result_matrix[i][j] = [0, 0, 255]
            #elif distance_to_red0_matrix[i][j] == min_distance_matrix[i][j] or \
            #    distance_to_red1_matrix[i][j] == min_distance_matrix[i][j]:
            #    result_matrix[i][j] = [0, 0, 255]

    return result_matrix









