from kmeansinfo import KmeansInfo
import numpy as np

def pick_closest_center(kmeans: KmeansInfo, color: list):
    # TODO NEED TO OPTIMIZE THIS!!!!
    """
    Pick the closest center to the color.
    I dont want to use yellow's red center currently!
    """

    distance_to_blue = np.linalg.norm(np.array(color) - np.array(kmeans.blue_center))
    distance_to_yellow = np.linalg.norm(np.array(color) - np.array(kmeans.yellow_center))
    distance_to_background0 = np.linalg.norm(np.array(color) - np.array(kmeans.background_centers[0]))
    distance_to_background1 = np.linalg.norm(np.array(color) - np.array(kmeans.background_centers[1]))
    distance_to_red0 = np.linalg.norm(np.array(color) - np.array(kmeans.red_centers[0]))
    #distance_to_red1 = np.linalg.norm(np.array(color) - np.array(kmeans.red_centers[1]))

    # find the minimum distance
    min_distance = min(
        distance_to_blue, 
        distance_to_yellow, 
        distance_to_background0, 
        distance_to_background1, 
        distance_to_red0, 
        #distance_to_red1
        )

    # return the color that corresponds to the minimum distance
    if min_distance == distance_to_blue:
        # blue is whole blue!
        return [255, 0, 0]
    elif min_distance == distance_to_yellow:
        # yellow is whole yellow!
        return [0, 255, 255]
    elif min_distance == distance_to_background0 or min_distance == distance_to_background1:
        # background is black!!!!
        return [0, 0, 0]
    elif min_distance == distance_to_red0:
        # red is whole red!
        return [0, 0, 255]
    else:
        raise Exception("something went wrong in pick_closest_center")








