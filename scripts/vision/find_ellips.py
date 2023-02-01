import cv2
import numpy as np
import enum
import tqdm
from sympy import Polygon
import itertools
from parallelbar import progress_map



img = cv2.imread("images/clustered_frame_23_jan.png")

blue_masked_img = np.zeros((len(img), len(img[0]), 3), np.uint8)
yellow_masked_img = np.zeros((len(img), len(img[0]), 3), np.uint8)
red_masked_img = np.zeros((len(img), len(img[0]), 3), np.uint8)

img_height = len(img)
img_width = len(img[0])
# mask out every color other than yellow, blue or red!
# writing this code in an optimized way, so it is a bit cryptic
for i in range(img_height):
    for j in range(img_width):
        # currently these are the values set in the kmeans_train for yellow!
        if img[i][j][2] == 255:
            if img[i][j][1] == 255:
                # color is [0, 255, 255]
                yellow_masked_img[i][j] = [255, 255, 255]
                # no need for these, since they are already black
                #blue_masked_img[i][j] = [0, 0, 0]
                #red_masked_img[i][j] = [0, 0, 0]
                img[i][j] = [0, 127, 127]

            else:
                # color is [0, 0, 255]
                red_masked_img[i][j] = [255, 255, 255]
                # no need for these, since they are already black
                #yellow_masked_img[i][j] = [0, 0, 0]
                #blue_masked_img[i][j] = [0, 0, 0]
                img[i][j] = [0, 0, 127]

        elif img[i][j][0] == 255:
            # color is [255, 0, 0]
            
            blue_masked_img[i][j] = [255, 255, 255]
            # no need for these, since they are already black
            #yellow_masked_img[i][j] = [0, 0, 0]
            #red_masked_img[i][j] = [0, 0, 0]
            img[i][j] = [127, 0, 0]


class ContourType(enum.Enum):
    ELLIPSE = 1
    CONVEX_HULL = 2
class ContourColor(enum.Enum):
    YELLOW = 1
    BLUE = 2
    RED = 3

def fit_contour(
        img: np.ndarray, 
        original_img: np.ndarray, 
        contour_type: ContourType, 
        contour_color: ContourColor) -> None:
        
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(img, 127, 255, 0)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contour_color == ContourColor.YELLOW:
        draw_color = (0, 255, 255)
    elif contour_color == ContourColor.BLUE:
        draw_color = (255, 0, 0)
    elif contour_color == ContourColor.RED:
        draw_color = (0, 0, 255)

    shapes = []
    for contour in contours:
        try:
            if contour_type == ContourType.ELLIPSE:
                ellipse = cv2.fitEllipse(contour)
                cv2.ellipse(original_img, ellipse, draw_color, 1)
                shapes.append(ellipse)
            elif contour_type == ContourType.CONVEX_HULL:
                hull = cv2.convexHull(contour)
                cv2.drawContours(original_img, [hull], 0, draw_color, 1)
                shapes.append(hull)
        except cv2.error:
            continue

    return shapes

yellow_shapes = fit_contour(yellow_masked_img, img, ContourType.CONVEX_HULL, ContourColor.YELLOW)
blue_shapes = fit_contour(blue_masked_img, img, ContourType.CONVEX_HULL, ContourColor.BLUE)
red_shapes = fit_contour(red_masked_img, img, ContourType.CONVEX_HULL, ContourColor.RED)

def find_differenting_blues(
        blue_shapes: list, 
        red_shapes: list, 
        img: np.ndarray
        ) -> list:
    # TODO: this is a very naive implementation, it should be optimized
    differenting_blues = []
    for blue_shape in tqdm.tqdm(blue_shapes):
        for red_shape in red_shapes:
            for point in red_shape:
                if cv2.pointPolygonTest(blue_shape, (float(point[0][0]), float(point[0][1])), False) >= 0:
                    differenting_blues.append(blue_shape)
                    cv2.drawContours(img, [blue_shape], 0, (255, 255, 255), 1)
                    break
    return differenting_blues

"""
print("starting to find differenting blues")
differentiated_blues = find_differenting_blues(blue_shapes, red_shapes, img)
print("found {} differenting blues".format(len(differentiated_blues)))
print(differentiated_blues)
"""

def is_blue_shape_differenting(
        blue_shape: np.ndarray, 
        red_shapes: list,
        ) -> bool:

    for red_shape in red_shapes:
        for point in red_shape:
            if cv2.pointPolygonTest(blue_shape, (float(point[0][0]), float(point[0][1])), False) >= 0:
                return True
    return False


def find_distance_between_polygons(
        polygon1: np.ndarray, 
        polygon2: np.ndarray) -> float:
    
    # TODO: this is not an optimized implementation (probably), it should be optimized
    #poly1 = Polygon([(float(point[0][0]), float(point[0][1])) for point in polygon1])
    #poly2 = Polygon([(float(point[0][0]), float(point[0][1])) for point in polygon2])

    tuple1 = tuple([(float(point[0][0]), float(point[0][1])) for point in polygon1])
    tuple2 = tuple([(float(point[0][0]), float(point[0][1])) for point in polygon2])
    # without this "pointer" operator, sympy gives an error
    # this is the first time I used this operator in Python, so I don't know if it's a good practice
    # https://stackoverflow.com/a/48583875
    poly1 = Polygon(*tuple1)
    poly2 = Polygon(*tuple2)

    return poly1.distance(poly2)

def calculate_costs_parallel(
    x
    #yellow_shapes: list,
    #blue_shapes: list,
):
    yellow_shapes, blue_shapes = x
    yellow_shapes, blue_shapes = x
    costs = []
    for yellow_shape in yellow_shapes:
        print("checking if blue shape is differenting")
        if len(yellow_shape) <= 2:
            continue
        for blue_shape in blue_shapes:
            costs.append(
                (
                    yellow_shape, 
                    blue_shape, 
                    find_distance_between_polygons(yellow_shape, blue_shape)
                )
            )
    return costs
    
# cost for a single yellow cell can be just distance
# for now just calculate costs for all blue cells
def calculate_costs(
        yellow_shapes: list, 
        blue_shapes: list, 
        red_shapes: list
        ) -> list:
    
    # for optimization
    # find differenting blues
    differenting_blues = []
    for blue_shape in blue_shapes:
        if is_blue_shape_differenting(blue_shape, red_shapes):
            differenting_blues.append(blue_shape)

    costs = []
    print("starting to calculate costs")
    # write the below loop in parallel 4 cpus


    # run this function in parallel 4 cpus
    import multiprocessing
    import pickle
    func_inputs = [(yellow_shapes, differenting_blues)]
    """
    with multiprocessing.Pool(processes=8) as pool:
        results = tqdm.tqdm(pool.imap_unordered(calculate_costs_parallel, func_inputs), total=len(yellow_shapes))
        results = list(results)
        costs = results[0]
    """
    
    for yellow_shape in tqdm.tqdm(yellow_shapes):
        if len(yellow_shape) <= 2:
            continue
        for blue_shape in differenting_blues:
            costs.append(
                (
                    yellow_shape, 
                    blue_shape, 
                    find_distance_between_polygons(yellow_shape, blue_shape)
                )
            )
    
    # save this to a file with pickle
    with open('costs.pickle', 'wb') as f:
        pickle.dump(costs, f)
    
    return costs

# find the costs for each blue cell
costs = calculate_costs(yellow_shapes, blue_shapes, red_shapes)
# change the format to be more convenient
# [blue_cell, [all_costs]]
costs = [[cost[1], [cost[2]]] for cost in costs]
# group the costs by blue cell
costs = [[key, [cost[1] for cost in group]] for key, group in itertools.groupby(costs, lambda cost: cost[0])]
print(costs)




# Show keypoints
cv2.imshow("", img)
#save img
cv2.imwrite("images/differentiating_blues_30_jan.png", img)
#cv2.imshow("Keypoints", im_with_keypoints)
cv2.waitKey(0)
    
