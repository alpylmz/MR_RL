import cv2
import numpy as np
import enum

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
    """
    This function has lots of commented out places,
    do not delete them, you can use it for debugging.
    """

    shapes = []
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(img, 127, 255, 0)
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    """
    if contour_color == ContourColor.YELLOW:
        draw_color = (0, 255, 255)
    elif contour_color == ContourColor.BLUE:
        draw_color = (255, 0, 0)
    elif contour_color == ContourColor.RED:
        draw_color = (0, 0, 255)
    """

    for contour in contours:
        try:
            if contour_type == ContourType.ELLIPSE:
                ellipse = cv2.fitEllipse(contour)
                #cv2.ellipse(original_img, ellipse, draw_color, 1)
                shapes.append(ellipse)
            elif contour_type == ContourType.CONVEX_HULL:
                hull = cv2.convexHull(contour)
                #cv2.drawContours(original_img, [hull], 0, draw_color, 1)
                shapes.append(hull)
        except cv2.error:
            continue

    return shapes




def execute_cell_separation_from_img(img: np.ndarray) -> np.ndarray:
    
    #img = cv2.imread("images/clustered_frame_23_jan.png")    
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

    """
    fit_contour(yellow_masked_img, img, ContourType.CONVEX_HULL, ContourColor.YELLOW)
    fit_contour(blue_masked_img, img, ContourType.CONVEX_HULL, ContourColor.BLUE)
    fit_contour(red_masked_img, img, ContourType.CONVEX_HULL, ContourColor.RED)
    """
    res = \
        fit_contour(yellow_masked_img, None, ContourType.CONVEX_HULL, None) + \
        fit_contour(blue_masked_img, None, ContourType.CONVEX_HULL, None) + \
        fit_contour(red_masked_img, None, ContourType.CONVEX_HULL, None)
    
    print(res)


# Show keypoints
#cv2.imshow("", img)
#cv2.imshow("Keypoints", im_with_keypoints)
#cv2.waitKey(0)
    
