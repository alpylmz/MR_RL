import pickle
import cv2

from utils import pick_closest_center

kmeans_info = pickle.load(open("kmeans_info.pkl", "rb"))
frame_name = "images/original_frame_cut.png"


def visualize_clusters(kmeans_info, frame_name):
    print("reading image")
    img = cv2.imread(frame_name)

    if img is None:
        print("Image not found")
        exit()

    print("visualizing clusters")
    color_minus_blue = img - kmeans_info.blue_center
    color_minus_yellow = img - kmeans_info.yellow_center
    color_minus_background0 = img - kmeans_info.background_centers[0]
    color_minus_background1 = img - kmeans_info.background_centers[1]
    color_minus_red0 = img - kmeans_info.red_centers[0]
    #color_minus_red1 = img - kmeans_info.red_centers[1]
    
    img = pick_closest_center(
        color_minus_blue,
        color_minus_yellow,
        color_minus_background0,
        color_minus_background1,
        color_minus_red0,
        #color_minus_red1
    )


    #cv2.imshow("original", cv2.imread(frame_name))
    #cv2.imshow("clusters", img)
    # save img
    #cv2.imwrite("images/clustered_frame_23_jan.png", img)
    #cv2.waitKey(0)

    return img

visualize_clusters(kmeans_info, frame_name)

