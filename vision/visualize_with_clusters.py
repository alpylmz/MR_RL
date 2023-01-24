import pickle
import cv2
from tqdm import tqdm

from kmeansinfo import KmeansInfo
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
    for i in tqdm(range(len(img))):
        for j in range(len(img[i])):
            img[i][j] = pick_closest_center(kmeans_info, img[i][j])

    cv2.imshow("original", cv2.imread(frame_name))
    cv2.imshow("clusters", img)
    cv2.waitKey(0)

visualize_clusters(kmeans_info, frame_name)

