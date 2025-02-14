import pickle
import cv2
import os
from sklearn.cluster import KMeans
from utils import cut_frame_unwanted_part

from kmeansinfo import KmeansInfo


frame_name = "images/original_frame_cut.png"
video_name = "videos/end_cut.mp4"
number_of_clusters = 4


def apply_kmeans(
    frame_name: str = None, 
    folder_name: str = None,
    frames = None
    ) -> KMeans:

    print("reading image(s)")
    imgs = []
    if frame_name is not None:
        imgs.append(cv2.imread(frame_name))
        if img is None:
            print("Image not found")
            exit()
    elif folder_name is not None:
        print("Did not test this yet, be careful")
        # read all the images in the folder
        # and apply kmeans to all of them
        # and save the kmeans info to a file
        for file_name in os.listdir(folder_name):
            if file_name.endswith(".jpg") or file_name.endswith(".png"):
                img = cv2.imread(folder_name + "/" + file_name)
                imgs.append(img)
    else:
        imgs = frames


    kmeans_list = []
    for img in imgs:
        for i in range(len(img)):
            for j in range(len(img[i])):
                # cut unwanted black places
                """
                print(i, j)
                if i <= 25:
                    if j <= 115 or len(img[i]) - j <= 180:
                        continue
                if len(img) - i <= 43 and len(img[i]) - j <= 170:
                    continue
                print("not skipped")
                """
                # skipping the cutted part
                if (img[i][j] != [0, 0, 0]).all():
                    kmeans_list.append(img[i][j])

    print("applying kmeans")
    return KMeans(n_clusters = number_of_clusters, random_state = 0, verbose = 1).fit(kmeans_list)
    
def identify_cluster_colors(kmeans: KMeans) -> KmeansInfo:
    print(f"identify {number_of_clusters} cluster colors")
    print("be careful that the color order is BGR")
    cluster_centers = kmeans.cluster_centers_
    blue_center = None
    yellow_center = None
    red_centers = []
    background_centers = None

    print("enter 0 for blue, 1 for yellow, 2 for background, red will be computed later")

    for i in range(len(cluster_centers)):
        print("cluster center: ", cluster_centers[i])
        
        while True:
            try:
                color = int(input("enter color: "))
            except:
                print("invalid input")
                continue
            break
        
        if color == 0:
            blue_center = cluster_centers[i]
        elif color == 1:
            yellow_center = cluster_centers[i]
        elif color == 2:
            background_centers = cluster_centers[i]

    # red computation
    red_of_blue = blue_center[2]
    red_of_yellow = yellow_center[2]
    red_centers.append([blue_center[0], blue_center[1], 256 - red_of_blue])
    red_centers.append([yellow_center[0], yellow_center[1], 256 - red_of_yellow])


    return KmeansInfo(cluster_centers, blue_center, yellow_center, red_centers, background_centers)

# read video and put all of the frames into 
frames = []
cap = cv2.VideoCapture(video_name)
while True:
    ret, frame = cap.read()
    if not ret:
        break
    frames.append(cut_frame_unwanted_part(frame))

kmeans = apply_kmeans(frames = frames)
kmeans_info = identify_cluster_colors(kmeans)
pickle.dump(kmeans_info, open("kmeans_info.pkl", "wb"))

"""
#cv2.imshow("original", cv2.imread(new_name))
#cv2.imshow("original", increase_hsv_value(cv2.imread(frame_name)))
cv2.imshow("original", cv2.imread(frame_name))
cv2.imshow("kmeans", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
"""




