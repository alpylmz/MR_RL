import pickle
import cv2
import os
from sklearn.cluster import KMeans

from kmeansinfo import KmeansInfo


frame_name = "original frame 2.png"
number_of_clusters = 4


def apply_kmeans(frame_name = None, folder_name = None):
    if frame_name is not None:
        img = cv2.imread(frame_name)

        if img is None:
            print("Image not found")
            exit()

        kmeans_list = []
        for i in range(len(img)):
            for j in range(len(img[i])):
                kmeans_list.append(img[i][j])

    elif folder_name is not None:
        print("Did not test this yet")
        # read all the images in the folder
        # and apply kmeans to all of them
        # and save the kmeans info to a file

        imgs = []
        for file_name in os.listdir(folder_name):
            if file_name.endswith(".jpg") or file_name.endswith(".png"):
                img = cv2.imread(folder_name + "/" + file_name)
                imgs.append(img)

        kmeans_list = []
        for img in imgs:
            for i in range(len(img)):
                for j in range(len(img[i])):
                    kmeans_list.append(img[i][j])

    kmeans = KMeans(n_clusters = number_of_clusters, random_state = 0).fit(kmeans_list)

    return kmeans

def identify_cluster_colors(kmeans):
    cluster_centers = kmeans.cluster_centers_
    blue_center = None
    yellow_center = None
    red_center = None
    background_centers = None

    print("enter 0 for blue, 1 for yellow, 2 for red, 3 for background")

    for i in range(len(cluster_centers)):
        print("cluster center: ", cluster_centers[i])
        
        while True:
            try:
                color = int(input("enter color: "))
                if color < 0 or color > 3:
                    print("invalid input")
                    continue
                break
            except:
                print("invalid input")
                continue
        
        if color == 0:
            blue_center = cluster_centers[i]
        elif color == 1:
            yellow_center = cluster_centers[i]
        elif color == 2:
            red_center = cluster_centers[i]
        elif color == 3:
            background_centers = cluster_centers[i]

    return KmeansInfo(cluster_centers, blue_center, yellow_center, red_center, background_centers)



kmeans = apply_kmeans(frame_name = frame_name)
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




