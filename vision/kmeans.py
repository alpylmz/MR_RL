import cv2
import os
from sklearn.cluster import KMeans
import numpy as np
from tqdm import tqdm
from PIL import ImageEnhance, Image

# the yellow is 169 162 116
# the red is 141 63 107

frame_name = "original frame 2.png"
contrast_increase = 1.2
new_name = frame_name[:-4] + f" contrasted {contrast_increase}.png"
number_of_clusters = 5
hsv_value_increase = 1.2

def increase_constrast(img):
    img = Image.fromarray(img)
    img = ImageEnhance.Contrast(img).enhance(contrast_increase)
    # if the file name frame_name does not exist
    # then save the image
    if not os.path.exists(new_name):
        img.save(new_name)
    return np.array(img)

def increase_hsv_value(img):
    # convert to hsv
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # increase the value
    img[:, :, 2] = img[:, :, 2] * hsv_value_increase
    # convert back to bgr
    img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
    return img


img = cv2.imread(frame_name)
img = increase_hsv_value(img)
if img is None:
    print("Image not found")
    exit()

kmeans_list = []
for i in range(len(img)):
    for j in range(len(img[i])):
        kmeans_list.append(img[i][j])

kmeans = KMeans(n_clusters = number_of_clusters, random_state = 0).fit(kmeans_list)

print("kmeans cluster centers: ")
print(kmeans.cluster_centers_)

print("example predict")
print(kmeans.predict(np.array([0, 0, 0]).reshape(1, -1)))


for i in tqdm(range(len(img))):
    img[i] = kmeans.cluster_centers_[kmeans.predict(img[i])]
    # for j in range(len(img[i])):
        # img[i][j] = kmeans.cluster_centers_[kmeans.predict(img[i][j].reshape(1, -1))]
        

#cv2.imshow("original", cv2.imread(new_name))
cv2.imshow("original", increase_hsv_value(cv2.imread(frame_name)))
#cv2.imshow("original", cv2.imread(frame_name))
cv2.imshow("kmeans", img)
cv2.waitKey(0)





