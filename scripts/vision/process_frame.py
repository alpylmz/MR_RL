import pickle
import cv2
import numpy as np

from utils import pick_closest_center

kmeans_info = pickle.load(open("kmeans_info.pkl", "rb"))
frame_name = "images/original_frame_cut.png"


def visualize_frame(kmeans_info, frame_name: str = None, frame: np.ndarray = None):
    print("reading image")
    if frame_name is not None:
        img = cv2.imread(frame_name)
    else:
        img = frame
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
    cv2.imshow("original", frame)
    cv2.imshow("clusters", img)
    # save img
    #cv2.imwrite("images/clustered_frame_23_jan.png", img)
    cv2.waitKey(0)
    return img

def visualize_video(kmeans_info, video_name):
    cap = cv2.VideoCapture(video_name)
    if not cap.isOpened():
        print("Video not found")
        exit()
    
    # create a new video file
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) * 2)
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('videos/output3.mp4', fourcc, 20.0, (width, height))

    # get total number of frames
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    i = 0
    # get the movie frame by frame
    while cap.isOpened():
        print("frame: ", i, "/", total_frames)
        i += 1
        ret, frame = cap.read()
        if ret:
            img = visualize_frame(kmeans_info, frame = frame)
            """
            print(img.shape)
            print(frame.shape)
            print(img.size)
            print(frame.size)
            print(img.dtype)
            print(frame.dtype)
            """
            #print(img)
            #print("--------------------------------------------------")
            #print(frame)
            # concatenate the original frame and the clustered frame
            img = cv2.hconcat([frame, img])
            out.write(img)
        else:
            break

# read video videos/example.mov
visualize_video(kmeans_info, "videos/end_cut.mp4")
#visualize_frame(kmeans_info, frame_name)

