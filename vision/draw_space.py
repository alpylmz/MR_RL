import cv2
from colorthief import ColorThief

THRESHOLD = 30

# read a.jpg
img = ColorThief("original frame.jpg")

# find the dominant color
dominant_color = img.get_color(quality=1)

# color palette
palette = img.get_palette(color_count=5)

print(dominant_color)
print(palette)

img = cv2.imread('frame.jpg')

# filter the image with close colors to dominant color
# and save it to a new file
dominant_plus_10 = [x + THRESHOLD for x in dominant_color]

for i in range(0, len(img)):
    for j in range(0, len(img[i])):
        
        # if the color values are below dominant_plus_10, make it all black
        # if img[i][j][0] < dominant_plus_10[0] and img[i][j][1] < dominant_plus_10[1] and img[i][j][2] < dominant_plus_10[2]:
        #    img[i][j] = [0, 0, 0]

        # yellow filter
        if abs(img[i][j][1] - 255) < THRESHOLD*5 and \
            abs(img[i][j][2] - 255) < THRESHOLD*5 and \
            abs(img[i][j][0]) < THRESHOLD*4:
            img[i][j] = [0, 255, 255]

        # blue filter
        elif abs(img[i][j][0] - 255) < THRESHOLD*3 \
            and abs(img[i][j][1]) < THRESHOLD*3 \
            and abs(img[i][j][2]) < THRESHOLD*3:
            img[i][j] = [255, 0, 0]
        
        # red filter
        elif abs(img[i][j][0] - 255) < THRESHOLD*4 and \
            abs(img[i][j][2] - 255) < THRESHOLD*4:
            img[i][j] = [255, 0, 255]

        # just take other colors
        else:
            img[i][j] = [0, 0, 0]        
        

# if the color values are close enough to the second dominant_color
# make them blue
"""
blue = palette[1]
for i in range(0, len(img)):
    for j in range(0, len(img[i])):
        if abs(img[i][j][0] - blue[0]) < THRESHOLD and \
            abs(img[i][j][1] - blue[1]) < THRESHOLD and \
            abs(img[i][j][2] - blue[2]) < THRESHOLD:
            img[i][j] = [255, 255, 255]
"""

"""
# yellow filtering
for i in range(0, len(img)):
    for j in range(0, len(img[i])):
        if abs(img[i][j][1] - 255) < THRESHOLD*5 and \
            abs(img[i][j][2] - 255) < THRESHOLD*5:
            img[i][j] = [0, 255, 255]

# red filtering
for i in range(0, len(img)):
    for j in range(0, len(img[i])):
        if abs(img[i][j][0] - 255) < THRESHOLD*5 and \
            abs(img[i][j][2] - 255) < THRESHOLD*5:
            img[i][j] = [255, 255, 255]
"""
# display img
cv2.imshow("original", cv2.imread('frame.jpg'))
cv2.imshow('image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()


