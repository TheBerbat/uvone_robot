import cv2
import math

path = '/home/rafaelm/Downloads/img_001.jpg'
img = cv2.imread(path)
height, width, channels = img.shape

init_point = (0,0)
final_point = (0,0)


def on_click(event, x, y, p1, p2):
    global init_point, final_point, height, width

    if event == cv2.EVENT_LBUTTONDOWN:
        print("Presionado",x, y)
        init_point = (x, y)
        
    if event == cv2.EVENT_LBUTTONUP:
        print("Solatdo",x, y)
        final_point = (x, y)

        vector = (final_point[0] - init_point[0], final_point[1] - init_point[1] )
        longitud = math.sqrt(vector[0]**2 + vector[1]**2) / (math.sqrt(height**2 + width**2) * 0.05)
        vector = (vector[0] / longitud, vector[1] / longitud)
        
        final_point = (int(init_point[0] + vector[0]), int(init_point[1] + vector[1]))


        cv2.arrowedLine(img, init_point, final_point, (100, 200, 0), 5)
        cv2.imshow("image", img)


cv2.namedWindow('image')
cv2.setMouseCallback('image', on_click)
cv2.imshow("image", img)

cv2.waitKey(0)  
  
cv2.destroyAllWindows()  