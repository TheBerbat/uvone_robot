#!/usr/bin/env python3
import cv2
import math
import yaml

origen = [0, 0, 0]
grid_size = 0

f = open("demofile2.txt", "w")

with open('/home/rafaelm/Desktop/mapa_test.yaml') as file:

    documents = yaml.load(file, Loader=yaml.FullLoader)

    for item, doc in documents.items():

        if item == 'origin':
            origen = doc

        if item == 'resolution':
            grid_size = doc

        if item == 'image':
            path = doc


img = cv2.imread(path)
height, width, channels = img.shape

init_point = (0,0)
final_point = (0,0)
init_point_off = (0,0)
final_point_off = (0,0)

def on_click(event, x, y, p1, p2):
    global init_point, final_point, init_point_off, final_point_off, height, width

    x_off = round(x*grid_size + origen[0],3)
    y_off = round((height - y)*grid_size + origen[1],3)

    if event == cv2.EVENT_LBUTTONDOWN:
        print("Presionado",x_off, y_off)
        init_point = (x, y)
        init_point_off = (x_off, y_off)
        
    if event == cv2.EVENT_LBUTTONUP:
        print("Soltado",x_off, y_off)
        final_point = (x, y)
        final_point_off = (x_off, y_off)

        vector = (final_point[0] - init_point[0], final_point[1] - init_point[1] )
        longitud = math.sqrt(vector[0]**2 + vector[1]**2) / (math.sqrt(height**2 + width**2) * 0.025)
        if longitud==0:
            return
        vector = (vector[0] / longitud, vector[1] / longitud)
        
        final_point = (int(init_point[0] + vector[0]), int(init_point[1] + vector[1]))


        f.write("%.4f  \t%.4f  \t%.4f\n" % (init_point_off[0], init_point_off[1], 1))


        cv2.arrowedLine(img, init_point, final_point, (100, 200, 0), 5)
        cv2.imshow("image", img)




cv2.namedWindow('image')
cv2.setMouseCallback('image', on_click)
cv2.imshow("image", img)

while(1):
    k = cv2.waitKey(33)
    if k==32:    # Esc key to stop
        break
    else:  # normally -1 returned,so don't print it
        continue
  
cv2.destroyAllWindows()

f.close()
