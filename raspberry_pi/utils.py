from typing import List
import cv2
import numpy as np
import math
from object_detector import Detection
import time

_MARGIN = 10  # pixels
_ROW_SIZE = 10  # pixels
_FONT_SIZE = 1
_FONT_THICKNESS = 1
_TEXT_COLOR = (0, 0, 255)  # red

ver_angle = 28
hor_angle = 37
obj_size = 0.6

def visualize(image: np.ndarray,detections: List[Detection],) -> np.ndarray:
    check = 0
    obj = 0
       
    for detection in detections:
        category = detection.categories[0]
        class_name = category.label
        if class_name=="person":


    # Draw bounding_box
            a = np.array((detection.bounding_box.left, detection.bounding_box.top),np.int32)
            b = np.array((detection.bounding_box.right,detection.bounding_box.bottom),np.int32)
            cv2.rectangle(image,(a[0],a[1]),(b[0],b[1]),(0,255,0),2)
            scale = 640/(b[0]-a[0])
            hor_img = scale*obj_size
            ver_img = hor_img*3/4
            d = hor_img/2/math.tan(37/2/180*math.pi)   
            obj_x = abs((b[0]+a[0])/2-320)/640 * hor_img
            obj_y = abs((b[1]+a[1])/2-240)/480 * ver_img
            distance = math.sqrt(d**2 + obj_x**2 + obj_y**2)
            print(distance)
            cv2.arrowedLine(image,(320,479), (int((a[0]+b[0])/2),int((a[1]+b[1])/2)),(0,0,255),3)
            cv2.putText(image,f'{round(distance,2)} m',(int((a[0]+b[0])/2),int((a[1]+b[1])/2)),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,(255,0,255))
            if distance < 1.5:
                check +=1
                c = (a+b)/2

                if(c[0] >= 320):
                    print("left: ",round(time.time(),2))
                    obj = 2
                else:
                    print("right: ",round(time.time(),2))
                    obj = 1 
    # Draw label and score
        #category = detection.categories[0]
        #class_name = category.label
        #probability = round(category.score, 2)
        #result_text = class_name + ' (' + str(probability) + ')'
        #text_location = (_MARGIN + detection.bounding_box.left,
        #             _MARGIN + _ROW_SIZE + detection.bounding_box.top)
        #cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_PLAIN,
        #        _FONT_SIZE, _TEXT_COLOR, _FONT_THICKNESS)
    if check == 0:
        print("safe: ",round(time.time(),2))
        obj = 0
    return image,obj
