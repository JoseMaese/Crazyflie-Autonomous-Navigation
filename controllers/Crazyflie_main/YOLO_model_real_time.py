# YOLO object detection
import cv2 as cv
import numpy as np
import time
import threading


WHITE = (255, 255, 255)
img = None
img0 = None
outputs = None

# Load names of classes and get random colors
classes = open('coco.names').read().strip().split('\n')
np.random.seed(42)
colors = np.random.randint(0, 255, size=(len(classes), 3), dtype='uint8')

# Give the configuration and weight files for the model and load the network.

#net = cv.dnn.readNetFromDarknet('yolov2-tiny.cfg', 'yolov2-tiny.weights')
#net = cv.dnn.readNetFromDarknet('yolov3-tiny.cfg', 'yolov3-tiny.weights')
#net = cv.dnn.readNetFromDarknet('yolov3.cfg', 'yolov3.weights')
net = cv.dnn.readNetFromDarknet('yolov7.cfg', 'yolov7-tiny.weights')

net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)

# determine the output layer
ln = net.getLayerNames()
ln = [ln[i - 1] for i in net.getUnconnectedOutLayers()]

def load_image(video_frame):
    
    img0 = video_frame
    img = img0.copy()

    blob = cv.dnn.blobFromImage(img, 1/255.0, (416, 416), swapRB=True, crop=False)
    
    net.setInput(blob)
    outputs = net.forward(ln)
    outputs = np.vstack(outputs)

    return outputs

def post_process(img0, outputs, conf, tiempo_nn):
    personaDetectada = 0
    t_detectada = tiempo_nn
    
    img=img0.copy()
    H, W = img.shape[:2]
    
    boxes = []
    confidences = []
    classIDs = []

    for output in outputs:
        scores = output[5:]
        classID = np.argmax(scores)
        confidence = scores[classID]
        if confidence > conf:
            x, y, w, h = output[:4] * np.array([W, H, W, H])
            p0 = int(x - w//2), int(y - h//2)
            p1 = int(x + w//2), int(y + h//2)
            boxes.append([*p0, int(w), int(h)])
            confidences.append(float(confidence))
            classIDs.append(classID)

    indices = cv.dnn.NMSBoxes(boxes, confidences, conf, conf-0.1)
    if len(indices) > 0:
        for i in indices.flatten():
            (x, y) = (boxes[i][0], boxes[i][1])
            (w, h) = (boxes[i][2], boxes[i][3])
            color = [int(c) for c in colors[classIDs[i]]]
            cv.rectangle(img, (x, y), (x + w, y + h), color, 2)
            text = "{}: {:.4f}".format(classes[classIDs[i]], confidences[i])
            #print(text)
            cv.putText(img, text, (x, y - 5), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            if text.split(":")[0].strip() == 'person':
                personaDetectada = 1
                #print('Persona detectada:', text.split(":")[1]) 
            else:
                personaDetectada = 0                   
    return img, personaDetectada, t_detectada
    
   
    
#cv.destroyAllWindows()