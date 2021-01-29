# parts of the code are based on https://www.pyimagesearch.com/2016/01/04/unifying-picamera-and-cv2-videocapture-into-a-single-class-with-opencv/
# before running the code install imutils for python3 (from https://github.com/pawelplodzpl/imutils)

# in case of problems with camera image run:
# sudo systemctl restart nvargus-daemon


import sys
import time
import imutils
from imutils.video import JetsonVideoStream
# from imutils.video import VideoStream
import serial

import jetson.inference
import jetson.utils

import numpy as np
import cv2
from math import pi

def translate(value, oldMin, oldMax, newMin=-100, newMax=100):
    # Figure out how 'wide' each range is
    oldRange = oldMax - oldMin
    newRange = newMax - newMin
    NewValue = (((value - oldMin) * newRange) / oldRange) + newMin
    return int(NewValue)

frameResolution = (848, 480)

HorizontalFOV = 62
VerticalFOV = 37


colorLower = (20, 100, 100)
colorUpper = (30, 255, 255)
colorTolerance = 3


usesPiCamera = False

paused = False

######## Initialization

# vs = VideoStream(usePiCamera=usesPiCamera, resolution=cameraResolution, framerate=60).start()
vs = JetsonVideoStream(outputResolution=frameResolution)
vs.start()
time.sleep(2.0)


modelPath = 'AI/mb2-ssd-lite.onnx'
labelsPath = 'AI/labels.txt'


net = jetson.inference.detectNet(argv=['--model={}'.format(modelPath),
                                       '--labels={}'.format(labelsPath),
                                       '--input-blob=input_0', '--output-cvg=scores', '--output-bbox=boxes'],# , '--input-flip=rotate-180'],
                                       threshold=0.5)


# # initialize serial communication
ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.05)

labels = {"BACKGROUND": 0, "Groot": 1}

# pause camera movement

ser.write(bytes('<stop, 0, 0>', 'utf-8') )

######## Main loop

while True:
    loopStart = time.time()
    if not paused:

        frame = vs.read()
        height, width = frame.shape[0:2]

        # convert to CUDA image
        img = frame.copy()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2RGBA).astype(np.float32)
        img = jetson.utils.cudaFromNumpy(frame)

        detections = net.Detect(img, width, height)
    
        groot = None
        for detection in detections:
            if detection.ClassID == labels["Groot"]:
                groot = detection
                break

        if groot != None:
            x1,y1,x2,y2 = (int(groot.Left), int(groot.Top), int(groot.Right), int(groot.Bottom))
            classID = groot.ClassID
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 0), thickness=3)
            w = x2 - x1
            h = y2 - y1
            x = x1
            y = y1

            biggestObjectMidPoint = ((x+ w//2), (y + h//2))
            cv2.rectangle(frame, (x, y), ((x+w), (y+h)), (0, 0, 255), thickness=3)
            cv2.circle(frame, biggestObjectMidPoint, 4, (255, 0, 0), thickness=3)
            screenMidPoint = width//2, height//2
            distanceVector = tuple(map(lambda x, y: x - y, biggestObjectMidPoint, screenMidPoint))
    
            yaw = translate(distanceVector[0], -width//2, width//2, -HorizontalFOV//2, HorizontalFOV//2) # up-down
            yawError = yaw / (HorizontalFOV/2) 
            pitch = translate(distanceVector[1], -height//2, height//2, -VerticalFOV//2, VerticalFOV//2) # left-right
            pitchError = pitch / (VerticalFOV/2)
            
            distanceToObject = (w * h) / (frameResolution[0] * frameResolution[1])
            print("Yaw error: {}, Pitch error: {}, Distance to object: {}\n".format(yawError, pitchError, distanceToObject/ 5 ))
                    
                    
            cv2.line(frame, screenMidPoint, biggestObjectMidPoint, (0, 0, 255))
            packet = '<servo, {}, {}, {}>'.format(yawError, pitchError, distanceToObject / 5)
            packetBytes = bytes(packet, 'utf-8')
                    
            ser.write(packetBytes)    
        else:
            packet = '<pause, 0, 0>'
            packetBytes = bytes(packet, 'utf-8')
            ser.write(packetBytes)

        # print out performance info
        # net.PrintProfilerTimes()
      
        cv2.imshow("video", frame)


    # handle keys 
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('p'):
        paused = not paused
    elif key == ord('d'):
        # pause/unpause camera movement
        packet = '<stop, 0, 0>'
        packetBytes = bytes(packet, 'utf-8')  
        ser.write(packetBytes)
    elif key == ord('f'):
        # pause/unpause camera movement
        packet = '<start, 0, 0>'
        packetBytes = bytes(packet, 'utf-8')  
        ser.write(packetBytes)


    loopEnd = time.time()
    
# cleanup

ser.close()
cv2.destroyAllWindows()

# to cleanly stop frame grabbing thread
vs.stop()
time.sleep(1.0)
sys.exit(0)

