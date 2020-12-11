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

import numpy as np
import cv2
from math import pi

THR_CIRCULARITY = 0.3
THR_SOLIDITY = 0.8
THR_ELLIPSE_SEMI_AXES_FACTOR = 0.2

def translate(value, oldMin, oldMax, newMin=-100, newMax=100):
    # Figure out how 'wide' each range is
    oldRange = oldMax - oldMin
    newRange = newMax - newMin
    NewValue = (((value - oldMin) * newRange) / oldRange) + newMin
    return int(NewValue)

def get_regular_objects(contours, minArea=0.0):
    regular_objects = []
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    
    for cnt in contours:
        try:
            if cv2.contourArea(cnt) < minArea:
                continue
            hull = cv2.convexHull(cnt)
            area = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            solidity = float(area)/hull_area if hull_area > 0 else 0
            print(f"is solidity greater than 0.8?: {solidity > THR_SOLIDITY}")
                
            (x, y), (MA, ma), angle = cv2.fitEllipse(cnt) 
            print(f"is contour circular?: {np.abs(MA/ma - 1) < THR_ELLIPSE_SEMI_AXES_FACTOR}")

            if(solidity > THR_SOLIDITY and np.abs(MA/ma - 1) < THR_ELLIPSE_SEMI_AXES_FACTOR):
                x,y,w,h = cv2.boundingRect(cnt)
                regular_objects.append((x,y,w,h))

        except:
            pass

    return regular_objects

def getCircularContours(contours, minimalCountourArea):
    circularContours = []

    if (contours is not None) and (len(contours) > 0):
        for contour in contours:
            # approximate contour with polygon where maximum distance from contour to approximated contour is 1% of perimeter
            approx = cv2.approxPolyDP(contour, 0.03*cv2.arcLength(contour, True), True)
            # get contour area
            area = cv2.contourArea(contour)
            # check if number of angles is greater then 8, contour area is bigger then minimal and the approximated contour is convex
            if (len(approx) >= 7) and (area > minimalCountourArea) and (cv2.isContourConvex(approx)):
                circularContours.append(approx)
            else:
                print(len(approx)) 
                print(area > minimalContourArea)   
    return circularContours

def get_ball_like_objects(contours, minArea=0.0):
    balls = []
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    for cnt in contours:
        try:
            if cv2.contourArea(cnt) < minArea:
                continue
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            center = (int(x),int(y))
            radius = int(radius)
            print(f"radius: {radius}")
            circle_area = radius**2 * pi
            area = cv2.contourArea(cnt)

            print(f"is circular: {abs(area/circle_area - 1) < THR_CIRCULARITY if circle_area > 0 else False} ")
            if(circle_area > 0 and abs(area/circle_area - 1) < THR_CIRCULARITY):
                x,y,w,h = cv2.boundingRect(cnt)
                balls.append((x,y,w,h))
            
        except:
            pass
    
    return balls

def getContours(image, mode=cv2.RETR_EXTERNAL):
    contours = cv2.findContours(image, mode, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    
    return contours

def preprocessMask(mask, iterations):
    kernel = np.ones((7,7),np.uint8)
    # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
    mask = cv2.dilate(mask, kernel, iterations=iterations)
    kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
    mask = cv2.erode(mask, kernel2, iterations=iterations)
    # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=iterations)
    # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=iterations)

    return mask

def getBoundingBoxes(contours, minArea):
    boundingBoxes = []
    
    if contours:
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        
        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > (minArea):
                x,y,w,h = cv2.boundingRect(contour)
                boundingBoxes.append((x,y,w,h))
    
    return boundingBoxes



######## Main variables

frameResolution = (960, 460)

HorizontalFOV = 62
VerticalFOV = 37


colorLower = (20, 100, 100)
colorUpper = (30, 255, 255)
colorTolerance = 3
roiSize = (frameResolution[0]//16, frameResolution[0]//16) # Region of Interest size


usesPiCamera = False

paused = False

######## Initialization

# vs = VideoStream(usePiCamera=usesPiCamera, resolution=cameraResolution, framerate=60).start()
vs = JetsonVideoStream(outputResolution=frameResolution)

vs.start()

# # initialize serial communication
ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.05)

time.sleep(2.0)


# pause camera movement

ser.write(bytes('<stop, 0, 0>', 'utf-8') )

######## Main loop

while True:
    loopStart = time.time()
    grabCircles = True
    if not paused:

        frame = vs.read() 
        height, width = frame.shape[0:2]

        blurred = cv2.GaussianBlur(frame, (5, 5), 0)

        frame_HSV = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        roi = frame_HSV[height//2 - roiSize[0]//2 : height //2 + roiSize[0]//2, width//2 - roiSize[1]//2 : width//2 + roiSize[1]//2, :]
        
        colorLowerWithTolerance = (colorLower[0] - colorTolerance,) + colorLower[1:]
        colorUpperWithTolerance = (colorUpper[0] + colorTolerance,) + colorUpper[1:]

        mask = cv2.inRange(frame_HSV, colorLowerWithTolerance, colorUpperWithTolerance)
        
        
        mask = preprocessMask(mask, 5)
        
        # get contours of objects in image
        contours = getContours(mask)
      
        # draw ROI on image
        xROI, yROI = width//2 - roiSize[1]//2 , height//2 - roiSize[0]//2
        cv2.rectangle(frame, (xROI, yROI), (xROI + roiSize[0], yROI + roiSize[1]), (0, 0, 0), thickness=3)
        minimalContourArea = (width * height)/256
        # draw bounding boxes, find largest object
        if not grabCircles:

            # get bounding boxes for contrours
            
            boundingBoxes = getBoundingBoxes(contours, minimalContourArea)

            for i, boundingBox in enumerate(boundingBoxes):
            
                x, y, w, h = boundingBox

                if i is 0:
                    biggestObjectMidPoint = ((x+ w//2), (y + h//2))
                    cv2.rectangle(frame, (x, y), ((x+w), (y+h)), (0, 0, 255), thickness=3)
                    cv2.circle(frame, biggestObjectMidPoint, 4, (255, 0, 0), thickness=3)

                    screenMidPoint = width//2, height//2
                    distanceVector = tuple(map(lambda x, y: x - y, biggestObjectMidPoint, screenMidPoint))

                    yaw = translate(distanceVector[0], -width//2, width//2, -HorizontalFOV//2, HorizontalFOV//2) # up-down
                    yawError = yaw / (HorizontalFOV/2) 
                    pitch = translate(distanceVector[1], -height//2, height//2, -VerticalFOV//2, VerticalFOV//2) # left-right
                    pitchError = pitch / (VerticalFOV/2)

                    print("Yaw error: {}, Pitch error: {}\n".format(yawError, pitchError))
                
                
                    cv2.line(frame, screenMidPoint, biggestObjectMidPoint, (0, 0, 255))
                    packet = '<servo, {}, {}>'.format(yawError, pitchError)
                    packetBytes = bytes(packet, 'utf-8')
                
                    ser.write(packetBytes)
                
                else:
                    cv2.rectangle(frame, (x, y), ((x+w), (y+h)), (255, 255, 0), thickness=2)
                 
        else:
            piłkas = get_ball_like_objects(getCircularContours(contours, minimalContourArea), minimalContourArea)
            for i, boundingBox in enumerate(piłkas):
                       
                x, y, w, h = boundingBox
                if i is 0:
                    biggestObjectMidPoint = ((x+ w//2), (y + h//2))
                    cv2.rectangle(frame, (x, y), ((x+w), (y+h)), (0, 0, 255), thickness=3)
                    cv2.circle(frame, biggestObjectMidPoint, 4, (255, 0, 0), thickness=3)
                    screenMidPoint = width//2, height//2
                    distanceVector = tuple(map(lambda x, y: x - y, biggestObjectMidPoint, screenMidPoint))
           
                    yaw = translate(distanceVector[0], -width//2, width//2, -HorizontalFOV//2, HorizontalFOV//2) # up-down
                    yawError = yaw / (HorizontalFOV/2) 
                    pitch = translate(distanceVector[1], -height//2, height//2, -VerticalFOV//2, VerticalFOV//2) # left-right
                    pitchError = pitch / (VerticalFOV/2)
           
                    print("Yaw error: {}, Pitch error: {}\n".format(yawError, pitchError))
                           
                           
                    cv2.line(frame, screenMidPoint, biggestObjectMidPoint, (0, 0, 255))
                    packet = '<servo, {}, {}>'.format(yawError, pitchError)
                    packetBytes = bytes(packet, 'utf-8')
                          
                    ser.write(packetBytes)
                           
                else:
                    cv2.rectangle(frame, (x, y), ((x+w), (y+h)), (255, 255, 0), thickness=2)
    
        cv2.imshow("video", frame)
        cv2.imshow("roi", roi)
        cv2.imshow("mask", mask)


    # handle keys 
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('a'):
        avg_h = 0
        avg_s = 0
        avg_v = 0
        i = 0
        for _, row in enumerate(roi):
            avg = np.average(row, 0)
            avg_h += avg[0]
            avg_s += avg[1]
            avg_v += avg[2]
            i+=1

        avg_h /= i
        avg_s /= i
        avg_v /= i
        print("HUE:{}, SAT:{}, VAL:{}".format(avg_h, avg_s, avg_v))
        colorLower = (max(0,avg_h), max(0, avg_s - 50), max(0,avg_v - 50))
        colorUpper = (min (255, avg_h), min(255, avg_s + 50), min(255, avg_v + 50))
    elif key == ord('z'):
        h = roi[:,:,0]
        s = roi[:,:,1]
        v = roi[:,:,2]
        colorLower = (int(np.min(h)), max(0, int(np.min(s)-20 )), max(0, int(np.min(v)-20)))
        colorUpper = (int(np.max(h)), min(255, int(np.max(s)+20)), min(255, int(np.max(v)+20)))
    elif key == ord('w'):
        colorTolerance = min(colorTolerance + 1, 50)
        print("New color range: {}".format(colorTolerance))
    elif key == ord('s'):
        colorTolerance = max(colorTolerance - 1, 0)
        print("New color range: {}".format(colorTolerance))
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



    #print(ser.read_all())

    loopEnd = time.time()
    # print("loop execution took {:3.2f}ms".format((loopEnd - loopStart)*1000))
    

    
# cleanup

ser.close()
cv2.destroyAllWindows()

# to cleanly stop frame grabbing thread
vs.stop()
time.sleep(1.0)
sys.exit(0)

