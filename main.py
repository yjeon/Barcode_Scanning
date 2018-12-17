from __future__ import print_function
import pyzbar.pyzbar as pyzbar
import numpy as np
import cv2
 
import imutils
 
def detect(image):
    # convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
 
    # compute the Scharr gradient magnitude representation of the images
    # in both the x and y direction using OpenCV 2.4
    ddepth = cv2.cv.CV_32F if imutils.is_cv2() else cv2.CV_32F
    gradX = cv2.Sobel(gray, ddepth=ddepth, dx=1, dy=0, ksize=-1)
    gradY = cv2.Sobel(gray, ddepth=ddepth, dx=0, dy=1, ksize=-1)
 
    # subtract the y-gradient from the x-gradient
    gradient = cv2.subtract(gradX, gradY)
    gradient = cv2.convertScaleAbs(gradient)
 
    # blur and threshold the image
    blurred = cv2.blur(gradient, (9, 9))
    (_, thresh) = cv2.threshold(blurred, 225, 255, cv2.THRESH_BINARY)
 
    # construct a closing kernel and apply it to the thresholded image
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 7))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
 
    # perform a series of erosions and dilations
    closed = cv2.erode(closed, None, iterations=4)
    closed = cv2.dilate(closed, None, iterations=4)
 
    # find the contours in the thresholded image
    cnts = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
 
    # if no contours were found, return None
    if len(cnts) == 0:
        return None
 
    # otherwise, sort the contours by area and compute the rotated
    # bounding box of the largest contour
    c = sorted(cnts, key=cv2.contourArea, reverse=True)[0]
    rect = cv2.minAreaRect(c)
    box = cv2.cv.BoxPoints(rect) if imutils.is_cv2() else cv2.boxPoints(rect)
    box = np.int0(box)
 
    # return the bounding box of the barcode
    return box
 
# Main 
if __name__ == '__main__':
 
  # Read image
    im = cv2.VideoCapture(0)

    if im.isOpened():
        rval, frame = im.read()
    else:
        rval = False
        print("Camera is not working!")

    while (rval):
        cv2.imshow("Frame", frame)
        rval, frame = im.read()
        
        box = detect(frame)

        if box is not None:
            cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)


        k = cv2.waitKey(1) & 0xFF

        # d = decode(frame)
        # display(frame,d)
        if k == 27:
            break
    # key = cv2.waitKey(20)
    # if key == ord('c'):
    #     cache = getImageName()
    #     #cv2.imwrite(cache, frame)
    #     #results = decodeFile(cache)
    #     #print "Total count: " + str(len(results))
    #     #for result in results:
    #      #   print "barcode format: " + formats[result[0]]
    #       #  print "barcode value: " + result[1] + "\n*************************"
    # elif key == 27:
    #     break

    #decodedObjects = decode(frame)
    #display(frame, decodedObjects)
    

    cv2.destroyAllWindows
  #decodedObjects = decode(im)
  #display(im, decodedObjects)

