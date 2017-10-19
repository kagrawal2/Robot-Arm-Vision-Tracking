# import the necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import cv2
import matplotlib.pyplot as plt
from colorsys import rgb_to_hsv


### Initialization Parameters for color range for the ball 
# (HSV)
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
# (HSV)
YELLOW_MIN = (8, 50, 50)
YELLOW_MAX = (45, 255, 255)

# initialize the known distance from the camera to the object, which
# in this case is 8 inches
KNOWN_DISTANCE = 8.0
# initialize the known object width, which in this case, the ball has 4 inch radius
KNOWN_WIDTH = 3.0
# Wait how many frames before initializing to calculate focal length
frame_init_wait = 15


ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
    help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
    help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
pts = deque(maxlen=args["buffer"])
distances = deque(maxlen=args["buffer"])
 
# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
    camera = cv2.VideoCapture(0)
 
# otherwise, grab a reference to the video file
else:
    camera = cv2.VideoCapture(args["video"])


### Calculate Depth using initial calibration for ball size and distance
def distance_to_camera(knownWidth, focalLength, perWidth):
    # compute and return the distance from the maker to the camera
    return (knownWidth * focalLength) / perWidth

init_ball_not_found = 0
focalLength = None  # Our initial calibration finds the focal length from initial calibration


# keep looping
while True:
    # grab the current frame
    (grabbed, frame) = camera.read()
    original_image = frame

    # if we are viewing a video and we did not grab a frame,
    # then we have reached the end of the video
    if args.get("video") and not grabbed:
        break
 
    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(frame, width=600)
    # plt.imshow(frame)
    # plt.show()
    # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
    # construct a mask for the color, then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, YELLOW_MIN, YELLOW_MAX)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    # plt.imshow(mask)
    # plt.show()
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
 
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 
        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),
                (0, 255, 255), 2)
            # cv2.circle(frame, center, 5, (0, 0, 255), -1)

            # Find the distance from the initial calibration 
            # (15 frames past in test example video)
            if init_ball_not_found == frame_init_wait:
                focalLength = (radius * KNOWN_DISTANCE) / KNOWN_WIDTH
                # print(focalLength)
                # plt.imshow(frame)
                # plt.show()
            elif focalLength:
                dist = distance_to_camera(KNOWN_WIDTH, focalLength, radius)
                distances.appendleft(dist)
                print(dist)
                cv2.putText(frame, "{} inches".format(int(dist)), (int(x), int(y)), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 1)
            init_ball_not_found += 1 
 
    # update the points queue
    pts.appendleft(center)
    # print(center)

    # loop over the set of tracked points
    for i in range(1, len(pts)):
        # if either of the tracked points are None, ignore
        # them
        if pts[i - 1] is None or pts[i] is None:
            continue
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        # thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        # cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
 
    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
 
    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break
 
# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()

