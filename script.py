import numpy as np
import cv2
import time
import sys

if len(sys.argv) == 1:
    filename = "sample-video.mkv"
else:
    filename = sys.argv[1]
cap = cv2.VideoCapture(filename)

ret, frame1 = cap.read()
frame1 = cv2.flip(frame1, flipCode=-1)
prvs = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
hsv = np.zeros_like(frame1)
hsv[...,1] = 255

def draw_flow(img, flow, step=16):
    h, w = img.shape[:2]
    # mgrid: get meshgrid, begin:end:step
    # next line results in [[8, 24, ..., h], [8, 24, ..., w]]
    y, x = np.mgrid[step//2:h:step, step//2:w:step].reshape(2,-1)
    fx, fy = flow[y,x].T
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.polylines(vis, lines, 0, (0, 0, 255))
    for (x1, y1), (x2, y2) in lines:
        cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    return vis
cumulative = np.zeros((frame1.shape[0], frame1.shape[1], 2))
flow = np.zeros((frame1.shape[0], frame1.shape[1], 2))
while(True):
    ret, frame2 = cap.read()
    frame2 = cv2.flip(frame2, flipCode=-1)
    next = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    flow = 3* cv2.calcOpticalFlowFarneback(prvs,next, flow, 0.5, 3, 15, 3, 7, 1.5, cv2.OPTFLOW_USE_INITIAL_FLOW)
    cumulative = flow*0.7 + 0.3*cumulative # exponential smoothing

    drawn = draw_flow(next, cumulative)
    cv2.imshow("frame2", drawn)
    # time.sleep(0.1)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break
    elif k == ord('s'):
        cv2.imwrite('optical_flow.png',drawn)
    prvs = next

cap.release()
cv2.destroyAllWindows()