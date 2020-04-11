import numpy as np
import cv2
import time
import sys
import cvxopt
import yaml

if len(sys.argv) == 1:
    filename = "sample-video.mkv"
else:
    filename = sys.argv[1]
cap = cv2.VideoCapture(filename)
frameRate = 30

def loadCameraParams(filename):
    print("loading camera parameters from {}".format(filename))
    with open(filename, "r") as f:
        data = yaml.load(f)
    width = data["image_width"]
    height = data["image_height"]
    cameraMatrix = np.array(data["camera_matrix"]["data"]).reshape((3,3))
    distortionCoefs = data["distortion_coefficients"]["data"]
    print("width:{}\theight:{}\ncamera matrix:\n{}\ndistortion coefficients:{}".format(width, height, cameraMatrix, distortionCoefs))
    return width, height, cameraMatrix, distortionCoefs

def constructM(x, y, w, h, cameraMatrix):
    f = cameraMatrix[0,0] # this lazy definition of f can only be used when (0,0) and (1,1) are, like, really similar
    M = np.zeros((2*x.shape[0], 2))
    M[np.arange(0, x.shape[0]*2, 2), 0] = f*(y-cameraMatrix[1,2])
    M[np.arange(0, x.shape[0]*2, 2), 1] = (y-cameraMatrix[1,2])*(x-cameraMatrix[0,2])
    M[np.arange(0, x.shape[0]*2, 2)+1, 1] = (x-cameraMatrix[0,2])**2
    M /= f*h
    print(M)
    return M

W, H, CAMERA_MATRIX, DISTORTION_COEFS = loadCameraParams("ELP_100_camera/ost.yaml")
X, Y = np.mgrid[8:W:16, H*3//4:H:16].reshape(2,-1) # use lower quarter of image
M = constructM(X, Y, W, H, CAMERA_MATRIX)

# calculate matrices used in QP
P = 2 * np.matmul(M.T, M)
print("P:\n{}".format(P))


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
    for (x1, y1), _ in lines:
        cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    return vis

def estimateVelocity(flow):
    fx, fy= flow[Y, X].T
    vec = np.zeros((X.shape[0]*2, 1))
    vec[np.arange(0, X.shape[0]*2, 2),0] = fx
    vec[np.arange(0, X.shape[0]*2, 2)+1,0] = fy
    q = -2 * np.matmul(vec.T, M).T
    _P = cvxopt.matrix(P)
    _q = cvxopt.matrix(q)
    sol = cvxopt.solvers.qp(_P, _q)
    return frameRate*np.array([sol["x"][0], sol["x"][1]])

h = 9 # height of the camera

if __name__=="__main__":
    cumulative = np.zeros((H, W, 2))
    flow = np.zeros((H, W, 2))
    cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
    while(True):
        ret, frame = cap.read()
        frame2 = cv2.undistort(frame, CAMERA_MATRIX, np.array(DISTORTION_COEFS))
        frame2 = cv2.flip(frame2, flipCode=-1)
        next = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
        try:
            flow = 3* cv2.calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 7, 1.5, cv2.OPTFLOW_USE_INITIAL_FLOW)
            cumulative = flow*0.4 + 0.6*cumulative # exponential smoothing
        except NameError:
            pass

        v = estimateVelocity(cumulative)
        print("vx:{}\tvz:{}\t[cm/s]".format(v[0], v[0]))

        drawn = draw_flow(next, cumulative)
        center = np.array([W//2, H//2])
        drawn = cv2.arrowedLine(drawn, tuple(center), tuple(center+(0.1*v).astype(int)), (255,0,0), thickness=2)
        cv2.imshow("frame", drawn)
        # time.sleep(0.1)
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break
        elif k == ord('s'):
            cv2.imwrite('optical_flow.png',drawn)
        prvs = next

    cap.release()
    cv2.destroyAllWindows()