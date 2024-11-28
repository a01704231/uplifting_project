import numpy as np
import cv2
import glob
import depthai as dai
from xarm.wrapper import XArmAPI

def fkine(theta_offset, d_offset, alpha, a, q, joint_type):
    FK = np.identity(4)
    for i in range(len(q)):
        if joint_type[i] == "R":
            theta = theta_offset[i] + q[i]
            d = d_offset[i]
        elif joint_type[i] == "P":
            theta = theta_offset[i]
            d = d_offset[i] + q[i]
        cz = np.cos(theta)
        sz = np.sin(theta)
        cx = np.cos(alpha[i])
        sx = np.sin(alpha[i])
        R1 = np.array([[cz, -sz, 0, 0],
                       [sz, cz, 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])
        R2 = np.array([[1, 0, 0, 0],
                       [0, cx, -sx, 0],
                       [0, sx, cx, 0],
                       [0, 0, 0, 1]])
        T1 = np.array([[1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 1, d],
                       [0, 0, 0, 1]])
        T2 = np.array([[1, 0, 0, a[i]],
                       [0, 1, 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])
        FK = FK @ (R1 @ T1 @ T2 @ R2)
    return FK

def move_robot():
    theta_offset = np.transpose(np.deg2rad([0, -90, -90, 0, 0, 0]))
    d_offset = np.transpose([243.3, 0.0, 0.0, 227.6, 0.0, 61.5])
    alpha = np.transpose(np.deg2rad([-90, 180, 90, 90, -90, 0]))
    a = np.transpose([0.0, 200.0, 87.0, 0.0, 0.0, 0.0])
    joint_type = ["R", "R", "R", "R", "R", "R"]
    n = 0
    key = 0
    pipeline = dai.Pipeline()
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setPreviewSize(416, 416)
    cam_rgb.setInterleaved(False)
    xout_video = pipeline.create(dai.node.XLinkOut)
    xout_video.setStreamName("video")
    cam_rgb.preview.link(xout_video.input)
    with dai.Device(pipeline) as device:
        video_queue = device.getOutputQueue(name = "video", maxSize = 4, blocking = False)
        while key != 113:
            in_video = video_queue.get()
            img = in_video.getCvFrame()
            cv2.imshow("image", img)
            key = cv2.waitKey(1) & 0xFF
            if key == 120:
                image = img
                nt = str(n)
                if n < 10:
                    nt = '0' + str(n)
                cv2.imwrite('img/img' + nt + '.jpg', image)
                q = np.array(arm.get_servo_angle(is_radian = True)[1][:6])
                T = fkine(theta_offset, d_offset, alpha, a, q, joint_type)
                text = str(T[0, 0]) + "\n" + str(T[0, 1]) + "\n" + str(T[0, 2]) + "\n" + str(T[1, 0]) + "\n" + str(T[1, 1]) + "\n" + str(T[1, 2]) + "\n" + str(T[2, 0]) + "\n" + str(T[2, 1]) + "\n" + str(T[2, 2]) + "\n" + str(T[0][3]) + "\n" + str(T[1][3]) + "\n" + str(T[2][3]) + "\n"
                f = open('poses.txt', 'a')
                f.write(text)
                f.close()
                n += 1
    cv2.destroyAllWindows()
    print("finish")

def get_Poses():
    global Poses_R, Poses_t
    f = open("poses.txt", "r")
    i = 1
    p = []
    for x in f:
        if np.abs(float(x)) < 0.000001:
            x = 0
        p.append(x)
        if (i == 12):
            Poses_R.append(np.array([[float(p[0]), float(p[1]), float(p[2])],
                                     [float(p[3]), float(p[4]), float(p[5])],
                                     [float(p[6]), float(p[7]), float(p[8])]]))
            Poses_t.append(np.array([float(p[9]), float(p[10]), float(p[11])]))
            p = []
            i = 0
        i += 1
    f.close()

def calibrate():
    point = np.zeros((chess_size[0] * chess_size[1], 3), np.float32)
    point[:, :2] = np.mgrid[0:chess_size[0], 0:chess_size[1]].T.reshape(-1, 2)
    obj_points = []
    img_points = []
    images = sorted(glob.glob('img/*.jpg'))
    for image in images:
        img = cv2.imread(image)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, chess_size, None)
        if ret == True:
            obj_points.append(point)
            #corners2 = corners
            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            img_points.append(corners2)
    ret, camera_mat, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, frame_size, None, None)    
    return camera_mat, dist, img_points

def get_parameters(camera_mat, dist, img_points, i):
    cb_corners = np.zeros((chess_size[0] * chess_size[1], 3), np.float32)
    cb_corners[:, :2] = np.mgrid[0:chess_size[0], 0:chess_size[1]].T.reshape(-1, 2) * squares_edge
    _, rvec, tvec = cv2.solvePnP(cb_corners, img_points[i], camera_mat, dist)
    R_target2cam = cv2.Rodrigues(rvec)[0]
    t_target2cam = tvec
    return R_target2cam, t_target2cam

def calibrate_handeye(camera_mat, dist, img_points):
    R_target2cam_list = []
    t_target2cam_list = []
    for i in range(len(img_points)):
        R_target2cam, t_target2cam = get_parameters(camera_mat, dist, img_points, i)
        R_target2cam_list.append(R_target2cam)
        t_target2cam_list.append(t_target2cam)
        print(Poses_t[i])
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(Poses_R, Poses_t, R_target2cam_list, t_target2cam_list)
    return R_cam2gripper, t_cam2gripper, R_target2cam_list, t_target2cam_list

global Poses_R, Poses_t, squares_edge, chess_size, frame_size
Poses_R = []
Poses_t = []
squares_edge = 13
chess_size = (8, 5)
frame_size = (416, 416)
arm = XArmAPI('192.168.1.177')
arm.motion_enable(enable = True)
arm.set_mode(0)
arm.set_state(0)

if __name__ == "__main__":
    #move_robot()
    get_Poses()
    m, d, img_p = calibrate()
    print("m: ", m)
    print("d: ", d)
    r_c, t_c, r_t, t_t = calibrate_handeye(m, d, img_p)
    print("r: ", r_c)
    print("t: ", t_c)
