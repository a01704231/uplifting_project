import numpy as np
from robodk import robolink, robomath
from xarm.wrapper import XArmAPI
import paho.mqtt.client as mqtt
import time
import sqlite3
from collections import Counter
from uuid import uuid4
import cv2
import csv
import os

def rot2rpyfull(T):
    R = T[0:3, 0:3]
    sy = np.sqrt(R[0,0] ** 2 + R[1,0] ** 2)
    singular = sy < 1e-6
    if not singular:
        x1 = np.arctan2(R[2,1], R[2,2])
        y1 = np.arctan2(-R[2,0], sy)
        z1 = np.arctan2(R[1,0], R[0,0])
        return x1, y1, z1
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
        return x, y, z

def fkine(q):
    FK = np.identity(4)
    for i in range(len(q)):
        theta = theta_offset[i] + q[i]
        d = d_offset[i]
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

def get_ik(t):
    p = robomath.Mat(np.array([[v, -v, 0, t[0]], [-v, -v, 0, t[1]], [0, 0, -1, t[2]], [0, 0, 0, 1]]).tolist())
    ik = robot.SolveIK_All(p)
    return ik

def robot_move0(t):
    j_move = arm.get_inverse_kinematics([t[0], t[1], t[2], np.rad2deg(roll_t), np.rad2deg(pitch_t), np.rad2deg(yaw_t)])
    arm.set_servo_angle(angle = j_move[1][:-1], speed = 25, wait = True)

def robot_moveL1(t):
    arm.set_position(x = t[0], y = t[1], z = arm.get_position()[1][2], roll = roll_t, pitch = pitch_t, yaw = yaw_t, speed = 25, is_radian = True, wait = True)
    arm.set_position(x = t[0], y = t[1], z = t[2], roll = roll_t, pitch = pitch_t, yaw = yaw_t, speed = 25, is_radian = True, wait = True)

def robot_moveL2(t):
    arm.set_position(x = arm.get_position()[1][0], y = arm.get_position()[1][1], z = t[2], roll = roll_t, pitch = pitch_t, yaw = yaw_t, speed = 25, is_radian = True, wait = True)
    arm.set_position(x = t[0], y = t[1], z = t[2], roll = roll_t, pitch = pitch_t, yaw = yaw_t, speed = 25, is_radian = True, wait = True)

def robot_moveJ(ik, t):
    rj = np.array(arm.get_servo_angle()[1][:-1])
    j_move = None
    for j in ik:
        j = [np.round(j[0], 4), np.round(j[1], 4), np.round(j[2], 4), np.round(j[3], 4), np.round(j[4], 4), np.round(j[5], 4)]
        j[3] = j[3] - 360 * int(j[3] / 360)
        j[0] = j[0] - 360 * int(j[0] / 360)
        j[5] = j[5] - 360 * int(j[5] / 360)
        j0_d = np.abs(rj[0] - j[0])
        j5_d = np.abs(rj[5] - j[5])
        if j[1] > 0 and j[2] < 180 and np.abs(j[3]) < 0.5 and j0_d < 136 and j5_d < 136:
            j_move = j
    try:
        arm.set_servo_angle(angle = j_move, speed = 25, wait = True)
    except:
        try:
            robot_move0(t)
            print("inverse kinematic error ", str(t))
        except:
            print("target unreachable ", str(t))

def get_T_camera_target():
    obj_w = 0
    obj_h = 0
    if fruit_i == 0:
        obj_w = apple_w
        obj_h = apple_h
    elif fruit_i == 1:
        obj_w = tang_w
        obj_h = tang_h
    elif fruit_i == 2:
        obj_w = kiwi_w
        obj_h = kiwi_h
    elif fruit_i == 3:
        obj_w = lime_w
        obj_h = lime_h
    if h > w:
        ww = obj_w
        obj_w = obj_h
        obj_h = ww
    x_min = cx - w/2
    x_max = cx + w/2
    y_min = cy - h/2
    y_max = cy + h/2
    img_points = np.array([[x_min, y_min],
                           [x_max, y_min],
                           [x_min, y_max],
                           [x_max, y_max]],
                           dtype=np.float32)
    obj_points = np.array([[0, 0, 0],
                           [obj_w, 0, 0],
                           [0, obj_h, 0],
                           [obj_w, obj_h, 0]],
                           dtype=np.float32)
    _, rvec, t = cv2.solvePnP(obj_points, img_points, cam_m, cam_d, cv2.SOLVEPNP_P3P)
    R = cv2.Rodrigues(rvec)[0]
    T = np.zeros((4, 4))
    T[:3, :3] = R
    T[:3, 3:] = t
    T[3, 3] = 1
    return T

def set_container(new_container):
    if (new_container == True):
        t = time.localtime()
        day = time.strftime("%d", t)
        month = time.strftime("%m", t)
        year = time.strftime("%Y", t)
        hour = time.strftime("%H", t)
        minute = time.strftime("%M", t)
        container_id = day + month + year + hour + minute
        return container_id
    
def count_containers(container):
    conn = sqlite3.connect(path)
    cursor = conn.cursor()
    cursor.execute("select * from pieces")
    rows = cursor.fetchall()
    container_id_counts = Counter(row[1] for row in rows)
    filled = [containers for containers, count in container_id_counts.items() if count == 4]
    conn.close()
    for contain in filled:
        if(container.startswith("0")):
            if(contain == int(container[1:len(container)])):
                return True
        else:
            if(contain == int(container)):
                return True
    return False

def count_error():
    global n_piece, count_p, start, status
    if piece_not_identified == "True":
        status = "error"
        start = time.time()
        count_p = True
    else:
        status = "in operation"
        end = time.time()
        if (end - start) >= 5 and count_p == True:
            n_piece += 1
            count_p = False

def count_piece():
    global n_apple, n_tang, n_kiwi, n_lime
    if fruit_i == 0:
        n_apple += 1
    elif fruit_i == 1:
        n_tang += 1
    elif fruit_i == 2:
        n_kiwi += 1
    elif fruit_i == 3:
        n_lime += 1

def save_data():
    t = time.localtime()
    hh = int(time.strftime("%H", t))
    mm = int(time.strftime("%M", t))
    ss = int(time.strftime("%S", t))
    data = (hh, mm, ss, n_apple, n_tang, n_kiwi, n_lime, n_piece, filled_containers, error_containers)
    conexion = sqlite3.connect(path)
    conexion.execute("insert into data(h, m, s, n_apple, n_tang, n_kiwi, n_lime, n_piece, filled_containers, error_containers) values (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)", data)
    conexion.commit()
    conexion.close()

def save_piece():
    id = str(uuid4())
    t = time.localtime()
    hh = int(time.strftime("%H", t))
    mm = int(time.strftime("%M", t))
    ss = int(time.strftime("%S", t))
    data = (container_id, hh, mm, ss, fruit, id, t5[0], t5[1], t5[2])
    conexion = sqlite3.connect(path)
    conexion.execute("insert into pieces(container_id, h, m, s, fruit_type, id, x, y, z) values (?, ?, ?, ?, ?, ?, ?, ?, ?)", data)
    conexion.commit()
    conexion.close()

def publish_1():
    global state_prev, pose_prev, status_prev
    if state_prev != state:
        client.publish("state", state)
        state_prev = state
    if str(list(np.round(pose_prev, 4))) != str(list(np.round(pose, 4))):
        v = np.round(pose[:3, 3], 4)
        roll, pitch, yaw = np.round(rot2rpyfull(pose), 4)
        client.publish("pose", str([v[0], v[1], v[2], roll, pitch, yaw]))
        pose_prev = pose
    if status_prev != status:
        client.publish("status", status)
        status_prev = status

def publish_2():
    global filled_containers, error_containers
    client.publish("apple", n_apple)
    client.publish("tangerine", n_tang) 
    client.publish("kiwi", n_kiwi)
    client.publish("lime", n_lime)
    client.publish("filled", filled_containers)
    client.publish("error", error_containers)

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("stop")
    client.subscribe("piece")
    client.subscribe("detect")
    client.subscribe("detection")

def on_message(client, userdata, message):
    global stop, piece_not_identified, detect, fruit, fruit_i, cx, cy, w, h
    if message.topic == "stop":
        stop = str(message.payload.decode('utf-8'))
    if (state == "find" or state == "verify" or state == "check_errors") and not begin:
        if message.topic == "piece":
            piece_not_identified = str(message.payload.decode('utf-8'))
        elif message.topic == "detect":
            detect = str(message.payload.decode('utf-8'))
        elif message.topic == "detection":
            detection = str(message.payload.decode('utf-8'))
            if detection != "":
                l = detection.split(', ')
                fruit = l[0][2:-1]
                cx = float(l[1])
                cy = float(l[2])
                w = float(l[3])
                h = float(l[4][:-1])
                if fruit == "Apple":
                    fruit_i = o_apple
                elif fruit == "Tangerine":
                    fruit_i = o_tang
                elif fruit == "Kiwi":
                    fruit_i = o_kiwi
                elif fruit == "Lime":
                    fruit_i = o_lime

global apple_w, tang_w, kiwi_w, lime_w, apple_h, tang_h, kiwi_h, lime_h, containers, v, roll_t, pitch_t, yaw_t, theta_offset, d_offset, alpha, a, cam_m, cam_d
apple_w = 55
tang_w = 50
kiwi_w = 70
lime_w = 50
apple_h = 55
tang_h = 50
kiwi_h = 50
lime_h = 50
containers = 3
x = [200, 350]
y = [200, 50, -150, -225]
z1 = 260
z2 = 140
t0 = [-300, 0, z1]
t3 = [212.132, 212.132, z1]
v = 0.707107
roll_t, pitch_t, yaw_t = rot2rpyfull(np.array([[v, -v, 0], [-v, -v, 0], [0, 0, -1]]))
theta_offset = np.transpose(np.deg2rad([0, -90, -90, 0, 0, 0]))
d_offset = np.transpose([243.3, 0.0, 0.0, 227.6, 0.0, 61.5])
alpha = np.transpose(np.deg2rad([-90, 180, 90, 90, -90, 0]))
a = np.transpose([0.0, 200.0, 87.0, 0.0, 0.0, 0.0])
cam_m = np.array([[640.08211832, 0, 225.00259728], [0, 640.8053746, 217.87483503], [0, 0, 1]], dtype = np.float32)
cam_d = np.array([[0.13054298, -0.47003759, 0.00482374, 0.00773899, 1.60409086]], dtype = np.float32)
T_f_c = np.array([[0.59778821, -0.80147952,  0.01672833, -42.13868071], [0.80160953,  0.59784482, -0.00193359, 40.52751636], [-0.00845121,  0.01456547,  0.9998582, 6.66498243], [0, 0, 0, 1]])

global state, pose, status, state_prev, pose_prev, status_prev, n_apple, n_tang, n_kiwi, n_lime, n_piece, filled_containers, error_containers, count_p, begin, start, o_apple, o_tang, o_kiwi, o_lime, t5
stop = ""
piece_not_identified = ""
detect = ""
fruit = ""
fruit_i = None
cx = None
cy = None
w = None
h = None
state = "find"
pose = np.array([])
status = ""
state_prev = ""
pose_prev = np.array([])
status_prev = ""
n_apple = 0
n_tang = 0
n_kiwi = 0
n_lime = 0
n_piece = 0
filled_containers = 0
error_containers = 0
count_p = False
begin = True
begin_d = True
container_id = None
new_container = True
gripper = "open"
ik_z1 = []
ik_z2 = []

path = str(os.getcwd()) + "/robot_data.db"
print(path)
conexion = sqlite3.connect(path)
conexion.execute("create table if not exists data(n integer primary key autoincrement, h real, m real, s real, n_apple real, n_tang real, n_kiwi real, n_lime real, n_piece real, filled_containers real, error_containers real)")
conexion.execute("create table if not exists pieces(n integer primary key autoincrement, container_id string, h real, m real, s real, id real, fruit_type real, x real, y real, z real)")
conexion.execute("create table if not exists control(apple integer, tang integer, kiwi integer, lime integer)")
conexion.commit()
conexion.close()
conexion = sqlite3.connect(path)
o_apple = int(conexion.cursor().execute("select apple from control").fetchall()[0][0])
o_tang = int(conexion.cursor().execute("select tang from control").fetchall()[0][0])
o_kiwi = int(conexion.cursor().execute("select kiwi from control").fetchall()[0][0])
o_lime = int(conexion.cursor().execute("select lime from control").fetchall()[0][0])
n_containers = int(conexion.cursor().execute("select containers from control").fetchall()[0][0])
conexion.close()
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect("192.168.1.128", 1883, 60)
client.loop_start()
print("connected")
client.publish("gripper", "")
client.publish("target", "")
client.publish("id", "")
client.publish("state", "")
client.publish("pose", "")
client.publish("status", "")
client.publish("apple", 0)
client.publish("tangerine", 0)
client.publish("kiwi", 0)
client.publish("lime", 0)
client.publish("filled", 0)
client.publish("error", 0)
rdk = robolink.Robolink()
robot = rdk.Item('uFactory Lite6')
rdk.setRunMode(run_mode = 6)
for j in range(len(y)):
    for i in range(len(x)):
        ik_z1.append(get_ik([x[i], y[j], z1]))
        ik_z2.append(get_ik([x[i], y[j], z2]))
ik_t0 = get_ik(t0)
ik_t3 = get_ik(t3)
rdk.Disconnect()
rdk.Finish()
arm = XArmAPI('192.168.1.177')
arm.motion_enable(enable = True)
arm.set_mode(0)
arm.set_state(0)
arm.set_servo_angle(angle = [0, 0, 30, 0, 30, 45], speed = 25, wait = True)
print("start")
start = time.time()
time_i = time.time()
time_f = time.time()
start_d = time.time()
end_d = time.time()
error = False

while filled_containers < n_containers:
    if (new_container):
        container_id = set_container(new_container)
        new_container = False
    pose = fkine(np.deg2rad(np.array(arm.get_servo_angle()[1][:-1])))
    count_error()
    publish_1()
    if stop == "True":
        state = "stop"
        status = "standby"
    if state == "find": 
        status = "in operation"
        if begin:
            robot_moveJ(ik_t3, t3)
            robot_moveJ(ik_t0, t0)
            begin = False
            begin_d = True
        if detect == "True":
            if begin_d:
                start_d = time.time()
                begin_d = False
            end_d = time.time()
            if (end_d - start_d >= 3):
                state = "pick"
                detect = ""
    elif state == "pick":
        T_b_f = fkine(np.deg2rad(np.array(arm.get_servo_angle()[1][:-1])))
        T_c_o = get_T_camera_target()
        t1 = T_b_f @ T_f_c @ T_c_o
        t1 = [t1[0][3] - 25, t1[1][3] -20, z2]
        client.publish("target", str(np.array(t1)))
        robot_moveL1(t1)
        client.publish("gripper", "close")
        arm.set_pause_time(10, wait = True)
        state = "prepare_to_place"
    elif state == "prepare_to_place":
        n = fruit_i
        t4 = [x[n % 2], y[n // 2], z1]
        t5 = [x[n % 2], y[n // 2], z2]
        client.publish("target", "")
        client.publish("id", container_id)
        robot_moveL2(t0)
        robot_moveJ(ik_t3, t3)
        robot_moveJ(ik_z1[n], t4)
        arm.set_pause_time(2, wait = True)
        state = "verify"
    elif state == "verify":
        if detect == "True":
            state = "discard_fruit"
            detect = ""
        elif detect == "False":
            state = "place_fruit"
    elif state == "discard_fruit":
        n = fruit_i + 4
        t6 = [x[n % 2], y[n // 2], z1]
        t7 = [x[n % 2], y[n // 2], z2]
        robot_moveJ(ik_z1[n], t6)
        robot_moveJ(ik_z2[n], t7)  
        client.publish("gripper", "open")
        arm.set_pause_time(5, wait = True)
        robot_moveJ(ik_z1[n], t6)
        state = "find"
        begin = True
        count_piece()
        save_piece()
        publish_2()
    elif state == "place_fruit":
        robot_moveJ(ik_z2[n], t5)
        client.publish("gripper", "open")
        arm.set_pause_time(5, wait = True)
        robot_moveJ(ik_z1[n], t4)
        state = "find"
        begin = True
        count_piece()
        save_piece()
        publish_2()
    elif state == "check_errors":
        error = False
        for k in range(4):
            n = k
            t4 = [x[n % 2], y[n // 2], z1]
            robot_moveJ(ik_z1[n], t4)
            if fruit_i != n:
                error_containers += 1
                error = True
                break
        if not error:
            filled_containers += 1
        state = "find"
        begin = True
        detect = ""
        save_piece()
        publish_2()
    elif state == "stop":
        arm.set_state(4)
    new_container = count_containers(container_id)
    if new_container:
        fruit_i = None
        state = "check_errors"
        begin = False
    time_f = time.time()
    if (time_f - time_i) > 5:
        save_data()
        time_i = time_f

with open("data.csv", mode = "w", newline = "") as f:
    writer = csv.writer(f)
    conn = sqlite3.connect(path)
    cursor = conn.cursor()
    cursor.execute("select * from data")
    for row in cursor:
        writer.writerow(row)
    conn.close()
with open("pieces.csv", mode = "w", newline = "") as f:
    writer = csv.writer(f)
    conn = sqlite3.connect(path)
    cursor = conn.cursor()
    cursor.execute("select * from pieces")
    for row in cursor:
        writer.writerow(row)
    conn.close()
