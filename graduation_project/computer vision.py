import cv2
import numpy as np
import math
import socket

exit_lopp = False
z_on_board = 27
width = 640
height = 480
# Specific geometry for bitbeambot:

e = 36.0
f = 81.84
re = 520
rf = 220

# Trigonometric constants
s = 165 * 2
sqrt3 = math.sqrt(3.0)
pi = 3.141592653
sin120 = sqrt3 / 2.0
cos120 = -0.5
tan60 = sqrt3
sin30 = 0.5
tan30 = 1.0 / sqrt3
# pts =np.zeros((4,2),int)
# c=0
angles = np.zeros((12, 3),float)
#angles=list([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])

'''def mouse_drawing(event,x,y,flags,params):
    global c
    if event==cv2.EVENT_LBUTTONDOWN:
        pts[c]=x,y
        c=c+1
        print(pts)
'''

'''
def send_data_to_plc(angles):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host = "192.168.0.1"
    port = 2000
    s.connect((host, port))
    for x in range(1,13):
        w = s.recv(1024)
        w=w.decode("utf_8")
        #if int(w) == 0:
        d1 = str(angles[0][0]) + str(angles[0][1]) + str(angles[0][2])
        s.send(d1.encode("utf_8"))
        elif int(w) == 1:
            d2 = str(angles[1][0]) + str(angles[1][1]) + str(angles[1][2])
            s.send(d2.encode("utf_8"))
        elif int(w) == 2:
            d3 = str(angles[2][0]) + str(angles[2][1]) + str(angles[2][2])
            s.send(d3.encode("utf_8"))
        elif int(w) == 3:
            d4 = str(angles[3][0]) + str(angles[3][1]) + str(angles[3][2])
            s.send(d4.encode("utf_8"))
        elif int(w) == 4:
            d5 = str(angles[4][0]) + str(angles[4][1]) + str(angles[4][2])
            s.send(d5.encode("utf_8"))
        elif int(w) == 5:
            d6 = str(angles[5][0]) + str(angles[5][1]) + str(angles[5][2])
            s.send(d6.encode("utf_8"))
        elif int(w) == 6:
            d7 = str(angles[6][0]) + str(angles[6][1]) + str(angles[6][2])
            s.send(d7.encode("utf_8"))
        elif int(w) == 7:
            d8 = str(angles[7][0]) + str(angles[7][1]) + str(angles[7][2])
            s.send(d8.encode("utf_8"))
        elif int(w) == 8:
            d9 = str(angles[8][0]) + str(angles[8][1]) + str(angles[8][2])
            s.send(d9.encode("utf_8"))
        elif int(w) == 9:
            d10 = str(angles[9][0]) + str(angles[9][1]) + str(angles[9][2])
            s.send(d10.encode("utf_8"))
        elif int(w) == 10:
            d11 = str(angles[10][0]) + str(angles[10][1]) + str(angles[10][2])
            s.send(d11.encode("utf_8"))
        elif int(w) == 11:
            d12 = str(angles[11][0]) + str(angles[11][1]) + str(angles[11][2])
            s.send(d12.encode("utf_8"))

    s.close()
'''


def get_object_location(only):
    # function that get targets location relative to camera frame
    column_sums = np.matrix(np.sum(only, 0))
    column_numbers = np.matrix(np.arange(width))
    column_mult = np.multiply(column_sums, column_numbers)
    total = np.sum(column_mult)
    total_total = np.sum(np.sum(only))

    column_location = total / total_total
    x_on_board = column_location * (350 / width)
    row_sums = np.matrix(np.sum(only, 1))
    row_sums = row_sums.transpose()
    row_numbers = np.matrix(np.arange(height))
    row_mult = np.multiply(row_sums, row_numbers)
    # row_mult = row_sums.dot(row_numbers))
    total1 = np.sum(row_mult)
    total_total1 = np.sum(np.sum(only))

    row_location = total1 / total_total1
    y_on_board = row_location * (220 / height)
    print(x_on_board, y_on_board)
    return x_on_board, y_on_board


def get_Transformation_coordinate(x_on_board, y_on_board, z_on_board):
    # transformation coordinate among base frame and camera frame
    R180_x = [[1, 0, 0], [0, np.cos(np.pi), -np.sin(np.pi)], [0, np.sin(np.pi), np.cos(np.pi)]]
    rad = (-90.0 / 180.0) * np.pi
    Rz = [[np.cos(rad), -np.sin(rad), 0], [np.sin(rad), np.cos(rad), 0], [0, 0, 1]]
    R0_c = np.dot(R180_x, Rz)
    do_c = [[-9.8], [-225.68], [-550]]
    H0_c = np.concatenate((R0_c, do_c), 1)
    H0_c = np.concatenate((H0_c, [[0, 0, 0, 1]]), 0)
    pc = [[x_on_board], [y_on_board], [z_on_board], [1]]
    p0 = np.dot(H0_c, pc)
    x_to_base = p0[0]
    y_to_base = p0[1]
    z_to_base = p0[2]
    return x_to_base, y_to_base, z_to_base


# Inverse kinematics
# Helper functions, calculates angle theta1 (for YZ-pane)
def angle_yz(x0, y0, z0, theta=None):
    y1 = -0.5 * 0.57735 * f  # f/2 * tg 30
    y0 -= 0.5 * 0.57735 * e  # shift center to edge
    # z = a + b*y
    a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2.0 * z0)
    b = (y1 - y0) / z0

    # discriminant
    d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf)
    if d < 0:
        return [1, 0]  # non-existing povar.  return error, theta

    yj = (y1 - a * b - math.sqrt(d)) / (b * b + 1)  # choosing outer povar
    zj = a + b * yj
    theta = math.atan(-zj / (y1 - yj)) * 180.0 / pi + (180.0 if yj > y1 else 0.0)

    return [0, theta]  # return error, theta


def inverse(x0, y0, z0):
    theta1 = 0
    theta2 = 0
    theta3 = 0
    status = angle_yz(x0, y0, z0)

    if status[0] == 0:
        theta1 = status[1]
        status = angle_yz(x0 * cos120 + y0 * sin120,
                          y0 * cos120 - x0 * sin120,
                          z0,
                          theta2)
    if status[0] == 0:
        theta2 = status[1]
        status = angle_yz(x0 * cos120 - y0 * sin120,
                          y0 * cos120 + x0 * sin120,
                          z0,
                          theta3)
    theta3 = status[1]

    return [status[0], theta1, theta2, theta3]


# print👍

cap = cv2.VideoCapture(1)
while True:
    _, frame = cap.read()

    cv2.imshow("image", frame)
    pts1 = np.float32([[37, 61], [605, 52], [34, 422], [619, 413]])
    pts2 = np.float32([[0, 0], [width, 0], [0, height], [width, height]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    frame = cv2.warpPerspective(frame, matrix, (width, height))
    # perspective transformation for the robot and get(ROI (region of interest))
    cv2.imshow("Perspective transformation", frame)
    red = np.matrix(frame[:, :, 2])
    green = np.matrix(frame[:, :, 1])
    blue = np.matrix(frame[:, :, 0])
    cv2.imshow('red pres', red)
    cv2.imshow('blue pres', green)
    cv2.imshow('green pres', blue)
    only = np.int16(red) - np.int16(green) - np.int16(blue)
    # cv2.imshow('red 1', only)
    only[only < 1] = 0
    only[only >= 1] = 255
    only = np.uint8(only)
    cv2.imshow('red', only)
    mask = np.ones((5, 5), np.uint8)
    opening = cv2.morphologyEx(only, cv2.MORPH_OPEN, mask)
    cv2.imshow("masked", opening)
    _, labels = cv2.connectedComponents(opening, 4)
    b = np.matrix(labels)
    print(np.amax(labels))
    for n in range(1, 13):
        obj = b == n
        # print(obj)
        obj = np.uint8(obj)
        obj[obj > 0] = 255
        cv2.imshow(str(n), obj)
        x_on_board, y_on_board = get_object_location(obj)
        x, y, z = get_Transformation_coordinate(x_on_board, y_on_board, z_on_board)
        print(x, y, z)
        error, th1, th2, th3 = inverse(x, y, z)
        print(th1, th2, th3)
        # print
        # print(angles)
        if error == 0:
            angles[n - 1][0] = round(th1,1)

            angles[n - 1][1] = round(th2,1)
            angles[n - 1][2] = round(th3,1)
            '''
            angles[n - 1][0] = format(angles[n - 1][0], '.1f')
            new_angel7=nf(angles[n - 1][0],2,1)
            angles[n - 1][0]=new_angel7
            print(angles[n - 1][0])
            angles[n - 1][1] = format(angles[n - 1][1], '.1f')
            new_angel8 = nf(angles[n - 1][1], 2, 1)
            angles[n - 1][1]=new_angel8
            print(angles[n - 1][1])
            angles[n - 1][2] = format(angles[n - 1][2], '.1f')
            new_angel9 = nf(angles[n - 1][2], 2, 1)
            angles[n - 1][2]=new_angel9
            print(angles[n - 1][2])
            '''
            '''
        if float(angles[n - 1][0]) > 1.0 and float(angles[n - 1][0]) < 10.0:
            angel1 = str(angles[n - 1][0])
            new_angel1 = angel1.zfill(4)
            angles[n - 1][0] = new_angel1
            print(angles[n - 1][0])
        elif float(angles[n - 1][1]) > 1.0 and float(angles[n - 1][1]) < 10.0:
            angel2 = str(angles[n - 1][1])
            new_angel2 = angel2.zfill(4)
            angles[n - 1][1] = new_angel2
            print(angles[n - 1][1])

        elif float(angles[n - 1][2]) > 1.0 and float(angles[n - 1][2]) < 10.0:
            angel3 = str(angles[n - 1][2])
            new_angel3 = angel3.zfill(4)
            angles[n - 1][2] = new_angel3
            print(angles[n - 1][2])
        '''
    if not np.all(angles == 0):
        print(angles)
        break

    k = cv2.waitKey(1)
    if k == 27:
        break

# send_data_to_plc(angles)


# cv2.destroyAllWindows()
cap.release()