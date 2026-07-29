import cv2
import numpy as np
import copy
import glob
import math
import time
from time import sleep
import Connection as conn

cap = cv2.VideoCapture(1)

import queue as Q

images = glob.glob('*.jpg')

def check_boundaries(ex, ey, nx, ny): #ex, ey :- end points of frame
    if nx > -1 and ny > -1 and nx < ex and ny < ey:
        return True
    else:
        return False

def printx(x):
    #print x
    pass

def check_obstacles(arr, ansx, ansy):  #function to check whether a given point is on obstacle or not
    if arr[ansx][ansy][0] == 255:
        return True
    else:
        return False

def feasible(arr, x, y):  #function to check if a point is feasible or not
    ex, ey, ez = arr.shape
    x = int(x)
    y = int(y)

    if check_boundaries(ex, ey, x, y):
        return not check_obstacles(arr, x, y)
    else:
        return False

def dist(sx, sy, x, y, theta, arr, q_star):  #distance of obstacle in direction theta in radians
    ansx = sx
    ansy = sy
    flag = True
    count = 1
    while True:
        if count > q_star:
            return (-1, -1)
        ansx = sx + count*math.sin(theta)
        ansy = sy + count*math.cos(theta)

        if check_boundaries(x, y, ansx, ansy) == False:
            break
        else:
            if check_obstacles(arr, ansx, ansy) == True:
                break
        count += 1


    return (ansx-sx,ansy- sy)

def obstacle_force(arr, sx, sy, q_star, theta1): #sx,sy :- source    dx, dy:- destination    q-star:- threshold distance of obstacles
    forcex = 0
    forcey = 0
    neta = 300000000000
    x, y , z= arr.shape
    for i in range(-8, 9):
        (ox,oy) = dist(sx, sy, x, y, (theta1 + i*math.pi/16 + 2*math.pi)%(2*math.pi), arr, q_star)
        theta = (theta1 + i*math.pi/16 + 2*math.pi)%(2*math.pi)
        fx = 0
        fy = 0
        #print 'ox ', ox, 'oy ', oy
        if ox == -1 or oy == -1:
            fx = 0
            fy = 0
        else:
            ox = math.fabs(ox)
            oy = math.fabs(oy)
            d = math.hypot(ox, oy)
            if d == 0:
                d = 1
            f = (neta*(1.0/q_star- 1.0/d))/(d*d)
            fx = f*math.sin(theta)
            fy = f*math.cos(theta)

        forcex += fx
        forcey += fy
    thet = math.atan2(forcex, forcey)
    arr1 = arr
    cv2.line(arr1, (sy, sx), (int(sy + 10*math.cos(thet)), int(sx + math.sin(thet))), (0, 255, 255), 1)
    cv2.imshow('arr', arr1)
    k = cv2.waitKey(20)
    return (forcex, forcey)

def goal_force(arr, sx, sy, dx, dy, d_star): # sx, sy :- source  dx, dy:- destination   d_star:- threshold distance from goal
    forcex = 0
    forcey = 0
    tau = 1000000  #constant
    printx('10')
    d = math.sqrt((dx-sx)*(dx-sx) + (dy-sy)*(dy-sy))
    if d > d_star:
        forcex += ((d_star*tau*math.sin(math.atan2(dx-sx, dy-sy))))
        forcey += ((d_star*tau*math.cos(math.atan2(dx-sx, dy-sy))))

    else:
        forcex += ((dx-sx)*tau)
        forcey += ((dy-sy)*tau)

    printx('11')
    return (forcex, forcey)

def path_planning(arr, sx1, sy1, dx, dy, theta):

    theta12= theta

    '''
    :param arr: input map
    :param sx1: source x
    :param sy1: source y
    :param dx: destination x
    :param dy: destination y
    :return: path
    '''

    #Parameters Declaration

    flx = 10000  #maximum total force in x
    fly = 10000  #maximum total force in y
    v = 5 #velocity magnitude
    t = 1 #time lapse
    x,y,z = arr.shape
    theta_const = math.pi*45/180  #maximum allowed turn angle
    q_star = 350
    d_star = 2

    if arr[sx1][sy1][0] == 255 or arr[dx][dy][0] == 255:
        return []
    sx = sx1
    sy = sy1
    '''
        if Q and P are two vectors and @ is angle between them

        resultant ,R = (P^2 + R^2 + 2*P*Q cos @)^(1/2)

        resultant, theta = atan((Q*sin @)/(P+Q*cos @))
    '''

    (fx, fy) = obstacle_force(arr, sx, sy, q_star, theta)
    (gx, gy) = goal_force(arr, sx, sy, dx, dy, d_star)

    tx = gx+fx
    ty = gy+fy
    if(tx < 0):
        tx = max(tx, -flx)
    else:
        tx = min(tx, flx)
    if(ty < 0):
        ty = max(ty, -fly)
    else:
        ty = min(ty, fly)
    theta1 = math.atan2(tx, ty)

    if arr[sx][sy][0] == 255:
        print(gx, gy, fx, fy)
        print('tx ', tx, ' ty ', ty, 'sx ', sx, ' sy ', sy)
        print(theta1*180/math.pi, theta*180/math.pi)

    P = v
    angle = theta1-theta  #angle between velocity and force vector

    Q = math.sqrt(tx*tx + ty*ty)

    theta2 = math.atan2((Q*math.sin(angle)),((P + Q*math.cos(angle))))   #resultant angle with velocity

    if theta2 < 0:
        theta2 = max(theta2, -theta_const)
    else:
        theta2 = min(theta2, theta_const)

    theta += theta2

    theta = (theta + 2*math.pi)%(2*math.pi)

    sx = sx + v*math.sin(theta)
    sy = sy + v*math.cos(theta)
    sx = int(sx)
    sy = int(sy)

    if not check_boundaries(x, y, sx, sy):
        print('out of boundaries' , sx, sy)
    print('sx ', sx, ' sy')
    return (sx, sy, theta2)

def show_image(im):
    cv2.imshow('image', im)
    k = cv2.waitKey(0)

def find_goal(img):
    # converting to HSV
    frame = copy.copy(img)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #show_image(hsv)

    lower_blue = np.array([113, 40, 29])
    upper_blue = np.array([123, 180, 255])

    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    #show_image(mask)
    result = cv2.bitwise_and(frame, frame, mask=mask)
    #show_image(result)
    blur = cv2.blur(result, (5, 5))

    bw = cv2.cvtColor(blur, cv2.COLOR_HSV2BGR)
    bw2 = cv2.cvtColor(bw, cv2.COLOR_BGR2GRAY)

    ret, th3 = cv2.threshold(bw2, 30, 255, cv2.THRESH_BINARY)
    # th3 = cv2.adaptiveThreshold(bw2,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
    # cv2.THRESH_BINARY,11,2)
    edges = cv2.Canny(th3, 100, 200)
    th4 = copy.copy(th3)

    perimeter = 0
    j = 0
    _res = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contours = _res[0] if len(_res) == 2 else _res[1]
    # print len(contours)
    # if(len(contours) > 5):
    #    continue
    cnt = np.array([])
    for i in range(len(contours)):
        if (perimeter < cv2.contourArea(contours[i])):
            perimeter = cv2.contourArea(contours[i])
            j = i;
            cnt = contours[j]
    if (len(cnt) == 0):
        return (-1, -1)
    cv2.drawContours(frame, cnt, -1, (0, 255, 0), 3)
    x = 0
    y = 0
    #print 'find goal'
    #print len(cnt), j
    #print cnt
    for i in range(len(cnt)):
        x = x + cnt[i][0][0]
        y = y + cnt[i][0][1]
    x = x/len(cnt)
    y = y/len(cnt)
    #print x, y
    x = int(x)
    y = int(y)
    cv2.circle(frame, (x, y), 5, (255, 0, 255), -1)

    cv2.imshow('image', frame)
    cv2.imwrite('goal.jpg', frame)
    k = cv2.waitKey(0)

    return (int(x), int(y))

def find_robot(frame):
    im = copy.copy(frame)
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    lower = np.array([50, 28, 0])
    upper = np.array([60, 168, 255])
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(im, im, mask=mask)
    blur = cv2.blur(result, (5, 5))
    bw = cv2.cvtColor(blur, cv2.COLOR_HSV2BGR)
    bw2 = cv2.cvtColor(bw, cv2.COLOR_BGR2GRAY)
    ret, th3 = cv2.threshold(bw2, 30, 255, cv2.THRESH_BINARY)
    edges = cv2.Canny(th3, 100, 200)
    th4 = copy.copy(th3)
    perimeter = 0
    j = 0
    _res = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contours = _res[0] if len(_res) == 2 else _res[1]
    cnt = np.array([])
    for i in range(len(contours)):
        if (perimeter < cv2.contourArea(contours[i])):
            perimeter = cv2.contourArea(contours[i])
            j = i;
            cnt = contours[j]

    x = 0
    y = 0
    for i in range(len(cnt)):
        x = x + cnt[i][0][0]
        y = y + cnt[i][0][1]
    x = x / len(cnt)
    y = y / len(cnt)
    #print x, y
    x = int(x)
    y = int(y)
    cv2.circle(im, (x, y), 5, (255, 0, 255), 2)
    cv2.imshow('img', im)
    k = cv2.waitKey(0)
    cv2.imwrite('robot.jpg', im)
    #show_image(im)
    return (int(x), int(y))

def get_direction():
    direction = 0
    return direction

def classify(img):
    cimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img2 = cv2.medianBlur(cimg, 13)

    ret, thresh1 = cv2.threshold(cimg, 100, 120, cv2.THRESH_BINARY)
    t2 = copy.copy(thresh1)

    x, y = thresh1.shape
    arr = np.zeros((x, y, 3), np.uint8)
    final_contours = []
    _res = cv2.findContours(t2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = _res[0] if len(_res) == 2 else _res[1]
    #cv2.imshow('image', image)
    #k = cv2.waitKey(0)
    for i in range(len(contours)):
        cnt = contours[i]
        if cv2.contourArea(cnt) > 3600 and cv2.contourArea(cnt) < 25000:
            cv2.drawContours(img, [cnt], -1, [0, 255, 255])
            cv2.fillConvexPoly(arr, cnt, [255, 255, 255])
            final_contours.append(cnt)
    cv2.imshow('arr', arr)
    k = cv2.waitKey(0)
    return arr

def negate(arr):
    (x, y, z) = arr.shape
    arr1 = np.zeros((x, y, 3), np.uint8)
    for i in range(x):
        for j in range(y):
            if arr[i][j][0] == 255:
                arr1[i][j] = [0, 0, 0]
            else:
                arr1[i][j] = [255, 255, 255]
    return arr1

def getAngle(po, im):
    angle = math.atan2(float(po[1][0]-po[0][0]),float(po[1][1]-po[0][1]))
    #angle = (angle + 2*math.pi)% (2*math.pi)
    return angle

count = 0

def main():
    counter = 1

    im = cv2.imread('sample1.jpg')
    input_image = copy.copy(im)
    (x, y, z) = im.shape
    print(x, y, z)
    img1 = copy.copy(im)
    arr = classify(img1)
    arr1 = negate(arr)
    cv2.imshow('arr', arr)
    k = cv2.waitKey(0)
    dx, dy = 50, 50
    sx1, sy1 = 500, 1000
    (dy, dx) = find_goal(im)
    (sy1, sx1) = find_robot(im)
    (sx2, sy2) = (sx1, sy1)
    print('sx1, sy1 : ', sx1, sy1)
    cv2.circle(img1, (sy1, sx1), 1, (255, 0 , 0))
    cv2.circle(img1, (dy, dx), 1, (255, 0, 0))
    cv2.imshow('arr', img1)
    k = cv2.waitKey(20)
    count = 0
    while True:

        if sx2 + 20 > dx and sx2-20 < dx and sy2 + 20 > dy and sy2-20 < dy:
            print('reached goal')
            break

        po = []
        po.append([sx1, sy1])
        po.append([sx2, sy2])
        direction = getAngle(po, im)
        if count == 0:
            direction = 0
        count = count + 1
        d1 = direction
        print(sx2, sy2, direction, 'attr')
        (x, y, theta2) = path_planning(arr, sx2, sy2, dx, dy, direction)
        print('theta2', theta2*180/math.pi)
        po = []
        po.append([sx2, sy2])
        po.append([x, y])
        theta = getAngle(po, im)
        direction = theta-d1
        cv2.line(im, (sy2, sx2), (int(sy2 + 20*math.cos(theta2)), int(sx2 + 20*math.sin(theta2))), (255, 255, 0), 2)
        cv2.line(im, (sy2, sx2), (y, x), (0, 0, 255), 2)
        cv2.line(input_image, (sy2, sx2), (y, x), (255, 255, 255), 1)
        cv2.imshow('im', im)
        cv2.imshow('input', input_image)
        k = cv2.waitKey(20)
        (sx1, sy1) = (sx2, sy2)
        (sx2, sy2) = (x, y)

    cv2.imwrite('output/2.jpg', input_image)
    cv2.imshow('arr', arr)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

main()