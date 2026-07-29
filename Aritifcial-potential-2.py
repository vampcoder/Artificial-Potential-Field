import cv2
import numpy as np
import copy
import glob
import math
import time
from time import sleep

images = glob.glob('*.jpg')

def check_boundaries(ex, ey, nx, ny): #ex, ey :- end points of frame
    if nx > -1 and ny > -1 and nx < ex and ny < ey:
        return True
    else:
        return False

def printx(x):
    #print x
    pass

def check_obstacles(arr, ansx, ansy):
    if arr[ansx][ansy][0] == 255:
        return True
    else:
        return False

def feasible(arr, x, y):
    ex, ey, ez = arr.shape
    x = int(x)
    y = int(y)

    if check_boundaries(ex, ey, x, y):
        return not check_obstacles(arr, x, y)
    else:
        return False

def obstacle(j, angle, cx, cy, arr):
    i = j+1
    while True:
        x = int(cx + i*math.sin(angle))
        y = int(cy + i*math.cos(angle))
        if not feasible(arr, x, y):
            break
        i += 1
    return i-j

def path(arr, sx, sy, dx, dy):
    sol = []
    rd = math.pi/8  #robot direction
    #robot size
    rx = 10
    ry = 10
    rspeed = 10   #robot speed
    mrspeed = 10  #max robot speed
    #s = 10  #safety distance
    macc = 10  #max acceleration
    maxt = 10*math.pi/180  #maximum turn
    dt = 30  #distance threshold
    k = 3   #degree of calculating potentials
    apscaling = 300000  #scaling factor for attractive potential
    rpscaling = 300000  #scaling factor for repulsive potential
    minApotential =  0.5 #minimum attractive potential
    rdiagonal = math.sqrt((rx/2)**2 + (ry/2)**2)

    ##################################
    ##################################

    ex, ey, ez = arr.shape  #boundary values

    #current position
    cx = sx
    cy = sy
    ctheta = rd #current direction
    pathfound = False

    if sx <= -1 or sy <= -1 or sx >= ex or sy >= ey:
        print ('No solution exist as source is either on obstacle or outside boundary')
        return sol

    if dx <= -1 or dy <= -1 or dx >= ex or dy >= ey:
        print('No solution exist as goal is either on obstacle or outside boundary')
        return sol

    pi = math.pi
    while not pathfound:
        #calculate distance from obstacle at front
        df = obstacle(rx/2, ctheta, cx,cy, arr)

        #calculate distance from obstacle at left
        dl = obstacle(ry/2, ctheta-pi/2, cx,cy, arr)

        #calculate distance from obstacle at right
        dr = obstacle(ry/2, ctheta+pi/2, cx,cy, arr)

        #calculate distance from front-right-diagonal
        dfrd = obstacle(rdiagonal, ctheta+pi/4, cx,cy, arr)

        #calculate distance from front-left-diagonal
        dfld = obstacle(rdiagonal, ctheta-pi/4, cx,cy, arr)

        #calculate angle and distance from goal

        gtheta = math.atan2(dx-cx, dy-cy)

        dg =  math.sqrt((cx-dx)*(cx-dx)+ (cy-dy)*(cy-dy))  #distance from goal

        if dg < dt:
            pathfound = True

        #calculate Potetials   :

        rPx = ((1.0/df)**k)*math.sin(ctheta) + ((1.0/dl)**k)*math.sin(ctheta-pi/2) + ((1.0/dr)**k)*math.sin(ctheta+pi/2) + ((1.0/dfld)**k)*math.sin(ctheta-pi/4) + ((1.0/dfrd)**k)*math.sin(ctheta+pi/4)
        rPy = ((1.0/df)**k)*math.cos(ctheta) + ((1.0/dl)**k)*math.cos(ctheta-pi/2) + ((1.0/dr)**k)*math.cos(ctheta+pi/2) + ((1.0/dfld)**k)*math.cos(ctheta-pi/4) + ((1.0/dfrd)**k)*math.cos(ctheta+pi/4)

        aPx = max(((1.0/dg)**k)*apscaling, minApotential)* math.sin(gtheta)
        aPy = max(((1.0/dg)**k)*apscaling, minApotential)* math.cos(gtheta)

        tPx = aPx-rPx*rpscaling
        tPy = aPy-rPy*rpscaling

        #calculate new direction
        ntheta = math.atan2(rspeed*math.sin(ctheta) + tPx, rspeed*math.cos(ctheta)+ tPy)-ctheta

        while ntheta > math.pi:
            ntheta -= 2*math.pi
        while ntheta < -math.pi:
            ntheta += 2*math.pi
        ntheta = min(maxt, ntheta)
        ntheta = max(-maxt, ntheta)

        ctheta += ntheta
        #print ctheta*180/pi

        #setting speed based on robot acceleration and speed
        speed = math.sqrt((rspeed*math.sin(ctheta)+tPx)**2+ (rspeed*math.cos(ctheta)+tPy)**2)
        speed = min(rspeed+macc, speed)
        rspeed = max(rspeed-macc, speed)
        rspeed = min(rspeed, mrspeed)
        rspeed = max(rspeed, 0)

        if rspeed == 0:
            print('Robot can\'t move')
            return sol

        #calculatig new positions
        cx = cx + (rspeed*math.sin(ctheta))
        cy = cy + (rspeed*math.cos(ctheta))

        if not feasible(arr, cx, cy):
            sol.append((int(cx), int(cy)))
            print('robot collides')
            return sol

        sol.append((int(cx), int(cy)))

    return sol

count = 0
def main():
    for im in images:

        img = cv2.imread(im)

        cimg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        img2 = cv2.medianBlur(cimg,13)

        ret,thresh1 = cv2.threshold(cimg,40,255,cv2.THRESH_BINARY)
        t2 = copy.copy(thresh1)

        x, y  = thresh1.shape
        print(x, y)
        arr = np.zeros((x, y, 3), np.uint8)
        final_contours= []
        _res = cv2.findContours(t2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        contours = _res[0] if len(_res) == 2 else _res[1]
        for i in range(len(contours)):
            cnt = contours[i]
            if cv2.contourArea(cnt) > 300 and cv2.contourArea(cnt) < 5000 :
                cv2.drawContours(img, [cnt],-1, [0, 255, 255])
                cv2.fillConvexPoly(arr, cnt, [255, 255, 255])
                final_contours.append(cnt)
        print('1')
        arr1 = np.zeros((x, y, 3), np.uint8)
        for i in range(x):
            for j in range(y):
                if arr[i][j][0] ==255:
                    arr1[i][j] = [0, 0, 0]
                else:
                    arr1[i][j] = [255, 255, 255]

        cv2.imwrite('count.bmp', arr1)

        sx = 30
        sy = 50
        dx = 100
        dy = 200

        cmax = 50
        start = time.perf_counter()
        #sol = path_planning(arr, sx, sy, dx, dy)
        sol = path(arr, sx, sy, dx, dy)
        #sol = [(100, 100),(100, 103), (100, 106), (100, 109), (100, 112), (100, 115)]
        if len(sol) == 0:
            print('No solution exist ')
            continue
        print('2')
        for i in sol:
        #    print arr[i[0],  i[1]]
          #  print i[0], i[1]
            arr[i[0], i[1]] = (255, 255, 0)
            img[i[0], i[1]] = (255, 0, 255)

        print('time: ',  time.perf_counter()-start)
        arr[sx][sy] = (0, 255, 255)
        arr[dx][dy] = (0, 255, 255)
       # cv2.circle(arr, (sol[len(sol)-1][1], sol[len(sol)-1][0]), 5, (0, 0, 255))
       # cv2.circle(arr, (sx, sy), 6, (0, 255, 255))
       # cv2.circle(arr, (dx, dy), 6, (0, 255, 255))

        #draw(math.pi/4, arr)
        cv2.imshow('image', img)
        cv2.imshow('arr', arr)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
main()