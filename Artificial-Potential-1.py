import cv2
import numpy as np
import copy
import glob
import math
import time
from time import sleep
import queue as Q

class pixel(object):
    def __init__(self, penalty, pointx, pointy): # parent is that pixel from which this current pixel is generated
        self.penalty = penalty
        self.pointx = int(pointx)
        self.pointy = int(pointy)

    def __cmp__(self, other): # comparable which will return self.penalty<other.penalty
        return cmp(self.penalty, other.penalty)

images = glob.glob('*.jpg')

def penalty(ox, oy, nx, ny, penalty): #ox, oy:- old points  nx, ny :- new points
    return penalty + math.sqrt((ox-nx)*(ox-nx)+ (oy-ny)*(oy-ny))

def check_boundaries(ex, ey, nx, ny): #ex, ey :- end points of frame
    if nx > -1 and ny > -1 and nx < ex and ny < ey:
        return True
    else:
        return False

def printx(x):
    #print x
    pass

def check_obstacles(arr, ansx, ansy):
    if arr[int(ansx)][int(ansy)][0] == 255:
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

def dist(sx, sy, x, y, theta, arr, q_star):  #distance of obstacle in direction theta in radians
    ansx = sx
    ansy = sy
    flag = True
    printx('6')
    count = 1
    while True:
        if count  > q_star:
            return (-1, -1)
        ansx = sx + count*math.sin(theta)
        ansy = sy + count*math.cos(theta)

        if check_boundaries(x, y, ansx, ansy) == False:
            break
        else:
            if check_obstacles(arr, ansx, ansy) == True:
                break
        count += 5



    printx('7')
    return (ansx-sx,ansy- sy)

flx = 10000
fly = 10000


def obstacle_force(arr, sx, sy, q_star): #sx,sy :- source    dx, dy:- destination    q-star:- threshold distance of obstacles
    forcex = 0
    forcey = 0
    neta = 30000
    x, y , z= arr.shape
    printx('8')
    #arr1 = np.zeros((x, y, 3), np.uint8)
    for i in range(8):
        (ox,oy) = dist(sx, sy, x, y, i*math.pi/4, arr, q_star)
     #   if ox == -1:
      #      cv2.line(arr, (sy, sx), (int(sy+q_star*math.sqrt(2)), int(sx+q_star*math.sqrt(2))), (255, 255, 255), 1, cv2.LINE_AA)
      #  else:
      #      cv2.line(arr, (sy, sx), (int(sy+oy), int(sx+ox)), (255, 255, 255), 1, cv2.LINE_AA)

        #print 'ox ', ox, 'oy ', oy, i*45
        theta = i*math.pi/4
        ox = math.fabs(ox)
        oy = math.fabs(oy)

        d = math.hypot(ox,oy)
        #print d, i*45
        #sleep(2)
        fx = 0
        fy = 0
        if ox == -1 or oy == -1:
            fx = 0
            fy = 0
        else:
            if d == 0:
                d = 1
            f = (neta*(1.0/q_star- 1.0/d))/(d*d)
            fx = f*math.sin(theta)
            fy = f*math.cos(theta)

        #    print 'fx ', fx, 'fy ',  fy, theta, f
        forcex += fx
        forcey += fy
       # sleep(1)



    printx('9')

    #cv2.imshow('img1', arr)
    #cv2.waitKey(0)
    #sleep(20)

    return (forcex, forcey)

def goal_force(arr, sx, sy, dx, dy, d_star): # sx, sy :- source  dx, dy:- destination   d_star:- threshold distance from goal
    forcex = 0
    forcey = 0
    tau = 1  #constant
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

def path_planning(arr, sx1, sy1, dx, dy):
    if arr[sx1][sy1][0] == 255 or arr[dx][dy][0] == 255:
        return []
    #print '3'
    v = 4 #velocity magnitude
    t = 1 #time lapse
    theta = 0 #initial angle
    x,y,z = arr.shape
    theta_const = math.pi*30/180

    sx = sx1
    sy = sy1

    sol = []
    sol.append((sx, sy))
    q_star = 50000
    d_star = 20000

    sx += int(v*math.sin(theta))
    sy += int(v*math.cos(theta))
    sol.append((sx, sy))

    '''
        if Q and P are two vectors and @ is angle between them

        resultant ,R = (P^2 + R^2 + 2*P*Q cos @)^(1/2)

        resultant, theta = atan((Q*sin @)/(P+Q*cos @))
    '''

    printx ('4')
    count  = 0
    while count < 1000:
        count += 1
        (fx, fy) = obstacle_force(arr, sx, sy, q_star)
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
        #tx *= -1
        #ty *= -1
        print(sx, sy, gx, gy, fx, fy, tx, ty)

        if arr[sx][sy][0] == 255:
            print(gx, gy, fx, fy)
            print('tx ', tx, ' ty ', ty, 'sx ', sx, ' sy ', sy)
            print(theta1*180/math.pi, theta*180/math.pi)
            sleep(10)

        #theta1 = math.atan2(tx, ty)
        P = v
        angle = theta1-theta

        Q = math.sqrt(tx*tx + ty*ty)

        theta2 = math.atan2((Q*math.sin(angle)),((P + Q*math.cos(angle))))   #resultant angle with velocity

        if theta2 < 0:
            theta2 = max(theta2, -theta_const)
        else:
            theta2 = min(theta2, theta_const)

        theta += theta2

        theta = (theta + 2*math.pi)%(2*math.pi)

        #print theta, theta2
        t1 = 0.0
        t2 = 0.0
        sx = sx + v*math.sin(theta)
        sy = sy + v*math.cos(theta)
        sx = int(sx)
        sy = int(sy)
        '''
        if theta < math.pi and theta >= 0:
            sx +=  int(math.ceil(math.sin(theta)))
        else:
            sx += int(math.floor(math.sin(theta)))

        if theta <= math.pi/2 and theta > -1*math.pi/2:
            sy += int(math.ceil(math.cos(theta)))
        else:
            sy += int(math.floor(math.cos(theta)))
        '''
      #  print theta
        if not check_boundaries(x, y, sx, sy):
            print('out of boundaries' , sx, sy)
            return sol

        if sx < dx+ 10 and sx > dx - 10 and sy < dy+10 and sy > dy-10:
            break

        sol.append((sx, sy))

    printx('5')
    return sol

def draw(theta, arr):
    x = 4
    y = 20
    ex, ey, ez = arr.shape
    while check_boundaries(ex, ey, x, y):
        arr[x][y] = (255, 255, 0)
        x += int(2*math.sin(theta))
        y += int(2*math.cos(theta))



def main():
    count = 0
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
        dx = 30
        dy = 200

        cmax = 50
        start = time.perf_counter()
        sol = path_planning(arr, sx, sy, dx, dy)
        #sol = path(arr, sx, sy, dx, dy)
        #sol = [(100, 100),(100, 103), (100, 106), (100, 109), (100, 112), (100, 115)]
        if len(sol) == 0:
            print('No solution exist ')
            continue
        print('2')
        for i in sol:
        #    print arr[i[0],  i[1]]
            #


        # print i[0], i[1]
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