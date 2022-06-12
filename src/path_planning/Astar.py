from PIL import Image
import numpy as np
from math import pow
from math import sqrt
from matplotlib import pyplot as plt

theWay = []

def read_img(name):
    im = Image.open(name)    #  read a file 
    return np.array(im)
    
data = read_img('/home/henrypham/UAV_ROS_CORE/src/path_planning/maps/newmap.pgm')


l = []

for i in range(250):
    k = []
    for j in range(250):
        k.append(0)
    l.append(k)
    
print(np.shape(l))

for i in range(250):
    for j in range(250):
        l[i][j] = data[950+i][950+j]

img = l

def ConvertList(way):
    l = []
    for i in range(len(way)-1, -1,-1):
        l.append(way[i])
    return l

def FindTheWay(beginX, beginY, endX, endY, Close):
    way = [(endX,endY)]
    while True: 
        if way[-1] == (beginX, beginY):
            return ConvertList(way)
            
        for Point in Close:
            if Point[0][0] == way[-1][0] and Point[0][1] == way[-1][1]:
                way.append((Point[1][0], Point[1][1]))
                break


class PriorityQueue():
    def __init__(self):
        self.queue = []
  
    def __str__(self):
        return ' ,'.join([str(i) for i in self.queue])
  
    # for checking if the queue is empty
    def isEmpty(self):
        return len(self.queue) == 0
    
    def FindOut(self, a):
        for i in range(len(self.queue)):
            if a == self.queue[i][0]:
                return True
        return False
    
    def top(self):
        return self.queue[0]
  
    # for inserting an element in the queue
    def insert(self, data):
        self.queue.append(data)
  
    # for popping an element based on Priority
    def delete(self, dic, matrixHx, endX, endY):
        try:
            max = 0
            for i in range(len(self.queue)):
                if abs(dic[self.queue[i][0]] - matrixHx[endY][endX]) < abs(dic[self.queue[max][0]] - matrixHx[endY][endX]):
                    max = i
            item = self.queue[max]
            del self.queue[max]
            return item
        except IndexError:
            print()
            exit()


def Astar(size, img, beginX, beginY, endX, endY):
    
    matrixGx = []
    for i in range(size): 
        l = []
        for j in range(size): 
            l.append(0)
        matrixGx.append(l)
    for i in range(size):
       for j in range(size):
            if img[i][j] == 0 or img [i][j] == 205:
                matrixGx[i][j] = 0;
            else:
                matrixGx[i][j] = (abs(beginX-j) + abs(beginY-i)) 
    # print("\nMatrixGx: ")    
    # for i in range(size):
    #     print(matrixGx[i])
    
    #Declare matrixHx:
    matrixHx = []
    for i in range(size): 
        l = []
        for j in range(size): 
            l.append(0)
        matrixHx.append(l)
    for i in range(size):
        for j in range(size):
            if img[i][j] == 0 or img [i][j] == 205:
                matrixHx[i][j] = 0
            else: 
                matrixHx[i][j] =  sqrt(pow(abs(beginX-(j+1)), 2) + pow(abs(beginY-(i+1)), 2))
    #Declare matrixFx:
    matrixFx = []
    for i in range(size+1): 
        l = []
        for j in range(size+1): 
            l.append(0)
        matrixFx.append(l)
    for i in range(size):
        for j in range(size):
            matrixFx[i][j] = (matrixGx[i][j] + matrixHx[i][j])
    
    #Print matrix Fx
    
    l = []
    for i in range(size):
        for j in range(size):
            l.append([(i,j), matrixFx[j][i]])
    data = dict(l)

    #Declare Open, close queue
    Open = PriorityQueue()
    Close = PriorityQueue()
    Open.insert( [( beginX, beginY) , (-1,-1) ] )
    while(True):
        if Open.isEmpty == True:
            print("Tim kiem that bai")
            return
        
        v = Open.delete(data,matrixHx , endX, endY)
        Close.insert(v)
        
        ##print("Duyet: ", v, data[v[0]])
        
        if v[0][0] == endX and v[0][1] == endY:
            print("Tim kiem thanh cong")
            Way = FindTheWay(beginX, beginY, endX, endY, Close.queue)
            for Point in Way:
                theWay.append(Point)
            return
        
        
        nearPoints = [(v[0][0]+1, v[0][1]),
                      (v[0][0], v[0][1]+1), 
                      (v[0][0]-1, v[0][1]),
                      (v[0][0], v[0][1]-1)
                      ]
        for Point in nearPoints:
            if Point[0] < 0 or Point[1] < 0 or Point[0] >= size or Point[1] >= size or Close.FindOut(Point) or matrixFx[Point[1]][Point[0]] == 0:
                continue
            if Open.FindOut(Point): 
                continue
            Open.insert([Point, v[0]])
            
Astar(250, img, 27, 43, 202,70 )
print(theWay)

for Point in theWay:
    img[Point[1]][Point[0]] = 100
    
plt.imshow(img, interpolation='nearest')
plt.show()