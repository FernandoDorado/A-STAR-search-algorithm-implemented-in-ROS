#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid

import matplotlib.pyplot as plt
import numpy as np
from heapq import *

plotting = False

def callback(data):
    global plotting
    my_map = data.data

    width = data.info.width # uint
    height = data.info.height # uint 
    resolution = data.info.resolution # float

    rospy.loginfo(str(width) + ' ' + str(height) + ' ' + str(resolution))
    
    im = np.zeros((height,width), dtype=np.float32)

    for ii in range(0, height):
        for jj in range(0, width):
            foo = my_map[ii*width+jj]/100.0
            if foo > 0:
                im[ii][jj] = foo

    print("loaded")

    if plotting == False:
        plotting = True
        im2 = np.flipud(im)
        print type(im2)
        print im2.shape
        print im2
        plt.figure()
        plt.imshow(im2)
        plt.show()
        im2 = im2.astype(int)
        mapa = im2.tolist()
        imaux = im2
        #print mapa
        inicio = (50,50)
        final = (500,220)
        tray = astar(im2, inicio, final)
        print(tray)
        print 'fin'


def heuristic(a, b):
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2

def astar(array, start, goal):

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start))
    
    while oheap:

        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j            
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
                
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))
                
    return False



def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('grid_drawer', anonymous=True)
 
    rospy.Subscriber('map', OccupancyGrid, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()




if __name__ == '__main__':
    listener()
