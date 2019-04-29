#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
import cv2

import matplotlib.pyplot as plt
import numpy as np
from heapq import *

plotting = False

def linea(ruta):
    xg=[]
    yg=[]
    for i in (range(0,len(ruta))):
        x=ruta[i][0]
        y=ruta[i][1]
        xg.append(x)
        yg.append(y)
    return xg,yg

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
        #print type(im2)
        #print im2.shape
        #print im2
        #plt.figure()
        #plt.imshow(im2)
        #plt.show()
        im2 = im2.astype(int)
        print("Introduzca el punto inicial")
        i2 = int(input())
        i1 = int(input())
        print("Introduzca el punto final")
        o2 = int(input())
        o1 = int(input())
        print("Generando trayectoria...")

        inicio = (i1,i2)
        final = (o1,o2)
        tray = a_planning(im2, inicio, final)
        
        tray = np.asarray(tray)
        #print(tray)
        xg,yg=linea(tray)
        fig,imx=plt.subplots()
        imx.imshow(im2)
        imx.plot(yg,xg,color="white")
        imx.scatter(inicio[1],inicio[0])
        imx.scatter(final[1],final[0])
        plt.show()

class punto:

    def __init__(self, padre=None, pos=None):
        self.padre = padre
        self.pos = pos

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, sig):
        return self.pos == sig.pos


def dev_tray(punto_actual):
    tray = []
    loc = punto_actual
    while loc is not None:
        tray.append(loc.pos)
        loc = loc.padre
    return tray[::-1]  # Giramos tray -> (1234) -> (4321)


def a_planning(mapa, inicio, final):

    # Inicializamos la lista abierta/cerrada
    lista_abierta = []
    lista_cerrada = []

    # Creamos una clase asociada al punto inicial y final
    punto_inicial = punto(None, inicio)
    punto_inicial.g = 0
    punto_inicial.h = 0
    punto_inicial.f = 0
    punto_final = punto(None, final) #El punto final nos servira para determinar la H y para determinar cuando hemos llegado al destino
    punto_final.g = 0
    punto_final.h = 0
    punto_final.f = 0


    # Anadimos el punto inicial a la lista abierta
    lista_abierta.append(punto_inicial)
    
    # Condicion de parada (para evitar bucle infinito)
    iteracciones = 0
    max_iter = 150000
    

    # No permitimos movimientos en digonales
    vecinos = ((0, -1), (0, 1), (-1, 0), (1, 0),)


    # No salimos del bucle hasta vaciar la lista
    while len(lista_abierta) > 0:
        iteracciones += 1
        
        # Obtenemos el punto actual
        punto_actual = lista_abierta[0]
        indice = 0

        for i, elemento in enumerate(lista_abierta):      #Con enumerate permitimos : [a,b,c,d] -> [1 a;2 b; 3 c;4 d]
            if elemento.f < punto_actual.f:
                punto_actual = elemento
                indice = i

        
        if iteracciones > max_iter:
            print("Problema al calcular la trayectoria")
            return dev_tray(punto_actual)

        # Eliminamos elemento de la lista abierta y lo anadimos a la lista cerrada
        lista_abierta.pop(indice)
        lista_cerrada.append(punto_actual)

        # En caso de haber llegado al final
        if punto_actual == punto_final:
            return dev_tray(punto_actual)

    
        hijos = []
        
        for nueva_pos in vecinos: # Recorremos los vecinos

            # Obtenemos caracterisicas del punto
            pos_punto = (punto_actual.pos[0] + nueva_pos[0], punto_actual.pos[1] + nueva_pos[1])

            # Nos aseguramos que estamos dentro del mapa
            #if pos_punto[0] > (len(mapa) - 50) or pos_punto[0] < 0 or pos_punto[1] > (len(mapa[len(mapa)-50]) -1) or pos_punto[1] < 50:
            #    continue

            # Nos aseguramos ed que el punto es valido
            if mapa[pos_punto[0]][pos_punto[1]] != 0:
                continue

            #Comprobamos si el vecino esta en la lista cerrada para evitar bucle infinito
            if punto(punto_actual, pos_punto) in lista_cerrada: 
                continue

            # Creamos nueva clase punto
            nuevo_punto = punto(punto_actual, pos_punto)

            # Anadimos
            hijos.append(nuevo_punto)

        # Recorremos a traves de los hijos
        for hijo in hijos:
            
            # Si el hijo esta en la lista cerrada
            if len([hijo_lcerrada for hijo_lcerrada in lista_cerrada if hijo_lcerrada == hijo]) > 0:
                continue

            # Calculamos las caract. del hijo
            hijo.g = punto_actual.g + 1
            hijo.h = ((hijo.pos[0] - punto_final.pos[0]) ** 2) + ((hijo.pos[1] - punto_final.pos[1]) ** 2)      
            hijo.f = hijo.g + hijo.h

            # Si el hijo esta todavia enla lista abierta
            if len([p_abierta for p_abierta in lista_abierta if hijo == p_abierta and hijo.g > p_abierta.g]) > 0:
                continue

            # Anadimos el hijo a la lista abierta
            lista_abierta.append(hijo)




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
