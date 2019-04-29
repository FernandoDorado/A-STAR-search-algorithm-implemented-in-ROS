#!/usr/bin/env python
# coding=utf-8

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid


class Map(object):
  def __init__(self):

    self.map_sub = rospy.Subscriber("map",OccupancyGrid, self.callback)
    print "Mapa obtenido"
    print self.map_sub


  def callback(self,mapmsg):
      print "Dentro callback"
      map = mapmsg.data
      print type(map)
      print "tamano mapa"
      map = np.array(map)
      print type(map)

      print map.shape
      
      #map = map.reshape((500,500))
      #print map
      row = mapmsg.info.width
      col = mapmsg.info.height

      print row,col
      
      im = np.zeros([row,col])
      print type(im)
      print im.shape

      #tem = np.zeros((row,col))
      i = 0
      j = 0

      for k in range(1,row*col):
        i = i + 1
        if k%row == 0:
          i = 0
          j = j + 1
          
        im[i,j] = map[k]

      print "he salido del bucle"
      print type(im)
      
      #cv2.imshow("map",im)
      #cv2.waitKey(0)
      plt.imshow(im)
      plt.show()
      return im


'''class punto():
  def __init__(self, padre = None, posicion = None):
    self.padre = padre
    self.posicion = posicion
    self.f = 0
    self.g = 0
    self.h = 0

  def __eq__(self,other):
    return self.posicion == otra.posicion
  

def algoritmoA(mapa, inicio, final):

  #Inicializamos
  punto_inicio = punto(None, inicio)
  punto_final = punto(None, final)
  punto_inicio.f = 0
  punto_inicio.g = 0
  punto_inicio.h = 0
  punto_final.f = 0
  punto_final.g = 0
  punto_final.h = 0

  lista_abierta = []
  lista_cerrada = [] 

  #Añadimos el punto de inicio a la lista abierta
  lista_abierta.append(punto_inicio)

  #Empezamos la búsqueda del camino

  while len(lista_abierta) > 0:

    punto_actual = lista_abierta[0]
    ind_actual = 0

    for i,j in enumerate(lista_abierta):
      if j.f < punto_actual.f:                #float
        punto_actual = j
        ind_actual = i


# Añadimos a la lista cerrada
lista_cerrada.append(punto_actual)

# Quitamos de la lista abierta
lista_abierta.pop(ind_actual)
  


if punto_actual == punto_final:
  trayectoria = []
  actual = punto_actual
  while actual is not None:
    trayectoria.append(punto_actual.posicion)
    actual = punto_actual.padre
  return trayectoria[::-1] # Return reversed path  





hijos = []
for nuevo_punto in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Casillas adyacentes

  posicion_punto = (punto_actual.posicion[0] + nueva_posicion[0], punto_actual.posicion[1] + nueva_posicion[1] )

  
  if posicion_punto[0] > (len(mapa)-1) or posicion_punto[0]<0 or posicion_punto[1] > (len(mapa[len(mapa)-1])-1) or posicion_punto[1]<0: 
    continue

  if mapa[posicion_punto[0]][posicion_punto[1]] !=0:
    continue


  #Creamos nuevo punto
  nuevo_punto = punto(punto_actual, posicion_punto)

  hijos.append(nuevo_punto)

  for hijo in hijos:

    for lista_cerrada_hijo in lista_cerrada:
      if hijo == lista_cerrada:
        continue
  
  hijo.g = punto_actual.g + 1
  hijo.h = ((hijo.posicion[0] - punto_final.posicion[0]) ** 2) + ((hijo.posicion[1] - punto_final.posicion[1]) ** 2)
  hijo.f = hijo.h + hijo.g

  #Si el hijo está en la lista abierta
  for punto_lista_abierta in lista_abierta:
    if hijo == punto_lista_abierta and hijo.g > punto_lista_abierta.g:
      continue

  #Añadimos el hijo a la lista abierta
  lista_abierta.append(hijo)'''










def main():
  rospy.init_node('map',anonymous=True)
  v=Map()
  rospy.spin()


  inicio = (0,0)
  final = (500,50)

  tray = astar(im, inicio, final)
  print(tray)

if __name__=='__main__':
  main()
