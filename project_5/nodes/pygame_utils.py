import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2

#Global Constants
x_dimension = 1000
y_dimension = 900
step_size = 7.0
iterations = 1000
thresh_radius=15
Config_Space=[(150, 300, 100, 100),(50, 100, 200, 100),(400,175,50,150)]


class State:
    x = 0
    y = 0
    cost=0  
    parent=None
    def __init__(self,x_coordinate, y_coordinate):
         self.x = x_coordinate
         self.y = y_coordinate

def environment(pygame,surface):
    blue=(0,0,255)
    for o in Config_Space: 
      pygame.draw.rect(surface,blue,o)

def euclid_dist(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

def next_state(p1,p2):
    if euclid_dist(p1,p2) < step_size:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        new_x = p1[0] + step_size*cos(theta)
        new_y = p1[1] + step_size*sin(theta)
        return new_x, new_y

def draw_tree(start,goal,node_list,pygame,surface):
  pink = 200, 20, 240
  nn = node_list[0]
  for p in node_list:
    if euclid_dist([p.x,p.y],[goal.x,goal.y]) < euclid_dist([nn.x,nn.y],[goal.x,goal.y]):
      nn = p
  while nn!=start:
      pygame.draw.line(surface,pink,[nn.x,nn.y],[nn.parent.x,nn.parent.y],5)
      nn=nn.parent
	

        
    
