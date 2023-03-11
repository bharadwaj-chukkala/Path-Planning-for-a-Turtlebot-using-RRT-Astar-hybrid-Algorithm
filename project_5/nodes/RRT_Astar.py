# Importing Libraries

import numpy as np
import math
import matplotlib.pyplot as plt
import time
import heapq
from pygame_utils import *
import matplotlib.patches as patches

plt.ion()

#Global constants
x_dimension = 1000
y_dimension = 900
step_size = 7.0
iterations = 1000
thresh_radius=15
Config_Space=[(150, 300, 100, 100),(50, 100, 200, 100),(400,175,50,150)]

### Node Class ###
class Node:

    def __init__(self, x, y, parent,current_theta, theta_diff,UL,UR, c2c, c2g, total_cost ):
        self.x = x
        self.y = y
        self.parent = parent
        self.current_theta = current_theta
        self.theta_diff = theta_diff
        self.UL = UL
        self.UR = UR
        self.c2c = c2c
        self.c2g = c2g
        self.total_cost = total_cost 
        
    def __lt__(self,other):
        return self.total_cost < other.total_cost

#### Check if node is valid or not ####
def Valid_move(x,y, r,c):
    
    if obstaclecheck(x, y, r, c):
        return False
    else:
        return True

### Checks if goal is being reached ###
def check_goal(current, goal):
    dt = dist((current.x, current.y), (goal.x, goal.y))
    if dt < 0.25:
        return True
    else:
        return False

### Function to calculate the euclidean distance between two coordinates ###
def dist(pos, goal):
    xp, yp = pos
    xg, yg = goal
    distance = np.sqrt((xp-xg)**2 + (yp-yg)**2)
    return distance

### Function to plot dubins curves ###
def plot_curve(Xi, Yi, Thetai, UL, UR,c, plot, Node_List, Path_List):
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.1
    cost = 0
    Xn = Xi
    Yn = Yi
    Thetan = 3.14 * Thetai / 180

    while t < 1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += r*0.5 * (UL + UR) * math.cos(Thetan) * dt
        Yn += r*0.5 * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
      
        if  Valid_move(Xn, Yn, r, c):
            if plot == 0:
                
                c2g = dist((Xs, Ys), (Xn, Yn))
                cost = cost + c2g
                Node_List.append((Xn, Yn))
                Path_List.append((Xs, Ys))
            if plot == 1:
                plt.plot([Xs, Xn], [Ys, Yn], color="red")
        else:
            return None
    Thetan = 180 * (Thetan) / 3.14
    return [Xn, Yn, Thetan, cost, Node_List, Path_List]


     
def obstaclecheck(x, y, r, c):
    tot = r + c

    
    obstacle1 = (x >= 2 - tot) and (x <=5.2 +tot) and (y >= 1.5 - tot) and (y <= 4.2 + tot)
    obstacle2 = (x >= 0.8 - tot) and (x <=5.2 +tot) and (y >= 5.8 - tot) and (y <= 8.2 + tot)
    obstacle3 = (x >= 6.5 - tot) and (x <=9.2 +tot) and (y >= 4.3 - tot) and (y <= 7.7 + tot)
    

    border1 = (x <= 0.1 + tot)
    border2 = (x >= 9.9- tot)
    border3 = (y <= 0.1 + tot)
    border4 = (y >= 9.9 - tot)

    if obstacle1 or obstacle2 or obstacle3  or border1 or border2 or border3 or border4:
        return True
    else:
        return False
        
######### GENERATE UNIQUE KEY ##########
def key(node):
    key = 1022*node.x + 111*node.y 
    return key


#### RRT-Astar Hybrid algorithm that finds optimal path in the generated tree  ####
def RRT_Astar(start_node, goal_node,rpm1,rpm2,r,clearance):

    if check_goal(start_node, goal_node):
        return 1,None,None
    
    start_node = start_node
    start_node_id = key(start_node)
    goal_node = goal_node

    Node_List = []
    Path_List = []
    
    unexplored_nodes = {}
    unexplored_nodes[start_node_id] = start_node
    explored_nodes = {}
    
    priority_list = []
    
    moves = [[rpm1, 0], 
             [0, rpm1], 
             [rpm1, rpm1], 
             [0, rpm2], 
             [rpm2, 0], 
             [rpm2, rpm2], 
             [rpm1, rpm2],
             [rpm2, rpm1]]
    
    heapq.heappush(priority_list, [start_node.total_cost, start_node])

    while (len(priority_list) != 0):

        present_node = (heapq.heappop(priority_list))[1]
        present_id = key(present_node)

        if check_goal(present_node, goal_node):
            goal_node.parent = present_node.parent
            goal_node.total_cost = present_node.total_cost
            print("Goal Node found")
            return 1,Node_List,Path_List
        
        if present_id in explored_nodes:  
            continue
        else:
            explored_nodes[present_id] = present_node
        
        del unexplored_nodes[present_id]
        
        for move in moves:
            X1 = plot_curve(present_node.x, present_node.y, present_node.current_theta, move[0], move[1],
                            clearance, 0, Node_List, Path_List)
            
            if (X1 != None):
                angle = X1[2]
                theta_thresh = 15
                x = (round(X1[0] * 10) / 10)
                y = (round(X1[1] * 10) / 10)
                th = (round(angle / theta_thresh) * theta_thresh)
                ct = present_node.theta_diff - th
                c2g = dist((x,y), (goal_node.x, goal_node.y))
                new_node = Node(x,y,present_node,th,ct,move[0],move[1],present_node.c2c+X1[3],c2g,present_node.c2c+X1[3]+c2g)
                new_node_id = key(new_node)

                if not Valid_move(new_node.x, new_node.y,r,clearance):
                    continue
                elif new_node_id in explored_nodes:
                    continue
                if new_node_id in unexplored_nodes:
                    if new_node.total_cost < unexplored_nodes[new_node_id].total_cost:
                        unexplored_nodes[new_node_id].total_cost = new_node.total_cost
                        unexplored_nodes[new_node_id].parent = new_node
                else:
                    unexplored_nodes[new_node_id] = new_node
                    heapq.heappush(priority_list, [ unexplored_nodes[new_node_id].total_cost, unexplored_nodes[new_node_id]])
            
    return 0,Node_List,Path_List

## ---- The fucntion call that creates the Sampling based Tree--- ###
def main_node():
    
    pygame.init()
    surface = pygame.display.set_mode((500,500), pygame.RESIZABLE)
    pygame.display.set_caption('RRT-Astar')
    white = 255, 255, 255
    black = 20, 20, 40
    surface.fill(white)
    environment(pygame,surface)
    node_list = []
    node_list.append(State(5.0,5.0)) # Start in the corner
    start=node_list[0]
    goal=State(450.0,450.0)
    for i in range(iterations):

        rand_list = []
        euclid_dist_values = []
        
        for j in range(4):
            rando = State(random.random()*x_dimension, random.random()*y_dimension)
            rand_list.append(rando)
       
        for each in rand_list:
            
            euclid_distance_each = euclid_dist ([each.x,each.y], [goal.x,goal.y])
            euclid_dist_values.append(euclid_distance_each)
              
        least_euclid_dist_index = euclid_dist_values.index(min(euclid_dist_values))
        rand = rand_list[least_euclid_dist_index]

        nn = node_list[0]
        for p in node_list:
          if euclid_dist([p.x,p.y],[rand.x,rand.y]) < euclid_dist([nn.x,nn.y],[rand.x,rand.y]):
            nn = p
        interpolatedState= next_state([nn.x,nn.y],[rand.x,rand.y])
	
        newnode = State(interpolatedState[0],interpolatedState[1])
        if Check_if_interescting(nn,rand,Config_Space):
          [newnode,nn]=choose_parent_node(nn,newnode,node_list)
          node_list.append(newnode)
          pygame.draw.line(surface,black,[nn.x,nn.y],[newnode.x,newnode.y])
          node_list=rewire_tree(node_list,newnode,pygame,surface)
          pygame.display.update()
        
          for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving because you requested it.")
    draw_tree(start,goal,node_list,pygame,surface)
    pygame.display.update()

#### Function to backtrack the optimal path #####
def Backtrack(goal_node):  

    x_path = []
    y_path = []
    theta_path = []
    RPM_Left_Wheel = []
    RPM_Right_Wheel = []
    x_path.append(goal_node.x)
    y_path.append(goal_node.y)
    theta_path.append(goal_node.current_theta)
    RPM_Left_Wheel.append(goal_node.UL)
    RPM_Right_Wheel.append(goal_node.UR)
    parent_node = goal_node.parent

    while parent_node != -1:
        x_path.append(parent_node.x)
        y_path.append(parent_node.y)
        theta_path.append(parent_node.current_theta)
        RPM_Left_Wheel.append(parent_node.UL)
        RPM_Right_Wheel.append(parent_node.UR)
        parent_node = parent_node.parent
        
    x_path.reverse()
    y_path.reverse()
    theta_path.reverse()
    RPM_Left_Wheel.reverse()
    RPM_Right_Wheel.reverse()

    RPM_Left = np.array(RPM_Left_Wheel)
    RPM_Right = np.array(RPM_Right_Wheel)
    x = np.asarray(x_path)
    y = np.asarray(y_path)
    theta = np.array(theta_path)
    return x,y,theta,RPM_Left,RPM_Right

### To check is orientation is valid ####
def validorient(theta):
    if((theta%30)==0):
        return theta
    else:
        return False

def counter_clock_check(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# Return true if line segments AB and CD intersect
def Check_if_interescting(nodeA,nodeB,Config_space):
    A=(nodeA.x,nodeA.y)
    B=(nodeB.x,nodeB.y)
    for o in Config_space:
      element=(o[0],o[1],o[0]+o[2],o[1]+o[3])
      
      p_1_s_1=(element[0],element[1])
      p_2_s_1=(element[0],element[3])
      inst1= counter_clock_check(A,p_1_s_1,p_2_s_1) != counter_clock_check(B,p_1_s_1,p_2_s_1) and counter_clock_check(A,B,p_1_s_1) != counter_clock_check(A,B,p_2_s_1)
      
      p_1_s_2=(element[0],element[1])
      p_2_s_2=(element[2],element[1])
      inst2= counter_clock_check(A,p_1_s_2,p_2_s_2) != counter_clock_check(B,p_1_s_2,p_2_s_2) and counter_clock_check(A,B,p_1_s_2) != counter_clock_check(A,B,p_2_s_2)
      
      p_1_s_3=(element[2],element[3])
      p_2_s_3=(element[2],element[1])
      inst3= counter_clock_check(A,p_1_s_3,p_2_s_3) != counter_clock_check(B,p_1_s_3,p_2_s_3) and counter_clock_check(A,B,p_1_s_3) != counter_clock_check(A,B,p_2_s_3)
      
      p_1_s_4=(element[2],element[3])
      p_2_s_4=(element[0],element[3])
      inst4= counter_clock_check(A,p_1_s_4,p_2_s_4) != counter_clock_check(B,p_1_s_4,p_2_s_4) and counter_clock_check(A,B,p_1_s_4) != counter_clock_check(A,B,p_2_s_4)
      
      if inst1==False and inst2==False and inst3==False and inst4==False:
        continue      
      else:
         return False
    return True

def choose_parent_node(nn,newnode,node_list):
        for p in node_list:
          cost_1 = p.cost+euclid_dist([p.x,p.y],[newnode.x,newnode.y])
          cost_2 = nn.cost+euclid_dist([nn.x,nn.y],[newnode.x,newnode.y])
          if Check_if_interescting(p,newnode,Config_Space) and euclid_dist([p.x,p.y],[newnode.x,newnode.y]) <thresh_radius and cost_1<cost_2:
            nn = p
        newnode.cost=nn.cost+euclid_dist([nn.x,nn.y],[newnode.x,newnode.y])
        newnode.parent=nn
        return newnode,nn

def rewire_tree(node_list,newnode,pygame,surface):
        white = 255, 240, 200
        black = 0, 0, 0
        for i in range(len(node_list)):
           p = node_list[i]
           if Check_if_interescting(p,newnode,Config_Space) and p!=newnode.parent and euclid_dist([p.x,p.y],[newnode.x,newnode.y]) <thresh_radius and newnode.cost+euclid_dist([p.x,p.y],[newnode.x,newnode.y]) < p.cost:
              pygame.draw.line(surface,white,[p.x,p.y],[p.parent.x,p.parent.y])  
              p.parent = newnode
              p.cost=newnode.cost+euclid_dist([p.x,p.y],[newnode.x,newnode.y]) 
              node_list[i]=p  
              pygame.draw.line(surface,black,[p.x,p.y],[newnode.x,newnode.y])                    
        return node_list

if __name__ == '__main__':

    width = 10
    height = 10
    robot_radius  = 0.038
    clearance = input("Enter obstacle clearance for robot ")
    clearance = float(clearance)

    Rpms = input("Enter left wheel and right wheel RPMs")
    RPM1,RPM2 = Rpms.split()
    RPM1 = int(RPM1)
    RPM2 = int(RPM2)

    start_coordinates = input("Enter start coordinates: ")
    s_x, s_y = start_coordinates.split()
    s_x = int(s_x)
    s_y = int(s_y)
    
    goal_coordinates = input("Enter goal coordinates: ")
    g_x, g_y = goal_coordinates.split()
    g_x = int(g_x)
    g_y = int(g_y)

    if not Valid_move(s_x, s_y, robot_radius,clearance):
        print("In valid start node or in Obstacle space")
        exit(-1)
        
    if not Valid_move(g_x, g_y, robot_radius,clearance):
        print("In valid goal node or in Obstacle space")
        exit(-1)
    
    start_theta = input("Enter Orientation of the robot at start node: ")
    s_t = int(start_theta)
    
    if not validorient(s_t):
        print("Orientation has to be a multiple of 30")
        exit(-1)

    
    timer_start = time.time()

    c2g = dist((s_x,s_y), (g_x, g_y))
    total_cost =  c2g
    start_node = Node(s_x, s_y,-1,s_t,0,0,0,0,c2g,total_cost)
    goal_node = Node(g_x, g_y, -1,0,0,0,0,c2g,0,total_cost)

    flag,Node_List,Path_List = RRT_Astar(start_node, goal_node,RPM1,RPM2,robot_radius,clearance)
                    
    if (flag)==1:
        x_path,y_path,theta_path,RPM_Left,RPM_Right = Backtrack(goal_node)
    else:
        print("not found")
        exit(-1)

    x_st = [0, 0]
    y_st = [0 ,1]


    l = 0
    for l in range(len(Node_List)):
        plt.plot([Path_List[l][0], Node_List[l][0]], [Path_List[l][1], Node_List[l][1]], color="blue")
        l+=1

    plt.plot(x_path,y_path, ':r')
    timer_stop = time.time()
    
    C_time = timer_stop - timer_start
    print("The Total Runtime is:  ", C_time)

    plt.show()
    # plt.pause(30)
    plt.close('all')
    
    print("publishing node initialising")
    
    
    
                


