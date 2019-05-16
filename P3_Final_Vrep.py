#!/usr/bin/env python
# coding: utf-8

# In[1]:


import math
import numpy as np
import cv2 as cv
from scipy.spatial import distance
import vrep
import time


# In[2]:


## ALL measurements are given in centimeters
r=3.8
l=23
dt=0.5
d=20


# In[3]:


def Goal(check_x,check_y):
    if (check_x-goal[0])**2+(check_y-goal[1])**2-49<=0:
        return 1
    else:
        return 0


# In[4]:


def obstacle_map(x,y):
    circle1 = (x-390)**2+(y-45)**2-(40+d)**2<=0
    circle2 = (x-438)**2+(y-274)**2-(40+d)**2<=0
    circle3 = (x-438)**2+(y-736)**2-(40+d)**2<=0
    circle4 = (x-390)**2+(y-965)**2-(40+d)**2<=0
    circle5 = (x-150)**2+(y-180)**2-(80+d)**2<=0
    circle6 = (x-310)**2+(y-180)**2-(80+d)**2<=0
    square1 = y>=100-d and y<=260+d and x>=150-d and x<= 310+d
    square2 = y>=0 and y<=183-d and x>=832-d and x<= 918+d
    square3 = y>=0 and y<=91-d and x<=1026+d and x>= 983-d
    square4 = y>=313-d and y<=389+d and x>=744-d and x<= 1110
    square5 = y>=495-d and y<=612+d and x>=1052-d and x<= 1110
    square6 = y>=612-d and y<=698+d and x>=1019-d and x<= 1110
    square7 = y>=765-d and y<=882+d and x>=1052-d and x<= 1110
    square8 = y>=899-d and y<=975+d and x>=927-d and x<= 1110
    square9 = y>=975-d and y<=1010 and x>=685-d and x<= 1110
    square10 = y>=917-d and y<=975+d and x>=779-d and x<= 896+d
    square11 = y>=823-d and y<=975+d and x>=474-d and x<= 748+d
    square12 = y>=626-d and y<=743+d and x>=785-d and x<= 937+d
    square13 = y>=669-d and y<=745+d and x>=529-d and x<= 712+d
    square14 = y>=512-d and y<=695+d and x>=438-d and x<= 529+d
    boundary1 = x>=0 and x<=d 
    boundary2 = y>=0 and y<=d
    boundary3 = x>=1110-d and x<=1110 
    boundary4 = y>=1010-d and y<=1010

    if circle1 or circle2 or circle3 or circle4 or circle5 or circle6 or square1 or square2 or square3 or square4 or square5 or square6 or square7 or square8 or square9 or square10 or square11 or square12 or square13 or square14 or boundary1 or boundary2 or boundary3 or boundary4:
        obs_output = 0
    else:
        obs_output = 1

    return obs_output


# In[7]:


def boundary_check(i,j):
    if (i<d or j>1109-d or j<d or i>1009-d):
        return 0
    else:
        return 1


# In[8]:


## Creating maze sized list to store parent details
parent_list=[]

for j in range (1110):
    column=[]
    for i in range (1010):
        column.append(0)
    parent_list.append(column)


# In[9]:


## ASSIGNING THE START POINT TO THE TURTLEBOT
print("Taking the starting point of the TurtleBot as [560,510]")

start=[560,510]

x_start=start[0]
y_start=start[1]


# In[10]:


## Taking user inputs as the goal position

x_goal=int(input("Please enter goal point x coordinate"))
y_goal=int(input("Please enter goal point y coordinate"))

goal_obs=obstacle_map(x_goal,y_goal)
goal_boundary=boundary_check(x_goal,y_goal)


while( ((goal_obs) and (goal_boundary)) !=1):
    print("Incorrect goal point! Please enter a valid goal point")
    x_goal=int(input("Please enter another goal point x coordinate"))
    y_goal=int(input("Please enter another goal point y coordinate"))
    goal_obs=obstacle_map(x_goal,y_goal)
    goal_boundary=boundary_check(x_goal,y_goal)

goal=[x_goal,y_goal]


# In[11]:


## Initializing all the maze size arrays
cost=np.array(np.ones((1110,1010)) * np.inf)
euclidean_distance_arr=np.array(np.ones((1110,1010)) * np.inf)
total_cost_arr=np.array(np.ones((1110,1010)) * np.inf)
visited_arr=np.array(np.zeros((1110,1010)))


# In[12]:


## This is the priority queue function
Q=[]
Q.append([x_start,y_start,0])
cost[x_start][y_start]=0
total_cost_arr[x_start][y_start]=0

def GetFirst(Q):
    index_min=0
    X_min = Q[0][0] 
    Y_min = Q[0][1]
    for i in range(len(Q)):
        x = Q[i][0]
        y = Q[i][1]
        if total_cost_arr[x,y] < total_cost_arr[X_min,Y_min]:
            index_min = i
            X_min = x 
            Y_min= y
    current_node = Q[index_min]
    Q.remove(Q[index_min])
    return current_node


# In[13]:


## Setting two RPMs and calculating rad/sec

RPM1=60
RPM2=30

#Conversion to radians per sec from rpm
r1=(2*22*RPM1)/(60*7)   
r2=(2*22*RPM2)/(60*7)


# In[14]:


## Motion formulas for differential drive

def Differential_motion(r1,r2,theta):
    dtheta=(r*(r1-r2)*dt)/l + theta
    dx=(r*(r1+r2)*math.cos(dtheta)*dt)/2
    dy=(r*(r1+r2)*math.sin(dtheta)*dt)/2
    return dtheta,dx,dy


# In[15]:


# Movements of the Turtle Bot Robot

def straight(i,j,theta):
    dtheta,dx,dy=Differential_motion(r1,r1,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)),'straight_0')
    return new_node

def straight_faster(i,j,theta):
    dtheta,dx,dy=Differential_motion(r2,r2,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)),'straight_1')
    return new_node

def right(i,j,theta):
    dtheta,dx,dy=Differential_motion(r1,0,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)),'right_0')
    return new_node

def right1(i,j,theta):
    dtheta,dx,dy=Differential_motion(r2,0,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)),'right_1')
    return new_node

def right2(i,j,theta):
    dtheta,dx,dy=Differential_motion(r1,r2,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)),'right_2')
    return new_node

def left(i,j,theta):
    dtheta,dx,dy=Differential_motion(0,r1,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)),'left_0')
    return new_node

def left1(i,j,theta):
    dtheta,dx,dy=Differential_motion(0,r2,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)),'left_1')
    return new_node

def left2(i,j,theta):
    dtheta,dx,dy=Differential_motion(r2,r1,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)),'left_2')
    return new_node


# In[16]:


## Generating new nodes according to the movements,checking conditions and appending to the queue list

visited_nodes=[]
current_node=[x_start,y_start,0]
while True: 
    if Goal(current_node[0],current_node[1])==1:
        goalbound=current_node
        break
    current_node=GetFirst(Q)
    straight_new=straight (current_node[0],current_node[1],current_node[2])
    status=boundary_check(straight_new[0],straight_new[1])
    flag=obstacle_map(straight_new[0],straight_new[1])
    if ( ((status) and (flag)) == 1):
        if visited_arr[straight_new[0],straight_new[1]]==0:
            visited_arr[straight_new[0],straight_new[1]]=1
            visited_nodes.append(straight_new)
            Q.append(straight_new)
            parent_list[straight_new[0]][straight_new[1]]=current_node
            
            cost[straight_new[0],straight_new[1]]=(cost[current_node[0],current_node[1]]+1)
            euclidean_distance_arr[straight_new[0],straight_new[1]]=math.sqrt(math.pow((straight_new[0]-goal[0]),2) + math.pow((straight_new[1]-goal[1]),2))
            total_cost_arr[straight_new[0],straight_new[1]]= cost[straight_new[0],straight_new[1]]+euclidean_distance_arr[straight_new[0],straight_new[1]]
        else:
            if cost[straight_new[0],straight_new[1]]>(cost[current_node[0],current_node[1]]+1):
                cost[straight_new[0],straight_new[1]]=(cost[current_node[0],current_node[1]]+1)
                euclidean_distance_arr[straight_new[0],straight_new[1]]=math.sqrt(math.pow((straight_new[0]-goal[0]),2) + math.pow((straight_new[1]-goal[1]),2))
                parent_list[straight_new[0]][straight_new[1]]=current_node
                total_cost_arr[straight_new[0],straight_new[1]]= cost[straight_new[0],straight_new[1]]+euclidean_distance_arr[straight_new[0],straight_new[1]]
    
    
    straight_faster_new=straight_faster(current_node[0],current_node[1],current_node[2])
    status=boundary_check(straight_faster_new[0],straight_faster_new[1])
    flag=obstacle_map(straight_faster_new[0],straight_faster_new[1])
    
    if ( ((status) and (flag)) == 1):
        if visited_arr[straight_faster_new[0],straight_faster_new[1]]==0:
            visited_arr[straight_faster_new[0],straight_faster_new[1]]=1
            visited_nodes.append(straight_faster_new)
            Q.append(straight_faster_new)
            parent_list[straight_faster_new[0]][straight_faster_new[1]]=current_node
            cost[straight_faster_new[0],straight_faster_new[1]]=(cost[current_node[0],current_node[1]]+1)
            euclidean_distance_arr[straight_faster_new[0],straight_faster_new[1]]=math.sqrt(math.pow((straight_faster_new[0]-goal[0]),2) + math.pow((straight_faster_new[1]-goal[1]),2))
            total_cost_arr[straight_faster_new[0],straight_faster_new[1]]= cost[straight_faster_new[0],straight_faster_new[1]]+euclidean_distance_arr[straight_faster_new[0],straight_faster_new[1]]
        else:
            if cost[straight_faster_new[0],straight_faster_new[1]]>(cost[current_node[0],current_node[1]]+1):
                cost[straight_faster_new[0],straight_faster_new[1]]=(cost[current_node[0],current_node[1]]+1)
                euclidean_distance_arr[straight_faster_new[0],straight_faster_new[1]]=math.sqrt(math.pow((straight_faster_new[0]-goal[0]),2) + math.pow((straight_faster_new[1]-goal[1]),2))
                parent_list[straight_faster_new[0]][straight_faster_new[1]]=current_node
                total_cost_arr[straight_faster_new[0],straight_faster_new[1]]= cost[straight_faster_new[0],straight_faster_new[1]]+euclidean_distance_arr[straight_faster_new[0],straight_faster_new[1]]
    
    right_new=right(current_node[0],current_node[1],current_node[2])
    status=boundary_check(right_new[0],right_new[1])
    flag=obstacle_map(straight_new[0],straight_new[1])

    if ( ((status) and (flag)) == 1):
        if visited_arr[right_new[0],right_new[1]]==0:
            visited_arr[right_new[0],right_new[1]]=1
            visited_nodes.append(right_new)
            Q.append(right_new)         
            parent_list[right_new[0]][right_new[1]]=current_node
            cost[right_new[0],right_new[1]]=(cost[current_node[0],current_node[1]]+1)
            euclidean_distance_arr[right_new[0],right_new[1]]=math.sqrt(math.pow((right_new[0]-goal[0]),2) + math.pow((right_new[1]-goal[1]),2))
            total_cost_arr[right_new[0],right_new[1]]= cost[right_new[0],right_new[1]]+euclidean_distance_arr[right_new[0],right_new[1]]

        else:
            if cost[right_new[0],right_new[1]]>(cost[current_node[0],current_node[1]]+1):
                cost[right_new[0],right_new[1]]=(cost[current_node[0],current_node[1]]+1)
                parent_list[right_new[0]][right_new[1]]=current_node
                euclidean_distance_arr[right_new[0],right_new[1]]=math.sqrt(math.pow((right_new[0]-goal[0]),2) + math.pow((right_new[1]-goal[1]),2))
                total_cost_arr[right_new[0],right_new[1]]= cost[right_new[0],right_new[1]]+euclidean_distance_arr[right_new[0],right_new[1]]
    
    
    
    left_new=left(current_node[0],current_node[1],current_node[2])
    status=boundary_check(left_new[0],left_new[1])
    flag=obstacle_map(left_new[0],left_new[1])
    if ( ((status) and (flag)) == 1):
        if visited_arr[left_new[0],left_new[1]]==0:
            visited_arr[left_new[0],left_new[1]]=1
            visited_nodes.append(left_new)
            Q.append(left_new)
            parent_list[left_new[0]][left_new[1]]=current_node
            euclidean_distance_arr[left_new[0],left_new[1]]=math.sqrt(math.pow((left_new[0]-goal[0]),2) + math.pow((left_new[1]-goal[1]),2))
            cost[left_new[0],left_new[1]]=(cost[current_node[0],current_node[1]]+1)
            total_cost_arr[left_new[0],left_new[1]]= cost[left_new[0],left_new[1]]+euclidean_distance_arr[left_new[0],left_new[1]]
        else:
            if cost[left_new[0],left_new[1]]>(cost[current_node[0],current_node[1]]+1):
                cost[left_new[0],left_new[1]]=(cost[current_node[0],current_node[1]]+1)
                parent_list[left_new[0]][left_new[1]]=current_node
                euclidean_distance_arr[left_new[0],left_new[1]]=math.sqrt(math.pow((left_new[0]-goal[0]),2) + math.pow((left_new[1]-goal[1]),2))
                total_cost_arr[left_new[0],left_new[1]]= cost[left_new[0],left_new[1]]+euclidean_distance_arr[left_new[0],left_new[1]]
    
    
    right1_new=right1(current_node[0],current_node[1],current_node[2])
    status=boundary_check(right1_new[0],right1_new[1])
    flag=obstacle_map(right1_new[0],right1_new[1])

    if ( ((status) and (flag)) == 1):   
        if visited_arr[right1_new[0],right1_new[1]]==0:
            visited_arr[right1_new[0],right1_new[1]]=1
            visited_nodes.append(right1_new)
            Q.append(right1_new)
            parent_list[right1_new[0]][right1_new[1]]=current_node
            euclidean_distance_arr[right1_new[0],right1_new[1]]=math.sqrt(math.pow((right1_new[0]-goal[0]),2) + math.pow((right1_new[1]-goal[1]),2))
            cost[right1_new[0],right1_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
            total_cost_arr[right1_new[0],right1_new[1]]= cost[right1_new[0],right1_new[1]]+euclidean_distance_arr[right1_new[0],right1_new[1]]
        else:
            if cost[right1_new[0],right1_new[1]]>(cost[current_node[0],current_node[1]]+math.sqrt(2)):
                cost[right1_new[0],right1_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
                parent_list[right1_new[0]][right1_new[1]]=current_node
                euclidean_distance_arr[right1_new[0],right1_new[1]]=math.sqrt(math.pow((right1_new[0]-goal[0]),2) + math.pow((right1_new[1]-goal[1]),2))
                total_cost_arr[right1_new[0],right1_new[1]]= cost[right1_new[0],right1_new[1]]+euclidean_distance_arr[right1_new[0],right1_new[1]]
    
    
    right2_new=right2(current_node[0],current_node[1],current_node[2])
    status=boundary_check(right2_new[0],right2_new[1])
    flag=obstacle_map(right2_new[0],right2_new[1])

    if ( ((status) and (flag)) == 1):    
        if visited_arr[right2_new[0],right2_new[1]]==0:
            visited_arr[right2_new[0],right2_new[1]]=1
            visited_nodes.append(right2_new)
            Q.append(right2_new)
            parent_list[right2_new[0]][right2_new[1]]=current_node
            euclidean_distance_arr[right2_new[0],right2_new[1]]=math.sqrt(math.pow((right2_new[0]-goal[0]),2) + math.pow((right2_new[1]-goal[1]),2))
            cost[right2_new[0],right2_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
            total_cost_arr[right2_new[0],right2_new[1]]= cost[right2_new[0],right2_new[1]]+euclidean_distance_arr[right2_new[0],right2_new[1]]
        else:
            if cost[right2_new[0],right2_new[1]]>(cost[current_node[0],current_node[1]]+math.sqrt(2)):
                cost[right2_new[0],right2_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
                parent_list[right2_new[0]][right2_new[1]]=current_node
                euclidean_distance_arr[right2_new[0],right2_new[1]]=math.sqrt(math.pow((right2_new[0]-goal[0]),2) + math.pow((right2_new[1]-goal[1]),2))
                total_cost_arr[right2_new[0],right2_new[1]]= cost[right2_new[0],right2_new[1]]+euclidean_distance_arr[right2_new[0],right2_new[1]]
            
    left1_new=left1(current_node[0],current_node[1],current_node[2])
    status=boundary_check(left1_new[0],left1_new[1])
    flag=obstacle_map(left1_new[0],left1_new[1])

    if ( ((status) and (flag)) == 1):
        if visited_arr[left1_new[0],left1_new[1]]==0:
            visited_arr[left1_new[0],left1_new[1]]=1
            visited_nodes.append(left1_new)
            Q.append(left1_new)
            parent_list[left1_new[0]][left1_new[1]]=current_node
            euclidean_distance_arr[left1_new[0],left1_new[1]]=math.sqrt(math.pow((left1_new[0]-goal[0]),2) + math.pow((left1_new[1]-goal[1]),2))
            cost[left1_new[0],left1_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
            total_cost_arr[left1_new[0],left1_new[1]]= cost[left1_new[0],left1_new[1]]+euclidean_distance_arr[left1_new[0],left1_new[1]]
        else:
            if cost[left1_new[0],left1_new[1]]>(cost[current_node[0],current_node[1]]+math.sqrt(2)):
                cost[left1_new[0],left1_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
                euclidean_distance_arr[left1_new[0],left1_new[1]]=math.sqrt(math.pow((left1_new[0]-goal[0]),2) + math.pow((left1_new[1]-goal[1]),2))
                parent_list[left1_new[0]][left1_new[1]]=current_node
                total_cost_arr[left1_new[0],left1_new[1]]= cost[left1_new[0],left1_new[1]]+euclidean_distance_arr[left1_new[0],left1_new[1]]
    
    left2_new=left2(current_node[0],current_node[1],current_node[2])
    status=boundary_check(left2_new[0],left2_new[1])
    flag=obstacle_map(left2_new[0],left2_new[1])

    if ( ((status) and (flag)) == 1):
        if visited_arr[left2_new[0],left2_new[1]]==0:
            visited_arr[left2_new[0],left2_new[1]]=1
            visited_nodes.append(left2_new)
            Q.append(left2_new)
            parent_list[left2_new[0]][left2_new[1]]=current_node
            euclidean_distance_arr[left2_new[0],left2_new[1]]=math.sqrt(math.pow((left2_new[0]-goal[0]),2) + math.pow((left2_new[1]-goal[1]),2))
            cost[left2_new[0],left2_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
            total_cost_arr[left2_new[0],left2_new[1]]= cost[left2_new[0],left2_new[1]]+euclidean_distance_arr[left2_new[0],left2_new[1]]
        else:
            if cost[left2_new[0],left2_new[1]]>(cost[current_node[0],current_node[1]]+math.sqrt(2)):
                cost[left2_new[0],left2_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
                parent_list[left2_new[0]][left2_new[1]]=current_node
                euclidean_distance_arr[left2_new[0],left2_new[1]]=math.sqrt(math.pow((left2_new[0]-goal[0]),2) + math.pow((left2_new[1]-goal[1]),2))
                total_cost_arr[left2_new[0],left2_new[1]]= cost[left2_new[0],left2_new[1]]+euclidean_distance_arr[left2_new[0],left2_new[1]]

print("Goal has been reached")


# In[17]:


start=[x_start,y_start,0]
path=[]
def path_trace(Goalbound,start):
    GN=goalbound
    path.append(goalbound)
    while (GN!=start):
        a=parent_list[GN[0]][GN[1]]
        path.append(a)
        GN=a

path_trace(goal,start)
del path[len(path)-1]
print('The path obtained is as follows',path)


# In[18]:


## Interfacing with V-Rep and giving the motions of the robot by popping from the obtained path

while path:
    p = path.pop()
    movement = p[3]
    
     # Establishing a connection with VREP through python
    vrep.simxFinish(-1)
    clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)

    if clientID!=-1:
        print ("Remote API connection secure")
        
        errorCode, Motor_left_VREP = vrep.simxGetObjectHandle(clientID, "wheel_left_joint", vrep.simx_opmode_oneshot_wait)
        errorCode, Motor_Right_VREP = vrep.simxGetObjectHandle(clientID, "wheel_right_joint", vrep.simx_opmode_oneshot_wait)
        if movement == 'straight_0':
            vrep.simxSetJointTargetVelocity(clientID, Motor_left_VREP, 60*(math.pi/30), vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, Motor_Right_VREP, 60*(math.pi/30), vrep.simx_opmode_streaming)
            time.sleep(0.4)
        
        elif movement == 'straight_1':
            vrep.simxSetJointTargetVelocity(clientID, Motor_left_VREP, 30*(math.pi/30), vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, Motor_Right_VREP, 30*(math.pi/30), vrep.simx_opmode_streaming)
            time.sleep(0.4)
        
        elif movement == 'left_0':
            vrep.simxSetJointTargetVelocity(clientID, Motor_left_VREP, 0*(math.pi/30), vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, Motor_Right_VREP, 30*(math.pi/30), vrep.simx_opmode_streaming)
            time.sleep(0.4)

        elif movement == 'left_1':
            vrep.simxSetJointTargetVelocity(clientID, Motor_left_VREP, 0*(math.pi/30), vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, Motor_Right_VREP, 60*(math.pi/30), vrep.simx_opmode_streaming)
            time.sleep(0.4)

        elif movement == 'left_2':
            vrep.simxSetJointTargetVelocity(clientID, Motor_Right_VREP, 60*(math.pi/30), vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, Motor_left_VREP, 30*(math.pi/30), vrep.simx_opmode_streaming)
            time.sleep(0.4)

    
        elif movement == 'right_0':
            vrep.simxSetJointTargetVelocity(clientID, Motor_left_VREP, 30*(math.pi/30), vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, Motor_Right_VREP, 0*(math.pi/30), vrep.simx_opmode_streaming)
            time.sleep(0.4)
            
        elif movement == 'right_1':
            vrep.simxSetJointTargetVelocity(clientID, Motor_left_VREP, 60*(math.pi/30), vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, Motor_Right_VREP, 0*(math.pi/30), vrep.simx_opmode_streaming)
            time.sleep(0.4)

        elif movement == 'right_2':
            vrep.simxSetJointTargetVelocity(clientID, Motor_left_VREP, 60*(math.pi/30), vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, Motor_Right_VREP, 30*(math.pi/30), vrep.simx_opmode_streaming)
            time.sleep(0.4)

        

vrep.simxSetJointTargetVelocity(clientID, Motor_left_VREP, 0*(math.pi/30), vrep.simx_opmode_streaming)
vrep.simxSetJointTargetVelocity(clientID, Motor_Right_VREP, 0*(math.pi/30), vrep.simx_opmode_streaming)

returnCode,handle = vrep.simxGetObjectHandle(clientID,'Turtlebot2',vrep.simx_opmode_blocking)
returnCode,position = vrep.simxGetObjectPosition(clientID,handle,-1,vrep.simx_opmode_blocking)

