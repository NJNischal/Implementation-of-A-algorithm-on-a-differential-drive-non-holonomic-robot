#!/usr/bin/env python
# coding: utf-8

# In[1]:


import math
import numpy as np
import cv2 as cv
from scipy.spatial import distance


# In[2]:


## ALL measurements are given in centimeters
r=3.8
l=23
dt=0.5
d=20


# In[3]:


def obs_map(x,y):
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


# In[4]:


obstacle_map_arr = np.ones((1010,1110,3),np.uint8)*255

for x in range(0,1110):
    for y in range(0,1010):
        if obs_map(x,y) == 0:
            obstacle_map_arr[y,x] = (0,0,0)


# In[5]:


def check_boundary(x,y):
    if (x<d or x>1009-d or y<d or y>1109-d ):
        return 0
    else:
        return 1


# In[6]:


parent_list=[]
for j in range (1110):
    column=[]
    for i in range (1010):
        column.append(0)
    parent_list.append(column)


# In[7]:


## Inputing THE START POINT TO THE TURTLEBOT
x_start=int(input("Please enter start point x coordinate"))
y_start=int(input("Please enter start point y coordinate"))

start_obs=obs_map(x_start,y_start)
start_boundary=check_boundary(x_start,y_start)

while( ((start_obs) and (start_boundary)) !=1):
    print("Incorrect start point! Please enter a valid start point")
    x_start=int(input("Please enter start point x coordinate"))
    y_start=int(input("Please enter start point y coordinate"))
    start_obs=obs_map(x_start,y_start)
    start_boundary=check_boundary(x_start,y_start)


start=[x_start,y_start]


# In[8]:


## TAKING INPUT OF THE GOAL POINT TO THE TURTLEBOT

x_goal=int(input("Enter the x-coordinate for the goal position"))
y_goal=int(input("Enter the y-coordinate for the goal position"))

goal_obs=obs_map(x_goal,y_goal)
goal_boundary=check_boundary(x_goal,y_goal)

while( ((goal_obs) and (goal_boundary)) !=1):
    print("Incorrect goal point! Please enter a valid goal point")
    x_goal=int(input("Enter another x-coordinate for the goal position"))
    y_goal=int(input("Enter another y-coordinate for the goal position"))
    goal_obs=obs_map(x_goal,y_goal)
    goal_boundary=check_boundary(x_goal,y_goal)

goal=[x_goal,y_goal]


# In[9]:


def Goal(x,y):
    if (x-goal[0])**2+(y-goal[1])**2-40<=0:
        return 1
    else:
        return 0


# In[10]:


cost=np.array(np.ones((1110,1010)) * np.inf)
euclidean_array=np.array(np.ones((1110,1010)) * np.inf)
totalcost=np.array(np.ones((1110,1010)) * np.inf)
visited=np.array(np.zeros((1110,1010)))


# In[11]:


# APPENDING NODES TO PRIORITY QUE

Q=[]
Q.append([x_start,y_start,0])
cost[x_start][y_start]=0
totalcost[x_start][y_start]=0

def Priority_Pop(Q):
    index_min=0
    #print('q',Q)
    X_min = Q[0][0] 
    Y_min = Q[0][1]
    for i in range(len(Q)):
        x = Q[i][0]
        y = Q[i][1]
        if totalcost[x,y] < totalcost[X_min,Y_min]:
            index_min = i
            X_min = x 
            Y_min= y
    current_node = Q[index_min]
    Q.remove(Q[index_min])
    return current_node


# In[12]:


RPM1=60
RPM2=30

#Conversion to radians per sec from rpm
r1=(2*22*RPM1)/(60*7)   
r2=(2*22*RPM2)/(60*7)


# In[13]:


def Differential_motion(r1,r2,theta):
    dtheta=(r*(r1-r2)*dt)/l + theta
    dx=(r*(r1+r2)*math.cos(dtheta)*dt)/2
    dy=(r*(r1+r2)*math.sin(dtheta)*dt)/2
    return dtheta,dx,dy


# In[14]:


# Movements of the Turtle Bot Robot

def straight(i,j,theta):
    dtheta,dx,dy=Differential_motion(r1,r1,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)))
    return new_node

def straight_faster(i,j,theta):
    dtheta,dx,dy=Differential_motion(r2,r2,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)))
    return new_node

def right(i,j,theta):
    dtheta,dx,dy=Differential_motion(r1,0,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)))
    return new_node

def right1(i,j,theta):
    dtheta,dx,dy=Differential_motion(r2,0,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)))
    return new_node

def right2(i,j,theta):
    if r1>r2:
        dtheta,dx,dy=Differential_motion(r1,r2,theta)
    else:
        dtheta,dx,dy=Differential_motion(r2,r2,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)))
    return new_node

def left(i,j,theta):
    dtheta,dx,dy=Differential_motion(0,r1,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)))
    return new_node

def left1(i,j,theta):
    dtheta,dx,dy=Differential_motion(0,r2,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)))
    return new_node

def left2(i,j,theta):
    if r1>r2:
        dtheta,dx,dy=Differential_motion(r2,r1,theta)
    else:
        dtheta,dx,dy=Differential_motion(r1,r2,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)))
    return new_node


# In[15]:


visited_node=[]
current_node=[x_start,y_start,0]
while True: 
    if Goal(current_node[0],current_node[1])==1:
        goalbound=current_node
        break
    current_node=Priority_Pop(Q)
    straight_new=straight (current_node[0],current_node[1],current_node[2])
    status=check_boundary(straight_new[0],straight_new[1])
    flag=obs_map(straight_new[0],straight_new[1])
    if ( ((status) and (flag)) == 1):
        if visited[straight_new[0],straight_new[1]]==0:
            visited[straight_new[0],straight_new[1]]=1
            visited_node.append(straight_new)
            Q.append(straight_new)
            parent_list[straight_new[0]][straight_new[1]]=current_node
            
            cost[straight_new[0],straight_new[1]]=(cost[current_node[0],current_node[1]]+1)
            euclidean_array[straight_new[0],straight_new[1]]=math.sqrt(math.pow((straight_new[0]-goal[0]),2) + math.pow((straight_new[1]-goal[1]),2))
            totalcost[straight_new[0],straight_new[1]]= cost[straight_new[0],straight_new[1]]+euclidean_array[straight_new[0],straight_new[1]]
        else:
            if cost[straight_new[0],straight_new[1]]>(cost[current_node[0],current_node[1]]+1):
                cost[straight_new[0],straight_new[1]]=(cost[current_node[0],current_node[1]]+1)
                euclidean_array[straight_new[0],straight_new[1]]=math.sqrt(math.pow((straight_new[0]-goal[0]),2) + math.pow((straight_new[1]-goal[1]),2))
                parent_list[straight_new[0]][straight_new[1]]=current_node
                totalcost[straight_new[0],straight_new[1]]= cost[straight_new[0],straight_new[1]]+euclidean_array[straight_new[0],straight_new[1]]
    
    
    straight_faster_new=straight_faster(current_node[0],current_node[1],current_node[2])
    status=check_boundary(straight_faster_new[0],straight_faster_new[1])
    flag=obs_map(straight_faster_new[0],straight_faster_new[1])
    if ( ((status) and (flag)) == 1):
        if visited[straight_faster_new[0],straight_faster_new[1]]==0:
            visited[straight_faster_new[0],straight_faster_new[1]]=1
            visited_node.append(straight_faster_new)
            Q.append(straight_faster_new)
            parent_list[straight_faster_new[0]][straight_faster_new[1]]=current_node
            cost[straight_faster_new[0],straight_faster_new[1]]=(cost[current_node[0],current_node[1]]+1)
            euclidean_array[straight_faster_new[0],straight_faster_new[1]]=math.sqrt(math.pow((straight_faster_new[0]-goal[0]),2) + math.pow((straight_faster_new[1]-goal[1]),2))
            totalcost[straight_faster_new[0],straight_faster_new[1]]= cost[straight_faster_new[0],straight_faster_new[1]]+euclidean_array[straight_faster_new[0],straight_faster_new[1]]
        else:
            if cost[straight_faster_new[0],straight_faster_new[1]]>(cost[current_node[0],current_node[1]]+1):
                cost[straight_faster_new[0],straight_faster_new[1]]=(cost[current_node[0],current_node[1]]+1)
                euclidean_array[straight_faster_new[0],straight_faster_new[1]]=math.sqrt(math.pow((straight_faster_new[0]-goal[0]),2) + math.pow((straight_faster_new[1]-goal[1]),2))
                parent_list[straight_faster_new[0]][straight_faster_new[1]]=current_node
                totalcost[straight_faster_new[0],straight_faster_new[1]]= cost[straight_faster_new[0],straight_faster_new[1]]+euclidean_array[straight_faster_new[0],straight_faster_new[1]]
    
    right_new=right(current_node[0],current_node[1],current_node[2])
    status=check_boundary(right_new[0],right_new[1])
    flag=obs_map(straight_new[0],straight_new[1])
    if ( ((status) and (flag)) == 1):
        if visited[right_new[0],right_new[1]]==0:
            visited[right_new[0],right_new[1]]=1
            visited_node.append(right_new)
            Q.append(right_new)         
            parent_list[right_new[0]][right_new[1]]=current_node
            cost[right_new[0],right_new[1]]=(cost[current_node[0],current_node[1]]+1)
            euclidean_array[right_new[0],right_new[1]]=math.sqrt(math.pow((right_new[0]-goal[0]),2) + math.pow((right_new[1]-goal[1]),2))
            totalcost[right_new[0],right_new[1]]= cost[right_new[0],right_new[1]]+euclidean_array[right_new[0],right_new[1]]

        else:
            if cost[right_new[0],right_new[1]]>(cost[current_node[0],current_node[1]]+1):
                cost[right_new[0],right_new[1]]=(cost[current_node[0],current_node[1]]+1)
                parent_list[right_new[0]][right_new[1]]=current_node
                euclidean_array[right_new[0],right_new[1]]=math.sqrt(math.pow((right_new[0]-goal[0]),2) + math.pow((right_new[1]-goal[1]),2))
                totalcost[right_new[0],right_new[1]]= cost[right_new[0],right_new[1]]+euclidean_array[right_new[0],right_new[1]]
    
    
    
    left_new=left(current_node[0],current_node[1],current_node[2])
    status=check_boundary(left_new[0],left_new[1])
    flag=obs_map(left_new[0],left_new[1])
    if ( ((status) and (flag)) == 1):
        if visited[left_new[0],left_new[1]]==0:
            visited[left_new[0],left_new[1]]=1
            visited_node.append(left_new)
            Q.append(left_new)
            parent_list[left_new[0]][left_new[1]]=current_node
            euclidean_array[left_new[0],left_new[1]]=math.sqrt(math.pow((left_new[0]-goal[0]),2) + math.pow((left_new[1]-goal[1]),2))
            cost[left_new[0],left_new[1]]=(cost[current_node[0],current_node[1]]+1)
            totalcost[left_new[0],left_new[1]]= cost[left_new[0],left_new[1]]+euclidean_array[left_new[0],left_new[1]]
        else:
            if cost[left_new[0],left_new[1]]>(cost[current_node[0],current_node[1]]+1):
                cost[left_new[0],left_new[1]]=(cost[current_node[0],current_node[1]]+1)
                parent_list[left_new[0]][left_new[1]]=current_node
                euclidean_array[left_new[0],left_new[1]]=math.sqrt(math.pow((left_new[0]-goal[0]),2) + math.pow((left_new[1]-goal[1]),2))
                totalcost[left_new[0],left_new[1]]= cost[left_new[0],left_new[1]]+euclidean_array[left_new[0],left_new[1]]
    
    
    right1_new=right1(current_node[0],current_node[1],current_node[2])
    status=check_boundary(right1_new[0],right1_new[1])
    flag=obs_map(right1_new[0],right1_new[1])
    if ( ((status) and (flag)) == 1):   
        if visited[right1_new[0],right1_new[1]]==0:
            visited[right1_new[0],right1_new[1]]=1
            visited_node.append(right1_new)
            Q.append(right1_new)
            parent_list[right1_new[0]][right1_new[1]]=current_node
            euclidean_array[right1_new[0],right1_new[1]]=math.sqrt(math.pow((right1_new[0]-goal[0]),2) + math.pow((right1_new[1]-goal[1]),2))
            cost[right1_new[0],right1_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
            totalcost[right1_new[0],right1_new[1]]= cost[right1_new[0],right1_new[1]]+euclidean_array[right1_new[0],right1_new[1]]
        else:
            if cost[right1_new[0],right1_new[1]]>(cost[current_node[0],current_node[1]]+math.sqrt(2)):
                cost[right1_new[0],right1_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
                parent_list[right1_new[0]][right1_new[1]]=current_node
                euclidean_array[right1_new[0],right1_new[1]]=math.sqrt(math.pow((right1_new[0]-goal[0]),2) + math.pow((right1_new[1]-goal[1]),2))
                totalcost[right1_new[0],right1_new[1]]= cost[right1_new[0],right1_new[1]]+euclidean_array[right1_new[0],right1_new[1]]
    
    
    right2_new=right2(current_node[0],current_node[1],current_node[2])
    status=check_boundary(right2_new[0],right2_new[1])
    flag=obs_map(right2_new[0],right2_new[1])
    if ( ((status) and (flag)) == 1):    
        if visited[right2_new[0],right2_new[1]]==0:
            visited[right2_new[0],right2_new[1]]=1
            visited_node.append(right2_new)
            Q.append(right2_new)
            parent_list[right2_new[0]][right2_new[1]]=current_node
            euclidean_array[right2_new[0],right2_new[1]]=math.sqrt(math.pow((right2_new[0]-goal[0]),2) + math.pow((right2_new[1]-goal[1]),2))
            cost[right2_new[0],right2_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
            totalcost[right2_new[0],right2_new[1]]= cost[right2_new[0],right2_new[1]]+euclidean_array[right2_new[0],right2_new[1]]
        else:
            if cost[right2_new[0],right2_new[1]]>(cost[current_node[0],current_node[1]]+math.sqrt(2)):
                cost[right2_new[0],right2_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
                parent_list[right2_new[0]][right2_new[1]]=current_node
                euclidean_array[right2_new[0],right2_new[1]]=math.sqrt(math.pow((right2_new[0]-goal[0]),2) + math.pow((right2_new[1]-goal[1]),2))
                totalcost[right2_new[0],right2_new[1]]= cost[right2_new[0],right2_new[1]]+euclidean_array[right2_new[0],right2_new[1]]
            
    left1_new=left1(current_node[0],current_node[1],current_node[2])
    status=check_boundary(left1_new[0],left1_new[1])
    flag=obs_map(left1_new[0],left1_new[1])
    if ( ((status) and (flag)) == 1):
        if visited[left1_new[0],left1_new[1]]==0:
            visited[left1_new[0],left1_new[1]]=1
            visited_node.append(left1_new)
            Q.append(left1_new)
            parent_list[left1_new[0]][left1_new[1]]=current_node
            euclidean_array[left1_new[0],left1_new[1]]=math.sqrt(math.pow((left1_new[0]-goal[0]),2) + math.pow((left1_new[1]-goal[1]),2))
            cost[left1_new[0],left1_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
            totalcost[left1_new[0],left1_new[1]]= cost[left1_new[0],left1_new[1]]+euclidean_array[left1_new[0],left1_new[1]]
        else:
            if cost[left1_new[0],left1_new[1]]>(cost[current_node[0],current_node[1]]+math.sqrt(2)):
                cost[left1_new[0],left1_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
                euclidean_array[left1_new[0],left1_new[1]]=math.sqrt(math.pow((left1_new[0]-goal[0]),2) + math.pow((left1_new[1]-goal[1]),2))
                parent_list[left1_new[0]][left1_new[1]]=current_node
                totalcost[left1_new[0],left1_new[1]]= cost[left1_new[0],left1_new[1]]+euclidean_array[left1_new[0],left1_new[1]]
    
    left2_new=left2(current_node[0],current_node[1],current_node[2])
    status=check_boundary(left2_new[0],left2_new[1])
    flag=obs_map(left2_new[0],left2_new[1])
    if ( ((status) and (flag)) == 1):
        if visited[left2_new[0],left2_new[1]]==0:
            visited[left2_new[0],left2_new[1]]=1
            visited_node.append(left2_new)
            Q.append(left2_new)
            parent_list[left2_new[0]][left2_new[1]]=current_node
            euclidean_array[left2_new[0],left2_new[1]]=math.sqrt(math.pow((left2_new[0]-goal[0]),2) + math.pow((left2_new[1]-goal[1]),2))
            cost[left2_new[0],left2_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
            totalcost[left2_new[0],left2_new[1]]= cost[left2_new[0],left2_new[1]]+euclidean_array[left2_new[0],left2_new[1]]
        else:
            if cost[left2_new[0],left2_new[1]]>(cost[current_node[0],current_node[1]]+math.sqrt(2)):
                cost[left2_new[0],left2_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
                parent_list[left2_new[0]][left2_new[1]]=current_node
                euclidean_array[left2_new[0],left2_new[1]]=math.sqrt(math.pow((left2_new[0]-goal[0]),2) + math.pow((left2_new[1]-goal[1]),2))
                totalcost[left2_new[0],left2_new[1]]= cost[left2_new[0],left2_new[1]]+euclidean_array[left2_new[0],left2_new[1]]

print("Goal reached")


# In[16]:


start=[x_start,y_start,0]
path=[]
def path_find(Goalbound,start):
    GN=goalbound
    print('GN',GN)
    path.append(goalbound)
    while (GN!=start):
        a=parent_list[GN[0]][GN[1]]
        path.append(a)
        GN=a

path_find(goal,start)
print('path',path)


# In[17]:


cv.circle(obstacle_map_arr,(int(goal[0]),int(goal[1])), (1), (0,0,255), -1);
cv.circle(obstacle_map_arr,(int(start[0]),int(start[1])), (1), (0,0,255), -1);

for i in visited_node:
    cv.circle(obstacle_map_arr,(int(i[0]),int(i[1])), (1), (255,0,0));
    output_img=cv.resize(obstacle_map_arr,None,fx=1,fy=1)
    cv.imshow('maze',output_img)
    cv.waitKey(1)

for i in path:
    cv.circle(obstacle_map_arr,(int(i[0]),int(i[1])), (1), (0,0,255));
    output_img=cv.resize(obstacle_map_arr,None,fx=1,fy=1)
    cv.imshow('maze',output_img)
    cv.waitKey(1)
    

cv.imwrite('Output_a_star.png',obstacle_map_arr)
cv.waitKey(0) 
cv.destroyAllWindows()

