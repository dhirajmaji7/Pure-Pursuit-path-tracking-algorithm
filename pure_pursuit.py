# -*- coding: utf-8 -*-
"""
Created on Tue Mar  2 22:15:15 2021

@author: dhirajmaji7
"""

import numpy as np
import math
import matplotlib.pyplot as plt

L = 2 #[m] Look ahead distance 
dt = 0.1 #[s] time tick
path_type = 1 # 0: straight line, 1: modified sine curve

class Current_State: #store the current state of the vehicle
    
    def __init__(self, x = 0.0, y = 0.0, v = 0.0, theta = 0.0):
        self.x = x
        self.y = y
        self.v = v
        self.theta = theta
        
    def update_state(self, deg):
        self.theta += deg 
        self.x += self.v * dt * math.cos(self.theta)
        self.y += self.v * dt * math.sin(self.theta)
        
        
class Previous_States: #store information of all the previous states
    
    def __init__(self):
        self.x = []
        self.y = []
        self.v = []
        self.theta = []
        self.t = []
    
    def add_to_list(self, state, t):
        self.x.append(state.x)
        self.y.append(state.y)
        self.v.append(state.v)
        self.theta.append(state.theta)
        self.t.append(t)
        
   
def path(): 
    p_x = np.arange(0, 50, 0.25)
    
    if path_type == 0: #straight line
        p_y = [0 for i in p_x] 
    elif path_type == 1: #modified sine curve
        p_y = [math.sin(i/5)*i/5 for i in p_x]
    
    #plt.subplots(1)
    #plt.plot(p_x, p_y,"-r")
    #plt.show()
    
    return p_x, p_y

def distance(current_state, d_x, d_y):
    dx = d_x - current_state.x
    dy = d_y - current_state.y
    d = np.sqrt(np.square(dx) + np.square(dy))
    return d
    

def goal_point(current_state, pp_x, pp_y, old_index):
    #calculate the nearest path point in the forward direction
    dist_x = [i_x - current_state.x for i_x in pp_x[old_index:]]
    dist_y = [i_y - current_state.y for i_y in pp_y[old_index:]]
    dist = np.sqrt(np.square(dist_x) + np.square(dist_y))
    d_index = np.argmin(dist)
    d_index += old_index
    #Finding the nearest path point just ahead of the Look-ahead distance
    while distance(current_state, pp_x[d_index], pp_y[d_index]) < L:
        d_index +=1
    
    return d_index

def curvature(current_state, c_y, c_index):
    curvature_y = c_y[c_index] - current_state.y
    degree = math.atan2((2 * curvature_y), (L**2))
    #print(degree)
    return degree
  
    
if __name__ == '__main__':
    #initializing variables
    time = 0.0
    current_index = 0
    
    #creating the path
    path_x, path_y = path()
    print("Path planning complete")
    goal_index = len(path_x) - 1
    
    #initializing our robot/vehicle object 
    current_location = Current_State(x = 0, y = -2, v = 1, theta = 0)
    
    #initializing object for storing the states data
    states_data = Previous_States()
    states_data.add_to_list(current_location, time)
    
    #loop for path tracking begins
    while current_index < goal_index:
        new_index = goal_point(current_location, path_x, path_y, current_index)
        steering_degree = curvature(current_location, path_y, new_index)
        
        steering_degree = steering_degree - current_location.theta
        current_location.update_state(steering_degree)
        
        time += dt
        states_data.add_to_list(current_location, time)
        #print(new_index)
        current_index = new_index
        
        plt.plot(path_x, path_y, "-r", label="course")
        plt.plot(states_data.x, states_data.y, "-b", label="trajectory")
        plt.plot(path_x[new_index], path_y[new_index], "xg", label="target")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.01)
            
    

   