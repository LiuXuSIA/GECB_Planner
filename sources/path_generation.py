"""
path_generation.py
author: 'liuxu'
email: liuxu172@mails.ucas.ac.cn
reference: https://github.com/AtsushiSakai/PythonRobotics
"""

import math
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
import path_smoothing
from  gaussian_process_map import loadData
import gaussian_process_map as gpm
from gaussian_process_map import RQkernel

class GridSearchPlanner:
    def __init__(self, data, resolution, w_elevation, w_gradient, w_distance):
        """
        Initialization
        resolution: grid resolution
        data: terrain data
        """
        print("planning start!")
        self.resolution = resolution
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.x_width, self.y_width = 0, 0
        self.w_elevation = w_elevation 
        self.w_gradient = w_gradient
        self.w_distance = w_distance
        self.motion = self.get_motion_model()
        self.calc_attributes_map(data)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

    def planning(self, sx, sy, gx, gy):
        """
        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        plt.title('Path generation', fontsize=15)
        plt.scatter(start_node.x, start_node.y, s=100,marker='^',color='red')
        plt.scatter(goal_node.x, goal_node.y,s=100,marker='v',color='red')
        plt.text(8, 0.5, 'the start state', fontsize=15)
        plt.text(13, 30.5, 'the goal state', fontsize=15)
        # plt.grid(True)
        plt.axis("equal")
        plt.pause(0.01)

        plt.title('Path generating', fontsize=15)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node 

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[c_id]

            # show graph
            plt.scatter(current.x, current.y, marker="x",s=70,c='darkorange')
            # plt.scatter(current.x, current.y,s=40,marker='*',color='red')#, ".c", )
            if len(closed_set.keys()) % 10 == 0:
                plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                # If the node is not safe, do nothing
                if not self.verify_node(current):
                    continue
                # the elevation change 3
                added_cost = np.abs(self.elevation_map[current.x][current.y] - \
                    self.elevation_map[current.x + self.motion[i][0]][current.y + self.motion[i][1]])*self.w_elevation
                # the traveled gradient  1
                added_cost += self.gradient_map[current.x + self.motion[i][0]][current.y + self.motion[i][1]]*self.w_gradient
                # the traveled distance   0.01
                added_cost += math.hypot(self.motion[i][0], self.motion[i][1])*self.w_distance
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + added_cost, c_id)
                n_id = self.calc_grid_index(node)

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [goal_node.x], [goal_node.y]
        rx_true, ry_true = [self.calc_grid_position(goal_node.x, self.min_x)], [self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(n.x)
            ry.append(n.y)
            parent_index = n.parent_index
        return rx, ry

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return int(np.floor((position - min_pos) / self.resolution))

    def calc_grid_index(self, node):
        return node.y * self.x_width + node.x

    def verify_node(self, node):
        if node.x + 1 > self.x_width - 1:
            return False
        elif node.x - 1 < 0:
            return False
        elif node.y + 1 > self.y_width - 1:
            return False
        elif node.y - 1 < 0:
            return False
        return True

    def calc_attributes_map(self, data):

        self.min_x = np.min(data[:,0])
        self.min_y = np.min(data[:,1])
        self.max_x = np.max(data[:,0])
        self.max_y = np.max(data[:,1])
        self.x_width = int(np.floor((self.max_x - self.min_x) / self.resolution)) + 1 
        self.y_width = int(np.floor((self.max_y - self.min_y) / self.resolution)) + 1

        print("x_width:", self.x_width)
        print("y_width:", self.y_width)
        
        self.elevation_map = -np.ones([self.x_width, self.y_width]) * np.inf
        for attribute in data:
            x = self.calc_xy_index(attribute[0], self.min_x)
            y = self.calc_xy_index(attribute[1], self.min_y)
            if self.elevation_map[x, y] < attribute[2]:
                self.elevation_map[x, y] = attribute[2]

        self.gradient_map = np.zeros([self.x_width, self.y_width])
        for attribute in data:
            x = self.calc_xy_index(attribute[0], self.min_x)
            y = self.calc_xy_index(attribute[1], self.min_y)
            if self.gradient_map[x, y] < attribute[3]:  #3 3
                self.gradient_map[x, y] = attribute[3]
            
        passability = 0.5*self.elevation_map.T+10.0*self.gradient_map.T

        plt.pcolormesh(passability, cmap=plt.cm.RdYlGn_r)
        # plt.pause(0.001)
    
    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0],
                  [0, 1],
                  [-1, 0],
                  [0, -1],
                  [-1, -1],
                  [-1, 1],
                  [1, -1],
                  [1, 1]]
        return motion


if __name__ == '__main__':

    quarry = 'dataSets\\quarry.xyz'

    data = loadData(quarry)

    X, Y = data[:,0:2], data[:,2]
    X2predicted = []
    for x in np.linspace(np.min(X[:,0])+2, np.max(X[:,0])-2, 200):
        for y in np.linspace(np.min(X[:,1])+3, np.max(X[:,1])-2, 200):
            X2predicted.append([x,y])
    X2predicted = np.array(X2predicted)
    
    gpr = gpm.GPR(kernel=RQkernel(length_scale_bounds=(10, 1e5), alpha_bounds=(10, 1e5)))
    gpr.fit(X, Y)
   
    Y_predicted, gradient_predicted = gpr.predict(X2predicted, return_gradient=True)
    gradients = np.sqrt(gradient_predicted[:,0]**2+gradient_predicted[:,1]**2)
    cost_map = np.c_[X2predicted, Y_predicted, gradients]

    sx = -89.3
    sy = -72.9
    gx = -9.06
    gy = 48.9
    grid_size = 4

    w_elevation = 0.1
    w_gradient = 0.5
    w_distance = 0.01
    
    path_planner = GridSearchPlanner(cost_map, grid_size, w_elevation, w_gradient, w_distance)

    rx, ry = path_planner.planning(sx, sy, gx, gy)
    
    path_ctrls = path_smoothing.ctrls_optimization(np.c_[rx, ry])
    path_smoothed = path_smoothing.bezier_curve(path_ctrls, n_points = 500)
    
    plt.title('Find goal', fontsize=15)
    plt.scatter(rx[1:-1], ry[1:-1], marker='o',s=80,color='blue',label='path points')
    plt.plot(rx, ry, "-r", linewidth=5,label='path')
    plt.plot(path_smoothed[:,0], path_smoothed[:,1], "-b", linewidth=2,label='trajectory')
    plt.pause(0.001)
    plt.show()