# -*- coding: utf-8 -*-

'''
gecb_planner.py
author: 'liuxu'
email: liuxu172@mails.ucas.ac.cn

'''

import gaussian_process_map as gpm
from  gaussian_process_map import loadData
from gaussian_process_map import RQkernel
import path_generation as pg
import path_smoothing as ps
import open3d as o3d
import numpy as np
from matplotlib import cm
import matplotlib.pyplot as plt

class gecb_planner():

    def __init__(self, data, resolution, sx, sy, gx, gy):
        self.data = data
        self.resolution = resolution
        self.sx = sx
        self.sy = sy
        self.gx = gx
        self.gy = gy

    def map_building(self):
        X, Y = self.data[:,0:2], self.data[:,2]
        X2predicted = []
        for x in np.linspace(np.min(X[:,0])+2, np.max(X[:,0])-2, 200):
            for y in np.linspace(np.min(X[:,1])+3, np.max(X[:,1])-2, 200):
                X2predicted.append([x,y])
        self.X2predicted = np.array(X2predicted)
        
        #the bounds of the kernel parameters have strong influence on the gradient calculation,
        gpr = gpm.GPR(kernel=RQkernel(length_scale_bounds=(1e-5, 1e5), alpha_bounds=(1e-5, 1e5)))
        gpr.fit(X, Y)
        # training_mse = np.sum((Y_test-Y_predicted)**2)/len(Y_test)
        # print("training_mse", training_mse)
        self.Y_predicted, gradient_predicted = gpr.predict(self.X2predicted, return_gradient=True)
        self.gradients = np.sqrt(gradient_predicted[:,0]**2+gradient_predicted[:,1]**2)

    def map_display(self):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.c_[self.X2predicted, self.Y_predicted])
        scalar2Rgb = cm.ScalarMappable(cmap = 'jet')
        pointsColor = scalar2Rgb.to_rgba(self.gradients)[:,0:3]
        pcd.colors = o3d.utility.Vector3dVector(pointsColor)
        o3d.visualization.draw_geometries([pcd])

    def path_generation(self, w_elevation, w_gradient, w_distance):
        cost_map = np.c_[self.X2predicted, self.Y_predicted, self.gradients]
        path_planner = pg.GridSearchPlanner(cost_map, self.resolution, w_elevation, w_gradient, w_distance)
        self.rx, self.ry = path_planner.planning(self.sx, self.sy, self.gx, self.gy)
        plt.title('Find goal', fontsize=15)
        plt.scatter(self.rx[1:-1], self.ry[1:-1], marker='o',s=80,color='blue',label='path points')
        plt.plot(self.rx, self.ry, "-r", linewidth=5,label='path')
        plt.pause(0.001)

    def path_smoothing(self):
        path_ctrls = ps.ctrls_optimization(np.c_[self.rx, self.ry])
        path_smoothed = ps.bezier_curve(path_ctrls, n_points = 500)
        plt.plot(path_smoothed[:,0], path_smoothed[:,1], "-b", linewidth=2,label='trajectory')
        plt.show()


if __name__ == "__main__":

    quarry = 'dataSets\\quarry.xyz'
    
    data =loadData(quarry)
    
    sx = -89.3
    sy = -72.9
    gx = -9.06
    gy = 48.9
    grid_size = 4

    w_elevation = 0.1
    w_gradient = 0.5
    w_distance = 0.01
    
    gecb = gecb_planner(data, grid_size, sx, sy, gx, gy)
    gecb.map_building()
    gecb.map_display()
    gecb.path_generation(w_elevation, w_gradient, w_distance)
    gecb.path_smoothing()