# -*- coding: utf-8 -*-

'''
gaussian_process_map.py
author: 'liuxu'
email: liuxu172@mails.ucas.ac.cn
reference: https://github.com/scikit-learn

'''

import numpy as np
from scipy.linalg import cholesky, cho_solve
import scipy.optimize
from scipy.spatial.distance import pdist, cdist, squareform
from matplotlib import cm
import open3d as o3d

def loadData(filePath):
    Data = []
    fr = open(filePath)
    initialData = fr.readlines()
    fr.close()
    for element in initialData:
        lineArr = element.strip().split(' ')
        Data.append([float(x) for x in lineArr])
    return np.array(Data)

class RQkernel(): # rational quadratic kernel, shown in Equation (3)
    def __init__(self, length_scale=1.0, alpha=1.0, length_scale_bounds=(1e-5, 1e5), alpha_bounds=(1e-5, 1e5)):
        self.length_scale = length_scale
        self.alpha = alpha
        self.length_scale_bounds = length_scale_bounds
        self.alpha_bounds = alpha_bounds

    @property
    def theta(self):
        return np.log(np.hstack([self.alpha, self.length_scale]))

    @theta.setter
    def theta(self, theta):
        self.alpha, self.length_scale = np.exp(theta)

    @property
    def bounds(self):
        return np.log(np.vstack([self.alpha_bounds, self.length_scale_bounds]))
        
    def __call__(self, X, Y=None, eval_gradient=False):
        X = np.atleast_2d(X)
        if Y is None:
            dists = squareform(pdist(X, metric='sqeuclidean'))
            tmp = dists / (2 * self.alpha * self.length_scale ** 2)
            base = (1 + tmp)
            K = base ** -self.alpha
            np.fill_diagonal(K, 1)
        else:
            dists = cdist(X, Y, metric='sqeuclidean')
            K = (1 + dists / (2 * self.alpha * self.length_scale ** 2)) ** -self.alpha

        if eval_gradient:
            # gradient with respect to length_scale
            length_scale_gradient = dists * K / (self.length_scale ** 2 * base)
            length_scale_gradient = length_scale_gradient[:, :, np.newaxis]

            # gradient with respect to alpha
            alpha_gradient = K * (-self.alpha * np.log(base) + dists / (2 * self.length_scale ** 2 * base))
            alpha_gradient = alpha_gradient[:, :, np.newaxis]

            return K, np.dstack((alpha_gradient, length_scale_gradient))
        else:
            return K

class GPR(): #Gaussian process regression
    def __init__(self, kernel=None, alpha=1e-10, optimizer="L-BFGS-B"):
        print("mapping start!")
        self.kernel = kernel
        self.alpha = alpha
        self.optimizer = optimizer

    def fit(self, X, y):
        self.X_train = np.copy(X)
        self.y_train = np.copy(y)
        
        def obj_func(theta):
            lml, grad = self.log_marginal_likelihood(theta, eval_gradient=True)
            return -lml, -grad

        opt_res = scipy.optimize.minimize(obj_func,self.kernel.theta,method=self.optimizer,jac=True,bounds=self.kernel.bounds)
        self.kernel.theta = opt_res.x
        K = self.kernel(self.X_train)
        K[np.diag_indices_from(K)] += self.alpha
        self.L_ = cholesky(K, lower=True)
        self.alpha_ = cho_solve((self.L_, True), self.y_train)
        return self

    def predict(self, X, return_std=False, return_cov=False, return_gradient=False):
        K_trans = self.kernel(X, self.X_train)
        y_mean = K_trans.dot(self.alpha_)   # the mean of the posterior Gaussian distribution, shown in Equation (1)
        if return_gradient: #calculate the gradients respect to x and y
            x_test, y_test = X[:,0].reshape(1,-1), X[:,1].reshape(1,-1)
            x_training, y_training = self.X_train[:,0].reshape(-1,1), self.X_train[:,1].reshape(-1,1)
            x_temp, y_temp = (x_test - x_training).T, (y_test - y_training).T
            gradient_x = ((K_trans**(1+1/self.kernel.alpha)*x_temp).dot(self.alpha_))*(-1/self.kernel.length_scale**2)
            gradient_y = ((K_trans**(1+1/self.kernel.alpha)*y_temp).dot(self.alpha_))*(-1/self.kernel.length_scale**2)
            gradient = np.c_[gradient_x, gradient_y]
            return y_mean, gradient
        else:
            return y_mean

    def log_marginal_likelihood(self, theta=None, eval_gradient=False):
        kernel = self.kernel
        kernel.theta = theta
        K, K_gradient = self.kernel(self.X_train, eval_gradient=True)
        K[np.diag_indices_from(K)] += self.alpha
        L = cholesky(K, lower=True)
    
        # Support multi-dimensional output of self.y_train
        y_train = self.y_train
        if y_train.ndim == 1:
            y_train = y_train[:, np.newaxis]

        alpha = cho_solve((L, True), y_train)

        # Compute log-likelihood
        log_likelihood_dims = -0.5 * np.einsum("ik,ik->k", y_train, alpha)
        log_likelihood_dims -= np.log(np.diag(L)).sum()
        log_likelihood_dims -= K.shape[0] / 2 * np.log(2 * np.pi)
        log_likelihood = log_likelihood_dims.sum(-1)

        if eval_gradient:
            tmp = np.einsum("ik,jk->ijk", alpha, alpha)
            tmp -= cho_solve((L, True), np.eye(K.shape[0]))[:, :, np.newaxis]
            log_likelihood_gradient_dims = 0.5 * np.einsum("ijl,ijk->kl", tmp, K_gradient)
            log_likelihood_gradient = log_likelihood_gradient_dims.sum(-1)

        return log_likelihood, log_likelihood_gradient

if __name__ == '__main__':

    quarry = 'dataSets\\quarry.xyz'

    data = loadData(quarry)

    X, Y = data[:,0:2], data[:,2]

    X2predicted = []
    for x in np.linspace(np.min(X[:,0])+2, np.max(X[:,0])-2, 260):
        for y in np.linspace(np.min(X[:,1])+3, np.max(X[:,1])-2, 260):
            X2predicted.append([x,y])
    X2predicted = np.array(X2predicted)

    gpr = GPR(kernel=RQkernel(length_scale_bounds=(1e-5, 1e5), alpha_bounds=(1e-5, 1e5)))
    gpr.fit(X, Y)
    Y_predicted, gradient_predicted = gpr.predict(X2predicted, return_gradient=True)
    gradients = np.sqrt(gradient_predicted[:,0]**2+gradient_predicted[:,1]**2)
    
    #rendered by the gradients
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.c_[X2predicted, Y_predicted])
    scalar2Rgb = cm.ScalarMappable(cmap = 'jet')
    pointsColor = scalar2Rgb.to_rgba(gradients)[:,0:3]
    pcd.colors = o3d.utility.Vector3dVector(pointsColor)
    o3d.visualization.draw_geometries([pcd])