# -*- coding: utf-8 -*-

'''
path_smootging.py
author: 'liuxu'
email: liuxu172@mails.ucas.ac.cn
reference: https://github.com/zhipeiyan/kappa-Curves
'''

'''
uasge:
1) run the code
2) click "left mouse button" to determine the control points
   Ensure there are at least four control points
3) press the "space key" to generate the curve
4) click the "right mouse button" to tweak the control points
5) press the "escape key" to clear the canvas
6) 2)->5)
'''

import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg, special

def bernstein_poly(n, i, t):
    return special.comb(n, i) * t ** i * (1 - t) ** (n - i)

def bezier_point(t, control_points):
    n = len(control_points) - 1
    return np.sum([bernstein_poly(n, i, t) * control_points[i] for i in range(n + 1)], axis=0)

def bezier_curve(control_points, n_points = 100):
    inters = []
    for control_point in control_points:
        for t in np.linspace(0, 1, n_points):
            inters.append(bezier_point(t, control_point))
    return np.array(inters)

def bezier_derivatives_control_points(control_points, n_order):
    # A derivative of a n-order bezier curve is a (n-1)-order bezier curve.
    w = {0: control_points}
    for i in range(n_order):
        for k in range(len(w[i])):
            n = len(w[i][k])
            if k == 0:
                w[i + 1] = [[(n - 1) * (w[i][k][j + 1] - w[i][k][j]) for j in range(n - 1)]]
            else:
                w[i + 1].append([(n - 1) * (w[i][k][j + 1] - w[i][k][j]) for j in range(n - 1)])
    return w

def bezier_curvature(dx, dy, ddx, ddy):
    #Compute curvature at one point given first and second derivatives.
    return (dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** (3 / 2)

'''
Solve the cubic equation shown in equation (17) in the form of
a*t^3 + b*t^2 + c*t + d = 0, 
and the solution will be in [0, 1]
'''
def max_t(c0, cp, c2):
    # the coefficients
    # Notice: there is an typo of coefficient a in the original manuscript.
    # The correct one should be:
    a = np.dot(c2 - c0, c2 - c0)
    b = 3 * np.dot(c2 - c0, c0 - cp)
    c = np.dot(3 * c0 - 2 * cp - c2, c0 - cp)
    d = - np.dot(c0 - cp, c0 - cp)

    # to transform the equation to be x^3 + p*x + q = 0, 
    # where x = t + b / 3a
    p = (3 * a * c - b ** 2) / (3 * a ** 2)
    q = (2 * b ** 3 - 9 * a * b * c + 27 * a ** 2 * d) / (27 * a ** 3)
    
    # discriminant of whether there are multiple roots 
    delta = -(4 * p ** 3 + 27 * q ** 2)
    if (delta <= 0):
        # single real root
	    return np.cbrt(-q / 2 + np.sqrt(q ** 2 / 4 + p ** 3 / 27)) + np.cbrt(-q / 2 - np.sqrt(q * q / 4 + p ** 3 / 27)) - b / 3 / a
    else:
        # three real roots
        for k in range(3):
            t = 2 * np.sqrt(-p / 3) * np.cos(1 / 3 * np.arccos(3 * q / 2 / p * np.sqrt(-3 / p)) - 2 * np.pi * k / 3) - b / 3 / a
            if 0 <= t and t <= 1:
                return t
    # error
    return -1

'''
Solve the lambda using equation (14)
'''
def lamb(c0, c1, c3, c4):
    def temp(A, B, C):
        mat = np.c_[B-A, C-B]
        return np.abs(np.linalg.det(mat))
    return np.sqrt(temp(c0, c1, c3)) / (np.sqrt(temp(c0, c1, c3)) + np.sqrt(temp(c1, c3, c4)))


# paras_iter_play -> if display the iteration of the curve
def ctrls_optimization(pts, max_iter = 50, paras_iter_play = False):
    print("smoothing start!")
    pts = np.array(pts)
    n = len(pts) - 2
    # return bezier control points, a vector of triple of 2d points, {{q0,q1,q2}, {q0,q1,q2}, ...}
    ctrls = np.ones([n,3]).tolist()
    for i in range(1, n + 1):
        ctrls[i - 1][0] = (pts[i] + pts[i - 1]) / 2
        ctrls[i - 1][1] = pts[i]
        ctrls[i - 1][2] = (pts[i] + pts[i + 1]) / 2

    ctrls[0][0] = pts[0]
    ctrls[n - 1][2] = pts[n + 1]

    t, ld = [0 for i in range(n)], [0 for i in range(n - 1)]
    matA = np.mat(np.zeros([n, n]))
    matB = pts[1:-1]*1.0

    for iter_i in range(max_iter):
        for i in range(n):
            t[i] = max_t(ctrls[i][0], pts[i + 1], ctrls[i][2])
        for i in range(n - 1):
            ld[i] = lamb(ctrls[i][0], ctrls[i][1], ctrls[i + 1][1], ctrls[i + 1][2])
    
        matA[0, 0] = 2 * (1 - t[0]) * t[0] + (1 - ld[0]) * t[0] * t[0]
        matA[0, 1] = ld[0] * t[0] * t[0]
        for i in range(1, n - 1):
            matA[i, i - 1] = (1 - ld[i - 1]) * (1 - t[i]) * (1 - t[i])
            matA[i, i] = ld[i - 1] * (1 - t[i]) * (1 - t[i]) + 2 * (1 - t[i]) * t[i] + (1 - ld[i]) * t[i] * t[i]
            matA[i, i + 1] = ld[i] * t[i] * t[i]
        matA[n - 1, n - 2] = (1 - ld[n - 2]) * (1 - t[n - 1]) * (1 - t[n - 1])
        matA[n - 1, n - 1] = ld[n - 2] * (1 - t[n - 1]) * (1 - t[n - 1]) + 2 * (1 - t[n - 1]) * t[n - 1]

        matB[0] = pts[1] - (1 - t[0]) * (1 - t[0]) * pts[0]
        matB[n - 1] = pts[n] - t[n - 1] * t[n - 1] * pts[n + 1]
        # linear solver of Ax=B
        corners = linalg.solve(matA, matB)
        for i in range(n): 
            ctrls[i][1] = corners[i]  
        for i in range(n - 1):
            ctrls[i + 1][0] = ctrls[i][2] = (1 - ld[i]) * ctrls[i][1] + ld[i] * ctrls[i + 1][1]

        if paras_iter_play: plot_curve_change(ctrls, iter_i)

    return ctrls

def plot_figure(t, input_points, control_points, curve_inters, derivatives_cp):
    control_points_list = []
    for items in control_points:
        for item in items:
            control_points_list.append(item.tolist())

    ax.scatter(np.array(input_points)[:,0], np.array(input_points)[:,1], marker='o', color = 'r', s = 50)
    ax.scatter(np.array(control_points_list)[:,0], np.array(control_points_list)[:,1], marker='o', color = 'b', s = 50)
    ax.scatter(curve_inters[:,0], curve_inters[:,1], marker='o', color = 'g', s = 3)

    tangents, normals = [], []
    t = [t] if isinstance(t, int) or isinstance(t, float) else t
    for i in range(len(control_points)):
        curvatures = []
        for t_i in t:
            point = bezier_point(t_i, control_points[i])
            dt = bezier_point(t_i, derivatives_cp[1][i])
            ddt = bezier_point(t_i, derivatives_cp[2][i])
            curvature = bezier_curvature(dt[0], dt[1], ddt[0], ddt[1])
            curvatures.append(curvature)
            radius = 1 / curvature
            # Normalize derivative
            dt /= np.linalg.norm(dt, 2)
            tangent = np.array([point, point + dt])
            normal = np.array([point, point - np.array([- dt[1], dt[0]]) * curvature])
            ax.plot(normal[:, 0], normal[:, 1], color='black')
            normals.append(point -  np.array([- dt[1], dt[0]]) * curvature)
        maxCurIndex = np.abs(curvatures).tolist().index(max(np.abs(curvatures)))
        point = bezier_point(t[maxCurIndex], control_points[i])
        dt = bezier_point(t[maxCurIndex], derivatives_cp[1][i])
        dt /= np.linalg.norm(dt, 2)
        normal = np.array([point, point - np.array([- dt[1], dt[0]]) * curvatures[maxCurIndex]])
        ax.plot(normal[:, 0], normal[:, 1], color='red')

    ax.plot(np.array(normals)[:, 0], np.array(normals)[:, 1])

#get the point by pressing
def on_mouse_press(event):
    global ind
    if event.inaxes == None:
        print("none")
        return 
    if event.button == 1:
        ax.scatter(event.xdata,event.ydata, c='r')
        point = []
        point.append(event.xdata)
        point.append(event.ydata)
        input_points.append(point)
        fig.canvas.draw()
        # print(input_points)
    elif event.button == 3:
        # print(input_points)
        xt, yt = np.array(input_points)[:,0], np.array(input_points)[:,1]
        d = np.sqrt((xt-event.xdata)**2 + (yt-event.ydata)**2)
        indseq = np.nonzero(np.equal(d, np.amin(d)))[0]
        ind = indseq[0]
        if d[ind] >=0.1:
            ind = None

def mouse_motion_callback(event):
    #on mouse movement
    if event.button != 3 or ind == None: 
        return

    input_points.pop(ind)
    input_points.insert(ind, [event.xdata, event.ydata])

    control_points = ctrls_optimization(input_points, max_iter = 50, paras_iter_play=False)
    curve_inters = bezier_curve(control_points, n_points = 500)
    derivatives_cp = bezier_derivatives_control_points(control_points, 2)
    t = np.linspace(0, 1, 30)
    plt.cla()
    plt.xlim(0, 15)
    plt.ylim(0, 15)
    plot_figure(t, input_points, control_points, curve_inters, derivatives_cp)
    fig.canvas.draw()

def on_key_press(event):
    if event.key == ' ':
        control_points = ctrls_optimization(input_points, max_iter = 50, paras_iter_play=False)
        curve_inters = bezier_curve(control_points, n_points = 500)
        derivatives_cp = bezier_derivatives_control_points(control_points, 2)
        t = np.linspace(0, 1, 30)
        plot_figure(t, input_points, control_points, curve_inters, derivatives_cp)
        fig.canvas.draw()
    elif event.key == 'escape':
        plt.cla()
        plt.xlim(0, 15)
        plt.ylim(0, 15)
        fig.canvas.draw()
        input_points.clear()

def plot_curve_change(ctrls, iter_i):
    curve_inters = bezier_curve(ctrls, n_points = 500)
    derivatives_cp = bezier_derivatives_control_points(ctrls, 2)
    t = np.linspace(0, 1, 30)
    plot_figure(t, input_points, ctrls, curve_inters, derivatives_cp)
    ax.set_title("curve generated by iter number:" + str(iter_i), fontsize = 12, pad = 10)
    # plt.axis("tight")
    plt.pause(0.5)
    ax.cla()
    plt.xlim(0, 15)
    plt.ylim(0, 15)

if __name__ == "__main__":

    ind = None                  # the index of the control point pressed by the mouse
    fig, ax = plt.subplots()
    input_points = []           # the control points inputted by the mouse
    plt.xlim(0, 15) 
    plt.ylim(0, 15)
    
    fig.canvas.mpl_connect("button_press_event", on_mouse_press)
    fig.canvas.mpl_connect('key_press_event', on_key_press)
    fig.canvas.mpl_connect('motion_notify_event', mouse_motion_callback)
    ax.set_title("curve generated", fontsize = 12, pad = 10)
    plt.show()