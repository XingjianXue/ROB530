"""
GTSAM Copyright 2010-2018, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)
See LICENSE for the license information
Simple robotics example using odometry measurements and bearing-range (laser) measurements
Author: Alex Cunningham (C++), Kevin Deng & Frank Dellaert (Python)
"""
# pylint: disable=invalid-name, E1101

from __future__ import print_function

import math

import gtsam
import gtsam.utils.plot as gtsam_plot
import matplotlib.pyplot as plt
import numpy as np
import g2o_reading

#problem 1.B
poses,edges = g2o_reading.read_g2o('input_INTEL_g2o.g2o')

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(gtsam.Point3(0.3, 0.3, 0.1))
#create graph
graph = gtsam.NonlinearFactorGraph()
#prior factor
#add factors
initial_estimate = gtsam.Values()
initial = gtsam.Values()

# initial.insert(1,gtsam.Pose2(1,2,3))
# initial.insert(2,gtsam.Pose2(1,2,3))
# initial.insert(3,gtsam.Pose2(1,2,3))
# initial.insert(4,gtsam.Pose2(1,2,3))
# print(initial)

#print(len(poses)) 1228
for each_pose in poses:
    if each_pose[0] == 0:
         graph.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(each_pose[1], each_pose[2], each_pose[3]), PRIOR_NOISE))

    initial_estimate.insert(each_pose[0],gtsam.Pose2(each_pose[1],each_pose[2],each_pose[3]))





for each_edge in edges:

    info_vector = each_edge[5]
    info_matrix = [[info_vector[0],info_vector[1],info_vector[2]],
    [info_vector[1],info_vector[3],info_vector[4]],
    [info_vector[2],info_vector[4],info_vector[5]]]
    cov_matrix = np.linalg.inv(np.array(info_matrix))

    ODO_NOISE = gtsam.noiseModel.Gaussian.Covariance(cov_matrix)
    graph.add(gtsam.BetweenFactorPose2(int(each_edge[0]), int(each_edge[1]), gtsam.Pose2(each_edge[2],each_edge[3],each_edge[4]), ODO_NOISE))


parameters = gtsam.GaussNewtonParams()
parameters.setRelativeErrorTol(1e-5)
parameters.setMaxIterations(100)

optimizer = gtsam.GaussNewtonOptimizer(graph, initial_estimate, parameters)
result = optimizer.optimize()



initial_poses_x = np.array(poses)[:,1]
initial_poses_y = np.array(poses)[:,2]
pose_num = len(initial_poses_x)
#print(pose_num) 1228


optimized_x = []
optimized_y = []
for i in range(pose_num):
    each_pose = result.atPose2(i).translation()
    optimized_x.append(each_pose[0])
    optimized_y.append(each_pose[1])


plt.plot(initial_poses_x,initial_poses_y,label='Unoptimized Trajectory',linewidth = 1)
plt.plot(optimized_x,optimized_y,label='Optimized Trajectory',linewidth = 1)
plt.legend()
plt.title('Optimized Trajectory and Unoptimized Trajectory for task1.B')
plt.axis('equal')
plt.show()



