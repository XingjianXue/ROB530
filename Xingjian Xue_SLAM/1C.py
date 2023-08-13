from __future__ import print_function

import math

import gtsam
import gtsam.utils.plot as gtsam_plot
import matplotlib.pyplot as plt
import numpy as np
import g2o_reading

poses,edges = g2o_reading.read_g2o('input_INTEL_g2o.g2o')



parameters = gtsam.ISAM2Params()
isam = gtsam.ISAM2(parameters)
result = isam.calculateEstimate()


for pose in poses:
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()
    if pose[0] == 0:
        initial_estimate.insert(pose[0],gtsam.Pose2(pose[1],pose[2],pose[3]))
        prior_model = gtsam.noiseModel.Diagonal.Variances(np.array([0.5,0.5,0.1]))
        graph.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(pose[1], pose[2], pose[3]), prior_model))
    else:
        prevPose = result.atPose2(pose[0] - 1)
        initial_estimate.insert(pose[0],prevPose)
        for each_edge in edges:
            if each_edge[1] == pose[0]:
                info_vector = each_edge[5]
                info_matrix = [[info_vector[0],info_vector[1],info_vector[2]],
                                [info_vector[1],info_vector[3],info_vector[4]],
                                [info_vector[2],info_vector[4],info_vector[5]]]
                cov_matrix = np.linalg.inv(np.array(info_matrix))
                ODO_NOISE = gtsam.noiseModel.Gaussian.Covariance(cov_matrix)   
                graph.add(gtsam.BetweenFactorPose2(int(each_edge[0]), int(each_edge[1]), gtsam.Pose2(each_edge[2],each_edge[3],each_edge[4]), ODO_NOISE))
    isam.update(graph,initial_estimate)
    result = isam.calculateEstimate()

initial_poses_x = np.array(poses)[:,1]
initial_poses_y = np.array(poses)[:,2]
pose_num = len(initial_poses_x)
optimized_x = []
optimized_y = []
for i in range(pose_num):
    each_pose = result.atPose2(i).translation()
    optimized_x.append(each_pose[0])
    optimized_y.append(each_pose[1])


plt.plot(initial_poses_x,initial_poses_y,label='Unoptimized Trajectory',linewidth = 1)
plt.plot(optimized_x,optimized_y,label='Optimized Trajectory',linewidth = 1)
plt.legend()
plt.title('Optimized Trajectory and Unoptimized Trajectory for task1.C')
plt.axis('equal')
plt.show()











