from __future__ import print_function

import math

import gtsam
import gtsam.utils.plot as gtsam_plot
import matplotlib.pyplot as plt
import numpy as np
import g2o_reading

#problem 1.C
poses,edges = g2o_reading.read_g2o_3D('parking-garage.g2o')

#PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(gtsam.Point3(0.3, 0.3, 0.1))
#PRIOR_NOISE = gtsam.noiseModel.Diagonal.Variances(np.array([0.1,0.1,0.1,0.1,0.1,0.1]))
PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([[0.1],[0.1],[0.1],[0.1],[0.1],[0.1]]))
#create graph
graph = gtsam.NonlinearFactorGraph()
initial_estimate = gtsam.Values()
for each_pose in poses:
    if each_pose[0] == 0:
        
        r = gtsam.Rot3.Quaternion(each_pose[7],each_pose[4],each_pose[5],each_pose[6])
        t = gtsam.Point3(each_pose[1], each_pose[2], each_pose[3])
        graph.add(gtsam.PriorFactorPose3(0, gtsam.Pose3(r,t),PRIOR_NOISE))
        
    r = gtsam.Rot3.Quaternion(each_pose[7],each_pose[4],each_pose[5],each_pose[6])
    t = gtsam.Point3(each_pose[1], each_pose[2], each_pose[3])
    initial_estimate.insert(each_pose[0],gtsam.Pose3(r,t))
for each_edge in edges:

    info_vector = each_edge[9]
    info_matrix = np.array([[info_vector[0],info_vector[1],info_vector[2],info_vector[3],info_vector[4],info_vector[5]],
                            [info_vector[1],info_vector[6],info_vector[7],info_vector[8],info_vector[9],info_vector[10]],
                            [info_vector[2],info_vector[7],info_vector[11],info_vector[12],info_vector[13],info_vector[14]],
                            [info_vector[3],info_vector[8],info_vector[12],info_vector[15],info_vector[16],info_vector[17]],
                            [info_vector[4],info_vector[9],info_vector[13],info_vector[16],info_vector[18],info_vector[19]],
                            [info_vector[5],info_vector[10],info_vector[14],info_vector[17],info_vector[19],info_vector[20]]])

    cov_matrix = np.linalg.inv(info_matrix)
    ODO_NOISE = gtsam.noiseModel.Gaussian.Covariance(cov_matrix)
    re = gtsam.Rot3.Quaternion(each_edge[8],each_edge[5],each_edge[6],each_edge[7])
    te = gtsam.Point3(each_edge[2], each_edge[3], each_edge[4])

    


    graph.add(gtsam.BetweenFactorPose3(each_edge[0], each_edge[1], gtsam.Pose3(re,te), ODO_NOISE))   
parameters = gtsam.GaussNewtonParams()
parameters.setRelativeErrorTol(1e-5)
parameters.setMaxIterations(100)

optimizer = gtsam.GaussNewtonOptimizer(graph, initial_estimate, parameters)
result = optimizer.optimize()

#opt_poses = gtsam.utilities.extractPose3(result)

initial_poses_x = np.array(poses)[:,1]
initial_poses_y = np.array(poses)[:,2]
initial_poses_z = np.array(poses)[:,3]
pose_num = len(initial_poses_x)
#print(pose_num) 1228
optimized_x = []
optimized_y = []
optimized_z = []

for i in range(pose_num):
    each_pose = result.atPose3(i).translation()
    optimized_x.append(each_pose[0])
    optimized_y.append(each_pose[1])
    optimized_z.append(each_pose[2])

ax = plt.axes(projection = '3d')
plt.plot(initial_poses_x,initial_poses_y,initial_poses_z,label='Unoptimized Trajectory',linewidth = 1)
plt.plot(optimized_x,optimized_y,optimized_z,label='Optimized Trajectory',linewidth = 1)
plt.legend()
plt.title('Optimized Trajectory and Unoptimized Trajectory for task2.B')
plt.gca().set_box_aspect((1,1,0.25))
#plt.axis('equal')
plt.show()




