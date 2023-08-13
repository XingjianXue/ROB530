from __future__ import print_function

import math

import gtsam
import gtsam.utils.plot as gtsam_plot
import matplotlib.pyplot as plt
import numpy as np
import g2o_reading

poses,edges = g2o_reading.read_g2o_3D('parking-garage.g2o')



parameters = gtsam.ISAM2Params()
isam = gtsam.ISAM2()
result = isam.calculateEstimate()


for pose in poses:
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()
    if pose[0] == 0:
        r = gtsam.Rot3.Quaternion(pose[7],pose[4],pose[5],pose[6])
        t = gtsam.Point3(pose[1], pose[2], pose[3])

        initial_estimate.insert(pose[0],gtsam.Pose3(r,t))
        prior_model = gtsam.noiseModel.Diagonal.Variances(np.array([0.5,0.5,0.1,0.1,0.1,0.1]))
        graph.add(gtsam.PriorFactorPose3(0, gtsam.Pose3(r,t),prior_model))
    else:
        prevPose = result.atPose3(pose[0] - 1)
        initial_estimate.insert(pose[0],prevPose)
        for each_edge in edges:
            if each_edge[1] == pose[0]:
                info_vector = each_edge[9]
                info_matrix = np.array([[info_vector[0],info_vector[1],info_vector[2],info_vector[3],info_vector[4],info_vector[5]],
                   [info_vector[1],info_vector[6],info_vector[7],info_vector[8],info_vector[9],info_vector[10]],
                   [info_vector[2],info_vector[7],info_vector[11],info_vector[12],info_vector[13],info_vector[14]],
                   [info_vector[3],info_vector[8],info_vector[12],info_vector[15],info_vector[16],info_vector[17]],
                   [info_vector[4],info_vector[9],info_vector[13],info_vector[16],info_vector[18],info_vector[19]],
                   [info_vector[5],info_vector[10],info_vector[14],info_vector[17],info_vector[19],info_vector[20]]])
                cov_matrix = np.linalg.inv(info_matrix)
                ODO_NOISE = gtsam.noiseModel.Gaussian.Covariance(cov_matrix) 

                r = gtsam.Rot3.Quaternion(each_edge[8],each_edge[5],each_edge[6],each_edge[7])
                t = gtsam.Point3(each_edge[2], each_edge[3], each_edge[4])
                graph.add(gtsam.BetweenFactorPose3(int(each_edge[0]), int(each_edge[1]), gtsam.Pose3(r,t), ODO_NOISE))
    isam.update(graph,initial_estimate)
    result = isam.calculateEstimate()

initial_poses_x = np.array(poses)[:,1]
initial_poses_y = np.array(poses)[:,2]
initial_poses_z = np.array(poses)[:,3]
pose_num = len(initial_poses_x)
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
plt.title('Optimized Trajectory and Unoptimized Trajectory for task2.C')
#plt.axis('equal')
plt.gca().set_box_aspect((1,1,0.25))
plt.show()











