import os
import gtsam
import numpy as np

def read_g2o(fn):
    vertices,edges = [],[]    
    with open(fn) as f:
        lines = f.readlines() 
    for line in lines:
        line = line.split() #split to string into individual element
        if line[0] == 'VERTEX_SE2':
                vertices.append([int(line[1]),float(line[2]),float(line[3]),float(line[4])])
        if line[0] == 'EDGE_SE2':
                info = [float(line[6]),float(line[7]),float(line[8]),float(line[9]),float(line[10]),float(line[11])]
                edges.append([int(line[1]),int(line[2]),float(line[3]),float(line[4]),float(line[5]),info])
    return vertices,edges

vertices,edges = read_g2o("input_INTEL_g2o.g2o")


def read_g2o_3D(fn):
    vertices,edges = [],[]    
    with open(fn) as f:
        lines = f.readlines() 
    for line in lines:
        line = line.split() #split to string into individual element
        if line[0] == 'VERTEX_SE3:QUAT':
                vertices.append([int(line[1]),  float(line[2]),float(line[3]),float(line[4]),  float(line[5]),float(line[6]),float(line[7]),float(line[8])])
        if line[0] == 'EDGE_SE3:QUAT':
                info = [float(line[10]),float(line[11]),float(line[12]),float(line[13]),float(line[14]),float(line[15]),float(line[16]),float(line[17]),float(line[18]),float(line[19]),float(line[20]),float(line[21]),float(line[22]),float(line[23]),float(line[24]),float(line[25]),float(line[26]),float(line[27]),float(line[28]),float(line[29]),float(line[30])]
                edges.append([int(line[1]),int(line[2]) ,float(line[3]),float(line[4]),float(line[5]),   float(line[6]),float(line[7]),float(line[8]),float(line[9]),info])
    return vertices,edges

vertices,edges = read_g2o("input_INTEL_g2o.g2o")










                
                
    

