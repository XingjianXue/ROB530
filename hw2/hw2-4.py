import numpy as np
import matplotlib.pyplot as plt
#a
# parameter setting
N = 10000
mu_sensor = [10, 0]
sigma_sensor = [0.5, 0.25]

# generate point clouds
r, theta = np.zeros(N), np.zeros(N)
x, y = np.zeros(N), np.zeros(N)
#############################################################################
#                    TODO: Implement your code here                         #
#############################################################################
# i) Sensor (r, theta) frame 
for i in range(N):
    r[i] = np.random.normal(mu_sensor[0],sigma_sensor[0])
    theta[i] = np.random.normal(mu_sensor[1],sigma_sensor[1])
    x[i] = r[i]*np.cos(theta[i])
    y[i] = r[i]*np.sin(theta[i])


# ii) Cartesian (x,y) coordinate frame


#############################################################################
#                            END OF YOUR CODE                               #
#############################################################################
# i) Observation in the sensor frame
plt.plot(r, theta, '.', markersize=2)
plt.axis('equal')
plt.grid(True)
plt.title('Sensor Frame Point Cloud')
plt.xlabel('Range (m)')
plt.ylabel('Bearing (rad)')
plt.show()

# ii) Observation in the Cartesian frame
plt.plot(x, y, '.', markersize=2)
plt.axis('equal')
plt.grid(True)
plt.title('Cartesian Frame Point Cloud')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.show()

#b
cov_sensor = np.matrix([[pow(sigma_sensor[0],2), 0],[0, pow(sigma_sensor[1],2)]])
Jacobian = np.zeros((2,2))
cov_cartesian = np.zeros((2,2))
#############################################################################
#                    TODO: Implement your code here                         #
#############################################################################
# Implement the Jacobians
Jacobian = np.matrix([[1, 0],[0, 10]])
cov_cartesian = Jacobian@cov_sensor@(np.transpose(Jacobian))


# Implement the linearized covariance in cartesian corridinates


#############################################################################
#                            END OF YOUR CODE                               #
#############################################################################
print('Cov_sensor:\n',cov_sensor)
print('Jacobian:\n', Jacobian)
print('\nSigma_cartesian:\n', cov_cartesian)

#c
def calculateEllipseXY(mu, Sigma, k2, N=20):
    """
    input:
    mu     is the [2x1] [x;y] mean vector
    Sigma  is the [2x2] covariance matrix
    k2     is the Chi-squared 2 DOF variable
    N      is the number of points to use (optional, default 20)
    
    output:
    x and y coordinates to draw the ellipse
    """
    # set up angles for drawing the ellipse
    angles = np.linspace(0, 2*np.pi, num=N)
    _circle = np.array([np.cos(angles), np.sin(angles)])

    # make sure it is a numpy array
    mu = np.array(mu)
    Sigma = np.array(Sigma)
        
    # cholesky decomposition
    L = np.linalg.cholesky(Sigma) # Cholesky factor of covariance
    
    # apply the transformation and scale of the covariance
    ellipse = np.sqrt(k2) * L @ _circle

    # shift origin to the mean
    x = mu[0] + ellipse[0, :].T
    y = mu[1] + ellipse[1, :].T

    return x, y

def draw_ellipse(mu, Sigma, k2, colorin='red'):
    """   
    input:
    mu       is the [2x1] [x;y] mean vector
    Sigma    is the [2x2] covariance matrix
    k2       is the Chi-squared 2 DOF variable
    Npoints  number of points to draw the ellipse (default 20)
    colorin  color for plotting ellipses, red for analytical contours, blue for sample contours
    
    --- h = draw_ellipse(mu, Sigma, k2)
    Draws an ellipse centered at mu with covariance Sigma and confidence region k2, i.e.,
    K2 = 1; # 1-sigma
    K2 = 4; # 2-sigma
    K2 = 9; # 3-sigma
    K2 = chi2inv(.50, 2); # 50% probability contour
    """
    Npoints = 20
    
    x, y = calculateEllipseXY(mu, Sigma, k2, Npoints)
    
    if k2 == 9:
        if colorin == 'red':
            plt.plot(x, y, linewidth=1.25, color=colorin, label='analytical contours')
        elif colorin == 'blue':
            plt.plot(x, y, linewidth=1.25, color=colorin, label='sample contours')
    else:
        plt.plot(x, y, linewidth=1.25, color=colorin)

# Sensor frame
plt.plot(r, theta, '.', markersize=2, label='data')
plt.axis('equal')
plt.grid(True)
plt.title('Sensor Frame Point Cloud')
plt.xlabel('Range (m)')
plt.ylabel('Bearing (rad)')
plt.legend()

#############################################################################
#                    TODO: Implement your code here                         #
#############################################################################
mu_sensor = np.matrix([[10],[0]])
#cov_sensor = np.matrix([[0.25, 0],[0, 0.0625]])

mu_r = sum(r)/np.size(r)
mu_theta = sum(theta)/np.size(theta)
mu_sensor_sample = np.matrix([[mu_r],[mu_theta]])

cov_r = 0
for i in r:
    cov_r += (i - mu_r)**2/(np.size(r)-1)
    
cov_theta = 0
for j in theta:
    cov_theta += (j - mu_theta)**2/(np.size(theta)-1)
cov_sensor_sample = np.matrix([[cov_r, 0],[0, cov_theta]])


#############################################################################
#                            END OF YOUR CODE                               #
#############################################################################

# plot the ellipses
for i in range(3):
    # analytical contous in red
    draw_ellipse(mu_sensor, cov_sensor, (i+1)**2, 'red')
    # sample contous in blue
    draw_ellipse(mu_sensor_sample, cov_sensor_sample, (i+1)**2, 'blue')

plt.legend()
plt.show()

# Cartesian frame
plt.plot(x, y, '.', markersize=2, label='data')
plt.axis('equal')
plt.grid(True)
plt.title('Cartesian Frame Point Cloud')
plt.xlabel('x (m)')
plt.ylabel('y (m)')

#############################################################################
#                    TODO: Implement your code here                         #
#############################################################################
mu_cartesian = np.matrix([[10],[0]])

mu_x = sum(x)/np.size(x)
mu_y = sum(y)/np.size(y)
mu_cartesian_sample = np.matrix([[mu_x],[mu_y]])


cov_x = 0
for i in x:
    cov_x += (i - mu_x)**2/(np.size(x)-1)
cov_y = 0
for j in y:
    cov_y += (j - mu_y)**2/(np.size(y)-1)
cov_cartesian_sample = np.matrix([[cov_x, 0],[0, cov_y]])
#############################################################################
#                            END OF YOUR CODE                               #
#############################################################################

# plot the ellipses
for i in range(3):
    # analytical contous in red
    draw_ellipse(mu_cartesian, cov_cartesian, (i+1)**2, 'red')    
    # sample contous in blue
    draw_ellipse(mu_cartesian_sample, cov_cartesian_sample, (i+1)**2, 'blue')

plt.legend()
plt.show()

#d
# counter for samples lie within the contour
count_sensor = np.zeros((3,1)) # count results of samples in the sensor frame
count_cartesian = np.zeros((3,1)) # count results of samples in the Cartesian frame

# Compute the Mahalabobis distance of samples, and count how many samples lie in the contour
#############################################################################
#                    TODO: Implement your code here                         #
#############################################################################
mu_sensor = np.matrix([[10],[0]])
#cov_sensor = np.matrix([[0.25, 0],[0, 0.0625]])
r1,r2,r3 = 0,0,0
for i in range(N):
    n = np.matrix([[r[i]],[theta[i]]])
    error_sensor = np.sqrt((n-mu_sensor).T*np.linalg.inv(cov_sensor)*(n-mu_sensor))
    if error_sensor < 1:
        r1 += 1
    if error_sensor < 2:
        r2 += 1
    if error_sensor < 3:
        r3 += 1

count_sensor[0] = r1
count_sensor[1] = r2
count_sensor[2] = r3

mu_cartesian = np.matrix([[10],[0]])
#cov_cartesian = np.matrix([[0.25, 0],[0, 6.25]])
k1,k2,k3 = 0,0,0

for i in range(N):
    m = np.matrix([[x[i]],[y[i]]])
    error_sensor = np.sqrt((m-mu_cartesian).T*np.linalg.inv(cov_cartesian)*(m-mu_cartesian))
    if error_sensor < 1:
        k1 += 1
    if error_sensor < 2:
        k2 += 1
    if error_sensor < 3:
        k3 += 1

count_cartesian[0] = k1
count_cartesian[1] = k2
count_cartesian[2] = k3


#############################################################################
#                            END OF YOUR CODE                               #
#############################################################################

# print out the result
Gaussian = [0.39, 0.86, 0.99]
print("         Sensor frame  Cartesian frame  Gaussian\n")
for k in range(3):
    print('%d-sigma%10.2f%16.2f%13.2f\n' % (k+1, count_sensor[k]/N, count_cartesian[k]/N, Gaussian[k]))



#f
for rho in [0.1, 0.5, 0.9]: # correlation coefficient, eg. 0.1, 0.5, 0.9 will lead to negative definite covariance matrix
    print('rho =', rho)
    r_theta = rho*(0.5*0.25)
    # Part A. generate point clouds
    r, theta = np.zeros(N), np.zeros(N)
    x, y = np.zeros(N), np.zeros(N)
    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################
    mu_sensor = [10,0]
    cov_sensor = [[0.25, r_theta],[r_theta, 0.0625]]
    data_sensor = np.random.multivariate_normal(mu_sensor,cov_sensor,N)
    for i in range(N):
        r[i] = data_sensor[i][0]
        theta[i] = data_sensor[i][1]
        x[i] = r[i]*np.cos(theta[i])
        y[i] = r[i]*np.sin(theta[i])

    
    Jacobian = np.zeros((2,2))
    cov_cartesian = np.zeros((2,2))
    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################
    # Implement the Jacobians
    Jacobian = np.matrix([[1, 0],[0, 10]])
    cov_cartesian = Jacobian@np.matrix([[0.25, r_theta],[r_theta, 0.0625]])@(np.transpose(Jacobian))

    # Part C. Draw ellipse
    # Sensor frame
    plt.plot(r, theta, '.', markersize=2, label='data')
    plt.axis('equal')
    plt.grid(True)
    plt.title('Sensor Frame Point Cloud')
    plt.xlabel('Range (m)')
    plt.ylabel('Bearing (rad)')
    plt.legend()
    mu_sensor = np.matrix([[10],[0]])
    cov_sensor = np.matrix([[0.25, r_theta],[r_theta, 0.0625]])

    mu_r = sum(r)/np.size(r)
    mu_theta = sum(theta)/np.size(theta)
    mu_sensor_sample = np.matrix([[mu_r],[mu_theta]])
    
    cov_sensor_sample = np.cov([r,theta], bias=False)
    print(cov_sensor_sample)
    # plot the ellipses
    for i in range(3):
        # analytical contous in red
        draw_ellipse(mu_sensor, cov_sensor, (i+1)**2, 'red')
        # sample contous in blue
        draw_ellipse(mu_sensor_sample, cov_sensor_sample, (i+1)**2, 'blue')

    plt.legend()
    plt.show()

    # Cartesian frame
    plt.plot(x, y, '.', markersize=2)
    plt.axis('equal')
    plt.grid(True)
    plt.title('Cartesian Frame Point Cloud')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    
    mu_cartesian = np.matrix([[10],[0]])

    mu_x = sum(x)/np.size(x)
    mu_y = sum(y)/np.size(y)
    mu_cartesian_sample = np.matrix([[mu_x],[mu_y]])
    
    cov_cartesian_sample = np.cov([x,y], bias=False)
    print(cov_cartesian_sample)

    # plot the ellipses
    for i in range(3):
        # analytical contous in red
        draw_ellipse(mu_cartesian, cov_cartesian, (i+1)**2, 'red')    
        # sample contous in blue
        draw_ellipse(mu_cartesian_sample, cov_cartesian_sample, (i+1)**2, 'blue')

    plt.legend()
    plt.show()