import matplotlib.pyplot as plt
import numpy as np
import pykitti

# Change this to the directory where you store KITTI data
Odo_Dir = 'KITTI_SAMPLE/ODOMETRY'
RAW_Dir = 'KITTI_SAMPLE/RAW'

# Specify the dataset to load
date = '2011_09_26'
drive = '0009'



# Load the data
data = pykitti.raw(RAW_Dir, date, drive, frames=range(0, 50, 1))

#get data (position matrices)
pos_matrix = data.oxts


# store X, Y, Z to plot

Xs = []
Ys = []
Zs = []


# get all x,y,z values in each frame
for i in range(50):
    Xs.append(pos_matrix[i][1][0][3])
    Ys.append(pos_matrix[i][1][1][3])
    Zs.append(pos_matrix[i][1][2][3])


#plot results
fig, axis = plt.subplots(1, 3, figsize = (18, 5))
axis[0].plot(Xs, Ys)
axis[0].set(xlabel = 'X values ', ylabel = 'Y values')
axis[0].axis('equal')

axis[1].plot(Ys, Zs)
axis[1].set(xlabel = 'Y values ', ylabel = 'Z values')
axis[1].axis('equal')

axis[2].plot(Xs, Zs)
axis[2].set(xlabel = 'X values ', ylabel = 'Z values')
axis[2].axis('equal')

plt .show()


# get lidar and position matrix in every frame
lidar_poses = []
pos_matrices = []

for i in range(50):

    # get lidar and matrices in each frame
    lidar_poses.append(np.array(data.get_velo(i)))
    pos_matrices.append(pos_matrix[i][1])



# extract transformation matrix from imu to Lidar
calibre = np.linalg.inv(data.calib.T_velo_imu)


for i in range(len(lidar_poses)):


    # remove depth inferior to 5 (by the first element 'x values')
    lidar_poses[i] =  lidar_poses[i][(lidar_poses[i][:, 0] >= 5)]
    lidar_poses[i][:, 3] = 1



# extract projection matrix from Lidar to camera for coloring and filtring points
calibre_camera = data.calib.T_cam2_velo

# extract K matrix of camera 2
Kw = data.calib.K_cam2

# variable to store all points of the scene with their colors
all_inside_points = []


for i in range(0,50):

    # get new points in velo coordinates
    new_points = np.array((pos_matrices[i] @ (calibre @ lidar_poses[i].T)))

    # set forth element to 1 for homogeneity purposes
    new_points[:, 3] = 1


    # calculate 2D points, K * [R|t] * X (Projected points in 2D)
    X_2D = np.array((Kw @ (calibre_camera[0:3, :] @ lidar_poses[i].T)))

    # transpose the array
    new_points = new_points.T

    # normalizing by the third value
    X_2D = X_2D / X_2D[2, :]
    X_2D = X_2D[0:2].T

    #get image info
    Image = np.array(data.get_cam2(i))

    #create mask that filters the points by the dimension of the image
    mask = ((X_2D[:, 0] > 0) & (X_2D[:, 0] < Image.shape[1]) & (X_2D[:, 1] > 0) & (X_2D[:, 1] < Image.shape[0]))

    # to store inside points (same mask to 2D & 3D points)
    new_points_inside = new_points[mask]
    X_2D_inside = X_2D[mask]


    # get pixel positions as type int
    pixel_position = X_2D_inside.astype(int)

    # store rgb color of each point in the image
    color_rgb = []

    for i in range(len(pixel_position)):
        color_rgb.append(Image[pixel_position[i, 1], pixel_position[i, 0]])

    color_rgb = np.vstack(color_rgb)

    #remove last element of the new points to be x,y,z
    new_points_inside = new_points_inside[:, :-1]

    # store position array and color array as one array of 6 elements
    new_points_position_color = np.concatenate((new_points_inside, color_rgb), axis = 1)

    #store the new points
    all_inside_points.append(new_points_position_color)


all_inside_points = np.vstack(all_inside_points)


import plyfile as ply

# transform to list of tuple
pos_3D_tuple = list(map(tuple, all_inside_points))


vertex = np.array(pos_3D_tuple, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')])
el = ply.PlyElement.describe(vertex, 'vertex')
ply.PlyData([el]).write('binary.ply')
