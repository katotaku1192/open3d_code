import numpy as np
import open3d as o3d
import copy
import math


def move_origin(pcd, odom_x, odom_y, odom_z, odom_qx, odom_qy, odom_qz, odom_qw):
    # linear transform from pose.position
    pcd.transform(np.asarray([[1,    0,   0,    -odom_x  ],
                              [0,    1,   0,    -odom_y  ],
                              [0,    0,   1,    -odom_z  ],
                              [0,    0,   0,    1        ]]))

    # rotate transform from pose.orientation
    odom_qxn = -odom_qx
    odom_qyn = -odom_qy
    odom_qzn = -odom_qz

    # transform from quaternion to rotate matrix
    r11 = odom_qxn**2 - odom_qyn**2 - odom_qzn**2 + odom_qw**2
    r12 = 2 * (odom_qxn*odom_qyn - odom_qzn*odom_qw)
    r13 = 2 * (odom_qxn*odom_qzn + odom_qyn*odom_qw)

    r21 = 2 * (odom_qxn*odom_qyn + odom_qzn*odom_qw)
    r22 = -odom_qxn**2 + odom_qyn**2 - odom_qzn**2 + odom_qw**2
    r23 = 2 * (odom_qyn*odom_qzn - odom_qxn*odom_qw)

    r31 = 2 * (odom_qxn*odom_qzn - odom_qyn*odom_qw)
    r32 = 2 * (odom_qyn*odom_qzn + odom_qxn*odom_qw)
    r33 = -odom_qxn**2 - odom_qyn**2 + odom_qzn**2 + odom_qw**2

    pcd.transform(np.asarray([[r11,  r12,  r13,   0  ],
                              [r21,  r22,  r23,   0  ],
                              [r31,  r32,  r33,   0  ], 
                              [0,    0,    0,     1  ]]))

    return pcd


def transform_tan(pcd_op, mark_x_my, mark_y_my, mark_z_my, mark_x_op, mark_y_op):        
    # input "opponent pcd" and "marker coordinate"
    # m:myself, o:opponent

    # rotate coordinate system 180[deg]
    pcd_op.transform(np.asarray([[-1,   0,   0,    0  ],
                                 [0,   -1,   0,    0  ],
                                 [0,    0,   1,    0  ], 
                                 [0,    0,   0,    1  ]]))


    # calclate alpha from tan addition theorim
    tanA = mark_y_my / mark_x_my
    tanB = mark_y_op / mark_x_op
    alpha = math.atan((tanB-tanA) / (1 + tanB * tanA))

    # transform maps
    t_cos = math.cos(alpha)
    t_sin = math.sin(alpha)
    T = np.asarray([[t_cos,  t_sin,  0,  mark_x_my ],
                    [-t_sin, t_cos,  0,  mark_y_my ],
                    [0,      0,      1,  mark_z_my ], 
                    [0,      0,      0,        1   ]])

    print "transform matrix :"
    print(T)
    pcd_op.transform(T)

    return pcd_op


def transform_cos(xa, ya, za, xb, yb): # not use
    # calculate alpha from cos theorim
    r2 = xb**2 + yb**2
    print(r2)
    # r2 = 12.96 (= 13)

    xbn = -xb
    ybn = -yb

    alpha = math.acos(((xa-xbn)**2 + (ya-ybn)**2 - 2*r2) / (-2*r2))
    print(alpha)

    # transform maps
    t_cos = math.cos(alpha)
    t_sin = math.sin(alpha)
    T = np.asarray([[t_cos,  0,  t_sin,   xa ],
                    [0,      1,  0,       za ],
                    [-t_sin, 0,  t_cos,   ya ], 
                    [0,      0,  0,       1  ]])
    
    return T


def display_inlier_outlier(cloud, ind):
    inlier_cloud = o3d.geometry.select_down_sample(cloud, ind)
    outlier_cloud = o3d.geometry.select_down_sample(cloud, ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])


def draw_registration_result(source, target):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)

    # make colored point cloud
    source_temp.paint_uniform_color([1, 0.706, 0])        # source is blue
    target_temp.paint_uniform_color([0, 0.651, 0.929])    # target is yellow

    # draw colored point cloud
    o3d.visualization.draw_geometries([source_temp, target_temp])

    # make combined point cloud
    combined = o3d.geometry.PointCloud()
    combined += source
    combined += target

    return combined



if __name__ == "__main__":
    # read pointcloud
    pcd_a = o3d.io.read_point_cloud("3_1.ply")
    pcd_b = o3d.io.read_point_cloud("3_2.ply")

    o3d.visualization.draw_geometries([pcd_a])
    o3d.visualization.draw_geometries([pcd_b])

    # downsample the combined point cloud
    print("Downsample the point cloud with a voxel")
    pcd_a = o3d.geometry.voxel_down_sample(pcd_a, voxel_size=0.02)
    pcd_b = o3d.geometry.voxel_down_sample(pcd_b, voxel_size=0.02)

    # remove noise
    print("Statistical outlier removal")
    pcd_a_fined, ind_a = o3d.geometry.statistical_outlier_removal(pcd_a, nb_neighbors=80, std_ratio=1.0)
    display_inlier_outlier(pcd_a, ind_a)
    pcd_b_fined, ind_b = o3d.geometry.statistical_outlier_removal(pcd_b, nb_neighbors=80, std_ratio=1.0)
    display_inlier_outlier(pcd_b, ind_b)

    # create origin and coordinate frame
    mesh_frame = o3d.geometry.create_mesh_coordinate_frame(size=1, origin=[0, 0, 0])

    # set odom value
    # RobotA
    odom_x_a = 3.1024259381322
    odom_y_a = 0.475000873676642
    odom_z_a = -0.0286154273850
    odom_qx_a = 0.004886388043213
    odom_qy_a = 0.00679809790261
    odom_qz_a = 0.410081469925729
    odom_qw_a = 0.912010449280415

    # RobotB
    odom_x_b = 3.67714720453505
    odom_y_b = -1.14520728247524
    odom_z_b = 0.053575426872284
    odom_qx_b = 0.011402542079772
    odom_qy_b = -0.011182338587292
    odom_qz_b = -0.649493785529
    odom_qw_b = 0.760199020176191

    # tarnsform coordinate system from odom
    pcd_a_fined = move_origin(pcd_a_fined, odom_x_a, odom_y_a, odom_z_a, odom_qx_a, odom_qy_a, odom_qz_a, odom_qw_a)
    pcd_b_fined = move_origin(pcd_b_fined, odom_x_b, odom_y_b, odom_z_b, odom_qx_b, odom_qy_b, odom_qz_b, odom_qw_b)

    o3d.visualization.draw_geometries([pcd_a_fined, mesh_frame])
    o3d.visualization.draw_geometries([pcd_b_fined, mesh_frame])


    # set marker coordinate from RobotA to RobotB
    mark_x_a = 2.33524929870932
    mark_y_a = 0.822841964535865
    mark_z_a = -0.030192081642858
    # set marker coordinate from RobotB to RobotA
    mark_x_b = 2.23317809665794
    mark_y_b = -1.02299736527862

    # draw each points
    point_a = o3d.geometry.create_mesh_sphere(radius=0.1)
    point_a.paint_uniform_color([0.1, 0.1, 0.9])
    point_a.transform(np.asarray([[1,  0,  0,  mark_x_b  ],
                                  [0,  1,  0,  mark_y_b ],
                                  [0,  0,  1,  0  ], 
                                  [0,  0,  0,  1  ]]))

    #o3d.visualization.draw_geometries([pcd_b, mesh_frame, point_a])




    # transform RobotB point cloud to RobotA point cloud
    pcd_b_fined = transform_tan(pcd_b_fined, mark_x_a, mark_y_a, mark_z_a, mark_x_b, mark_y_b)

    # draw colored point cloud and make combined point cloud
    pcd_combined = draw_registration_result(pcd_a_fined, pcd_b_fined)

    # draw combined point cloud(include noise)
    o3d.visualization.draw_geometries([pcd_combined, mesh_frame])

    o3d.io.write_point_cloud("3_combined.ply", pcd_combined)