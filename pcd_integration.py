import numpy as np
import open3d as o3d
import copy
import math


if __name__ == "__main__":

    # read A438 point cloud
    pcd_combined = o3d.geometry.PointCloud()
    
    for i in range(18):
        pcd = o3d.geometry.PointCloud()

        if i != 13:
            for j in range(4):
                pcd += o3d.io.read_point_cloud("{}_{}.pcd".format(i*30, j+1))

            pcd.transform(np.asarray([[1,  0,  0,  0    ],
                                      [0,  1,  0,  i*0.3],
                                      [0,  0,  1,  0    ], 
                                      [0,  0,  0,  1    ]]))

            pcd_combined += pcd


    pcd_combined.transform(np.asarray([[math.cos(math.pi/6),  0,  math.sin(math.pi/6),  0],
                                       [0,                    1,  0,                    0],
                                       [-math.sin(math.pi/6), 0,  math.cos(math.pi/6),  0], 
                                       [0,                    0,  0,                    1]]))

    o3d.visualization.draw_geometries([pcd_combined])

    # read A232 pointcloud
    pcd_A232 = o3d.io.read_point_cloud("combined_A232.ply")

    # crop A438 point cloud
    vol = o3d.visualization.read_selection_polygon_volume("crop_pcd.json")
    pcd_combined = vol.crop_point_cloud(pcd_combined)

    # crop A232 point cloud
    pcd_A232 = vol.crop_point_cloud(pcd_A232)

    # down sample A438 point cloud
    pcd_combined.voxel_down_sample(voxel_size=0.01)

    o3d.visualization.draw_geometries([pcd_combined])
    o3d.visualization.draw_geometries([pcd_A232])

    o3d.io.write_point_cloud("answer.pcd", pcd_combined)
    o3d.io.write_point_cloud("cropped_A232.pcd", pcd_A232)
