import numpy as np
import open3d as o3d
import copy
import math


if __name__ == "__main__":
    # read point cloud
    pcd_combined = o3d.io.read_point_cloud("combined.pcd")
    pcd_A232 = o3d.io.read_point_cloud("cropped_A232.pcd")
    
    

    voxel = o3d.geometry.VoxelGrid.create_from_point_cloud_within_bounds(pcd_combined, 0.1, [0.0,0.0,0.0], [10,10,10])
    voxel_A232 = o3d.geometry.VoxelGrid.create_from_point_cloud_within_bounds(pcd_A232, 0.1, [0.0,0.0,0.0], [10,10,1])
    
    

    print(voxel.get_voxel([3.0001,3.0001,3.0001]), voxel_A232.get_voxel([3.0001,3.0001,3.0001]))
    print(voxel.get_voxel([3.0,3.0,3.0]), voxel_A232.get_voxel([3.0,3.0,3.0]))
    print(voxel.get_voxel([2.9999,2.9999,2.9999]), voxel_A232.get_voxel([2.9999,2.9999,2.9999]))
    print(voxel.get_voxel([0.07,0.0,0.0]), voxel_A232.get_voxel([0.07,0.0,0.0]))
    print(voxel.get_voxel([0.08,0.0,0.0]), voxel_A232.get_voxel([0.08,0.0,0.0]))
    print(voxel_A232.get_voxel([-0.00001,0.0,0.0]), voxel.get_voxel([-0.00001,0.0,0.0]))
    #print(voxel.get_voxels())


    o3d.visualization.draw_geometries([voxel, voxel_A232])

    queries = np.asarray([[0.0,0.0,0.0],[5.0,1.0,1.0]])
    output = voxel.check_if_included(o3d.utility.Vector3dVector(queries))
    output = voxel_A232.check_if_included(o3d.utility.Vector3dVector(queries))
    print(output)

    print(voxel)
