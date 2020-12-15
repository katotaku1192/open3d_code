import numpy as np
import open3d as o3d
import copy
import math


if __name__ == "__main__":
    # read answer data
    print("Read answer point cloud...")
    pcd_answer = o3d.io.read_point_cloud("combined.pcd")
    # color answer data
    pcd_answer.paint_uniform_color([0, 1, 0])
    # read map data
    print("Read map point cloud...")
    pcd_map = o3d.io.read_point_cloud("combined_A232.ply")

    # After alignment both clouds...
    print("Alignment point cloud...")

    # crop answer data (2m)
    print("Cropping point cloud...")
    vol = o3d.visualization.read_selection_polygon_volume("crop_pcd_2m.json")
    pcd_answer = vol.crop_point_cloud(pcd_answer)
    # crop map data (2m)
    pcd_map = vol.crop_point_cloud(pcd_map)

    # make answer data voxel grid (0.1m)
    print("Make voxel grid...")
    voxel_answer = o3d.geometry.VoxelGrid.create_from_point_cloud_within_bounds(pcd_answer, 0.1, [0.0,0.0,0.0], [10,10,10])
    # make map data voxel grid (0.1m)
    voxel_map = o3d.geometry.VoxelGrid.create_from_point_cloud_within_bounds(pcd_map, 0.1, [0.0,0.0,0.0], [10,10,10])
     
    print(voxel_map)
    #o3d.visualization.draw_geometries([voxel_map])
    print(voxel_answer)
    #o3d.visualization.draw_geometries([voxel_answer])
    #o3d.visualization.draw_geometries([voxel_answer, voxel_map])

    TP = 0
    TN = 0
    FP = 0
    FN = 0

    for i in range(-10, 10):
        for j in range(-10, 10):
            for k in range(-10, 10):
                queries = np.asarray([[i/10+0.05, j/10+0.05, k/10+0.05]])
                answer_tf = voxel_answer.check_if_included(o3d.utility.Vector3dVector(queries))
                map_tf = voxel_map.check_if_included(o3d.utility.Vector3dVector(queries))

                if answer_tf[0] and map_tf[0]:
                    TP += 1
                elif answer_tf[0] == False and map_tf[0] == False:
                    TN += 1
                elif answer_tf[0] == False and map_tf[0] == True:
                    FP += 1
                elif answer_tf[0] == True and map_tf[0] == False:
                    FN += 1

    print("TP =", TP,", TN =", TN,", FP =", FP,", FN =", FN)
    accuracy = (TP + TN) / (TP + TN + FP + FN)
    IoU = TP / (TP + FP + FN)
    print("accuracy =", accuracy)
    print("Iou =", IoU)

    #queries = np.asarray([[0.0,0.0,0.0],[5.0,1.0,1.0]])
    #output = voxel.check_if_included(o3d.utility.Vector3dVector(queries))
    #output = voxel_A232.check_if_included(o3d.utility.Vector3dVector(queries))
    #print(output)

    #print(voxel)