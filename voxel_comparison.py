import open3d as o3d
import numpy as np



if __name__ == "__main__":

    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])

    # read answer and map
    pcd_answer = o3d.io.read_point_cloud("A232_answer_pcd.pcd")
    #o3d.visualization.draw_geometries([pcd_answer, mesh_frame])

    pcd_map = o3d.io.read_point_cloud("2_combined.ply")
    #o3d.visualization.draw_geometries([pcd_map, mesh_frame])

    # crop answer and map
    vol = o3d.visualization.read_selection_polygon_volume("crop_pcd_A232.json")
    pcd_answer = vol.crop_point_cloud(pcd_answer)
    pcd_map = vol.crop_point_cloud(pcd_map)

    o3d.visualization.draw_geometries([pcd_answer, mesh_frame])
    o3d.visualization.draw_geometries([pcd_map, pcd_answer,mesh_frame])


    # make answer data voxel grid (0.1m)
    print("Make voxel grid...")
    voxel_answer = o3d.geometry.VoxelGrid.create_from_point_cloud_within_bounds(pcd_answer, 0.1, [0.0,0.0,0.0], [10,10,10])
    # make map data voxel grid (0.1m)
    voxel_map = o3d.geometry.VoxelGrid.create_from_point_cloud_within_bounds(pcd_map, 0.1, [0.0,0.0,0.0], [10,10,10])

    print(voxel_answer)
    o3d.visualization.draw_geometries([voxel_answer])
    print(voxel_map)
    o3d.visualization.draw_geometries([voxel_map])

    # count voxels
    TP = 0
    TN = 0
    FP = 0
    FN = 0

    for i in range(0, 80):
        for j in range(-35, 50):
            for k in range(-7, 13):
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
