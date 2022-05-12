import numpy as np
import pyoctomap
from pathlib import Path

radar_scan =  "/home/alex/Development/mmw2lidar/octo_data/run0/upsampled_radar_scan/"
radar_poses = "/home/alex/Development/mmw2lidar/octomap_scripts/run_0_quat.txt"

scan_globs = Path(radar_scan).glob("**/*.txt")
scan_files = [x for x in scan_globs if x.is_file()]
scan_files.sort()
poses = np.loadtxt(Path(radar_poses).resolve().as_posix(), delimiter=",").astype(np.float32)

assert len(poses) == len(scan_files)

tree = pyoctomap.OcTree(0.1)

for idx in range(len(poses)):
    this_pcd_np = np.loadtxt(scan_files[idx], delimiter=",").reshape((-1, 3)).astype(np.float32)
    pcd = pyoctomap.Pointcloud(this_pcd_np)
    rot = pyoctomap.Quaternion(poses[idx, 3], poses[idx, 4], poses[idx, 5], poses[idx, 6])
    trans = pyoctomap.Vector3(poses[idx, 0:3])
    pose = pyoctomap.Pose6D(trans, rot)

    tree.insertPointCloud(pcd, pose)

pcd = tree.getPoints().numpy()
# np.savetxt("run0.txt", pcd)

import pyvista as pv

p = pv.Plotter()
data = pv.PolyData(pcd)
original_data = pv.PolyData(this_pcd_np)
p.add_mesh(data, scalars=pcd[:, 2])
# p.add_mesh(original_data, 'r')
p.show()