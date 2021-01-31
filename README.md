# Monocular-Visual-SLAM
To run the visual Odometry:
```
python run_VO.py --path ../KITTI/KITTI_gray/dataset/sequences/00
```
where `path` is the path to the KITTI dataset (directory structure of code and data to be updated)


## Task List
- [x] Build visual-odometry frontend with ORB descriptors and 2D-2D feature correspondences 
- [x] Build G2O and Pangolin (check [Installation.md](Installation/Installation.md)) on few tips on installation and troubleshooting guide 
- [x] Check working of front end of the slam system
- [x] Integrate pose-graph optimization backend using g2o (static)
- [x] Set up 3D plotter for visualisation of frames and point cloud
- [ ] Integrate pose-graph optimization backend on-the-fly (dyanamic)