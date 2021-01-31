# Monocular-Visual-SLAM

Minimal implementation of monocular SLAM with pose graph optimisation (loop closing yet to be implemented)

To run the visual SLAM:

```
python run_slam.py --path ../KITTI/KITTI_gray/dataset/sequences/00 \
                   --optimize \
                   --local_window 10 \
                   --num_iter 100
```
where `path` is the path to the KITTI dataset (directory structure of code and data to be updated)


## Task List
- [x] Build visual-odometry frontend with ORB descriptors and 2D-2D feature correspondences 
- [x] Build G2O and Pangolin (check [Installation.md](Installation/Installation.md)) on few tips on installation and troubleshooting guide 
- [x] Check working of front end of the slam system
- [x] Integrate pose-graph optimization backend using g2o (static)
- [x] Set up 3D plotter for visualisation of frames and point cloud
- [x] Integrate pose-graph optimization backend on-the-fly (dyanamic)
- [x] Intergrated bundled pose graph optimization (backend)