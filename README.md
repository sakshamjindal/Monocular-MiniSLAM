# Monocular-Visual-Odometry





## Troubleshoot

Error encountered during installation of g2o

```
rm /usr/bin/libGL.so
sudo ln -s /usr/lib/x86_64-linux-gnu/libGL.so.1  /usr/bin/libGL.so
```

```
rm /usr/lib/x86_64-linux-gnu/libGL.so
ln -s /usr/lib/x86_64-linux-gnu/mesa/libGL.so.1 /usr/lib/x86_64-linux-gnu/libGL.soD
```


## Task List
[x] Build visual-odometry frontend with ORB descriptors and 2D-2D feature correspondences \
[x] Build G2O and Pangolin (check Installation.md) on few tips on installation and troubleshooting guide \
[] Integrate real-time pose-graph optimization backend using g2o
[] Set up 3D plotter for visualisation of frames and point cloud




