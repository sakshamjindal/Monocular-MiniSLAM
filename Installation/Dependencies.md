# Installation

### Build dependencies for g20
```
sudo su
apt-get install libsuitesparse-dev 
apt-get install qtdeclarative5-dev 
apt-get install qt5-qmake 
apt-get install -y libqglviewer-dev

apt-get install libegl1-mesa-dev
apt install ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev
```

### Build dependencies for Pangolin
```
sudo su
apt-get install libglew-dev
apt install ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev
apt install graphviz
apt-get install doxygen
```

### Requiments
```
numpy
opencv
matplotlib
PyOpenGL 
PyOpenGL_accelerate
g2o [fork](https://github.com/sakshamjindal/Python-G2O)
pangolin [fork](https://github.com/sakshamjindal/Pangolin-Python)
```


### Troubleshoot

- Error encountered during installation of g2o

```
rm /usr/bin/libGL.so
sudo ln -s /usr/lib/x86_64-linux-gnu/libGL.so.1  /usr/bin/libGL.so
```

- Error encountered during installation of Panglolin

```
rm /usr/lib/x86_64-linux-gnu/libGL.so
ln -s /usr/lib/x86_64-linux-gnu/mesa/libGL.so.1 /usr/lib/x86_64-linux-gnu/libGL.soD
```

- Error while importing g2o
Solution : https://stackoverflow.com/questions/52747902/python-import-successful-in-one-terminal-but-fails-in-another