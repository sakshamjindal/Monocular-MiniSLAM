# Installations

The entire repository was built as a docker image and the pipeline was run on a docker container. I had to face a lot of issues with installations of pangolin and g2o. Further, there were import issues with import of the python bindings. Also, there were issues with forwarding of GUI components (example a pangolin display or matlotlib plot) from inside the container to the host machine. This markdown serves as a guide on how to build the entire repository using docker and run the pipeline as if you were running it on your local without any heachaches involved in building the dependencies
 
Although, I have supplied a Dockerfile for building end-to-end the application. Here, I am noting few steps that I undertook to build it step-by-step for future reference.

 **Step 1. Build the g2o**
 ```
git clone https://github.com/ElliotHYLee/g2opy
cd g2opy
docker build -t env_g2o:latest .
 ```
 You will be able to see a folder `g2opy` inside `/home/` directory

***Step 2. Build Pangolin on top of g2o docker image**
```
git clone https://github.com/sakshamjindal/Pangolin-Python
cd Pangolin-Python
docker build -t env_g2o_pangolin:latest .
```
 You will be able to see a folder `pangolin` inside `/home/` directory

**Step 3. Mount the local directory and forward GUI components from inside the container**
```
docker run -dit -P --name slam_box --net=host -e DISPLAY -v /tmp/.X11-unix -v ~/mono-slam:/home/slam env_g2o_pangolin:latest
docker exec -it slam_box bash
```
You will be able to see your working directory `~/mono-slam` inside `/home/` directory

Some links that were helpful in step 3 were a [youtube video](https://www.youtube.com/watch?v=RDg6TRwiPtg&t=37s&ab_channel=PiotrekChmielowski)