# sb_sim2real_ep
code for ICRA2022 sim2real challenge
sunshine boy wodefa

### installation
install scipy
```
python3 -m pip install -U scipy
```
clone this repository into the client docker
```
# in the docker client
cd ~/ep_ws/src
git clone https://github.com/yanseim/sb_sim2real_ep.git
```
compile the ROS workspace
```
cd ~/ep_ws
catkin_make
source devel/setup.bash
```
our packages are in folder /sb_sim2real_ep

### Usage
#### the whole procedure
start the server
```
# in docker server
cd ~/ros_x_habitat_ws/src/ros_x_habitat/
python3 src/scripts/roam_with_joy.py --hab-env-config-path ./configs/roam_configs/pointnav_rgbd_roam_mp3d_test_scenes.yaml

```
launch the file
```
roslaunch sb_brain procedure.launch
```