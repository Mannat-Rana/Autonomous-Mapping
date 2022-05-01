# Autonomous Mapping ![SHIELD](https://img.shields.io/badge/Project%20Status%3A-Complete-green?style=for-the-badge) ![ros](https://camo.githubusercontent.com/4c117e738ecff5825b1031d601ac04bc70cc817805ba6ce936c0c556ba8e14f0/68747470733a2f2f696d672e736869656c64732e696f2f7374617469632f76313f7374796c653d666f722d7468652d6261646765266d6573736167653d524f5326636f6c6f723d323233313445266c6f676f3d524f53266c6f676f436f6c6f723d464646464646266c6162656c3d) ![PYTHON](https://camo.githubusercontent.com/3df944c2b99f86f1361df72285183e890f11c52d36dfcd3c2844c6823c823fc1/68747470733a2f2f696d672e736869656c64732e696f2f7374617469632f76313f7374796c653d666f722d7468652d6261646765266d6573736167653d507974686f6e26636f6c6f723d333737364142266c6f676f3d507974686f6e266c6f676f436f6c6f723d464646464646266c6162656c3d) 

## About
The goal of this application is to develop an autonomous mapping solution for a 2-wheeled robot. This application is developed as the final project in the course SES 598: Autonomous Exploration Systems at Arizona State University. The solution uses RTABMap to develop a quality 3D map while avoiding obstacles.

Code was tested on Ubuntu 20.04 running ROS Noetic.

## Videos

## To Install ROS Package

```bash
cd ~/catkin_ws/src
https://github.com/Mannat-Rana/Autonomous-Mapping.git
catkin build
sudo chmod +x Autonomous-Mapping/reactive_mapping/src/reactive_mapping_node.py
cd ~/catkwin_ws
source devel/setup.bash
```

## To Run
```bash
roslaunch my_robot world.launch
rosrun reactive_mapping reactive_mapping_node.py
```
