# visual_servo
We use visp_ros and gazebo to complete an visual servo simulation
The development environment used for this project is Ubuntu 18.04 + ROS Melodic + Visp_ros + Gazebo
## 1.Install
Firstly, you need install ROS and create the workspace.   
How to install ROS on Ubuntu: http://wiki.ros.org/melodic/Installation/Ubuntu  
How to create a ROS workspace: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment  
```bash
sudo apt-get install libvisp-dev
sudo apt-get install ros-melodic-visp
sudo apt-get install ros-melodic-vision-visp
```
You can also install ViSP, Visp_ros and vision_visp from source for Ubuntu. The tutorial of installation is here:   
ViSP: https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-install-ubuntu.html  
Visp_ros: http://wiki.ros.org/visp_ros/Tutorials/Howto_install_visp_ros  
vision_visp: http://wiki.ros.org/vision_visp  

## 2. Download the project file
```bash
cd ~/catkin_ws/src
git clone git://github.com/EmptyCity1995/visual_servo.git
catkin_make
source ./devel/setup.bash
```
## 3. Create a target model
```bash
cd ~/catkin_ws/src/visual_servo/ar_tags/scripts
./generate_markers_model.py -i ../images/
```

## 4. Run gazebo and add the pionner3dx model
```bash
roslaunch visual_servo gazebo_pioneer_amcl.launch
```

## 5. Insert the target model in the gazebo simulation environment 
This is shown in the following picture
![](https://github.com/EmptyCity1995/visual_servo/blob/main/20210305100039977.png)


## 6.Compile and run the visual servo program
```bash
cd ~/catkin_ws/src/visual_servo/src/
mkdir -p build
cd build
cmake ..
make 
./tutorial-ros-pioneer-visual-servo
```

## Video demonstration
![](https://github.com/EmptyCity1995/visual_servo/blob/main/exsample_video.mp4)
