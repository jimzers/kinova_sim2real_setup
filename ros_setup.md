# ROS setup



First make your workspace and python environment. (note: we use `export ROS_PYTHON_VERSION=3` a lot, this tells ROS that we wanna use python3. Even when we're using a python3 virtual environment)

```
# if you have a conda environment up already: we're getting rid of it
conda deactivate

cd ~/

# make python virtual environment
python3 -m venv kinova_venv

# activate environment
. kinova_venv/bin/activate

# install some python packages woohoo
pip install -U pip wheel
pip install numpy torch scipy pandas
pip uninstall em
pip install empy

export ROS_PYTHON_VERSION=3
```

Set up the ROS workspace and MoveIt! dependencies

```
mkdir -p ~/kinova_rl/src

sudo apt-get install ros-melodic-moveit ros-melodic-trac-ik -y
```

Then setup the Kinova arm with Aruco Meshes (Haonan's old code):

```
cd ~/kinova_rl/src
git clone https://github.com/Kinovarobotics/kinova-ros.git kinova-ros

mkdir ~/kinova_rl/misc
cd ~/kinova_rl/misc
git clone https://github.com/OSUrobotics/Kinova-simulation-in-Rviz

# move the kinova scripts
cp -r ~/kinova_rl/misc/Kinova-simulation-in-Rviz/kinova_scrpits/ ~/kinova_rl/src/kinova-ros/

# replace kinova meshes with the aruco-attached meshes
cp -r ~/kinova_rl/misc/Kinova-simulation-in-Rviz/meshes/ ~/kinova_rl/src/kinova-ros/kinova_description/

# replace kinova urdf with aruco-attached urdf
cp -r ~/kinova_rl/misc/Kinova-simulation-in-Rviz/urdf/ ~/kinova_rl/src/kinova-ros/kinova_description/

# replace launch files for our virtual robot.
cp ~/kinova_rl/misc/Kinova-simulation-in-Rviz/j2s7s300_virtual_robot_demo.launch ~/kinova_rl/src/kinova-ros/kinova_moveit/robot_configs/j2s7s300_moveit_config/launch/j2s7s300_virtual_robot_demo.launch

cd ~/kinova_rl

export ROS_PYTHON_VERSION=3

# finally... build time
catkin_make

# accessing the arm via usb
sudo cp ~/kinova_rl/src/kinova-ros/kinova_driver/udev/10-kinova-arm.rules /etc/udev/rules.d/

```

Make all the scripts runnable

```
chmod -R +x ~/kinova_rl/src/kinova-ros/kinova_scrpits/
```

You'll also need to change some deprecated code or your code will crash. Change instances of `xarco.py` to `xarco`.

```
vim ~/kinova_rl/src/kinova-ros/kinova_moveit/robot_configs/j2s7s300_moveit_config/launch/planning_context.launch
```



Sanity check running a visualizer. You'll need 3 terminals (described as terminals A, B, C):

TODO: Add what the actual files do.



Terminal A:

```
cd ~/
conda deactivate
. kinova_venv/bin/activate

cd ~/kinova_rl
source devel/setup.bash
roslaunch j2s7s300_moveit_config j2s7s300_virtual_robot_demo.launch
```

Terminal B:

Goes into `kinova-ros/kinova_scrpits/visualization



The launch file: Starts up 3 nodes

```
conda deactivate

cd ~/kinova_rl
source devel/setup.bash
roslaunch kinova_scripts rob514_visual.launch
```

Terminal C:

```
cd ~/
conda deactivate
. kinova_venv/bin/activate

cd ~/kinova_rl
source devel/setup.bash

rosrun kinova_scripts joint_angles.py 9
```



this code might be useful for identifying aruco markers and stuff.

TODO: Figure out what the purpose of this code is (ofc not just the filename...)

```
cd ~/
conda deactivate
. kinova_venv/bin/activate

cd ~/kinova_rl
source devel/setup.bash
python misc/Kinova-simulation-in-Rviz/identify\ aruco\ markers\ in\ image.py 
```

Code for editing random pieces of stuff. Can probably ignore this section.

```
vim src/kinova-ros/kinova_moveit/robot_configs/j2s7s300_moveit_config/launch/j2s7s300_virtual_robot_demo.launch


vim ~/kinova_rl/src/kinova-ros/kinova_moveit/robot_configs/j2s7s300_moveit_config/launch/planning_context.launch
^^^ change from xarco.py to xarco
```





### Sim-to-real-kinova Setup

TODO: finish talking about camera calibration + arm calibration procedure.

```
cd ~
git clone https://github.com/OSUrobotics/sim-to-real-kinova.git
cd sim-to-real-kinova
git checkout full_workspace
rm -rf build/ devel/
sudo apt-get install ros-melodic-vision-opencv ros-melodic-cv-bridge ros-melodic-usb-cam -y
```



okay magically everything works. make sure you have your virtual environment activated. now is the fun part.

```
export ROS_PYTHON_VERSION=3
. ~/kinova_venv/bin/activate
cd ~/sim-to-real-kinova
catkin_make
```

If your program successfully built, hurray! If not, it's probably related to OpenCV. One problem I ran into: Check the capitalization of the package naming - it's a bit verbose. And point your build to where you have OpenCV (typically `/usr/share/OpenCV/`) in `CMakeLists.txt`.

Anyways, try this out:

In terminal A:

```
roscore
```

In terminal B:

```
export ROS_PYTHON_VERSION=3
conda deactivate
. ~/kinova_venv/bin/activate
cd ~/sim-to-real-kinova
rosrun usb_cam usb_cam_node _camera_name:='usb_cam' _camera_frame_id:='usb_cam' 
```

In terminal C:

```
cd ~/
export ROS_PYTHON_VERSION=3
conda deactivate
. ~/kinova_venv/bin/activate
cd ~/sim-to-real-kinova
source devel/setup.bash

python ~/sim-to-real-kinova/src/sim-to-real-kinova-master/object_detection_pkg/src/object_pose_and_distance_estimation.py
```





and if you have your calibration setup right, you should start publishing to  the ROS topic `/usb_cam/image_raw`

go to `~/sim-to-real-kinova/src/sim-to-real-kinova-master/object_detection_pkg/src/camera_cal.py` and make sure line 24 is pointed to the right directory with all your images.

```
# if you want 
python ~/sim-to-real-kinova/src/sim-to-real-kinova-master/object_detection_pkg/src/camera_cal.py

# if you want to see object pose + estimation code
python ~/sim-to-real-kinova/src/sim-to-real-kinova-master/object_detection_pkg/src/object_pose_and_distance_estimation.py
```







Camera setup: Camera calibration stuff, usb cam, lalala

http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

https://github.com/NVlabs/Deep_Object_Pose/blob/master/doc/camera_tutorial.md



Extra goodies:

You (may ) need to build OpenCV from source. Try not to cry.

```
mkdir ~/ihateopencv
cd ~/ihateopencv
wget -O opencv.zip https://github.com/opencv/opencv/archive/master.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/master.zip
unzip opencv.zip
unzip opencv_contrib.zip
mkdir ~/ihateopencv/build && cd ~/ihateopencv/build

cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-master/modules ../opencv-master
cmake --build .
```

