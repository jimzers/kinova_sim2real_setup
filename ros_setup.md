# ROS setup



First make your workspace and python environment. (note: we use `export ROS_PYTHON_VERSION=3` a lot, this tells ROS that we wanna use python3. Even when we're using a python3 virtual environment)

(Note: make sure you have python3, python3-pip, python3-venv, python3-catkin-pkg installed beforehand with apt)


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
pip install numpy torch scipy pandas opencv-python opencv-contrib-python
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
vim src/kinova-ros/kinova_moveit/robot_configs/j2s7s300_moveit_config/launch/j2s7s300_virtual_robot_demo.launchTest_11_9


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





and if you have your calibration setup right, you should start publishing to  the ROS topic `/usb_cam/image_rawTest_11_9`

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







# Doing camera calibration

Step 1: Take some pictures

Make a new folder for pictures:

```
mkdir ~/sim-to-real-kinova/calibration_photos
```

Then, take some photos and place them in that folder.

Step 2: Setup your camera launch files

Go to `cam_launch.launch` (in `src/sim-to-real-kinova-master`) and make sure your `video_device` param.

```
ls /dev/video*
# if you can't figure out which video channel to use, try them all lol
```

Changing example:

```
<param name="video_device" value="/dev/video0" />
```

Step 3: Actual calibration. Should pop out a `camera_mtx.npy` and `dist_mtx.npy` at the end of it all.

```
cd ~/
export ROS_PYTHON_VERSION=3
conda deactivate
. ~/kinova_venv/bin/activate
cd ~/sim-to-real-kinova
source devel/setup.bash

python ~/sim-to-real-kinova/src/sim-to-real-kinova-master/object_detection_pkg/src/camera_cal.py
```

Step 4: You're done. Make sure any code that utilizes OpenCV / AruCo stuff is pointed to the correct `camera_mtx.npy` and `dist_mtx.npy`.



TODO: write more documentation (when brain is fried)

If you want to run `grasp_classifier_test.py`:

Step 1: Teleoperate and get sample joints.



Places where you'll need to change the joint angles (in `kinova_gripper_env.py`:

`self.lift_pose`: The lifting pose?

`self.goal_pose`: The goal pose?

`self.home_angle`: The pose to start out

`self.pre_grasp_angle`: Useless... The pose right before attempting a grasp.

And you'll need to supply at least one array of joint angles in `grasp_classifier_test.py` in the `robot_grasp_joints` variable.



Step 5: Running the `grasp_classifier_test.py` code:



Terminal A:

```
cd ~/
conda deactivate
. kinova_venv/bin/activate

cd ~/sim-to-real-kinova
source devel/setup.bash
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2s7s300
```

Terminal B:

```
cd ~/
conda deactivate
. kinova_venv/bin/activate

cd ~/sim-to-real-kinova
source devel/setup.bash
roslaunch j2s7s300_moveit_config j2s7s300_demo.launch
```

Terminal C:

```
cd ~/
conda deactivate
. kinova_venv/bin/activate

cd ~/sim-to-real-kinova
source devel/setup.bash
roslaunch grasp_classifier_individual_sim_2_real kinova_grasp_classifier_test_sim2real.launch
```





TODO: finish this shit





TODO:

1. Camera calibration
2. Get `~/sim-to-real-kinova/src/sim-to-real-kinova-master/grasp_classifier_individual_sim_2_real/src/grasp_classfier_test.py` to work as sanity check. It tests the metrics of aruco markers iirc.
   1. Need to teleoperate the arm and hard code 4 values (confirm values with nigel) in `kinova_gripper_env.py`:
      1. starting grasp position for object
      2. lifting position for object
      3. move to goal position (moves right)
      4. move back to original place
   2. add a few sample "pre-starting" positions in `grasp_classifier_test.py`
3. Figure out Haonan's stuff more and more.



Unrelated step: Teleoperate to make the joint angles.

Joint position can be observed by echoing two topics: `/'${kinova_robotType}_driver'/out/joint_angles` (in degree) and `/'${kinova_robotType}_driver'/out/state/position` (in radians including finger information)

**eg**: `rostopic echo -c /m1n4s200_driver/out/joint_state` will print out joint names (rad), position, velocity (rad/s) and effort (Nm) information.



Pain train begins...

Terminal A:

```
cd ~/
conda deactivate
. kinova_venv/bin/activate

cd ~/sim-to-real-kinova
source devel/setup.bash
roslaunch j2s7s300_moveit_config j2s7s300_demo.launch
```

```
cd ~/
conda deactivate
. kinova_venv/bin/activate

cd ~/sim-to-real-kinova
source devel/setup.bash
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2s7s300
```

Make a directory for the transform matrix dictionary first:

```
mkdir ~/sim-to-real-kinova/src/kinova-ros/kinova_scrpits/src/data/
```








```
cd ~/
conda deactivate
. kinova_venv/bin/activate

cd ~/sim-to-real-kinova
source devel/setup.bash
rosrun kinova_scripts transfromMatrix.py 0
python2 ~/sim-to-real-kinova/src/kinova-ros/kinova_scrpits/src/transfromMatrix.py

python3 ~/sim-to-real-kinova/src/kinova-ros/kinova_scrpits/src/arm_calibration.py
```

0 - intial location

1 - front left

2 - front right

3 - back right

4 - back left

5 - center







### Running adam's openai gym test



some other terminal:

```
roscore
```



Terminal A:

```
cd ~/
conda deactivate
. kinova_venv/bin/activate

cd ~/sim-to-real-kinova
source devel/setup.bash
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2s7s300
```

Terminal B:

```
cd ~/
conda deactivate
. kinova_venv/bin/activate

cd ~/sim-to-real-kinova
source devel/setup.bash
roslaunch j2s7s300_moveit_config j2s7s300_demo.launch
```

Terminal C:

```
cd ~/
conda deactivate
. kinova_venv/bin/activate

cd ~/sim-to-real-kinova
source devel/setup.bash
roslaunch openai_gym_kinova test_adams_gym.launch
```

