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
pip install pandas imageio

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

0 - intial locationprint('rotation vectors',rvecs)
print('transl')

1 - front left

2 - front right

3 - back right

4 - back left

5 - center







### Running adam's openai gym test

Echoing joint states:

```
cd ~/
conda deactivate
. kinova_venv/bin/activate

cd ~/sim-to-real-kinova
source devel/setup.bash
rostopic echo -c /j2s7s300_driver/out/joint_state
```



some other terminal:

```
roscoreprint('rotation vectors',rvecs)
print('transl')
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

```
rostopic pub -r 100 /j2s7s300_driver/in/joint_velocity  kinova_msgs/JointVelocity "{joint1: 2.0, joint2: 2.0, joint3: 2.0}"
FingerTorq
rostopic pub -r 100 /j2s7s300_driver/in/cartesian_velocity kinova_msgs/PoseVelocity "{twist_linear_x: 0.0, twist_linear_y: 0.0, twist_linear_z: 0.0, twist_angular_x: 0.0, twist_angular_y: 0.0, twist_angular_z: 0.0, finger1: 1.0, finger2: 1.0, finger3: 1.0}"
joint_angle_client

rostopic pub -r 100 /j2s7s300_driver/in/cartesian_velocity kinova_msgs/PoseVelocity "{twist_linear_x: 0.0, twist_linear_y: 0.0, twist_linear_z: 0.0, twist_angular_x: 0.0, twist_angular_y: 0.0, twist_angular_z: 0.0, finger1: 0.0, finger2: 0.0, finger3: 0.0}"


```

rosrun kinova_demo fingers_action_client.py j2s7s300 percent 100 100 100



note: i had to get the right kinova_msgs and kinova_driver folder. you have to use this commit:

https://github.com/Kinovarobotics/kinova-ros/commit/5a9cf3eb05bf957f9d9c95fb9c49c6ce8d48da04

but you can only access from here: https://github.com/f371xx/kinova-ros

I know, it makes no fucking sense. it even got merged.

```
THIS WROKS TO CLOSE THE FINGERS YAY




rostopic pub -r 100 /j2s7s300_driver/in/cartesian_velocity_with_finger_velocity kinova_msgs/PoseVelocityWithFingerVelocity "{twist_linear_x: 0.0, twist_linear_y: 0.0, twist_linear_z: 0.0, twist_angular_x: 0.0, twist_angular_y: 0.0, twist_angular_z: 0.0, finger1: 1000.0, finger2: 1000.0, finger3: 1000.0}"

```



```
cd ~/
conda deactivate
. kinova_venv/bin/activate

cd ~/sim-to-real-kinova
source devel/setup.bash
roslaunch openai_gym_kinova test_adams_cartesian
```





```
# move the kinova scripts
cp -r ~/kinova_rl/misc/Kinova-simulation-in-Rviz/kinova_scrpits/ ~/sim-to-real-kinova/src/kinova-ros/

# replace kinova meshes with the aruco-attached meshes
cp -r ~/kinova_rl/misc/Kinova-simulation-in-Rviz/meshes/ ~/sim-to-real-kinova/src/kinova-ros/kinova_description/

# replace kinova urdf with aruco-attached urdf
cp -r ~/kinova_rl/misc/Kinova-simulation-in-Rviz/urdf/ ~/sim-to-real-kinova/src/kinova-ros/kinova_description/

# replace launch files for our virtual robot.
cp ~/kinova_rl/misc/Kinova-simulation-in-Rviz/j2s7s300_virtual_robot_demo.launch ~/sim-to-real-kinova/src/kinova-ros/kinova_moveit/robot_configs/j2s7s300_moveit_config/launch/j2s7s300_virtual_robot_demo.launch
```







### Refactoring hell / Documentation

1. Steps on how to setup up your orientations (run log_cartesian_orientation.launch, writing stuff)
2. Turn arm control (go to pose) into an action







# Running the gym environment

1. Setup your camera calibrationjoint_angle_client
2. Setup your ROS terminals
3. Setup your orientations for home, pregrasp, lift reward, etc.
4. Run test code



### setting up your camera calibration

copy paste above section

### Setup your ROS terminals

Terminal A:

```
roscore
```

Terminal B:

```
cd ~/
conda deactivate
. kinova_venv/bin/activate

cd ~/sim-to-real-kinova
source devel/setup.bash
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2s7s300
```

Terminal C:

```
cd ~/
conda deactivate
. kinova_venv/bin/activate

cd ~/sim-to-real-kinova
source devel/setup.bash
roslaunch j2s7s300_moveit_config j2s7s300_demo.launch
```



### Setting up your orientations.

Terminal D:

```
cd ~/
conda deactivate
. kinova_venv/bin/activate

cd ~/sim-to-real-kinova
source devel/setup.bash
roslaunch openai_gym_kinova log_cartesian_orientation.launch
```

You should begin seeing some terminal output. This should contain positions, and orientations. Use this information to form orientations for your 4 positions:

`[pos_x, pos_y, pos_z, orientation_x, orientation_y, orientation_z, orientation_w]`

joint_angle_clientjoint_angle_client

The order that the arm will go along:

1. `self.home_orientation_cartesian` - the initial position. pick a spot that will make reaching the pregrasp position easy.
2. `self.pre_grasp_orientation_cartesian` - the position where the open gripper will begin to close.
3. grasp planner used here
4. `self.manual_lift_pose_cartesian` - the arm manually lifts the object to this position. pick a higher position from your pregrasp orientation
5. `self.check_reward_orientation_cartesian` - should move the object to the right side of the camera, where a cropped region evaluates the success of the grasp. The arm and aruco markers should completely fit inside the cropped image (when running `log_cartesian_orientation.launch`, a window containing the image view of the reward detection algorithm should pop up for your reference.)

Starting around line 56.



### Test out your stuff

Terminal D:

```
cd ~/
conda deactivate
. kinova_venv/bin/activate

cd ~/sim-to-real-kinova
source devel/setup.bash
roslaunch openai_gym_kinova test_adams_cartesian_path.launch
```

rosrun kinova_demo fingers_action_client.py j2s7s300 percent 100 100 100



setup gym shit

```
pip install gym

cd ~/
mkdir pip_fodder
cd pip_fodder
```



Set up mujoco py

```
# get osmesa and opengl shit
sudo apt-get install libosmesa6-dev

cd ~/
mkdir ~/pip_fodder
cd ~/pip_fodder

wget -O mjpro150.zip https://www.roboti.us/download/mjpro150_linux.zip
mkdir ~/.mujoco
unzip mjpro150.zip -d ~/.mujoco

cp ~/mjkey.txt ~/.mujoco
cp ~/mjkey.txt ~/.mujoco/mjpro150/bin

export LD_LIBRARY_PATH=/home/mechagodzilla/.mujoco/mjpro150/bin:$LD_LIBRARY_PATH
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so

wget -O mujoco_py.zip https://github.com/openai/mujoco-py/archive/refs/tags/1.50.1.0.zip
unzip mujoco_py.zip 
cd mujoco-py-1.50.1.0/
pip install -e .

pip install gym

```



Extra notes:

If your arm is too fucking slow, use `joint_limits.yaml` to make it go fASTER!!! change the `max_velocity` param for each joint

https://github.com/ros-planning/moveit/issues/797 < talks about cartesian path planning



Additional information to supplement URDFs are in SRDFs. fun fact I found my collision parameters there.



Missing GL version?

```
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so
```



Making the right camera load up:

```
a = self._sim.render(width=w, height=h, depth=True, mode='offscreen', camera_name='camera')
```

And in every case where you define a new viewer:

```
self._viewer = MjViewer(self._sim)
self._viewer.cam.fixedcamid = 0  # set to the camera provided by the simulator!!!
self._viewer.cam.type = 2  # constant for CAMERA_FIXED
```

And in your XML file that you load into mujoco, replace your camera tag with this:

```
<camera euler="-0 0 0" mode="fixed" name="camera" pos="0 0 1" />
```



testing file:

check out this:

```
python src/sim-to-real-kinova-master/openai_gym_kinova/src/gym_kinova_gripper/envs/kinova_description/test.py
```





In simulation:

positive y: move into the hand

negative y: move away from hand

positive x: moves towards two finger side

negative x: moves toward single finger side





In real life:

positive y:  in the direction into the hand

negative y: in the direction away from the hand

positive x: in the direction of the single finger

negative x: in the direction of the two fingers

positive z: up

negative z: down



notes: if you move hand in one set of experiments and the object in another, you can invert the x axis and y axis



note: in the 



```
"""
For action space:
1. How fast the fingers close => how many timesteps to hit the "closed" state?
2. 

For reward space:
1. Draw a plot based on the XY noise successes:

there are 4 scenarios
1. agreement:
- both succeed
- both fail

2. disagreement:
- sim succeeds reality fails
- sim fails reality succeeds

For observation space:
For all individual states:
1. print graph of deltas between two states. should look flat.
2. print both sim and real states going at each other...
3. 


How to start:
0. collect the two logging directories
1. data structure to store dual replay buffers for both sim and real.
- call by trial numbers (and the corresponding noise assigned)
- actions
- states (call by a specific variable)
- rewards?

2. load past csvs?

3. do reward comparisons
4. do observation comparisons
5. do action comparions


extras:
overlay simulation over real world gif???

"""
```





```
pip install imageio-ffmpeg

sudo apt-get install python3-tk
```





TODO:

add dots



Adding to the mujoco thing:

Add forcelimited and forcerange to the actuators. This changes how much force the actuator can actually output. Forcerange will define the min and max amounts the actuator can give.

```
forcelimited="true" forcerange="0 1"
```



Friction parameter:

first 2 are tangental

first 2 are sliding

last 1 is rolling





stiffness

detection problesm: lighting issues with aruco. adjust shade or whatever



```
RuntimeError: CUDA error: CUBLAS_STATUS_INVALID_VALUE when calling `cublasSgemm( handle, opa, opb, m, n, k, &alpha, a, lda, b, ldb, &beta, c, ldc)`
```

https://discuss.pytorch.org/t/runtimeerror-cuda-error-cublas-status-invalid-value-when-calling-cublassgemm-handle-opa-opb-m-n-k-alpha-a-lda-b-ldb-beta-c-ldc/124544

NOT ENOUGH VRAM HOLY MOLY



For the peoples getting this error and ending up on this post, please  know that it can also be caused if you have a mismatch between the  dimension of your input tensor and the dimensions of your nn.Linear  module. (ex. x.shape = (a, b) and nn.Linear(c, c, bias=False) with c not matching)



`pip install wandb`

read instructions on setting up weights and biases at https://docs.wandb.ai/guides/integrations/pytorch 

https://docs.wandb.ai/guides/track/log



useless code:

```
class ExpertPIDController(object):
    """
    Fun fact: This isn't really a PID controller. It's just a hardcoded nudging strategy!

    """

    def __init__(self, obj_init_pos):
        self.prev_f1jA = 0.0
        self.prev_f2jA = 0.0
        self.prev_f3jA = 0.0
        self.step = 0.0
        self.init_obj_pose = obj_init_pos  # X position of object
        # self.init_obj_pose = self._sim.data.get_geom_xpos(object)
        # self.init_dot_prod = obj_init_dot_prod  # dot product of object wrt palm
        self.f1_vels = []
        self.f2_vels = []
        self.f3_vels = []
        self.wrist_vels = []

    def _count(self):
        self.step += 1

    def center_action(self, constant_velocity, wrist_lift_velocity, finger_lift_velocity, curr_obj_pose, lift_check):
        """ Object is in a center location within the hand, so lift with constant velocity or adjust for lifting """
        wrist, f1, f2, f3 = 0, constant_velocity, constant_velocity, constant_velocity

        # cartesian distance from original starting position
        curr_dist_traveled = np.sqrt((curr_obj_pose[:2] - self.init_obj_pose[
                                                          :2]) ** 2)  # we only index 2, ensuring we only grab x and y dimensions

        # Check if change in object dot product to wrist center versus the initial dot product is greater than 0.01
        if curr_dist_traveled > 0.05:
            # print("CHECK 2: Obj dot product to wrist has changed more than 0.01")
            # Start lowering velocity of finger 2 and 3 so the balance of force is equal (no tipping)
            f1, f2, f3 = constant_velocity, (constant_velocity / 2), (constant_velocity / 2)

        # Lift check determined by grasp check (distal finger tip movements)
        # and this check has occurred over multiple time steps
        if lift_check is True:
            # Ready to lift, so slow down Finger 1 to allow for desired grip
            # (where Fingers 2 and 3 have dominance)
            # print("Check 2A: Object is grasped, ready for lift")
            f1, f2, f3 = (finger_lift_velocity / 2), finger_lift_velocity, finger_lift_velocity
        return np.array([f1, f2, f3])

    def right_action(self, pid, states, min_velocity, wrist_lift_velocity, finger_lift_velocity, curr_obj_pose,
                     lift_check, velocities):
        """ Object is in an extreme right-side location within the hand, so Finger 2 and 3 move the
        object closer to the center """
        # cartesian distance from original starting position
        curr_dist_traveled = np.sqrt((curr_obj_pose[:2] - self.init_obj_pose[
                                                          :2]) ** 2)  # we only index 2, ensuring we only grab x and y dimensions

        # Only Small change in object dot prod to wrist from initial position, must move more
        # Object has not moved much, we want the fingers to move closer to the object to move it
        if curr_dist_traveled < 0.05:
            """ PRE-contact """
            # print("CHECK 5: Only Small change in object dot prod to wrist, moving f2 & f3")
            f1 = 0.0  # frontal finger doesn't move
            f2 = pid.touch_vel(curr_obj_pose, states[79], velocities)  # f2_dist dot product to object
            f3 = f2  # other double side finger moves at same speed
            wrist = 0.0
        else:
            """ POST-contact """
            # now finger-object distance has been changed a decent amount.
            # print("CHECK 6: Object dot prod to wrist has Changed More than 0.01")
            # Goal is 1 b/c obj_dot_prod is based on comparison of two normalized vectors
            if abs(1 - obj_dot_prod) > 0.01:
                # print("CHECK 7: Obj dot prod to wrist is > 0.01, so moving ALL f1, f2 & f3")
                # start to close the PID stuff
                f1 = min_velocity  # frontal finger moves slightly
                f2 = pid.velocity(obj_dot_prod, velocities)  # get PID velocity
                f3 = f2  # other double side finger moves at same speed
                wrist = 0.0
            else:  # goal is within 0.01 of being reached:
                # print("CHECK 8: Obj dot prod to wrist is Within reach of 0.01 or less, Move F1 Only")
                # start to close from the first finger
                f1 = pid.touch_vel(obj_dot_prod, states[78], velocities)  # f1_dist dot product to object
                f2 = 0.0
                f3 = 0.0
                wrist = 0.0

            # print("Check 9a: Check for grasp (small distal finger movement)")
            # Lift check determined by grasp check (distal finger tip movements)
            # and this check has occurred over multiple time steps
            if lift_check is True:
                # print("CHECK 9: Yes! Good grasp, move ALL fingers")
                f1, f2, f3 = (finger_lift_velocity / 2), finger_lift_velocity, finger_lift_velocity

        return np.array([f1, f2, f3])

    def left_action(self, pid, states, min_velocity, wrist_lift_velocity, finger_lift_velocity, obj_dot_prod,
                    lift_check, velocities):
        """ Object is in an extreme left-side location within the hand, so Finger 1 moves the
                object closer to the center """
        # Only Small change in object dot prod to wrist from initial position, must move more
        if abs(obj_dot_prod - self.init_dot_prod) < 0.01:
            """ PRE-contact """
            # print("CHECK 11: Only Small change in object dot prod to wrist, moving F1")
            f1 = pid.touch_vel(obj_dot_prod, states[78], velocities)  # f1_dist dot product to object
            f2 = 0.0
            f3 = 0.0
            wrist = 0.0
        else:
            """ POST-contact """
            # now finger-object distance has been changed a decent amount.
            # print("CHECK 12: Object dot prod to wrist has Changed More than 0.01")
            # Goal is 1 b/c obj_dot_prod is based on comparison of two normalized vectors
            if abs(1 - obj_dot_prod) > 0.01:
                # print("CHECK 13: Obj dot prod to wrist is > 0.01, so kep moving f1, f2 & f3")
                f1 = pid.velocity(obj_dot_prod, velocities)
                f2 = min_velocity  # 0.05
                f3 = min_velocity  # 0.05
                wrist = 0.0
            else:
                # Goal is within 0.01 of being reached:
                # print("CHECK 14: Obj dot prod to wrist is Within reach of 0.01 or less, Move F2 & F3 Only")
                # start to close from the first finger
                # nudge with thumb
                f2 = pid.touch_vel(obj_dot_prod, states[79], velocities)  # f2_dist dot product to object
                f3 = f2
                f1 = 0.0
                wrist = 0.0

            # print("Check 15a: Check for grasp (small distal finger movement)")
            # Lift check determined by grasp check (distal finger tip movements)
            # and this check has occurred over multiple time steps
            if lift_check is True:
                # print("CHECK 15b: Good grasp - moving ALL fingers")
                f1, f2, f3 = (finger_lift_velocity / 2), finger_lift_velocity, finger_lift_velocity
        return np.array([f1, f2, f3])

    def PDController(self, lift_check, curr_obj_pos, action_space, velocities):
        """ Position-Dependent (PD) Controller that is dependent on the x-axis coordinate position of the object to
        determine the individual finger velocities.
        """
        pid = PID(action_space)  # Define pid controller
        obj_dot_prod = obj_dot_prod  # Dot product of object wrt palm

        # Define action (finger velocities)
        f1 = 0.0  # far out finger, on single side
        f2 = 0.0  # double side finger - right top
        f3 = 0.0  # double side finger - right bottom
        wrist = 0.0

        # Velocity variables (for readability)
        constant_velocity = velocities["constant_velocity"]
        wrist_lift_velocity = velocities["wrist_lift_velocity"]
        finger_lift_velocity = velocities["finger_lift_velocity"]
        min_velocity = velocities["min_velocity"] + 1.3  # Add 1.3 so fingers are always moving
        max_velocity = velocities["max_velocity"]

        # Note: only comparing initial X position of object. because we know
        # the hand starts at the same position every time (close to origin)

        # Check if the object is near the center area (less than x-axis 0.03)
        if abs(self.init_obj_pose) <= 0.03:
            # print("CHECK 1: Object is near the center")
            controller_action = self.center_action(constant_velocity, wrist_lift_velocity, finger_lift_velocity,
                                                   obj_dot_prod, lift_check)
        else:
            # print("CHECK 3: Object is on extreme left OR right sides")
            # Object on right hand side, move 2-fingered side
            # Local representation: POS X --> object is on the RIGHT (two fingered) side of hand
            if self.init_obj_pose > 0.0:
                # print("CHECK 4: Object is on RIGHT side")
                controller_action = self.right_action(pid, states, min_velocity, wrist_lift_velocity,
                                                      finger_lift_velocity, obj_dot_prod, lift_check, velocities)

            # object on left hand side, move 1-fingered side
            # Local representation: NEG X --> object is on the LEFT (thumb) side of hand
            else:
                # print("CHECK 10: Object is on the LEFT side")
                controller_action = self.left_action(pid, states, min_velocity, wrist_lift_velocity,
                                                     finger_lift_velocity, obj_dot_prod, lift_check, velocities)

        self._count()
        controller_action = check_vel_in_range(controller_action, min_velocity, max_velocity, finger_lift_velocity)

        # print("f1: ", controller_action[0], " f2: ", controller_action[1], " f3: ", controller_action[2])
        self.f1_vels.append(f1)
        self.f2_vels.append(f2)
        self.f3_vels.append(f3)
        self.wrist_vels.append(wrist)

        return controller_action, self.f1_vels, self.f2_vels, self.f3_vels, self.wrist_vels


def check_vel_in_range(action, min_velocity, max_velocity, finger_lift_velocity):
    """ Checks that each of the finger/wrist velocies values are in range of min/max values """
    for idx in range(len(action)):
        if idx > 0:
            if action[idx] < min_velocity:
                if action[idx] != 0 or action[idx] != finger_lift_velocity or action[idx] != finger_lift_velocity / 2:
                    action[idx] = min_velocity
            elif action[idx] > max_velocity:
                action[idx] = max_velocity

    return action


class PID(object):
    def __init__(self, min_speed=0, max_speed=1.5):
        self.kp = 1
        self.kd = 1
        self.ki = 1
        self.prev_err = 0.0
        self.sampling_time = 15
        self.action_range = [min_speed, max_speed]
        self.max_speed = max_speed

        self.set_point = 0  # ideally, which

        # TODO: wtf, the velocity is not set properly...,

        # how it used to work: we wanted to reduce the orientation between the two fingers...
        # do we want a separate pid controller for each finger?

    def velocity(self, dot_prod, velocities):
        err = 0 - dot_prod
        diff = (err) / self.sampling_time
        vel = err * self.kp + diff * self.kd
        vel = (vel / 1.25)  # 1.25 means dot product equals to 1

        # Scale the velocity to the maximum velocity -
        # the PID was determined originally with a max of self.max_speed rad/sec
        action = (vel / self.max_speed) * velocities["max_velocity"]

        return action

    def joint(self, dot_prod):
        err = 1 - dot_prod
        diff = (err) / self.sampling_time
        joint = err * self.kp + diff * self.kd
        action = (joint / 1.25) * 2  # 1.25 means dot product equals to 1
        return action

    def touch_vel(self, obj_dotprod, finger_dotprod, velocities):
        err = obj_dotprod - finger_dotprod  # this really should be set point - variable point
        diff = err / self.sampling_time
        vel = err * self.kp + diff * self.kd

        action = (vel / self.max_speed) * velocities["max_velocity"]

        return action


# function to get the dot product. Only used for the pid controller
def _get_dot_product(self, obj_state=None):
    if obj_state == None:
        obj_state = self._get_obj_pose()
    hand_pose = self._sim.data.get_body_xpos("j2s7s300_link_7")
    obj_state_x = abs(obj_state[0] - hand_pose[0])
    obj_state_y = abs(obj_state[1] - hand_pose[1])
    obj_vec = np.array([obj_state_x, obj_state_y])
    obj_vec_norm = np.linalg.norm(obj_vec)
    obj_unit_vec = obj_vec / obj_vec_norm

    center_x = abs(0.0 - hand_pose[0])
    center_y = abs(0.0 - hand_pose[1])
    center_vec = np.array([center_x, center_y])
    center_vec_norm = np.linalg.norm(center_vec)
    center_unit_vec = center_vec / center_vec_norm

    dot_prod = np.dot(obj_unit_vec, center_unit_vec)
    return dot_prod ** 20  # cuspy to get distinct reward

```







lmao rewriting the coordinates

```
new_coord_arr = []

with open('CubeM.txt', 'r') as file:
    for line in file:
        line_coord_arr = line.strip().split(',')
        line_coord_arr[2] = '0.0555'  # new height
        new_coord_str = ','.join(line_coord_arr)
#         print(new_coord_str)
        new_coord_arr.append(new_coord_str)
    
with open('CylinderM.txt', 'w') as file:
    for line in new_coord_arr:
        file.write(line + '\n')
        

with open('Cone1M.txt', 'w') as file:
    for line in new_coord_arr:
        file.write(line + '\n')
        
        
with open('Vase1M.txt', 'w') as file:
    for line in new_coord_arr:
        file.write(line + '\n')
```

