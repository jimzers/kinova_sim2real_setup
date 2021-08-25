# Sim2Real Transfer with a Kinova Hand



### Intro / Foreword (LOL)

This document highlights my experiences with the sim2real transfer project at OSU. In this doc you'll find a lot of commands for running stuff, as well as intructions and things to avoid...



Outline:

1. Folder structure and important files
2. Setup (technical)
   1. Prereqs
   2. ROS + Python setup
3. Experimental process
   1. Overview
   2. Running experiments
4. Random notes
   1. Hacks
   2. Stuff I didn't fix (sorry guys!)
   3. Rants







### Folder structure

Most of my work is in `openai_gym_kinova`. I'll talk about it here:





### Other important files

`object_pose_and_distance_estimation.py`: use the one in <>

`reward_detection.py`: use the one in <>



### Setup

Refer to ROS setup for more info...

#### Prerequisites

1. You should already know how to use: python virtualenvs, pip, catkin, building stuff in ROS, environment variables. google search these if you don't
2. 






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



```
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









### Experimental setup

1. Setting up the arm: do all the setup instructoins (ROS software, python virtualenv + mujoco, camera, kinova arm + hand)
   1. look at ROS
2. Training the RL algorithm
   1. look at stephanies docs
   2. look at nautilus docs (honestly just ask adam to set this one up for you)
3. Evaluating the RL algorithm: simulation
   1. main_experiment_sim.py
4. Evaluating the RL algorithm: real world
   1. main_experiment.py
5. Running analysis scripts + comparison scripts
   1. visualize_irl.py (rename this lol)
   2. compare_sim_and_real_analysis.py







