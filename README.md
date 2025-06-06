<div align="center">

# PL-EVIO：Robust Monocular Event-based Visual Inertial Odometry with Point and Line Features 

**Weipeng Guan**<sup>†</sup>, **Peiyu Chen**<sup>†</sup>, **Yuhan Xie**, **Peng Lu**<sup>*</sup>

**Adaptive Robotic Controls Lab (ArcLab)**, **The University of Hong Kong**.

</div>


[comment]: <> (  <h2 align="center">PAPER</h2>)
  <h3 align="center">
  <a href="https://ieeexplore.ieee.org/abstract/document/10287884">Paper</a> 
  | <a href="https://kwanwaipang.github.io/PL-EVIO/">Website</a> 
  | <a href="https://www.bilibili.com/video/BV12t4y1L7eK/?spm_id_from=333.1387">Demo</a> 
  </h3>

<div align="center">
  <img src="https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/raw/main/PL-EVIO/PLEVIO_flip_3.gif" width="90%" />
</div>


## Abstract
<div align="justify">
Robust state estimation in challenge situations is still an unsolved problem, especially achieving onboard pose feedback control for aggressive motion. 
In this paper, we propose robust and real-time event-based visual-inertial odometry (VIO) that incorporates event, image, and inertial measurements. 
Our approach utilizes line-based event features to provide additional structure and constraint information in human-made scenes, while point-based event and image features complement each other through well-designed feature management. 
To achieve reliable state estimation, we tightly couple the point-based and line-based visual residuals from the event camera, the point-based visual residual from the standard camera, and the residual from IMU pre-integration using a keyframe-based graph optimization framework. 
Experiments in the public benchmark datasets show that our method can achieve superior performance compared with the state-of-the-art image-based or event-based VIO. 
Furthermore, we demonstrate the effectiveness of our pipeline through onboard closed-loop quadrotor aggressive flight and large-scale outdoor experiments. Videos of the evaluations can be found on our website.
</div>


## 1. Prerequisites
1.1 Ubuntu 20.04 with ROS Noetic.


1.2 Ceres Solver Follow [Ceres Installation](http://ceres-solver.org/installation.html), remember to make install and use the version 1.14.0 ([Download Link](dependences/ceres-solver-1.14.0.zip)).


1.3 we use catkin build, and all the dependency files are stored within the folder `dependences`.

## 2. Build
~~~
mkdir -p catkin_ws_dvs/src
cd catkin_ws_dvs
catkin config --init --mkdirs --extend /opt/ros/noetic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release
cd ~/catkin_ws_dvs/src
git clone git@github.com:arclab-hku/PL-EVIO_open.git --recursive
~~~

You should modifie your `.bashrc` file through `gedit ~/.bashrc`, add the following codes in it:
~~~
source ~/catkin_ws_dvs/devel/setup.bash
alias EVIObuild='cd ~/catkin_ws_dvs/src && catkin build PL-EVIO_estimator feature_tracker pose_graph -DCMAKE_BUILD_TYPE=Release -j8'
~~~

After that, run the `source ~/.bashrc ` and `EVIObuild` command in your terminal.

## 3. Run on Dataset

### 3.1 Run on HKU-dataset
#### 3.1.1 Download our rosbag files ([HKU-dataset](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM))
Our datasets for evaluation can be download from our One-drive or Baidu-Disk. 
We have released all the rosbag files for evaluating PL-EVIO, with the introduction of these datasets can be found on this [page](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#Dataset-for-monocular-evio).
</br>
For the convenience of the community, we also release the raw results of our methods in the form of rosbag ([link](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/blob/main/Results_for_comparison.md)). 

#### 3.1.2 Run our examples
After you have downloaded our bag files, you can now run our example:
~~~
roslaunch PL-EVIO_estimator PL-EVIO.launch 
rosbag play YOUR_DOWNLOADED.bag
~~~

### 3.2 Run on Your Event Camera
#### 3.2.1 Driver Installation
We thanks the [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros) and [DV ROS](https://gitlab.com/inivation/dv/dv-ros) for their intructions of event camera driver.
We add some modification for the code, and the driver code of the event camera is available in [link](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/tree/main/driver_code).
User can choose either one.

##### For rpg_dvs_ros
* Step 1: Install libcaer (add required repositories as per [iniVation documentation](https://inivation.gitlab.io/dv/dv-docs/docs/getting-started.html#ubuntu-linux) first):
~~~
sudo apt-get install libcaer-dev
~~~

*Step 2: Create a catkin workspace and copy the driver code:
~~~
mkdir -p ~/catkin_ws_dvs/src
cd ~/catkin_ws_dvs
catkin config --init --mkdirs --extend /opt/ros/noetic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release`
cd ~/catkin_ws_dvs/src
~~~

And then copy the code from our link, or directly use the driver code in the dependences folder

* Step 3: Build the packages:
~~~
catkin build davis_ros_driver  (if you are using the DAVIS)
catkin build dvxplorer_ros_driver  (if you are using the DVXplorer)

source ~/catkin_ws_dvs/devel/setup.bash
~~~

* Step 4: After source your environment, you can open your event camera:
~~~
roslaunch dvs_renderer davis_mono.launch` (if you are using the DAVIS)
roslaunch dvs_renderer dvxplorer_mono.launch` (if you are using the DVXplorer)
~~~

##### For DV ROS
* Step 1: Instalizing DV software libraries:
~~~
sudo add-apt-repository ppa:inivation-ppa/inivation
sudo apt update
sudo apt install dv-processing dv-runtime-dev gcc-10 g++-10
~~~

* Step 2: It is build using catkin tools, run the following commands from your catkin workspace:
~~~
mkdir -p ~/catkin_ws_dvs/src
cd ~/catkin_ws_dvs
catkin config --init --mkdirs --extend /opt/ros/noetic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release`
cd ~/catkin_ws_dvs/src
~~~

And then copy the code from our link.

* Step 3: Modifying your `.bashrc` file, add the following codes in it:
~~~
source ~/catkin_dvs_ws/devel/setup.bash

alias dvsbuild='cd ~/catkin_dvs_ws && catkin build dv_ros_accumulation dv_ros_capture dv_ros_imu_bias dv_ros_messaging dv_ros_runtime_modules dv_ros_tracker dv_ros_visualization -DCMAKE_BUILD_TYPE=Release --cmake-args -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10'

alias dvsrun='cd ~/catkin_dvs_ws/src/Event_based_VO-VIO-SLAM/driver_code/dv-ros-master/script && sh run.sh'
~~~

* Step 4: the user can directly run the following command `dvsbuild` or `dvsrun` in the terminal to build the project and run your event camera, respectively.

**Tips**: Users need to adjust the lens of the camera, such as the focal length, aperture.
Filters are needed for avoiding the interfere from infrared light under the motion capture system.
For the dvxplorer, the sensitive of event generation should be set, e.g. `bias_sensitivity`.
Users can visualize the event streams to see whether it is similiar to the edge map of the testing environments, and then fine-tune it.


#### 3.2.2 Sensor calibration
In order to launch PL-EVIO on your own hardware setup, you need to have a carefully calibration of the extrinsic among Event, Image and IMU. We recommend you using the following the link ([DVS-IMU Calibration and Synchronization](https://arclab-hku.github.io/ecmd/calibration/)) to kindly calibrate your sensors.


## Acknowledgement
This work was supported by General Research Fund under Grant 17204222, and in part by the Seed Fund for Collaborative Research and General Funding Scheme-HKU-TCL Joint Research Center for Artificial
Intelligence.

We use ([VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)) as our baseline code. Thanks Dr. Qin Tong, Prof. Shen, etc. very much.

If you find this work is helpful in your research, a simple star or citation of our works should be the best affirmation for us. :blush:

~~~
@article{GWPHKU:PL-EVIO,
  title={PL-EVIO: Robust Monocular Event-based Visual Inertial Odometry with Point and Line Features},
  author={Guan, Weipeng and Chen, Peiyu and Xie, Yuhan and Lu, Peng},
  journal={IEEE Transactions on Automation Science and Engineering},
  volume={21},
  number={4},
  pages={6277--6293},
  year={2023},
  publisher={IEEE}
}
~~~

## License
The source code is released under GPLv3 license. 
We are still working on improving the code reliability. 
If you are interested in our project for commercial purposes, please contact [Dr. Peng LU](https://arclab.hku.hk/People.html) for further communication.
