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

```sh
# rm -rf ~/.catkin_tools #if you have another catkin build workspace
mkdir -p catkin_ws_evio/src
cd catkin_ws_evio
catkin config --init --mkdirs --extend /opt/ros/noetic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release
cd ~/catkin_ws_evio/src
git clone git@github.com:ERGlab/PLEVIO.git --recursive
```

You should modifie your `.bashrc` file through `gedit ~/.bashrc`, add the following codes in it:

```sh
source ~/catkin_ws_evio/devel/setup.bash
alias pleviobuild='cd ~/catkin_ws_evio/src && catkin build evio_estimator feature_tracker pose_graph -DCMAKE_BUILD_TYPE=Release -j8'
```

After that, run the `source ~/.bashrc ` and `pleviobuild` command in your terminal.

## 3. Run on Dataset
We have released all the rosbag files for evaluating PL-EVIO, with the introduction of these datasets can be found on this [page](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#Dataset-for-monocular-evio).
</br>
For the convenience of the community, we also release the raw results of our methods in the form of rosbag ([link](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/blob/main/Results_for_comparison.md)). 


<!-- ******************************************************* -->
### 3.1 Run on Stereo HKU-dataset
#### 3.1.1 Download our rosbag files ([Stereo HKU-dataset](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#Dataset-for-stereo-evio))

#### 3.1.2 Run our examples
After you have downloaded our bag files, you can now run our example:

```sh
roslaunch evio_estimator plevio_stereo_hku.launch 
rosbag play --pause --clock ~/dataset/HKU_aggressive_small_flip.bag
```


<!-- ******************************************************* -->
### 3.2 Run on Mono HKU-dataset
#### 3.3.1 Download our rosbag files ([Mono HKU-dataset](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#Dataset-for-monocular-evio))

#### 3.3.2 Run vicon_hdr4 as examples
After you have downloaded our bag files, you can now run:

```sh
roslaunch evio_estimator plevio_mono_hku.launch 
rosbag play --pause --clock ~/dataset/vicon_hdr4.bag
```


<!-- ******************************************************* -->
### 3.3 Run on DAVIS240C dataset
#### 3.3.1 Download rosbag files ([davis240c](https://rpg.ifi.uzh.ch/davis_data.html))

#### 3.3.2 Run boxes_translation as examples
After you have downloaded the bag files, you can now run:

```sh
roslaunch evio_estimator plevio_davis240c.launch 
rosbag play --pause --clock ~/dataset/boxes_translation.bag
```

### 3.4 Run on Your Event Camera
* We recommend to follow the instruction on ([DVS-IMU Calibration and Synchronization](https://arclab-hku.github.io/ecmd/calibration/)) to calibrate your sensors.
* Refer to our [ESVIO](https://github.com/arclab-hku/ESVIO)


<!-- ******************************************************* -->
## Acknowledgement
This work was supported by General Research Fund under Grant 17204222, and in part by the Seed Fund for Collaborative Research and General Funding Scheme-HKU-TCL Joint Research Center for Artificial
Intelligence.
We use ([VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)) as our backbone code. Thanks Dr. Qin Tong, Prof. Shen, etc. very much.

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
