#!/bin/bash

ROS_LAUNCH_FILE=${1:-linefeatures.launch}
DATA_PATH=${2:-/data/rosbag/ICCV2025/MVSEC/indoor_flying1_data.bag}
OUTPUT_DIR=${3:-~/data/LineSegments/ICCV2025/rosbag_mvsec_indoor1/c2f-efio/csvs}
START_TRIAL=${4:-1}
END_TRIAL=${5:-30}
PLAY_SPEED=${6:-0.2}
DURATION=${7:-10}
START_TIME=${8:-0}
CONFIG_FILE=${9:-/home/plevio/catkin_ws/src/PL-EVIO/config/mono_davis346/davis346_mvsec.yaml}

for i in $(seq $START_TRIAL $END_TRIAL); do
    docker exec plevio /bin/bash -c "cd ~/catkin_ws && source devel/setup.bash && roslaunch evio_estimator $ROS_LAUNCH_FILE bag_file:=$DATA_PATH play_speed:=$PLAY_SPEED start_time:=$START_TIME duration:=$DURATION config_path:=$CONFIG_FILE"

    docker cp plevio:/home/plevio/lines.csv "$OUTPUT_DIR/$i.csv"
done