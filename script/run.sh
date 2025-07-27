#! /bin/bash

gnome-terminal --tab -e 'bash -c "roscore;exec bash"'
sleep 3s

# #################********************* Stereo HKU ********************************##############
gnome-terminal --tab -e 'bash -c "roslaunch evio_estimator plevio_stereo_hku.launch;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause --clock /home/cpy/Datasets/ESVIO/HKU_aggressive_translation.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause --clock /home/cpy/Datasets/ESVIO/HKU_aggressive_small_flip.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause --clock /home/cpy/Datasets/ESVIO/hku_hdr_tran_rota.bag;exec bash"'
gnome-terminal --window -e 'bash -c "rosbag play --pause --clock /home/cpy/Datasets/ESVIO/hku_dark_normal.bag;exec bash"'

# #################*********************在346下验证PL-EVIO********************************##############
# gnome-terminal --tab -e 'bash -c "roslaunch evio davis_open.launch;exec bash"'
# gnome-terminal --tab -e 'bash -c "roslaunch evio_estimator plevio_mono_hku.launch;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause /home/cpy/Datasets/vicon_darktolight1.bag;exec bash"'

# #################*********************在240下验证PL-EVIO********************************##############
# gnome-terminal --tab -e 'bash -c "roslaunch evio_estimator plevio_davis240c.launch;exec bash"'
# # gnome-terminal --tab -e 'bash -c "roslaunch evio davis240c_evaluation.launch;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause /home/kwanwaipang/dataset/davis240c/poster_6dof_hku.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause --clock /home/cpy/Datasets/boxes_6dof.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause --clock /home/cpy/Datasets/boxes_translation.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause --clock /home/cpy/Datasets/dynamic_translation.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause --clock /home/cpy/Datasets/dynamic_6dof.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause --clock /home/cpy/Datasets/hdr_boxes.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause --clock /home/cpy/Datasets/hdr_poster.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause --clock /home/cpy/Datasets/poster_6dof.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause --clock /home/cpy/Datasets/poster_translation.bag;exec bash"'
sleep 16s