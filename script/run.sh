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

# # evo_ape bag pl_eio_result*.bag /dvs_vicon/gt_pose /pose_graph/evio_loop -v --align --n_to_align 251
# # evo_traj bag pl_eio_result*.bag /pose_graph/evio_loop --ref=/dvs_vicon/gt_pose -v --align --n_to_align 251 --plot --plot_mode=xyz

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

# 下面用evo包
# evo_ape bag davis240c*.bag /optitrack/davis /pose_graph/evio_loop -v --align --n_to_align 1000
# evo_traj bag davis240c*.bag /pose_graph/evio_loop --ref=/optitrack/davis -v --align --n_to_align 1000 --plot --plot_mode=xyz

# evo_traj bag dynamic_translation.bag /pose_graph/evio_loop --ref=/optitrack/davis -v --align --n_to_align 1000 --plot --plot_mode=xyz

# # #################*********************testing the HKU_LAB********************************##############
# gnome-terminal --tab -e 'bash -c "roslaunch evio_estimator plevio_davis346.launch;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause /home/cpy/Datasets/HKU/20220816dvs_fix_circle.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause /home/cpy/Datasets/HKU/20220816dvs_fix_eight.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause /home/cpy/Datasets/HKU/20220816dvs_varing_circle.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause /home/cpy/Datasets/HKU/20220816dvs_varing_eight.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause /home/cpy/Datasets/HKU/Vicon_dvs_fix_eight.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause /home/cpy/Datasets/HKU/Vicon_dvs_varing_eight.bag;exec bash"'
  # gnome-terminal --window -e 'bash -c "rosbag play --pause /home/cpy/Datasets/HKU/EVIO_offline_fix_eight_1.5.bag;exec bash"'
  # gnome-terminal --window -e 'bash -c "rosbag play --pause /home/cpy/Datasets/HKU/EVIO_offline_varing_eight_1.5.bag;exec bash"'
  # gnome-terminal --window -e 'bash -c "rosbag play --pause /home/cpy/Datasets/HKU/EVIO_online_fix_eight_1.5.bag;exec bash"'
  # gnome-terminal --window -e 'bash -c "rosbag play --pause /home/cpy/Datasets/HKU/EVIO_online_fix_eight_3.0.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause /home/cpy/Datasets/HKU/EVIO_online_varing_eight_1.5.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause /home/cpy/Datasets/HKU/HKU_outside1.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause /home/cpy/Datasets/HKU/HKU_outside2.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause /home/cpy/Datasets/HKU/HKU_flip_1.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause /home/cpy/Datasets/HKU/HKU_flip_fail_16_04_11.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play -s 10 --pause /home/cpy/Datasets/HKU/HKU_flip_2.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play -s 10 --pause /home/cpy/Datasets/HKU/HKU_flip_calculate.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play -s 3 --pause /home/cpy/2022-10-17-16-14-48.bag;exec bash"'

  

# # #################*********************testing the fpv********************************##############
# gnome-terminal --tab -e 'bash -c "roslaunch evio_estimator fpv.launch;exec bash"'
# # # # gnome-terminal --tab -e 'bash -c "roslaunch evio fpv_evaluation.launch;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause /home/kwanwaipang/dataset/fpv/Indoor_forward/indoor_forward_3_davis_with_gt.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause /home/kwanwaipang/dataset/fpv/Indoor_45_degree/indoor_45_2_davis_with_gt.bag;exec bash"'
# # gnome-terminal --window -e 'bash -c "rosbag play --pause /home/kwanwaipang/dataset/fpv/Outdoor_forward/outdoor_forward_1_davis_with_gt.bag;exec bash"'
# # gnome-terminal --window -e 'bash -c "rosbag play --pause /home/kwanwaipang/dataset/fpv/Outdoor_45_degree/outdoor_45_1_davis_with_gt.bag;exec bash"'


# evo_ape bag pl_evio_result*.bag /groundtruth/pose /pose_graph/evio_loop -va
# evo_traj bag pl_evio_result*.bag /pose_graph/evio_loop --ref=/groundtruth/pose -va --plot --plot_mode=xyz

# 保存txt
# evo_traj bag pl_evio_result*.bag /groundtruth/pose --save_as_tum
# evo_ape tum groundtruth_pose.txt pl_evio_result_loop.txt -va --plot --plot_mode xyz 
#################*********************testing the fpv********************************##############








#################*********************visualization********************************##############
#  gnome-terminal --tab -e 'bash -c "roslaunch evio evio.launch;exec bash"'

#################*********************recording dataset********************************##############
# gnome-terminal --tab -e 'bash -c "roslaunch evio vicon.launch;exec bash"'
# gnome-terminal --tab -e 'bash -c "roslaunch evio davis_test.launch;exec bash"'
# gnome-terminal --tab -e 'bash -c "roslaunch evio dvxplorer_record.launch;exec bash"'

#################*********************recording two dvs and vicon********************************##############
# gnome-terminal --tab -e 'bash -c "ROS_NAMESPACE=davis346 roslaunch evio davis_open.launch;exec bash"'
# gnome-terminal --tab -e 'bash -c "ROS_NAMESPACE=dvxplorer roslaunch evio dvxplorer_open.launch;exec bash"'
# sleep 3s
# gnome-terminal --tab -e 'bash -c "roslaunch evio davis_dvxplorer_record.launch;exec bash"'

# gnome-terminal --tab -e 'bash -c "roslaunch evio result_record_withoutvicon.launch;exec bash"'


#################*********************davis 346********************************##############
# gnome-terminal --tab -e 'bash -c "ROS_NAMESPACE=dvs roslaunch evio davis_open.launch;exec bash"'
# sleep 3s
# gnome-terminal --tab -e 'bash -c "roslaunch evio_estimator davis346.launch;exec bash"'
# # # # sleep 3s
# # # ######################data set for hku davis346
# gnome-terminal --window -e 'bash -c "rosbag play --pause ~/dataset/hku_event_camera/davis346+dvxplorer/vicon_test_aggressive.bag;exec bash"'

# #################*********************evaluation********************************##############
# gnome-terminal --tab -e 'bash -c "roslaunch evio davis346_dvxplorer_evaluation.launch;exec bash"'

# python3 /home/kwanwaipang/catkin_ws/src/rpg_trajectory_evaluation/scripts/dataset_tools/bag_to_pose.py dvs_vicon_result.bag /pose_graph/imu_evio_loop --msg_type PoseStamped --output stamped_traj_estimate.txt
# python3 /home/kwanwaipang/catkin_ws/src/rpg_trajectory_evaluation/scripts/dataset_tools/bag_to_pose.py dvs_vicon_result.bag /dvs_vicon/gt_pose --msg_type PoseStamped --output stamped_groundtruth.txt
# python3 /home/kwanwaipang/catkin_ws/src/rpg_trajectory_evaluation/scripts/analyze_trajectory_single.py /home/kwanwaipang/dataset/hku_event_camera/davis346+dvxplorer/vicon_result_record/vicon_test

# evo_ape bag dvs_vicon_result.bag /dvs_vicon/gt_pose /pose_graph/imu_evio_loop -v --align --n_to_align 251
# evo_traj bag dvs_vicon_result.bag /pose_graph/imu_evio_loop --ref=/dvs_vicon/gt_pose -v --align --n_to_align 251 --plot --plot_mode=xyz
# evo_traj bag dvs_vicon_result.bag /pose_graph/imu_evio_loop --ref=/dvs_vicon/gt_pose -v --align_origin --plot --plot_mode=xyz
# evo_traj bag /home/cpy/Datasets/HKU/EVIO_evo_online_fix_eight_3.0.bag /pose_graph/evio_odometry --ref=/cpy_uav/viconros/odometry -va -p --plot_relative_time
# evo_ape bag /home/cpy/Datasets/HKU/EVIO_evo_online_fix_eight_3.0.bag /pose_graph/evio_odometry /cpy_uav/viconros/odometry -va -p --plot_full_ref

# #################*********************testing the fpv********************************##############
# gnome-terminal --tab -e 'bash -c "roslaunch evio_estimator fpv.launch;exec bash"'
# # gnome-terminal --tab -e 'bash -c "roslaunch evio fpv_evaluation.launch;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause /home/kwanwaipang/dataset/fpv/Indoor_forward/indoor_forward_3_davis_with_gt.bag;exec bash"'
# # gnome-terminal --window -e 'bash -c "rosbag play --pause /home/kwanwaipang/dataset/fpv/Indoor_45_degree/indoor_45_2_davis_with_gt.bag;exec bash"'
# # gnome-terminal --window -e 'bash -c "rosbag play --pause /home/kwanwaipang/dataset/fpv/Outdoor_forward/outdoor_forward_1_davis_with_gt.bag;exec bash"'
# # gnome-terminal --window -e 'bash -c "rosbag play --pause /home/kwanwaipang/dataset/fpv/Outdoor_45_degree/outdoor_45_1_davis_with_gt.bag;exec bash"'


# evo_ape bag pl_evio_result*.bag /groundtruth/pose /pose_graph/evio_loop -va
# evo_traj bag pl_evio_result*.bag /pose_graph/evio_loop --ref=/groundtruth/pose -va --plot --plot_mode=xyz

# 保存txt
# evo_traj bag pl_evio_result*.bag /groundtruth/pose --save_as_tum
# evo_ape tum groundtruth_pose.txt pl_evio_result_loop.txt -va --plot --plot_mode xyz 


#################*********************双目********************************##############
# gnome-terminal --tab -e 'bash -c "roslaunch evio_estimator esvio.launch;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause /home/kwanwaipang/dataset/fusionportable/20220216_garden_day.bag;exec bash"'



# #################*********************dvxplorer********************************##############
# # gnome-terminal --tab -e 'bash -c "ROS_NAMESPACE=dvxplorer roslaunch evio dvxplorer_open.launch;exec bash"'
# # sleep 3s
# # gnome-terminal --tab -e 'bash -c "roslaunch evio evio.launch;exec bash"'
# gnome-terminal --tab -e 'bash -c "roslaunch evio_estimator dvxplorer.launch;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause /home/kwanwaipang/dataset/hku_event_camera/dvxplorer_live_1.bag;exec bash"'

# # gnome-terminal --tab -e 'bash -c "roslaunch evio_estimator davis346.launch;exec bash"'
# # gnome-terminal --tab -e 'bash -c "roslaunch evio davis346_dvxplorer_evaluation.launch;exec bash"'
# # gnome-terminal --window -e 'bash -c "rosbag play --pause /home/kwanwaipang/dataset/hku_event_camera/davis346+dvxplorer/vicon_record/vicon_lighttodark1.bag;exec bash"'


# # evo_ape bag pl_eio_result*.bag /dvs_vicon/gt_pose /pose_graph/evio_loop -v --align --n_to_align 251
# # evo_traj bag pl_eio_result.bag /pose_graph/imu_evio_loop --ref=/dvs_vicon/gt_pose -v --align_origin --plot --plot_mode=xyz
# # evo_traj bag pl_eio_result*.bag /pose_graph/evio_loop --ref=/dvs_vicon/gt_pose -v --align --n_to_align 251 --plot --plot_mode=xyz


# ####################################测试
# gnome-terminal --window -e 'bash -c "rosbag play --pause /home/kwanwaipang/dataset/hku_event_camera/davis346+dvxplorer/indoor_aggressive_test_1.bag;exec bash"'



# gnome-terminal --window -e 'bash -c "rosbag play -s 8 --pause /home/kwanwaipang/dataset/hku_event_camera/dvxplorer_mini/vicon_flight_12.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play -s 20 --pause /home/kwanwaipang/dataset/hku_event_camera/dvxplorer_mini/vicon_flight_9.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play -s 22 --pause /home/kwanwaipang/dataset/hku_event_camera/dvxplorer_mini/vicon_flight_hover.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "roslaunch evio dvxplorer_mini_evaluation.launch;exec bash"'

# # # sleep 3s
# # # # # # ######################data set for hku dvxplorer
# gnome-terminal --window -e 'bash -c "rosbag play --pause -s 63 ~/dataset/hku_event_camera/dvxplorer/indoor_outdoor_2.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause -s 10 ~/dataset/hku_event_camera/dvxplorer/indoor_outdoor_1.bag;exec bash"'

#################*********************evaluation********************************##############
# gnome-terminal --tab -e 'bash -c "roslaunch evio davis346_dvxplorer_evaluation.launch;exec bash"'
# gnome-terminal --tab -e 'bash -c "rosrun image_view image_view image:=/feature_tracker/feature_img __ns:=/aa;exec bash"'
# gnome-terminal --tab -e 'bash -c "rosrun image_view image_view image:=/feature_tracker/feature_img_two __ns:=/bb;exec bash"'

# python3 /home/kwanwaipang/catkin_ws/src/rpg_trajectory_evaluation/scripts/dataset_tools/bag_to_pose.py dvs_vicon_result.bag /pose_graph/imu_evio_loop --msg_type PoseStamped --output stamped_traj_estimate.txt
# python3 /home/kwanwaipang/catkin_ws/src/rpg_trajectory_evaluation/scripts/dataset_tools/bag_to_pose.py dvs_vicon_result.bag /dvs_vicon/gt_pose --msg_type PoseStamped --output stamped_groundtruth.txt
# python3 /home/kwanwaipang/catkin_ws/src/rpg_trajectory_evaluation/scripts/analyze_trajectory_single.py /home/kwanwaipang/dataset/hku_event_camera/davis346+dvxplorer/vicon_result_record/orb-slam3/vicon_test1

# evo_ape bag dvs_vicon_result.bag /dvs_vicon/gt_pose /pose_graph/imu_evio_loop -v --align --n_to_align 251
# evo_traj bag dvs_vicon_result.bag /pose_graph/imu_evio_loop --ref=/dvs_vicon/gt_pose -v --align --n_to_align 251 --plot --plot_mode=xyz
# evo_traj bag dvs_vicon_result.bag /pose_graph/imu_evio_loop --ref=/dvs_vicon/gt_pose -v --align_origin --plot --plot_mode=xyz


########outdoor test
# gnome-terminal --window -e 'bash -c "rosbag play -r 0.8 --pause ~/dataset/hku_event_camera/dvxplorer/outdoor_6.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play -r 0.8 --pause ~/dataset/hku_event_camera/dvxplorer/outdoor_5.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause -r 0.8 -s 18 ~/dataset/hku_event_camera/dvxplorer/outdoor_4.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause -r 0.8 ~/dataset/hku_event_camera/dvxplorer/outdoor_3.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play -s 3 --pause ~/dataset/hku_event_camera/dvxplorer/outdoor_2.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause -s 50 ~/dataset/hku_event_camera/dvxplorer/outdoor_1.bag;exec bash"'

#################*********************dvxplorer********************************##############


#################*********************dvxplorer-mini********************************##############
# gnome-terminal --tab -e 'bash -c "ROS_NAMESPACE=dvxplorer_mini roslaunch evio dvxplorer_open.launch;exec bash"'
# sleep 3s
# gnome-terminal --tab -e 'bash -c "roslaunch evio_estimator dvxplorer_mini.launch;exec bash"'
# gnome-terminal --tab -e 'bash -c "roslaunch evio dvxplorer_mini_record.launch;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play -s 16 --pause /home/kwanwaipang/dataset/hku_event_camera/dvxplorer_mini/vicon_flight_1.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play -s 16 --pause /home/kwanwaipang/dataset/hku_event_camera/dvxplorer_mini/vicon_flight_2.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play -s 12 --pause /home/kwanwaipang/dataset/hku_event_camera/dvxplorer_mini/vicon_flight_8.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "roslaunch evio dvxplorer_mini_evaluation.launch;exec bash"'
# evo_ape bag vicon_flight_result.bag /gwp_uav/viconros/odometry /pose_graph/imu_evio_loop -v --align --n_to_align 476
# evo_traj bag vicon_flight_result.bag /pose_graph/imu_evio_loop --ref=/gwp_uav/viconros/odometry -v --align --n_to_align 476 --plot --plot_mode=xyz
# python3 /home/kwanwaipang/catkin_ws/src/rpg_trajectory_evaluation/scripts/dataset_tools/bag_to_pose.py vicon_flight_result.bag /pose_graph/imu_evio_loop --msg_type PoseStamped --output stamped_traj_estimate.txt
# python3 /home/kwanwaipang/catkin_ws/src/rpg_trajectory_evaluation/scripts/dataset_tools/bag_to_pose.py vicon_flight_result.bag /evio_estimator/gt_pose --msg_type PoseStamped --output stamped_groundtruth.txt
# python3 /home/kwanwaipang/catkin_ws/src/rpg_trajectory_evaluation/scripts/analyze_trajectory_single.py /home/kwanwaipang/dataset/hku_event_camera/dvxplorer_mini/result/666



#################*********************ORBSLAM3********************************##############
# cd ~/catkin_ws/ORB_SLAM3 && ./build_ros.sh
# gnome-terminal --tab -e 'bash -c "rosrun ORB_SLAM3 Mono_Inertial ~/catkin_ws/ORB_SLAM3/Vocabulary/ORBvoc.txt  ~/catkin_ws/ORB_SLAM3/Examples/Monocular-Inertial/davis346.yaml ture;exec bash"'
# gnome-terminal --tab -e 'bash -c "rosrun ORB_SLAM3 Mono ~/catkin_ws/ORB_SLAM3/Vocabulary/ORBvoc.txt  ~/catkin_ws/ORB_SLAM3/Examples/Monocular/davis346.yaml;exec bash"'
# gnome-terminal --tab -e 'bash -c "rosrun image_view image_view image:=/davis346/image_raw;exec bash"'
# gnome-terminal --tab -e 'bash -c "roslaunch orbslam3_ros davis346_monoimu.launch;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --pause ~/dataset/hku_event_camera/davis346+dvxplorer/vicon_test10.bag;exec bash"'
# gnome-terminal --tab -e 'bash -c "roslaunch evio orbslam3_evaluation.launch;exec bash"'

# evo_ape bag orbslam3.bag /dvs_vicon/gt_pose /orbslam3/pose -v --align --n_to_align 90
# evo_traj bag orbslam3.bag /orbslam3/pose --ref=/dvs_vicon/gt_pose -v --align --n_to_align 90 --plot --plot_mode=xyz
# evo_ape bag vicon_test2.bag /dvs_vicon/gt_pose /orbslam3/pose -v --align --n_to_align 110

# gnome-terminal --window -e 'bash -c "rosbag play --pause ~/dataset/hku_event_camera/davis346+dvxplorer/vicon_test_aggressive.bag;exec bash"'
# python3 /home/kwanwaipang/catkin_ws/src/rpg_trajectory_evaluation/scripts/analyze_trajectory_single.py /home/kwanwaipang/dataset/hku_event_camera/davis346+dvxplorer/vicon_result_record/orb-slam3/vicon_test
# evo_ape tum stamped_groundtruth.txt stamped_traj_estimate.txt -v --align --n_to_align 116
# evo_traj tum stamped_traj_estimate.txt --ref=stamped_groundtruth.txt -v --align --n_to_align 116 --plot --plot_mode=xyz
# stamped_traj_estimate.txt



#################*********************davis240c********************************##############
# gnome-terminal --tab -e 'bash -c "roslaunch evio_estimator davis240c.launch;exec bash"'
# # # # sleep 3s
# # # #####################ata set for ETH (davis240c)
# gnome-terminal --window -e 'bash -c "rosbag play --pause ~/dataset/hku_event_camera/hku_dataset_davis240c/dynamic_translation_hku.bag;exec bash"'
# gnome-terminal --tab -e 'bash -c "roslaunch evio davis240c_evaluation.launch;exec bash"'

# evo_ape bag davis240c.bag /optitrack/davis /pose_graph/imu_evio_loop -v --align --n_to_align 1000
# evo_ape bag davis240c.bag /optitrack/davis /pose_graph/imu_evio_loop -v --align_origin
# evo_traj bag davis240c.bag /pose_graph/imu_evio_loop --ref=/optitrack/davis -v --align_origin  --plot --plot_mode=xyz
# evo_traj bag davis240c.bag /pose_graph/imu_evio_loop --ref=/optitrack/davis -v --align --n_to_align 1000 --plot --plot_mode=xyz

# python3 /home/kwanwaipang/catkin_ws/src/rpg_trajectory_evaluation/scripts/dataset_tools/bag_to_pose.py davis240c.bag /pose_graph/imu_evio_loop --msg_type PoseStamped --output stamped_traj_estimate.txt
# python3 /home/kwanwaipang/catkin_ws/src/rpg_trajectory_evaluation/scripts/dataset_tools/bag_to_pose.py davis240c.bag /optitrack/davis --msg_type PoseStamped --output stamped_groundtruth.txt
# python3 /home/kwanwaipang/catkin_ws/src/rpg_trajectory_evaluation/scripts/analyze_trajectory_single.py /home/kwanwaipang/dataset/ours_result_davis240c/with_loop/boxes_6dof_hku


#################*********************change the dataset of davis240c********************************##############
# gnome-terminal --tab -e 'bash -c "rosparam set use_sim_time true;exec bash"'
# sleep 3s
# gnome-terminal --tab -e 'bash -c "rosrun evio vins_node;exec bash"'
# sleep 3s
# gnome-terminal --tab -e 'bash -c "roslaunch evio davis240c_recording.launch;exec bash"'
# sleep 3s
# gnome-terminal --window -e 'bash -c "rosbag play --clock --pause ~/dataset/eth_dataset_davis240c/shapes_translation.bag;exec bash"'




sleep 16s
#################*********************debug********************************##############
# gnome-terminal --tab -e 'bash -c "rosrun --prefix \"gdb -ex run --args\" evio vins_node ~/catkin_ws_dvs/src/EVIO/config/davis_346/davis_346_imu_config.yaml;exec bash"'


