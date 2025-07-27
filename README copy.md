# EVIO (noetic版本)   based on vins-mono  
https://github.com/arclab-hku/EVIO   

## 配置过程直接运行
```
$  mkdir -p catkin_ws_dvs/src

$ cd catkin_ws_dvs

$ catkin config --init --mkdirs --extend /opt/ros/kinetic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release (for kinetic)

$ catkin config --init --mkdirs --extend /opt/ros/melodic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release (for melodic)

$ catkin config --init --mkdirs --extend /opt/ros/noetic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release (for noetic)

$ sudo apt-get install python3-vcstool

$ vcs-import < EVIO/dependencies.yaml

# .bashrc文件中应该加入：
# alias eviobuild='cd /home/kwanwaipang/catkin_ws_dvs/src && catkin build evio -DCMAKE_BUILD_TYPE=Release'
alias eviobuild='cd /home/kwanwaipang/catkin_ws_dvs/src && catkin build evio_estimator feature_tracker pose_graph -DCMAKE_BUILD_TYPE=Release'
alias eviorun='cd /home/kwanwaipang/catkin_ws_dvs/src/EVIO/script && sh run.sh'
alias cm='cd ~/catkin_ws && catkin_make' 

$ cd catkin_ws_dvs/src

$ catkin build dvs_render  

$ eviobuild

$ eviorun  (运行代码。记得修改rosbag的位置，详细见“run.sh”)
```
1. 需要拉取上面的依赖并且编译dvs_render  
2. 数据集的存放路径要修改：~/dataset/gwphku/hku_davis346_2021-10-29-20-19-47.bag
3. 千万千万不要安装 https://github.com/uzh-rpg/rpg_dvs_ros

编译前试试下面的：
'''
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

catkin build vins -DCMAKE_BUILD_TYPE=Release
'''

## 参考
1. token for evio: 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
2. VINS-FUSION   https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
3. VINS-Mono的解析https://github.com/xieqi1/VINS-Mono-noted
4. ESVO：https://github.com/HKUST-Aerial-Robotics/ESVO  
5. kalibra校正记录：https://blog.csdn.net/gwplovekimi/article/details/120948986
6. opencv中图像增强：https://www.cnblogs.com/jukan/p/7815722.html
7. 验证的数据集：http://rpg.ifi.uzh.ch/davis_data.html
8. (mvsec)https://daniilidis-group.github.io/mvsec/download/
9. (DSEC) https://dsec.ifi.uzh.ch/
10. 获取eth的结果http://rpg.ifi.uzh.ch/ultimateslam.html

## 任务


## 进度计划


'''
python2 /home/kwanwaipang/catkin_ws/src/rpg_trajectory_evaluation/scripts/dataset_tools/bag_to_pose.py /home/kwanwaipang/dataset/gwphku/evaluation_boxes_translation_2021-11-17-16-56-16.bag /evio_estimator/evio_pose --msg_type PoseStamped --output stamped_traj_estimate.txt

python2 /home/kwanwaipang/catkin_ws/src/rpg_trajectory_evaluation/scripts/dataset_tools/bag_to_pose.py /home/kwanwaipang/dataset/gwphku/evaluation_boxes_translation_2021-11-17-16-56-16.bag /optitrack/davis --msg_type PoseStamped --output stamped_groundtruth.txt

# 验证

python3 /home/kwanwaipang/catkin_ws/src/rpg_trajectory_evaluation/scripts/analyze_trajectory_single.py /home/kwanwaipang/dataset/ours_result_davis240c/with_loop/boxes_translation_hku

# evo包验证
evo_ape tum stamped_groundtruth.txt stamped_traj_estimate.txt -v --align_origin

evo_ape tum stamped_groundtruth.txt stamped_traj_estimate.txt -v --align_origin -r angle_deg 

evo_traj tum stamped_traj_estimate.txt --ref=stamped_groundtruth.txt -v --align_origin --plot --plot_mode=xyz
###############################################################################################################
evo_ape bag davis240c.bag /optitrack/davis /pose_graph/imu_evio_loop -va --plot --plot_mode=xyz

evo_ape bag davis240c.bag /optitrack/davis /pose_graph/imu_evio_loop -v --align_origin --plot --plot_mode=xyz -r angle_deg

evo_traj bag davis240c.bag /pose_graph/imu_evio_loop --ref=/optitrack/davis -v --align_origin  --plot --plot_mode=xyz


# 转换csv到txt
python3 /home/kwanwaipang/catkin_ws/src/rpg_trajectory_evaluation/scripts/dataset_tools/asl_groundtruth_to_pose.py evio_result_loop.csv

python3 /home/kwanwaipang/catkin_ws/src/rpg_trajectory_evaluation/scripts/dataset_tools/asl_groundtruth_to_pose.py traj_gt.csv



# 多视觉几何（计算pose）、惯性传感器（IMU预积分在integration_base.h）

# 标定的过程 （思路，后面做精度测试的时候，必须做这步）
1. 使用Kalibr标定双目的内外参数
2. 使用IMU标定工具（如kalibr_allan和imu_utils）标定出IMU的内参数
3. 使用Kalibr标定IMU与双目之间的外参数（需要用到imu的内参书，连续时间下）
4. 将标定的结果写入VINS-FUSION的配置文件中（注意IMU内参连续时间到离散时间的转换）



# 其他记录#
'''
### EventArray相对于Event的区别主要是：
Header header
uint32 height         # image height, that is, number of rows
 uint32 width          # image width, that is, number of columns
    // # an array of events
Event[] events

//# A DVS event
uint16 x
uint16 y
 time ts
bool polarity
'''

### calcOpticalFlowPyrLK光流的应用
cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
 //prev_img, cur_img, prev_pts,三个都是前面给定的
//所以应该得到的是cur_pts，status就是匹配的效果
 //cur_pts输出二维点的矢量（具有单精度浮点坐标），包含第二图像中输入特征的计算新位置;
// 当传递OPTFLOW_USE_INITIAL_FLOW标志时，向量cur_pts必须与输入中的大小相同。（但此处没有输入，前面根后面有）
//status输出状态向量（无符号字符）;如果找到相应特征的流，则向量的每个元素设置为1，否则设置为0。
//err ：输出错误的矢量; 向量的每个元素都设置为相应特征的错误，错误度量的类型可以在flags参数中设置; 如果未找到流，则未定义错误（使用status参数查找此类情况）
'''
prev_img   //第一个8位输入图像或金字塔（）
cur_img    与prevImg相同大小和相同类型的第二个输入图像或金字塔
prev_pts 需要找到流的2D点的矢量，点坐标必须是单精度浮点数 （也就是上一帧的点）
cur_pts ：输出二维点的矢量（具有单精度浮点坐标），包含第二图像中输入特征的计算新位置;可以理解为，当前帧匹配到的点的数目。
status ：输出状态向量（无符号字符）;如果找到相应特征的流，则向量的每个元素设置为1，否则设置为0。然后根据匹配的情况，匹配不成功的点去掉
err ：输出错误的矢量; 向量的每个元素都设置为相应特征的错误，错误度量的类型可以在flags参数中设置; 如果未找到流，则未定义错误（使用status参数查找此类情况） 代码中并无真正使用
winSize--> 在计算局部连续运动的窗口尺寸（在图像金字塔中）,default=Size(21, 21);
maxLevel--> 图像金字塔层数，0表示不使用金字塔, default=3;
'''

### goodFeaturesToTrack的应用
https://blog.csdn.net/xdfyoga1/article/details/44175637
void cv::goodFeaturesToTrack(
		cv::InputArray image, // 输入图像（CV_8UC1 CV_32FC1）
		cv::OutputArray corners, // 输出角点vector
		int maxCorners, // 最大角点数目
		double qualityLevel, // 质量水平系数（小于1.0的正数，一般在0.01-0.1之间）
		double minDistance, // 最小距离，小于此距离的点忽略
		cv::InputArray mask = noArray(), // mask=0的点忽略
		int blockSize = 3, // 使用的邻域数
		bool useHarrisDetector = false, // false ='Shi Tomasi metric'
		double k = 0.04 // Harris角点检测时使用
	);

### resize 的使用，解决图像size不一致的问题
https://www.cnblogs.com/tcysky/p/6215784.html

# 初始化git #
echo "# EVIO" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/KwanWaiPang/EVIO.git
git push -u origin main

# EVIO
