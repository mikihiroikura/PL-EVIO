#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

extern ros::Publisher pub_loop_image;
extern ros::Publisher pub_img,pub_match, pub_match_two;
extern ros::Publisher pub_time_surface;
extern ros::Publisher pub_restart;
extern ros::Publisher corner_pub;//把当前事件帧的所有feature发布出去

extern ros::Publisher pub_match_two_line,pub_feature_line;//线特征相关的

void registerPub(ros::NodeHandle &n);
void pubLoopImage(const cv::Mat &imgTrack, const double t);

