#include "visualization.h"
// #include <dvs_msgs/Event.h>
// #include <dvs_msgs/EventArray.h>
#include "../dvs_msgs/Event.h"
#include "../dvs_msgs/EventArray.h"

using namespace ros;
using namespace Eigen;
ros::Publisher pub_loop_image;
ros::Publisher pub_img,pub_match, pub_match_two;
ros::Publisher pub_time_surface;
ros::Publisher pub_restart;
ros::Publisher corner_pub;//把当前事件帧的所有feature发布出去
ros::Publisher pub_match_two_line,pub_feature_line;//线特征相关的

void registerPub(ros::NodeHandle &n)
{
    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);//发布feature  (实际发出去的是 /feature_tracker/feature)
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);//发布feature tracking的image
    pub_match_two=n.advertise<sensor_msgs::Image>("feature_img_two",1000);
    pub_time_surface = n.advertise<sensor_msgs::Image>("timesurface_map", 1000);//发布timesurface
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);

    corner_pub= n.advertise<dvs_msgs::EventArray>("event_feature_loop",1000);//把当前事件信息的所有角点发布到pose graph用于回环检测

    pub_loop_image = n.advertise<sensor_msgs::Image>("loop_image", 1000);//发布timesurface

    pub_match_two_line=n.advertise<sensor_msgs::Image>("linefeature_img_two",1000);//发布两张图片叠在一起的线特征matching的结果
    pub_feature_line = n.advertise<sensor_msgs::PointCloud>("linefeature",     1000);//发布线feature(当前帧的特征)

}

/**
 * @brief 发布用于回环检测的
 */
void pubLoopImage(const cv::Mat &timesurface, const double t)
{//发布回环用的图像

    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(t);

    sensor_msgs::ImagePtr TimeSurfaceImg = cv_bridge::CvImage(header, "mono8", timesurface).toImageMsg();
    pub_loop_image.publish(TimeSurfaceImg);
}
