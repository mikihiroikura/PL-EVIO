#include "feature_manager.h"

Eigen::Matrix2d stereo_FeaturePerFrame::getOmega() // inverse of covariance matrix
{
    Eigen::Matrix2d sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();

    if(getDepth() <= 0) {
        ROS_ERROR("feature_manager.cpp: something is wrong!"); 
        return sqrt_info; 
    }

    if(!g_use_stereo_correction)
        return sqrt_info; 

    if(!gc_succeed)
        return sqrt_info; 

    /*// Chapter 6 "Statistical Optimization for Geometric Computation"
    double epsilon = 1.5 / FOCAL_LENGTH; 
    
    Eigen::Matrix3d R = Rrl.transpose();
    Vector3d h = Trl;  
    Eigen::Matrix3d G = Utility::skewSymmetric(h); 
    Eigen::Matrix3d Gt = G.transpose(); 
    Eigen::Matrix3d Pk = Eigen::Matrix3d::Identity(); 
    Pk(2,2) = 0; 
    Eigen::Vector3d x1 = Pk*G*pointRight;
    Eigen::Matrix3d XX = x1*x1.transpose(); 
    Eigen::Vector3d x2 = Pk*Gt*point; 
    double de = x1.squaredNorm() + x2.squaredNorm(); 
    Eigen::Matrix2d XX2 = XX.block<2,2>(0,0)/de; 
    Eigen::Matrix2d cov = Eigen::Matrix2d::Identity() - XX2; 
    Eigen::Matrix2d cc = Eigen::Matrix2d::Zero(); 
    cc(0,0) = sqrt(cov(0,0)); 
    cc(1,1) = sqrt(cov(1,1)); 
    cov = epsilon * cc; 

    sqrt_info = cov.inverse();*/
    return sqrt_info; 
}

Eigen::Matrix2d stereo_FeaturePerFrame::getOmegaRight() // inverse of covariance matrix
{
    Eigen::Matrix2d sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();

    if(getDepth() <= 0) {
        ROS_ERROR("feature_manager.cpp: something is wrong!"); 
        return sqrt_info; 
    }

    double depth = getDepth(); 
    double weighting = -SQ(depth - 1)/81. + 1.; 
    if(weighting <0 ) weighting = 0.01;

    if(!g_use_stereo_correction)
        return weighting*sqrt_info; 

    /*// Chapter 6 "Statistical Optimization for Geometric Computation"
    double epsilon = 1.5 / FOCAL_LENGTH; 
    
    Eigen::Matrix3d R = Rrl.transpose();
    Vector3d h = Trl;  
    Eigen::Matrix3d G = Utility::skewSymmetric(h); 
    Eigen::Matrix3d Gt = G.transpose(); 
    Eigen::Matrix3d Pk = Eigen::Matrix3d::Identity(); 
    Pk(2,2) = 0; 
    Eigen::Vector3d x1 = Pk*G*pointRight;
    Eigen::Vector3d x2 = Pk*Gt*point; 
    Eigen::Matrix3d XX = x2*x2.transpose(); 
    double de = x1.squaredNorm() + x2.squaredNorm(); 
    Eigen::Matrix2d XX2 = XX.block<2,2>(0,0)/de; 
    Eigen::Matrix2d cov = Eigen::Matrix2d::Identity() - XX2; 
    Eigen::Matrix2d cc = Eigen::Matrix2d::Zero(); 
    cc(0,0) = sqrt(cov(0,0)); 
    cc(1,1) = sqrt(cov(1,1)); 
    cov = epsilon * cc; 

    sqrt_info = cov.inverse(); */
    return weighting*sqrt_info; 
}

// triangulation to compute depth 
double stereo_FeaturePerFrame::getDepth()
{
    if(!is_stereo) return -1; 
    if(dpt > 0) return dpt; 
    if(point(0) < pointRight(0)){
        is_stereo = false; // this is false stereo match, since depth < 0 
        dpt = -1; 
        return -1; 
    }

    // check the triangulated depth 
    Eigen::Matrix<double, 3, 4> leftPose = Eigen::Matrix<double, 3, 4>::Zero();
    leftPose.leftCols<3>() = Eigen::Matrix3d::Identity(); 

    Eigen::Matrix<double, 3, 4> rightPose = Eigen::Matrix<double, 3, 4>::Zero(); 
    rightPose.leftCols<3>() = Rrl; 
    rightPose.rightCols<1>() = Trl; 

    Eigen::Vector2d point0, point1; 
    Eigen::Vector3d point3d; 
    point0 = point.head(2); 
    point1 = pointRight.head(2); 

    ((FeatureManager*)0)->triangulatePoint(leftPose, rightPose, point0, point1, point3d); 

    double depth = point3d.z(); 
    // if(depth <= 0.3 || depth >= 7){ // too small or too large depth, not reliable 
    // TODO: find out optimal threshold for distant point 
    if(depth <= 1. || depth >= 7.){ // barely no object so close to the camera 
        // ROS_ERROR("feature_manager.cpp: what? depth: %lf", depth); 
        // cout<<"feature_id: "<< feat_id <<"point0: "<<point0.transpose()<<" point1: "<<point1.transpose()<<" point3d: "<<point3d.transpose()<<endl;
        is_stereo = false; 
        dpt = -1; 

        return -1; 
    }

    // check reprojection error 
    Eigen::Vector3d proj_pt0, proj_pt1; 
    proj_pt0 = point3d / depth; 
    proj_pt1 = Rrl * point3d + Trl; 
    if(proj_pt1.z()<= 1.){
        is_stereo = false; 
        dpt = -1; 
        return -1; 
    }
    proj_pt1 = proj_pt1/proj_pt1.z(); 

    Eigen::Vector2d err_pt0 = proj_pt0.head(2) - point0; 
    Eigen::Vector2d err_pt1 = proj_pt1.head(2) - point1; 

    double ep0_norm = err_pt0.norm(); 
    double ep1_norm = err_pt1.norm(); 

    if((err_pt0.norm() > 2./FOCAL_LENGTH) || (err_pt1.norm() > 2./FOCAL_LENGTH)){
        is_stereo = false; 
        dpt = -1; 
   
        // for debug 
        // static int cnt =0; 
        // ROS_ERROR("really, we have %d wrong triangulations err_pt0: %lf  err_pt1: %lf", ++cnt, err_pt0.norm(), err_pt1.norm()); 
        return - 1; 
    }

    dpt = depth; 

    if(!g_use_stereo_correction) // don't apply geometric correction 
        return dpt; 

    // stereo correction, Chapter 6 "Statistical Optimization for Geometric Computation"
    Vector3d np0 = point; 
    Vector3d np1 = pointRight; 

    Eigen::Matrix3d R = Rrl.transpose();
    Vector3d h = Trl;  
    Eigen::Matrix3d G = Utility::skewSymmetric(h)*R; 
    Eigen::Matrix3d Gt = G.transpose(); 
    Eigen::Matrix3d Pk = Eigen::Matrix3d::Identity(); 
    Pk(2,2) = 0; 

    double fe = np0.transpose() * G * np1; 
    // double de1 = np1.transpose() * Gt * G * np1; 
    // double de2 = np0.transpose() * G * Gt * np0;
    // double de = de1 + de2; 
    Vector3d ve1 =  Pk*Gt*np0;
    Vector3d ve2 =  Pk*G*np1;
    double de = ve1.squaredNorm() + ve2.squaredNorm();
    Vector3d delta_np0 = fe * Pk * G * np1; 
    Vector3d delta_np1 = fe * Pk * Gt * np0; 

    // temporary solution, needs to improve 
    // TODO: update uv and uvRight as well 

    Vector3d new_p0 = np0 - delta_np0/de; 
    Vector3d new_p1 = np1 - delta_np1/de; 

    point0 = new_p0.head(2); 
    point1 = new_p1.head(2); 

    ((FeatureManager*)0)->triangulatePoint(leftPose, rightPose, point0, point1, point3d); 
    
    // check reprojection error 
    proj_pt0 = point3d / point3d.z(); 
    proj_pt1 = Rrl * point3d + Trl; 
    if(proj_pt1.z()<= 1.){
        is_stereo = false; 
        dpt = -1; 
        return -1; 
    }
    proj_pt1 = proj_pt1/proj_pt1.z(); 

    err_pt0 = proj_pt0.head(2) - point0; 
    err_pt1 = proj_pt1.head(2) - point1; 
    double new_ep0_norm = err_pt0.norm();
    double new_ep1_norm = err_pt1.norm(); 
    if(new_ep0_norm > ep0_norm || new_ep1_norm > ep1_norm){
        gc_succeed = false; 
        return depth ; 
    }

    point = new_p0; 
    pointRight = new_p1; 
    depth = point3d.z(); 
    dpt = depth; 
    gc_succeed = true; 
    return depth; 
}

int stereo_FeaturePerId::endFrame()//点特征的
{
    return start_frame + feature_per_frame.size() - 1;
}

int FeaturePerId::endFrame()//点特征的
{
    return start_frame + feature_per_frame.size() - 1;
}

int Event_FeaturePerId::endFrame()//事件点特征的
{
    return start_frame + Event_feature_per_frame.size() - 1;
}


int lineFeaturePerId::endFrame()//线特征的
{
    return start_frame + linefeature_per_frame.size() - 1;
}

//*********************************************************************************************************
FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    feature.clear();//点特征清空
    stereo_feature.clear();//双目点特征清空
    Event_feature.clear();//事件点特征
}

int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature)
    {

        it.used_num = it.feature_per_frame.size();

        if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2)
        {
            cnt++;
        }
    }
    return cnt;
}

int FeatureManager::stereo_getFeatureCount()
{
    int cnt = 0;
    for (auto &it : stereo_feature)
    {

        it.used_num = it.feature_per_frame.size();

        if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2)
        {
            cnt++;
        }
    }
    return cnt;
}

int FeatureManager::getEventFeatureCount()
{
    int cnt = 0;
    for (auto &it : Event_feature)
    {

        it.used_num = it.Event_feature_per_frame.size();

        if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2)
        {
            cnt++;
        }
    }
    return cnt;
}


int FeatureManager::getLineFeatureCount()//统计线特征的数量
{
    int cnt = 0;
    for (auto &it : linefeature)
    {

        it.used_num = it.linefeature_per_frame.size();

        if (it.used_num >= LINE_MIN_OBS && it.start_frame < WINDOW_SIZE - 2 && it.is_triangulation)
        {
            cnt++;
        }
    }
    return cnt;
}

bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td)
{
    ROS_DEBUG("input feature: %d", (int)image.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    for (auto &id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });

        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
        }
    }

    if (frame_count < 2 || last_track_num < 20)
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const map<int, vector<pair<int, Vector4d>>> &lines, double td)
{
    // ROS_DEBUG("input feature: %d", (int)image.size());
    // ROS_DEBUG("num of feature: %d", getFeatureCount());
    // ROS_WARN("input feature: %d", (int)image.size());
    // ROS_WARN("input line feature: %d", (int)lines.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount()+getLineFeatureCount()); // 已有的点特征与线特征的数目

    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    //遍历点特征
    for (auto &id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });

        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
        }
    }

    for (auto &id_line : lines)   //遍历当前帧上的线特征
    {
        lineFeaturePerFrame f_per_fra(id_line.second[0].second);  // 观测

        int feature_id = id_line.first;
        //cout << "line id: "<< feature_id << "\n";
        auto it = find_if(linefeature.begin(), linefeature.end(), [feature_id](const lineFeaturePerId &it)
        {
            return it.feature_id == feature_id;    // 在feature里找id号为feature_id的特征
        });

        if (it == linefeature.end())  // 如果之前没存这个特征，说明是新的
        {
            linefeature.push_back(lineFeaturePerId(feature_id, frame_count));
            linefeature.back().linefeature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)//如果存在则在相应id的线特征中添加当前帧的观测
        {
            it->linefeature_per_frame.push_back(f_per_fra);
            it->all_obs_cnt++;//一共观测了多少次
            // last_track_num++;//（检测视差加入线特征数量的统计）
        }
    }

    if (frame_count < 2 || last_track_num < 20)
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

//同时加入图像特征、事件点特征、事件线特征
bool FeatureManager::addFeatureCheckParallax(int frame_count, 
                                             const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, //图像特征
                                             const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &event, //事件点特征
                                             const map<int, vector<pair<int, Vector4d>>> &lines, //事件线特征
                                             double td)
{
    // ROS_WARN("input feature------image feature: %d, event feature: %d, event line feature: %d", (int)image.size(), (int)event.size(), (int)lines.size());
    // ROS_WARN("useful------image feature: %d, event feature: %d, event line feature: %d", getFeatureCount(), getEventFeatureCount(), getLineFeatureCount());
    // ROS_DEBUG("num of feature: %d", getFeatureCount()+getEventFeatureCount()+getLineFeatureCount()); // 已有的图像点特征+事件点特征与线特征的数目

    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    //遍历图像点特征
    for (auto &id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });

        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
        }
    }

    //遍历事件点特征
    for (auto &id_pts : event)
    {
        Event_FeaturePerFrame f_per_fra(id_pts.second[0].second, td);//当有image的时候，额外多定义了一个Event_FeaturePerFrame，实际内容跟原来的一样，只是名字变了

        int feature_id = id_pts.first;//获取ID
        auto it = find_if(Event_feature.begin(), Event_feature.end(), [feature_id](const Event_FeaturePerId &it)
                          {
            return it.feature_id == feature_id; // 在feature里找id号为feature_id的特征
                          });

        if (it == Event_feature.end()) // 如果之前没存这个特征，说明是新的
        {
            Event_feature.push_back(Event_FeaturePerId(feature_id, frame_count));
            Event_feature.back().Event_feature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)//如果有找到，则增加观测数量
        {
            it->Event_feature_per_frame.push_back(f_per_fra);
            // last_track_num++;//是否需要事件点特征来辅助进行关键帧的选择？
        }
    }

    //遍历当前帧上的线特征
    for (auto &id_line : lines) 
    {
        lineFeaturePerFrame f_per_fra(id_line.second[0].second);  // 观测

        int feature_id = id_line.first;
        //cout << "line id: "<< feature_id << "\n";
        auto it = find_if(linefeature.begin(), linefeature.end(), [feature_id](const lineFeaturePerId &it)
        {
            return it.feature_id == feature_id;    // 在feature里找id号为feature_id的特征
        });

        if (it == linefeature.end())  // 如果之前没存这个特征，说明是新的
        {
            linefeature.push_back(lineFeaturePerId(feature_id, frame_count));
            linefeature.back().linefeature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)//如果存在则在相应id的线特征中添加当前帧的观测
        {
            it->linefeature_per_frame.push_back(f_per_fra);
            it->all_obs_cnt++;//一共观测了多少次
            // last_track_num++;//（检测视差加入线特征数量的统计）
        }
    }

    if (frame_count < 2 || last_track_num < 20)
        return true;

    for (auto &it_per_id : feature)//图像特征(仅仅采用图像特征)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

// 双目
//同时加入图像特征、事件点特征、事件线特征
bool FeatureManager::stereo_addFeatureCheckParallax(int frame_count, 
                                             const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, //图像特征
                                             const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &event, //事件点特征
                                             const map<int, vector<pair<int, Vector4d>>> &lines, //事件线特征
                                             double td)
{
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    last_average_parallax = 0;
    long_track_num = 0; 
    new_feature_num = 0; 
    //遍历图像点特征
    for (auto &id_pts : image)
    {
        stereo_FeaturePerFrame f_per_fra(id_pts.second[0].second, td);
        f_per_fra.feat_id = id_pts.first; 
        if(id_pts.second[0].first != 0)
            ROS_ERROR("what? second[0].first = %d", id_pts.second[0].first );
        assert(id_pts.second[0].first == 0);
        if(id_pts.second.size() == 2)
        {
            f_per_fra.rightObservation(id_pts.second[1].second);//右边的观测
            assert(id_pts.second[1].first == 1);
        }

        int feature_id = id_pts.first;//获取特征点的id
        auto it = find_if(stereo_feature.begin(), stereo_feature.end(), [feature_id](const stereo_FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });

        if (it == stereo_feature.end())
        {
            stereo_feature.push_back(stereo_FeaturePerId(feature_id, frame_count));
            stereo_feature.back().feature_per_frame.push_back(f_per_fra);
            new_feature_num++;
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
            if( it-> feature_per_frame.size() >= 4)
                long_track_num++;
        }
    }

    //遍历事件点特征
    for (auto &id_pts : event)
    {
        Event_FeaturePerFrame f_per_fra(id_pts.second[0].second, td);//当有image的时候，额外多定义了一个Event_FeaturePerFrame，实际内容跟原来的一样，只是名字变了

        int feature_id = id_pts.first;//获取ID
        auto it = find_if(Event_feature.begin(), Event_feature.end(), [feature_id](const Event_FeaturePerId &it)
                          {
            return it.feature_id == feature_id; // 在feature里找id号为feature_id的特征
                          });

        if (it == Event_feature.end()) // 如果之前没存这个特征，说明是新的
        {
            Event_feature.push_back(Event_FeaturePerId(feature_id, frame_count));
            Event_feature.back().Event_feature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)//如果有找到，则增加观测数量
        {
            it->Event_feature_per_frame.push_back(f_per_fra);
            // last_track_num++;//是否需要事件点特征来辅助进行关键帧的选择？
        }
    }

    //遍历当前帧上的线特征
    for (auto &id_line : lines) 
    {
        lineFeaturePerFrame f_per_fra(id_line.second[0].second);  // 观测

        int feature_id = id_line.first;
        //cout << "line id: "<< feature_id << "\n";
        auto it = find_if(linefeature.begin(), linefeature.end(), [feature_id](const lineFeaturePerId &it)
        {
            return it.feature_id == feature_id;    // 在feature里找id号为feature_id的特征
        });

        if (it == linefeature.end())  // 如果之前没存这个特征，说明是新的
        {
            linefeature.push_back(lineFeaturePerId(feature_id, frame_count));
            linefeature.back().linefeature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)//如果存在则在相应id的线特征中添加当前帧的观测
        {
            it->linefeature_per_frame.push_back(f_per_fra);
            it->all_obs_cnt++;//一共观测了多少次
            // last_track_num++;//（检测视差加入线特征数量的统计）
        }
    }

    // if (frame_count < 2 || last_track_num < 20 || long_track_num < 40 || new_feature_num > 0.5 * last_track_num)
    //    return true;
    if (frame_count < 2 || last_track_num < 20)
        return true;

    for (auto &it_per_id : stereo_feature)//图像特征(仅仅采用图像特征)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

void FeatureManager::debugShow()
{
    ROS_DEBUG("debug show");
    for (auto &it : feature)
    {
        ROS_ASSERT(it.feature_per_frame.size() != 0);
        ROS_ASSERT(it.start_frame >= 0);
        ROS_ASSERT(it.used_num >= 0);

        ROS_DEBUG("%d,%d,%d ", it.feature_id, it.used_num, it.start_frame);
        int sum = 0;
        for (auto &j : it.feature_per_frame)
        {
            ROS_DEBUG("%d,", int(j.is_used));
            sum += j.is_used;
            printf("(%lf,%lf) ",j.point(0), j.point(1));
        }
        ROS_ASSERT(it.used_num == sum);
    }
}

//只有初始化IMU与视觉对齐的时候需要用
vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

vector<pair<Vector3d, Vector3d>> FeatureManager::getCorrespondingWithDepth(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : stereo_feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;
            a.z() = it.feature_per_frame[idx_l].getDepth(); 

            b = it.feature_per_frame[idx_r].point;
            b.z() = it.feature_per_frame[idx_r].getDepth();
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}

void FeatureManager::stereo_setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : stereo_feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}

void FeatureManager::Event_setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : Event_feature)
    {
        it_per_id.used_num = it_per_id.Event_feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}

void FeatureManager::removeFailures()
{
    //对于图像特征点
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            feature.erase(it);
    }

    //对于双目的情况
    for (auto it = stereo_feature.begin(), it_next = stereo_feature.begin();
    it != stereo_feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            stereo_feature.erase(it);
    }

    // 对于事件特征点
    for (auto it = Event_feature.begin(), it_next = Event_feature.begin();
         it != Event_feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            Event_feature.erase(it);//删除对应的点
    }
}

//只有初始化视觉跟IMU的时候才需要
void FeatureManager::clearDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
    }

    //对于双目
    for (auto &it_per_id : stereo_feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
    }
}

VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}

VectorXd FeatureManager::stereo_getDepthVector()
{
    VectorXd dep_vec(stereo_getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : stereo_feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}

//对事件特征的深度进行设置
VectorXd FeatureManager::Event_getDepthVector()
{
    VectorXd dep_vec(getEventFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : Event_feature)
    {
        it_per_id.used_num = it_per_id.Event_feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}

// 接下来是事件线特征的
MatrixXd FeatureManager::getLineOrthVectorInCamera()
{
    MatrixXd lineorth_vec(getLineFeatureCount(),4);
    int feature_index = -1;
    for (auto &it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;

        lineorth_vec.row(++feature_index) = plk_to_orth(it_per_id.line_plucker);

    }
    return lineorth_vec;
}

void FeatureManager::setLineOrthInCamera(MatrixXd x)
{
    int feature_index = -1;
    for (auto &it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;

        //std::cout<<"x:"<<x.rows() <<" "<<feature_index<<"\n";
        Vector4d line_orth = x.row(++feature_index);
        it_per_id.line_plucker = orth_to_plk(line_orth);// transfrom to camera frame

        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        /*
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
         */
    }
}

MatrixXd FeatureManager::getLineOrthVector(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    MatrixXd lineorth_vec(getLineFeatureCount(),4);
    int feature_index = -1;
    for (auto &it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;

        int imu_i = it_per_id.start_frame;

        ROS_ASSERT(NUM_OF_CAM == 1);

        Eigen::Vector3d twc = Ps[imu_i] + Rs[imu_i] * tic[0];   // twc = Rwi * tic + twi
        Eigen::Matrix3d Rwc = Rs[imu_i] * ric[0];               // Rwc = Rwi * Ric

        Vector6d line_w = plk_to_pose(it_per_id.line_plucker, Rwc, twc);  // transfrom to world frame
        // line_w.normalize();
        lineorth_vec.row(++feature_index) = plk_to_orth(line_w);
        //lineorth_vec.row(++feature_index) = plk_to_orth(it_per_id.line_plucker);

    }
    return lineorth_vec;
}

void FeatureManager::setLineOrth(MatrixXd x,Vector3d P[], Matrix3d R[], Vector3d tic[], Matrix3d ric[])
{
    int feature_index = -1;
    for (auto &it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;

        Vector4d line_orth_w = x.row(++feature_index);
        Vector6d line_w = orth_to_plk(line_orth_w);

        int imu_i = it_per_id.start_frame;
        ROS_ASSERT(NUM_OF_CAM == 1);

        Eigen::Vector3d twc = P[imu_i] + R[imu_i] * tic[0];   // twc = Rwi * tic + twi
        Eigen::Matrix3d Rwc = R[imu_i] * ric[0];               // Rwc = Rwi * Ric

        it_per_id.line_plucker = plk_from_pose(line_w, Rwc, twc); // transfrom to camera frame
        //it_per_id.line_plucker = line_w; // transfrom to camera frame

        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        /*
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
         */
    }
}

double FeatureManager::reprojection_error( Vector4d obs, Matrix3d Rwc, Vector3d twc, Vector6d line_w ) {

    double error = 0;

    Vector3d n_w, d_w;
    n_w = line_w.head(3);
    d_w = line_w.tail(3);

    Vector3d p1, p2;
    p1 << obs[0], obs[1], 1;
    p2 << obs[2], obs[3], 1;

    Vector6d line_c = plk_from_pose(line_w,Rwc,twc);
    Vector3d nc = line_c.head(3);
    double sql = nc.head(2).norm();
    nc /= sql;

    error += fabs( nc.dot(p1) );
    error += fabs( nc.dot(p2) );

    return error / 2.0;
}

//线特征的三角化
void FeatureManager::triangulateLine(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    //std::cout<<"linefeature size: "<<linefeature.size()<<std::endl;
    for (auto &it_per_id : linefeature)        // 遍历每个特征，对新特征进行三角化
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();    // 已经有多少帧看到了这个特征
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2))   // 看到的帧数少于2， 或者 这个特征最近倒数第二帧才看到， 那都不三角化
            continue;

        if (it_per_id.is_triangulation)       // 如果已经三角化了
            continue;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        ROS_ASSERT(NUM_OF_CAM == 1);

        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];   // twc = Rwi * tic + twi
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];               // Rwc = Rwi * Ric

        double d = 0, min_cos_theta = 1.0;
        Eigen::Vector3d tij;
        Eigen::Matrix3d Rij;
        Eigen::Vector4d obsi,obsj;  // obs from two frame are used to do triangulation

        // plane pi from ith obs in ith camera frame
        Eigen::Vector4d pii;
        Eigen::Vector3d ni;      // normal vector of plane    
        for (auto &it_per_frame : it_per_id.linefeature_per_frame)   // 遍历所有的观测， 注意 start_frame 也会被遍历
        {
            imu_j++;

            if(imu_j == imu_i)   // 第一个观测是start frame 上
            {
                obsi = it_per_frame.lineobs;
                Eigen::Vector3d p1( obsi(0), obsi(1), 1 );
                Eigen::Vector3d p2( obsi(2), obsi(3), 1 );
                pii = pi_from_ppp(p1, p2,Vector3d( 0, 0, 0 ));
                ni = pii.head(3); ni.normalize();
                continue;
            }

            // 非start frame(其他帧)上的观测
            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];

            Eigen::Vector3d t = R0.transpose() * (t1 - t0);   // tij
            Eigen::Matrix3d R = R0.transpose() * R1;          // Rij
            
            Eigen::Vector4d obsj_tmp = it_per_frame.lineobs;

            // plane pi from jth obs in ith camera frame
            Vector3d p3( obsj_tmp(0), obsj_tmp(1), 1 );
            Vector3d p4( obsj_tmp(2), obsj_tmp(3), 1 );
            p3 = R * p3 + t;
            p4 = R * p4 + t;
            Vector4d pij = pi_from_ppp(p3, p4,t);
            Eigen::Vector3d nj = pij.head(3); nj.normalize(); 

            double cos_theta = ni.dot(nj);
            if(cos_theta < min_cos_theta)
            {
                min_cos_theta = cos_theta;
                tij = t;
                Rij = R;
                obsj = obsj_tmp;
                d = t.norm();
            }
            // if( d < t.norm() )  // 选择最远的那俩帧进行三角化
            // {
            //     d = t.norm();
            //     tij = t;
            //     Rij = R;
            //     obsj = it_per_frame.lineobs;      // 特征的图像坐标
            // }

        }
        
        // if the distance between two frame is lower than 0.1m or the parallax angle is lower than 15deg , do not triangulate.
        // if(d < 0.1 || min_cos_theta > 0.998) 
        if(min_cos_theta > 0.998)
        // if( d < 0.2 ) 
            continue;

        // plane pi from jth obs in ith camera frame
        Vector3d p3( obsj(0), obsj(1), 1 );
        Vector3d p4( obsj(2), obsj(3), 1 );
        p3 = Rij * p3 + tij;
        p4 = Rij * p4 + tij;
        Vector4d pij = pi_from_ppp(p3, p4,tij);

        Vector6d plk = pipi_plk( pii, pij );
        Vector3d n = plk.head(3);
        Vector3d v = plk.tail(3);

        //Vector3d cp = plucker_origin( n, v );
        //if ( cp(2) < 0 )
        // {
        //   //  cp = - cp;
        //   //  continue;
        // }

        //Vector6d line;
        //line.head(3) = cp;
        //line.tail(3) = v;
        //it_per_id.line_plucker = line;

        // plk.normalize();
        it_per_id.line_plucker = plk;  // plk in camera frame
        it_per_id.is_triangulation = true;

        //  used to debug
        Vector3d pc, nc, vc;
        nc = it_per_id.line_plucker.head(3);
        vc = it_per_id.line_plucker.tail(3);


        Matrix4d Lc;
        Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

        Vector4d obs_startframe = it_per_id.linefeature_per_frame[0].lineobs;   // 第一次观测到这帧
        Vector3d p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
        Vector3d p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
        Vector2d ln = ( p11.cross(p21) ).head(2);     // 直线的垂直方向
        ln = ln / ln.norm();

        Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
        Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
        Vector3d cam = Vector3d( 0, 0, 0 );

        Vector4d pi1 = pi_from_ppp(cam, p11, p12);
        Vector4d pi2 = pi_from_ppp(cam, p21, p22);

        Vector4d e1 = Lc * pi1;
        Vector4d e2 = Lc * pi2;
        e1 = e1/e1(3);
        e2 = e2/e2(3);

        Vector3d pts_1(e1(0),e1(1),e1(2));
        Vector3d pts_2(e2(0),e2(1),e2(2));

        Vector3d w_pts_1 =  Rs[imu_i] * (ric[0] * pts_1 + tic[0]) + Ps[imu_i];
        Vector3d w_pts_2 =  Rs[imu_i] * (ric[0] * pts_2 + tic[0]) + Ps[imu_i];
        it_per_id.ptw1 = w_pts_1;
        it_per_id.ptw2 = w_pts_2;

        //if(isnan(cp(0)))
        // {

        //     //it_per_id.is_triangulation = false;

        //     //std::cout <<"------------"<<std::endl;
        //     //std::cout << line << "\n\n";
        //     //std::cout << d <<"\n\n";
        //     //std::cout << Rij <<std::endl;
        //     //std::cout << tij <<"\n\n";
        //     //std::cout <<"obsj: "<< obsj <<"\n\n";
        //     //std::cout << "p3: " << p3 <<"\n\n";
        //     //std::cout << "p4: " << p4 <<"\n\n";
        //     //std::cout <<pi_from_ppp(p3, p4,tij)<<std::endl;
        //     //std::cout << pij <<"\n\n";

        // }


    }

//    removeLineOutlier(Ps,tic,ric);
}

//点特征的三角化
void FeatureManager::triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (it_per_id.estimated_depth > 0)
            continue;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        ROS_ASSERT(NUM_OF_CAM == 1);
        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }

    }
}

void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
              design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

void FeatureManager::triangulateStereo()
{
    for (auto &it_per_id : stereo_feature)
    {
        if (it_per_id.estimated_depth > 0)
            continue;
        double depth = it_per_id.feature_per_frame[0].getDepth(); 
        if(depth > 0){
            it_per_id.estimated_depth = depth;
        }
    }
}

void FeatureManager::stereo_triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : stereo_feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (it_per_id.estimated_depth > 0)
            continue;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        // ROS_ASSERT(NUM_OF_CAM == 1);
        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }

    }
}

// 事件点特征的三角化
void FeatureManager::Event_triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : Event_feature)
    {
        it_per_id.used_num = it_per_id.Event_feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (it_per_id.estimated_depth > 0)
            continue;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        ROS_ASSERT(NUM_OF_CAM == 1);
        Eigen::MatrixXd svd_A(2 * it_per_id.Event_feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.Event_feature_per_frame)
        {
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }

    }
}

// 去除点特征的outlier(好像没有用到)
void FeatureManager::removeOutlier()
{
    ROS_BREAK();
    int i = -1;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        i += it->used_num != 0;
        if (it->used_num != 0 && it->is_outlier == true)
        {
            feature.erase(it);
        }
    }
}

void FeatureManager::removeLineOutlier(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{

    for (auto it_per_id = linefeature.begin(), it_next = linefeature.begin();
         it_per_id != linefeature.end(); it_per_id = it_next)
    {
        it_next++;
        it_per_id->used_num = it_per_id->linefeature_per_frame.size();
        if (!(it_per_id->used_num >= LINE_MIN_OBS && it_per_id->start_frame < WINDOW_SIZE - 2 && it_per_id->is_triangulation))
            continue;

        int imu_i = it_per_id->start_frame, imu_j = imu_i -1;

        ROS_ASSERT(NUM_OF_CAM == 1);

        Eigen::Vector3d twc = Ps[imu_i] + Rs[imu_i] * tic[0];   // twc = Rwi * tic + twi
        Eigen::Matrix3d Rwc = Rs[imu_i] * ric[0];               // Rwc = Rwi * Ric

        // 计算初始帧上线段对应的3d端点
        Vector3d pc, nc, vc;
        nc = it_per_id->line_plucker.head(3);
        vc = it_per_id->line_plucker.tail(3);

 //       double  d = nc.norm()/vc.norm();
 //       if (d > 5.0)
        {
 //           std::cerr <<"remove a large distant line \n";
 //           linefeature.erase(it_per_id);
 //           continue;
        }

        Matrix4d Lc;
        Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

        Vector4d obs_startframe = it_per_id->linefeature_per_frame[0].lineobs;   // 第一次观测到这帧
        Vector3d p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
        Vector3d p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
        Vector2d ln = ( p11.cross(p21) ).head(2);     // 直线的垂直方向
        ln = ln / ln.norm();

        Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
        Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
        Vector3d cam = Vector3d( 0, 0, 0 );

        Vector4d pi1 = pi_from_ppp(cam, p11, p12);
        Vector4d pi2 = pi_from_ppp(cam, p21, p22);

        Vector4d e1 = Lc * pi1;
        Vector4d e2 = Lc * pi2;
        e1 = e1/e1(3);
        e2 = e2/e2(3);

        //std::cout << "line endpoint: "<<e1 << "\n "<< e2<<"\n";
        if(e1(2) < 0 || e2(2) < 0)
        {
            linefeature.erase(it_per_id);
            continue;
        }
        if((e1-e2).norm() > 10)
        {
            linefeature.erase(it_per_id);
            continue;
        }

/*
        // 点到直线的距离不能太远啊
        Vector3d Q = plucker_origin(nc,vc);
        if(Q.norm() > 5.0)
        {
            linefeature.erase(it_per_id);
            continue;
        }
*/
        // 并且平均投影误差不能太大啊
        Vector6d line_w = plk_to_pose(it_per_id->line_plucker, Rwc, twc);  // transfrom to world frame

        int i = 0;
        double allerr = 0;
        Eigen::Vector3d tij;
        Eigen::Matrix3d Rij;
        Eigen::Vector4d obs;

        //std::cout<<"reprojection_error: \n";
        for (auto &it_per_frame : it_per_id->linefeature_per_frame)   // 遍历所有的观测， 注意 start_frame 也会被遍历
        {
            imu_j++;

            obs = it_per_frame.lineobs;
            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];

            double err =  reprojection_error(obs, R1, t1, line_w);

//            if(err > 0.0000001)
//                i++;
//            allerr += err;    // 计算平均投影误差

            if(allerr < err)    // 记录最大投影误差，如果最大的投影误差比较大，那就说明有outlier
                allerr = err;
        }
//        allerr = allerr / i;
        if (allerr > 3.0 / 500.0)
        {
//            std::cout<<"remove a large error\n";
            linefeature.erase(it_per_id);
        }
    }
}

// 增加了线特征的处理过程
void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    //对图像点特征进行处理
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;  
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }

    //双目的情况下处理
    for (auto it = stereo_feature.begin(), it_next = stereo_feature.begin();
        it != stereo_feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;  
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2)
            {
                stereo_feature.erase(it);
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }

    //对事件点特征进行处理
    for (auto it = Event_feature.begin(), it_next = Event_feature.begin();
         it != Event_feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->Event_feature_per_frame[0].point;  
            it->Event_feature_per_frame.erase(it->Event_feature_per_frame.begin());
            if (it->Event_feature_per_frame.size() < 2)
            {
                Event_feature.erase(it);
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
    }

    // 对事件线特征进行处理（线特征的深度暂时没有通过雷达获取）
    for (auto it = linefeature.begin(), it_next = linefeature.begin();
         it != linefeature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)    // 如果特征不是在这帧上初始化的，那就不用管，只要管id--
        {
            it->start_frame--;
        }
        else
        {
/*
            //  used to debug
            Vector3d pc, nc, vc;
            nc = it->line_plucker.head(3);
            vc = it->line_plucker.tail(3);

            Matrix4d Lc;
            Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

            Vector4d obs_startframe = it->linefeature_per_frame[0].lineobs;   // 第一次观测到这帧
            Vector3d p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
            Vector3d p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
            Vector2d ln = ( p11.cross(p21) ).head(2);     // 直线的垂直方向
            ln = ln / ln.norm();

            Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
            Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
            Vector3d cam = Vector3d( 0, 0, 0 );

            Vector4d pi1 = pi_from_ppp(cam, p11, p12);
            Vector4d pi2 = pi_from_ppp(cam, p21, p22);

            Vector4d e1 = Lc * pi1;
            Vector4d e2 = Lc * pi2;
            e1 = e1/e1(3);
            e2 = e2/e2(3);

            Vector3d pts_1(e1(0),e1(1),e1(2));
            Vector3d pts_2(e2(0),e2(1),e2(2));

            Vector3d w_pts_1 =  marg_R * pts_1 + marg_P;
            Vector3d w_pts_2 =  marg_R * pts_2 + marg_P;

            std::cout<<"-------------------------------\n";
            std::cout << w_pts_1 << "\n" <<w_pts_2 <<"\n\n";
            Vector4d obs_startframe = it->linefeature_per_frame[0].lineobs;   // 第一次观测到这帧
            */
//-----------------
            it->linefeature_per_frame.erase(it->linefeature_per_frame.begin());  // 移除观测
            if (it->linefeature_per_frame.size() < 2)                     // 如果观测到这个帧的图像少于两帧，那这个特征不要了
            {
                linefeature.erase(it);
                continue;
            }
            else  // 如果还有很多帧看到它，而我们又把这个特征的初始化帧给marg掉了，那就得把这个特征转挂到下一帧上去, 这里 marg_R, new_R 都是相应时刻的相机坐标系到世界坐标系的变换
            {
                it->removed_cnt++;
                // transpose this line to the new pose
                Matrix3d Rji = new_R.transpose() * marg_R;     // Rcjw * Rwci
                Vector3d tji = new_R.transpose() * (marg_P - new_P);
                Vector6d plk_j = plk_to_pose(it->line_plucker, Rji, tji);
                it->line_plucker = plk_j;
            }
//-----------------------
/*
            //  used to debug
            nc = it->line_plucker.head(3);
            vc = it->line_plucker.tail(3);

            Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

            obs_startframe = it->linefeature_per_frame[0].lineobs;   // 第一次观测到这帧
            p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
            p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
            ln = ( p11.cross(p21) ).head(2);     // 直线的垂直方向
            ln = ln / ln.norm();

            p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
            p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
            cam = Vector3d( 0, 0, 0 );

            pi1 = pi_from_ppp(cam, p11, p12);
            pi2 = pi_from_ppp(cam, p21, p22);

            e1 = Lc * pi1;
            e2 = Lc * pi2;
            e1 = e1/e1(3);
            e2 = e2/e2(3);

            pts_1 = Vector3d(e1(0),e1(1),e1(2));
            pts_2 = Vector3d(e2(0),e2(1),e2(2));

            w_pts_1 =  new_R * pts_1 + new_P;
            w_pts_2 =  new_R * pts_2 + new_P;

            std::cout << w_pts_1 << "\n" <<w_pts_2 <<"\n";
*/
        }
    }

}

void FeatureManager::removeBack()
{
    // 对图像点特征进行处理
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }

    //对于双目
    for (auto it = stereo_feature.begin(), it_next = stereo_feature.begin();
        it != stereo_feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                stereo_feature.erase(it);
        }
    }

    // 对事件点特征进行处理
    for (auto it = Event_feature.begin(), it_next = Event_feature.begin();
         it != Event_feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->Event_feature_per_frame.erase(it->Event_feature_per_frame.begin());
            if (it->Event_feature_per_frame.size() == 0)
                Event_feature.erase(it);
        }
    }


    // 对线特征进行处理（跟点特征的处理一样）
    for (auto it = linefeature.begin(), it_next = linefeature.begin();
         it != linefeature.end(); it = it_next)
    {
        it_next++;

        // 如果这个特征不是在窗口里最老关键帧上观测到的，由于窗口里移除掉了一个帧，所有其他特征对应的初始化帧id都要减1左移
        // 例如： 窗口里有 0,1,2,3,4 一共5个关键帧，特征f2在第2帧上三角化的， 移除掉第0帧以后， 第2帧在窗口里的id就左移变成了第1帧，这是很f2的start_frame对应减1
        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->linefeature_per_frame.erase(it->linefeature_per_frame.begin());  // 删掉特征ft在这个图像帧上的观测量
            if (it->linefeature_per_frame.size() == 0)                       // 如果没有其他图像帧能看到这个特征ft了，那就直接删掉它
                linefeature.erase(it);
        }
    }
}

void FeatureManager::removeFront(int frame_count)
{
     // 对图像点特征进行处理
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }

    //对于双目
    for (auto it = stereo_feature.begin(), it_next = stereo_feature.begin(); it != stereo_feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
                stereo_feature.erase(it);
        }
    }


    // 对事件点特征进行处理
    for (auto it = Event_feature.begin(), it_next = Event_feature.begin(); it != Event_feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->Event_feature_per_frame.erase(it->Event_feature_per_frame.begin() + j);
            if (it->Event_feature_per_frame.size() == 0)
                Event_feature.erase(it);
        }
    }

    // 对线特征进行处理
    for (auto it = linefeature.begin(), it_next = linefeature.begin(); it != linefeature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)  // 由于要删去的是第frame_count-1帧，最新这一帧frame_count的id就变成了i-1
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;    // j指向第i-1帧
            if (it->endFrame() < frame_count - 1)
                continue;
            it->linefeature_per_frame.erase(it->linefeature_per_frame.begin() + j);   // 删掉特征ft在这个图像帧上的观测量
            if (it->linefeature_per_frame.size() == 0)                            // 如果没有其他图像帧能看到这个特征ft了，那就直接删掉它
                linefeature.erase(it);
        }
    }
}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}

double FeatureManager::compensatedParallax2(const Event_FeaturePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const Event_FeaturePerFrame &frame_i = it_per_id.Event_feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const Event_FeaturePerFrame &frame_j = it_per_id.Event_feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}

double FeatureManager::compensatedParallax2(const stereo_FeaturePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const stereo_FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const stereo_FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}
