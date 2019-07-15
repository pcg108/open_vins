#include "RosVisualizer.h"


using namespace ov_msckf;



RosVisualizer::RosVisualizer(ros::NodeHandle &nh, VioManager* app, Simulator *sim) : _nh(nh), _app(app), _sim(sim) {


    // Setup our transform broadcaster
    mTfBr = new tf::TransformBroadcaster();

    // Setup pose and path publisher
    pub_poseimu = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ov_msckf/poseimu", 2);
    ROS_INFO("Publishing: %s", pub_poseimu.getTopic().c_str());
    pub_pathimu = nh.advertise<nav_msgs::Path>("/ov_msckf/pathimu", 2);
    ROS_INFO("Publishing: %s", pub_pathimu.getTopic().c_str());

    // 3D points publishing
    pub_points_msckf = nh.advertise<sensor_msgs::PointCloud2>("/ov_msckf/points_msckf", 2);
    ROS_INFO("Publishing: %s", pub_points_msckf.getTopic().c_str());
    pub_points_slam = nh.advertise<sensor_msgs::PointCloud2>("/ov_msckf/points_slam", 2);
    ROS_INFO("Publishing: %s", pub_points_msckf.getTopic().c_str());
    pub_points_aruco = nh.advertise<sensor_msgs::PointCloud2>("/ov_msckf/points_aruco", 2);
    ROS_INFO("Publishing: %s", pub_points_aruco.getTopic().c_str());
    pub_points_sim = nh.advertise<sensor_msgs::PointCloud2>("/ov_msckf/points_sim", 2);
    ROS_INFO("Publishing: %s", pub_points_sim.getTopic().c_str());

    // Our tracking image
    pub_tracks = nh.advertise<sensor_msgs::Image>("/ov_msckf/trackhist", 2);
    ROS_INFO("Publishing: %s", pub_tracks.getTopic().c_str());

    // Groundtruth publishers
    pub_posegt = nh.advertise<geometry_msgs::PoseStamped>("/ov_msckf/posegt", 2);
    ROS_INFO("Publishing: %s", pub_posegt.getTopic().c_str());
    pub_pathgt = nh.advertise<nav_msgs::Path>("/ov_msckf/pathgt", 2);
    ROS_INFO("Publishing: %s", pub_pathgt.getTopic().c_str());

    // Load groundtruth if we have it
    if (nh.hasParam("path_gt")) {
        std::string path_to_gt;
        nh.param<std::string>("path_gt", path_to_gt, "");
        DatasetReader::load_gt_file(path_to_gt, gt_states);
    }

}



void RosVisualizer::visualize() {


    // publish current image
    publish_images();

    // Return if we have not inited
    if(!_app->intialized())
        return;

    // publish state
    publish_state();

    // publish points
    publish_features();

    // Publish gt if we have it
    publish_groundtruth();

}



void RosVisualizer::visualize_final() {


    // TODO: publish our calibration final results


    // Publish RMSE if we have it
    if(!gt_states.empty()) {
        ROS_INFO("\033[0;95mRMSE average: %.3f (deg) orientation\033[0m",summed_rmse_ori/summed_number);
        ROS_INFO("\033[0;95mRMSE average: %.3f (m) position\033[0m",summed_rmse_pos/summed_number);
    }

}



void RosVisualizer::publish_state() {

    // Get the current state
    State* state = _app->get_state();

    // Create pose of IMU (note we use the bag time)
    geometry_msgs::PoseWithCovarianceStamped poseIinM;
    poseIinM.header.stamp = ros::Time(state->timestamp());
    poseIinM.header.seq = poses_seq_imu;
    poseIinM.header.frame_id = "global";
    poseIinM.pose.pose.orientation.x = state->imu()->quat()(0);
    poseIinM.pose.pose.orientation.y = state->imu()->quat()(1);
    poseIinM.pose.pose.orientation.z = state->imu()->quat()(2);
    poseIinM.pose.pose.orientation.w = state->imu()->quat()(3);
    poseIinM.pose.pose.position.x = state->imu()->pos()(0);
    poseIinM.pose.pose.position.y = state->imu()->pos()(1);
    poseIinM.pose.pose.position.z = state->imu()->pos()(2);

    // Finally set the covariance in the message
    Eigen::Matrix<double,6,6> covariance = state->Cov().block(state->imu()->pose()->id(),state->imu()->pose()->id(),6,6);
    poseIinM.pose.covariance[0] = covariance(0,0); // 0
    poseIinM.pose.covariance[1] = covariance(0,1);
    poseIinM.pose.covariance[2] = covariance(0,2);
    poseIinM.pose.covariance[3] = covariance(0,3);
    poseIinM.pose.covariance[4] = covariance(0,4);
    poseIinM.pose.covariance[5] = covariance(0,5);
    poseIinM.pose.covariance[6] = covariance(1,0); // 1
    poseIinM.pose.covariance[7] = covariance(1,1);
    poseIinM.pose.covariance[8] = covariance(1,2);
    poseIinM.pose.covariance[9] = covariance(1,3);
    poseIinM.pose.covariance[10] = covariance(1,4);
    poseIinM.pose.covariance[11] = covariance(1,5);
    poseIinM.pose.covariance[12] = covariance(2,0); // 2
    poseIinM.pose.covariance[13] = covariance(2,1);
    poseIinM.pose.covariance[14] = covariance(2,2);
    poseIinM.pose.covariance[15] = covariance(2,3);
    poseIinM.pose.covariance[16] = covariance(2,4);
    poseIinM.pose.covariance[17] = covariance(2,5);
    poseIinM.pose.covariance[18] = covariance(3,0); // 3
    poseIinM.pose.covariance[19] = covariance(3,1);
    poseIinM.pose.covariance[20] = covariance(3,2);
    poseIinM.pose.covariance[21] = covariance(3,3);
    poseIinM.pose.covariance[22] = covariance(3,4);
    poseIinM.pose.covariance[23] = covariance(3,5);
    poseIinM.pose.covariance[24] = covariance(4,0); // 4
    poseIinM.pose.covariance[25] = covariance(4,1);
    poseIinM.pose.covariance[26] = covariance(4,2);
    poseIinM.pose.covariance[27] = covariance(4,3);
    poseIinM.pose.covariance[28] = covariance(4,4);
    poseIinM.pose.covariance[29] = covariance(4,5);
    poseIinM.pose.covariance[30] = covariance(5,0); // 5
    poseIinM.pose.covariance[31] = covariance(5,1);
    poseIinM.pose.covariance[32] = covariance(5,2);
    poseIinM.pose.covariance[33] = covariance(5,3);
    poseIinM.pose.covariance[34] = covariance(5,4);
    poseIinM.pose.covariance[35] = covariance(5,5);
    pub_poseimu.publish(poseIinM);

    // Append to our pose vector
    geometry_msgs::PoseStamped posetemp;
    posetemp.header = poseIinM.header;
    posetemp.pose = poseIinM.pose.pose;
    poses_imu.push_back(posetemp);

    // Create our path (imu)
    nav_msgs::Path arrIMU;
    arrIMU.header.stamp = ros::Time::now();
    arrIMU.header.seq = poses_seq_imu;
    arrIMU.header.frame_id = "global";
    arrIMU.poses = poses_imu;
    pub_pathimu.publish(arrIMU);

    // Move them forward in time
    poses_seq_imu++;

    // Publish our transform on TF
    tf::StampedTransform trans;
    trans.stamp_ = ros::Time::now();
    trans.frame_id_ = "global";
    trans.child_frame_id_ = "imu";
    tf::Quaternion quat(state->imu()->quat()(0),state->imu()->quat()(1),state->imu()->quat()(2),state->imu()->quat()(3));
    trans.setRotation(quat);
    tf::Vector3 orig(state->imu()->pos()(0),state->imu()->pos()(1),state->imu()->pos()(2));
    trans.setOrigin(orig);
    mTfBr->sendTransform(trans);


    // Loop through each camera calibration and publish it
    for(const auto &calib : state->get_calib_IMUtoCAMs()) {
        // need to flip the transform to the IMU frame
        Eigen::Vector4d q_CtoI = Inv(calib.second->quat());
        Eigen::Vector3d p_IinC = -calib.second->Rot().transpose()*calib.second->pos();
        // publish
        tf::StampedTransform trans;
        trans.stamp_ = ros::Time::now();
        trans.frame_id_ = "imu";
        trans.child_frame_id_ = "cam"+std::to_string(calib.first);
        tf::Quaternion quat(q_CtoI(0),q_CtoI(1),q_CtoI(2),q_CtoI(3));
        trans.setRotation(quat);
        tf::Vector3 orig(p_IinC(0),p_IinC(1),p_IinC(2));
        trans.setOrigin(orig);
        mTfBr->sendTransform(trans);
    }

}



void RosVisualizer::publish_images() {

    // Get our trackers
    TrackBase *trackFEATS = _app->get_track_feat();
    TrackBase *trackARUCO = _app->get_track_aruco();

    // Get our image of history tracks
    cv::Mat img_history;
    trackFEATS->display_history(img_history,255,255,0,255,255,255);
    if(trackARUCO != nullptr) {
        trackARUCO->display_history(img_history, 0, 255, 255, 255, 255, 255);
        trackARUCO->display_active(img_history, 0, 255, 255, 255, 255, 255);
    }

    // Create our message
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    sensor_msgs::ImagePtr exl_msg = cv_bridge::CvImage(header, "bgr8", img_history).toImageMsg();

    // Publish
    pub_tracks.publish(exl_msg);

}




void RosVisualizer::publish_features() {

    // Get our good features
    std::vector<Eigen::Vector3d> feats_msckf = _app->get_good_features_MSCKF();

    // Declare message and sizes
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = "global";
    cloud.header.stamp = ros::Time::now();
    cloud.width  = 3*feats_msckf.size();
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = false; // there may be invalid points

    // Setup pointcloud fields
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1,"xyz");
    modifier.resize(3*feats_msckf.size());

    // Iterators
    sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");

    // Fill our iterators
    for(const auto &pt : feats_msckf) {
        *out_x = pt(0); ++out_x;
        *out_y = pt(1); ++out_y;
        *out_z = pt(2); ++out_z;
    }

    // Publish
    pub_points_msckf.publish(cloud);

    //====================================================================
    //====================================================================

    // Get our good features
    std::vector<Eigen::Vector3d> feats_slam = _app->get_features_SLAM();

    // Declare message and sizes
    sensor_msgs::PointCloud2 cloud_SLAM;
    cloud_SLAM.header.frame_id = "global";
    cloud_SLAM.header.stamp = ros::Time::now();
    cloud_SLAM.width  = 3*feats_slam.size();
    cloud_SLAM.height = 1;
    cloud_SLAM.is_bigendian = false;
    cloud_SLAM.is_dense = false; // there may be invalid points

    // Setup pointcloud fields
    sensor_msgs::PointCloud2Modifier modifier_SLAM(cloud_SLAM);
    modifier_SLAM.setPointCloud2FieldsByString(1,"xyz");
    modifier_SLAM.resize(3*feats_slam.size());

    // Iterators
    sensor_msgs::PointCloud2Iterator<float> out_x_SLAM(cloud_SLAM, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y_SLAM(cloud_SLAM, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z_SLAM(cloud_SLAM, "z");

    // Fill our iterators
    for(const auto &pt : feats_slam) {
        *out_x_SLAM = pt(0); ++out_x_SLAM;
        *out_y_SLAM = pt(1); ++out_y_SLAM;
        *out_z_SLAM = pt(2); ++out_z_SLAM;
    }

    // Publish
    pub_points_slam.publish(cloud_SLAM);

    //====================================================================
    //====================================================================

    // Get our good features
    std::vector<Eigen::Vector3d> feats_aruco = _app->get_features_ARUCO();

    // Declare message and sizes
    sensor_msgs::PointCloud2 cloud_ARUCO;
    cloud_ARUCO.header.frame_id = "global";
    cloud_ARUCO.header.stamp = ros::Time::now();
    cloud_ARUCO.width  = 3*feats_aruco.size();
    cloud_ARUCO.height = 1;
    cloud_ARUCO.is_bigendian = false;
    cloud_ARUCO.is_dense = false; // there may be invalid points

    // Setup pointcloud fields
    sensor_msgs::PointCloud2Modifier modifier_ARUCO(cloud_ARUCO);
    modifier_ARUCO.setPointCloud2FieldsByString(1,"xyz");
    modifier_ARUCO.resize(3*feats_aruco.size());

    // Iterators
    sensor_msgs::PointCloud2Iterator<float> out_x_ARUCO(cloud_ARUCO, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y_ARUCO(cloud_ARUCO, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z_ARUCO(cloud_ARUCO, "z");

    // Fill our iterators
    for(const auto &pt : feats_aruco) {
        *out_x_ARUCO = pt(0); ++out_x_ARUCO;
        *out_y_ARUCO = pt(1); ++out_y_ARUCO;
        *out_z_ARUCO = pt(2); ++out_z_ARUCO;
    }

    // Publish
    pub_points_aruco.publish(cloud_ARUCO);


    //====================================================================
    //====================================================================

    // Skip the rest of we are not doing simulation
    if(_sim == nullptr)
        return;

    // Get our good features
    std::unordered_map<size_t,Eigen::Vector3d> feats_sim = _sim->get_map();

    // Declare message and sizes
    sensor_msgs::PointCloud2 cloud_SIM;
    cloud_SIM.header.frame_id = "global";
    cloud_SIM.header.stamp = ros::Time::now();
    cloud_SIM.width  = 3*feats_sim.size();
    cloud_SIM.height = 1;
    cloud_SIM.is_bigendian = false;
    cloud_SIM.is_dense = false; // there may be invalid points

    // Setup pointcloud fields
    sensor_msgs::PointCloud2Modifier modifier_SIM(cloud_SIM);
    modifier_SIM.setPointCloud2FieldsByString(1,"xyz");
    modifier_SIM.resize(3*feats_sim.size());

    // Iterators
    sensor_msgs::PointCloud2Iterator<float> out_x_SIM(cloud_SIM, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y_SIM(cloud_SIM, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z_SIM(cloud_SIM, "z");

    // Fill our iterators
    for(const auto &pt : feats_sim) {
        *out_x_SIM = pt.second(0); ++out_x_SIM;
        *out_y_SIM = pt.second(1); ++out_y_SIM;
        *out_z_SIM = pt.second(2); ++out_z_SIM;
    }

    // Publish
    pub_points_sim.publish(cloud_SIM);

}



void RosVisualizer::publish_groundtruth() {

    // Our groundtruth state
    Eigen::Matrix<double,17,1> state_gt;

    // Check that we have the timestamp in our GT file [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
    if(_sim == nullptr && (gt_states.empty() || !DatasetReader::get_gt_state(_app->get_state()->timestamp(), state_gt, gt_states))) {
        return;
    }

    // Get the simulated groundtruth
    if(_sim != nullptr && !_sim->get_state(_app->get_state()->timestamp(),state_gt)) {
        return;
    }

    // Get the GT and system state state
    Eigen::Matrix<double,16,1> state_ekf = _app->get_state()->imu()->value();

    // Create pose of IMU
    geometry_msgs::PoseStamped poseIinM;
    poseIinM.header.stamp = ros::Time(_app->get_state()->timestamp());
    poseIinM.header.seq = poses_seq_gt;
    poseIinM.header.frame_id = "global";
    poseIinM.pose.orientation.x = state_gt(1,0);
    poseIinM.pose.orientation.y = state_gt(2,0);
    poseIinM.pose.orientation.z = state_gt(3,0);
    poseIinM.pose.orientation.w = state_gt(4,0);
    poseIinM.pose.position.x = state_gt(5,0);
    poseIinM.pose.position.y = state_gt(6,0);
    poseIinM.pose.position.z = state_gt(7,0);
    pub_posegt.publish(poseIinM);

    // Append to our pose vector
    poses_gt.push_back(poseIinM);

    // Create our path (imu)
    nav_msgs::Path arrIMU;
    arrIMU.header.stamp = ros::Time::now();
    arrIMU.header.seq = poses_seq_gt;
    arrIMU.header.frame_id = "global";
    arrIMU.poses = poses_gt;
    pub_pathgt.publish(arrIMU);

    // Move them forward in time
    poses_seq_gt++;

    // Publish our transform on TF
    tf::StampedTransform trans;
    trans.stamp_ = ros::Time::now();
    trans.frame_id_ = "global";
    trans.child_frame_id_ = "truth";
    tf::Quaternion quat(state_gt(1,0),state_gt(2,0),state_gt(3,0),state_gt(4,0));
    trans.setRotation(quat);
    tf::Vector3 orig(state_gt(5,0),state_gt(6,0),state_gt(7,0));
    trans.setOrigin(orig);
    mTfBr->sendTransform(trans);

    //==========================================================================
    //==========================================================================

    // Difference between positions
    double dx = state_ekf(4,0)-state_gt(5,0);
    double dy = state_ekf(5,0)-state_gt(6,0);
    double dz = state_ekf(6,0)-state_gt(7,0);
    double rmse_pos = std::sqrt(dx*dx+dy*dy+dz*dz);

    // Quaternion error
    Eigen::Matrix<double,4,1> quat_gt, quat_st, quat_diff;
    quat_gt << state_gt(1,0),state_gt(2,0),state_gt(3,0),state_gt(4,0);
    quat_st << state_ekf(0,0),state_ekf(1,0),state_ekf(2,0),state_ekf(3,0);
    quat_diff = quat_multiply(quat_st,Inv(quat_gt));
    double rmse_ori = (180/M_PI)*2*quat_diff.block(0,0,3,1).norm();


    //==========================================================================
    //==========================================================================

    // Get covariance of pose
    Eigen::Matrix<double,6,6> covariance = _app->get_state()->Cov().block(_app->get_state()->imu()->pose()->id(),_app->get_state()->imu()->pose()->id(),6,6);

    // Calculate NEES values
    double ori_nees = 2*quat_diff.block(0,0,3,1).dot(covariance.block(0,0,3,3).inverse()*2*quat_diff.block(0,0,3,1));
    Eigen::Vector3d errpos = state_ekf.block(4,0,3,1)-state_gt.block(5,0,3,1);
    double pos_nees = errpos.transpose()*covariance.block(3,3,3,3).inverse()*errpos;

    //==========================================================================
    //==========================================================================

    // Update our average variables
    summed_rmse_ori += rmse_ori;
    summed_rmse_pos += rmse_pos;
    summed_nees_ori += ori_nees;
    summed_nees_pos += pos_nees;
    summed_number++;

    // Nice display for the user
    ROS_INFO("\033[0;95merror to gt => %.3f, %.3f (deg,m) | average error => %.3f, %.3f (deg,m) | called %d times \033[0m",rmse_ori,rmse_pos,summed_rmse_ori/summed_number,summed_rmse_pos/summed_number, (int)summed_number);
    ROS_INFO("\033[0;95mnees => %.1f, %.1f (ori,pos) | average nees = %.1f, %.1f (ori,pos) \033[0m",ori_nees,pos_nees,summed_nees_ori/summed_number,summed_nees_pos/summed_number);


    //==========================================================================
    //==========================================================================



}






