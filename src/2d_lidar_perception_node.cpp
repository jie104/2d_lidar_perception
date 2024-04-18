//
// Created by zxj on 2024/4/16.
//
#include "../include/laser_detect_pallet.h"
#include <glog/logging.h>
#include <sensor_msgs/LaserScan.h>


int main(int argc,char **argv){
    google::InitGoogleLogging(argv[0]);
    google::SetStderrLogging(google::INFO);
    google::InstallFailureSignalHandler();

    LOG(INFO) << "hello world!";

    ros::init(argc, argv, "2d_lidar_perception");//ros初始化

    ///雷达信息
    perception_module::InstallPara install_para
        {-1.57,1.57,1,2,3,0.1,0.1};

    ///slam发送的栈板位姿
    Eigen::Vector3d pallet_pose_in_world(1,2,3);

    ///车体位姿
    Eigen::Vector3d car_pose_in_world(2,3,4);

    perception_module::laser_detect_pallet<sensor_msgs::LaserScanPtr>
            perception(install_para,pallet_pose_in_world,car_pose_in_world);

    ros::spin();

}
