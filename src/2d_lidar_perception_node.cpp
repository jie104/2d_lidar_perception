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
    perception_module::InstallPara install_para
        {-1.57,1.57,1,2,3,0.1,0.1};
    perception_module::laser_detect_pallet<sensor_msgs::LaserScanPtr> perception(install_para);

    ros::spin();

}
