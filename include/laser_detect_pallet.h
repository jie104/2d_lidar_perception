//
// Created by zxj on 2024/4/16.
//

#ifndef LASER_DETECT_WS_LASER_DETECT_PALLET_H
#define LASER_DETECT_WS_LASER_DETECT_PALLET_H

#include <Eigen/Dense>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include  <cmath>
#include <glog/logging.h>

namespace perception_module{

struct InstallPara{
    double laser_angle_min;
    double laser_angle_max;
    double laser_coord_x;
    double laser_coord_y;
    double laser_coord_yaw;
    double backlash_angle;
    double trailing_angle;
};

struct RadarPointInfo{
    Eigen::Vector2d point;
};

struct ClusterPoint{
    int min_index;
    float min_index_range;
    int max_index;
    float max_index_range;
    std::vector<RadarPointInfo> infos;

    Eigen::Vector2d computeCenterPoint() {
        Eigen::Vector2d center_point;
        center_point.setZero();
        for (auto &info:infos) {
            center_point += info.point;
        }
        if (infos.size()) {
            center_point = center_point * (1/(double) infos.size());
        }
        return center_point;
    }
};
typedef std::shared_ptr<ClusterPoint> ClusterPoint_Ptr;


template <class ScanType>
class laser_detect_pallet {
public:
    laser_detect_pallet(InstallPara &install_para);
    ~laser_detect_pallet(){}

    void computeCircleInfo(ScanType &scan);

    void scanCallback(ScanType scan);

    void filterOutBoarderPoint(ScanType &scan,InstallPara& install_para);

    void filterLowIntenPoint(ScanType &scan);

    void filterScanTrailingPoint(ScanType& scan,std::vector<bool> &points_state,int step = 2,double min_thresh = 0.02);

    void extractClusters(ScanType &scan,std::vector<bool> &points_state,std::vector<ClusterPoint_Ptr> &clusters);

    void dilateClusters(ScanType &scan, std::vector<ClusterPoint_Ptr> &clusters);

    void dilateClusterBoarder(ScanType &scan,ClusterPoint_Ptr& cluster,int& cluster_index,int dilate_size,int direction = 1);


private:
    std::vector<double> check_rack_circle_;
    const double check_dist_offset_ = 0.2;
    const double radius_thresh_ = 5.0;//计算允许过滤的最远距离,单位m
    const int min_cluster_thresh = 3;
    const double dilate_cluster_angle_offset = 1.0*M_PI/180.0;
    const double dilate_cluster_dist_thresh = 0.03;
    const double low_inten_thresh = 50;


    //雷达安装位置
    double laser_coord_x_;
    double laser_coord_y_;
    double laser_coord_yaw_;
    ros::NodeHandle node_;
    ros::Subscriber scan_sub_;
    InstallPara install_para_;


};
}


#endif //LASER_DETECT_WS_LASER_DETECT_PALLET_H
