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
#include <Eigen/Core>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud.h>
#include "../../../devel/include/laser_perception/status.h"

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


struct ClusterPoint {
    int min_index;
    float min_index_range;
    int max_index;
    float max_index_range;
    Eigen::Vector2d mean;
    std::vector<RadarPointInfo> infos;

    //计算聚类点平均值
    Eigen::Vector2d computeCenterPoint() {
        Eigen::Vector2d center_point;
        center_point.setZero();
        for (auto &info:infos) {
            center_point += info.point;
        }
        if (infos.size()) {
            mean = center_point * (1/(double) infos.size());
        }else{
            mean.setZero();
        }
        return mean;
    }
};

typedef std::shared_ptr<ClusterPoint> ClusterPoint_Ptr;


template <class ScanType>
class laser_detect_pallet {
public:

    laser_detect_pallet(InstallPara &install_para,Eigen::Vector3d &pallet_pose_in_world,
                        Eigen::Vector3d &car_pose_in_world);
    ~laser_detect_pallet(){}

    void computeCircleInfo(ScanType &scan);

    void scanCallback(ScanType scan);

    void filterOutBoarderPoint(ScanType &scan,InstallPara& install_para);

    void filterLowIntenPoint(ScanType &scan);

    void filterScanTrailingPoint(ScanType& scan,std::vector<bool> &points_state,int step = 2,double min_thresh = 0.02);

    void extractClusters(ScanType &scan,std::vector<bool> &points_state,std::vector<ClusterPoint_Ptr> &clusters);

    void dilateClusters(ScanType &scan, std::vector<ClusterPoint_Ptr> &clusters);

    void dilateClusterBoarder(ScanType &scan,ClusterPoint_Ptr& cluster,int& cluster_index,int dilate_size,int direction = 1);

    void getRackPointPair(std::vector<ClusterPoint_Ptr> &clusters,
                          std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> &rack_points);

    Eigen::Vector3d getDetectPose(std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> &rack_points);

    void pubPose(ScanType &scan,Eigen::Vector3d &pose);

    //模板类模版函数初始化
    template<class  Quaternion>
    void EulerToQuaternion(double yaw, double roll, double pitch, Quaternion &quaternion_);

    Eigen::Vector3d world_to_laser(const Eigen::Vector3d &pose_in_world);

    std::vector<Eigen::Vector3d> getDectectPoses(std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> &rack_point_pairs);

    void pubPoses(ScanType &scan,std::vector<Eigen::Vector3d> &true_poses);

    void pubClusterPoints(ScanType &scan,const std::vector<ClusterPoint_Ptr> &clusters);

    void pubClusterMean(ScanType &scan,const std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> &rack_point_pairs);

    bool doReq(laser_perception::status::Request& req,laser_perception::status::Response& resp);

    void PubCircle(std::vector<double> &check_rack_circle,ScanType scan);

private:
    void combineClusters(std::vector<ClusterPoint_Ptr> &clusters);
//    double Deg2Rad(double degree){
//        return degree*M_PI/180;
//    }
//
//    double Rad2Deg(double rad){
//        return rad*180/M_PI;
//    }


    bool cur_detect_status_;
    bool is_detect_;

    std::vector<double> check_rack_circle_;
    const double check_dist_offset_ = 0.2;
    const double radius_thresh_ = 5.0;//计算允许过滤的最远距离,单位m
    const int min_cluster_thresh = 3;
    const double dilate_cluster_angle_offset = 1.0*M_PI/180.0;
    const double dilate_cluster_dist_thresh = 0.03;
    const double low_inten_thresh = 50;
    const double rack_length_=2.56;  //单位m,todo:待确定
    const double rack_length_thresh_=0.1;   //todo:待确定
    const double range_detect_thresh_=0.05; //单位m,todo:待确定
    const double angle_detect_thresh_=0.0871;   //对应sin5°，todo:待确定
    const int min_detect_point_num_=3;

    const int detect_count_=10; //识别次数
    int cur_count_;

    ros::NodeHandle node_;
    ros::Subscriber scan_sub_;
    ros::Publisher pallet_pose_pub_;
    ros::Publisher clusters_pub_;
    ros::Publisher cluster_mean_pub_;
    ros::ServiceServer server_;
    ros::Publisher circle_pub_;

    InstallPara install_para_;

    Eigen::Vector3d pallet_pose_in_world_;
    Eigen::Vector3d car_pose_in_world_;
    std::vector<Eigen::Vector3d> detect_pallet_poses_;
};
}


#endif //LASER_DETECT_WS_LASER_DETECT_PALLET_H
