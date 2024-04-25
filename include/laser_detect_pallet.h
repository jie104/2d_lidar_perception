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
#include <numeric>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>

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

struct EdgeInfo{
    ClusterPoint_Ptr this_cluster;
    ClusterPoint_Ptr other_cluster;
    Eigen::Vector2d norm;
    float length;
    void computerEdge(const ClusterPoint_Ptr &this_cluster,const ClusterPoint_Ptr& other_cluster){
        norm = other_cluster->mean - this_cluster->mean;    //向量以this_cluster->mean为起点
        this->this_cluster = this_cluster;
        this->other_cluster = other_cluster;
        length = norm.norm();
        norm.normalize();
    }
};
typedef std::shared_ptr<EdgeInfo> EdgeInfo_Ptr;

struct PalletInfo{
    std::vector<ClusterPoint_Ptr> points;
    std::vector<Eigen::Vector2d> line_points;
    ClusterPoint_Ptr left_point;
    ClusterPoint_Ptr right_point;
    ClusterPoint_Ptr corner_point;
    Eigen::Vector2d mean;
    Eigen::Vector2d direction;

    void computePallet(const EdgeInfo_Ptr& edge_1,const EdgeInfo_Ptr& edge_2){
        corner_point = edge_1->this_cluster;
        left_point = edge_1->other_cluster;
        right_point = edge_2->other_cluster;
        points.clear();
        points.push_back(corner_point);
        points.push_back(left_point);
        points.push_back(right_point);
        mean = (left_point->mean + right_point->mean+corner_point->mean)/3;

        line_points.clear();
        line_points.push_back(left_point->mean);
        line_points.push_back(right_point->mean);
        line_points.push_back(corner_point->mean);
        FitLine(line_points,direction);
//        Eigen::Vector2d left_to_corner= calNormalVector(left_point->mean,corner_point->mean);
//        Eigen::Vector2d left_to_right= calNormalVector(left_point->mean,right_point->mean);
//        Eigen::Vector2d corner_to_right= calNormalVector(corner_point->mean,right_point->mean);
//
//        direction=(left_to_corner+left_to_right+corner_to_right)/3;
//        direction.norm();
//
        Eigen::Vector2d detect_direction=Eigen::Vector2d (std::cos(0),std::sin(0));
        if(direction.dot(detect_direction) <0){
            direction=-direction;
        }
    }

//    Eigen::Vector2d calNormalVector(Eigen::Vector2d &v1,Eigen::Vector2d &v2){
//        return Eigen::Vector2d(v2.y()-v1.y(),v1.x()-v2.x());
//    }
//
    bool FitLine(std::vector<Eigen::Matrix<double, 2, 1>>& data,Eigen::Matrix<double, 2, 1>& dir,double eps = 0.2) {
        if (data.size() < 2) {
            return false;
        }

        Eigen::Matrix<double, 2, 1> origin =
                std::accumulate(data.begin(), data.end(), Eigen::Matrix<double, 2, 1>::Zero().eval()) / data.size();

        Eigen::Matrix<double,3,2> Y(data.size(), 2);
        for (int i = 0; i < data.size(); ++i) {
            Y.row(i) = (data[i] - origin).transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix<double,3,2>> svd(Y, Eigen::ComputeFullV);
        dir = svd.matrixV().col(1);
        return true;
    }




};
typedef std::shared_ptr<PalletInfo> PalletInfo_Ptr;


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

    void pubClusterPoints(ScanType &scan,const std::vector<PalletInfo_Ptr> triangles);

    void pubClusterMean(ScanType &scan,const std::vector<PalletInfo_Ptr> triangles);

    bool doReq(laser_perception::status::Request& req,laser_perception::status::Response& resp);

    void PubCircle(std::vector<double> &check_rack_circle,ScanType scan);

    void filterNearRangePoints(ScanType scan);

    void detectPose(const std::vector<ClusterPoint_Ptr> &points,std::vector<PalletInfo_Ptr>& triangles);

    void findPallet(const std::vector<EdgeInfo_Ptr> &edges,std::vector<PalletInfo_Ptr>& pallets);


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
    const double rack_length_=0.44;  //单位m,todo:待确定
    const double rack_length_thresh_=0.03;   //todo:待确定
    const double range_detect_thresh_=0.05; //单位m,todo:待确定
    const double angle_detect_thresh_=0.0871;   //对应sin5°，todo:待确定
    const int min_detect_point_num_=3;
    const float near_filter_range_=1.0;
    const double pallet_detect_angle_thresh_=-0.95; //约161°

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
