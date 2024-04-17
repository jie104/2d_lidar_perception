//
// Created by zxj on 2024/4/16.
//
#include "../include/laser_detect_pallet.h"

namespace perception_module{
    template <class ScanType>
    laser_detect_pallet<ScanType>::laser_detect_pallet(InstallPara &install_para)
        : install_para_(install_para){
        scan_sub_ = node_.subscribe("scan", 5, &laser_detect_pallet::scanCallback, this);

    }

    template <class ScanType>
    void laser_detect_pallet<ScanType>::scanCallback(ScanType scan) {
        if(check_rack_circle_.empty()) {
            computeCircleInfo(scan);
        }
        std::vector<bool> points_state(scan->ranges.size(),true);
        filterOutBoarderPoint(scan, install_para_);
        filterLowIntenPoint(scan);
        filterScanTrailingPoint(scan,points_state,2,0.02);
        std::vector<ClusterPoint_Ptr> clusters;
        extractClusters(scan, points_state, clusters);
        dilateClusters(scan, clusters);


    }

    template <class ScanType>
    void laser_detect_pallet<ScanType>::filterScanTrailingPoint(ScanType& scan,std::vector<bool> &points_state,int step,double min_thresh) {//注意，这里如果拖尾部分有一个点被滤掉了，那其他点则无法识别出为拖尾点了，因为，这里有距离判断
        auto& ranges = scan->ranges;

        double cos_increment = cos(scan->angle_increment * (double) step * 2.0);
        double theta_thresh = sin((double) scan->angle_increment * (double) step * 2.0) / sin(0.17);//临界值,用于识别断点

        int scan_size = ranges.size() - step;
        for (int i = step; i < scan_size; i++) {
            if (ranges[i] == 0 || ranges[i] > check_rack_circle_[i]) {
                continue;
            }

            double dist_1 = std::sqrt(ranges[i+step] * ranges[i+step] + ranges[i] * ranges[i] -
                                      2 * ranges[i+step] * ranges[i] * cos_increment);

            double range_thresh_1 = ranges[i + step] * theta_thresh + min_thresh;
            if(dist_1 > range_thresh_1) {
                int remove_gap = step;
                for (int j = -remove_gap; j <= remove_gap; ++j) {
                    points_state[i+j] = false;
                }
            }
        }
    }

    template <class ScanType>
    void laser_detect_pallet<ScanType>::extractClusters(ScanType &scan,std::vector<bool> &points_state,std::vector<ClusterPoint_Ptr> &clusters){
        auto& ranges = scan->ranges;
        auto range_min = scan->range_min;
        auto range_max = scan->range_max;
        double curr_angle = scan->angle_min;
        double angle_incre = scan->angle_increment;
        int index = 0;
        clusters.clear();
        clusters.emplace_back(new (ClusterPoint));
        for (auto &range:ranges) {
            if (range < range_min || range > check_rack_circle_[index]||!points_state[index]) {
                if (clusters.back()->infos.empty()) {

                }else if(clusters.back()->infos.size()<= min_cluster_thresh){//孤立点
                    clusters.back()->infos.clear();
                }else {
                    clusters.emplace_back(new (ClusterPoint));
                }
            }else {
                if (clusters.back()->infos.empty()) {
                    clusters.back()->min_index = index;
                    clusters.back()->min_index_range = range;
                }
                clusters.back()->infos.emplace_back();
                clusters.back()->max_index = index;
                clusters.back()->max_index_range = range;
                auto &info = clusters.back()->infos.back();
                info.point = Eigen::Vector2d(range * cos(curr_angle), range * sin(curr_angle));
            }
            index++;
            curr_angle += angle_incre;
        }
        if (clusters.back()->infos.empty()) {
            clusters.resize(clusters.size() - 1);
        }
    }

    template <class ScanType>
    void laser_detect_pallet<ScanType>::dilateClusters(ScanType &scan, std::vector<ClusterPoint_Ptr> &clusters){
        double increment = scan->angle_increment;
        auto &ranges = scan->ranges;
        int max_size = scan->ranges.size() - 1;
        int delta_trailing_size = round(dilate_cluster_angle_offset / scan->angle_increment);
        for(auto& cluster:clusters){
            auto curr_trailing_size =
                    cluster->min_index - delta_trailing_size > 0 ? delta_trailing_size : cluster->min_index;
            dilateClusterBoarder(scan, cluster, cluster->min_index, curr_trailing_size, -1);
            curr_trailing_size = cluster->max_index + delta_trailing_size > max_size ? max_size - cluster->max_index
                                                                                     : delta_trailing_size;
            dilateClusterBoarder(scan, cluster, cluster->max_index, curr_trailing_size, 1);
        }
    }

    template <class ScanType>
    void laser_detect_pallet<ScanType>::dilateClusterBoarder(ScanType &scan,ClusterPoint_Ptr& cluster,int& cluster_index,int dilate_size,int direction){
        const int beg_index = cluster_index;
        auto &ranges = scan->ranges;
        double min_angle = scan->angle_min;
        double increment = scan->angle_increment;
        auto ref_range = ranges[beg_index];
        for (int i = 1; i < dilate_size; ++i) {
            auto first_edge_index = beg_index + i * direction;
            auto range = ranges[first_edge_index];
            auto delta_dist =
                    range * range + ref_range * ref_range - 2.0 * range * ref_range * cos(i * increment);
            if (delta_dist <= dilate_cluster_dist_thresh * dilate_cluster_dist_thresh) {
                cluster_index = first_edge_index;
                cluster->infos.emplace_back();
                auto &info = cluster->infos.back();
                double curr_angle = first_edge_index * increment + min_angle;
                info.point = Eigen::Vector2d(range * cos(curr_angle), range * sin(curr_angle));
            }
        }
    }


    template <class ScanType>
    void laser_detect_pallet<ScanType>::computeCircleInfo(ScanType &scan) {
        check_rack_circle_.clear();
        check_rack_circle_.resize(scan->ranges.size());
        double min_angle = scan->angle_min;
        double angle_incre = scan->angle_increment;
        double angle = min_angle;

        //将以雷达安装位置为原点的坐标系上的点云转化到转化到标准坐标系上，即原点(0,0),即将雷达坐标系的点云转到车体中心
        Eigen::Affine2f laser_tf(
                Eigen::Translation2f(laser_coord_x_, laser_coord_y_) * Eigen::Rotation2Df(laser_coord_yaw_));
        Eigen::Vector2f center_pose = laser_tf.inverse() * Eigen::Vector2f(0, 0);   //车体中心在雷达坐标系下的坐标
        double a_0 = center_pose[0];
        double b_0 = center_pose[1];

        double r_0_2 = a_0 * a_0 + b_0 * b_0;
        double r_0 = sqrt(r_0_2);
        double  phi = atan2(b_0, a_0);
    //        LOG(INFO) << "radius:" << radius_thresh << "," << a_0 << "," << b_0 << "," << phi;
        for (auto &range:check_rack_circle_) {//将所有角度的range均计算出过滤的最远距离
            //利用极坐标直线公式:r*r - 2*r*r0cos(theta-phi)+r0*r0-a*a,

            double cos_delta_theta = cos(angle - phi);
            double b = -2 * r_0 * cos_delta_theta;
            double k = r_0_2 - radius_thresh_ * radius_thresh_;
            double delta_2 = b * b - 4 * k;
            if (delta_2 < 0) {
                LOG(WARNING) << "the delta_2 is err!" << delta_2;
                range = 0;
            } else {
                double range_1 = (-b + sqrt(delta_2)) / 2.0;
                double range_2 = (-b - sqrt(delta_2)) / 2.0;

                range = std::max(range_1, range_2);
                if (range < 0) {
                    LOG(WARNING) << "the range is err!" << range;
                    range = 0;
                }
            }
            angle += angle_incre;
        }
    }

    template <class ScanType>
    void laser_detect_pallet<ScanType>::filterOutBoarderPoint(ScanType &scan,InstallPara& install_para){
        auto& ranges = scan->ranges;
        auto range_min = scan->range_min;
        auto range_max = scan->range_max;
        double curr_angle = scan->angle_min;
        double angle_incre = scan->angle_increment;
        for (auto &range:ranges) {
            if (range < range_min || range > range_max) {
                range = 0;
            }else {
                if (curr_angle < install_para.laser_angle_min || curr_angle > install_para.laser_angle_max) {
                    range = 0;
                }
            }
            curr_angle += angle_incre;
        }
    }

    template <class ScanType>
    void laser_detect_pallet<ScanType>::filterLowIntenPoint(ScanType &scan){
        int scan_size = scan->ranges.size();
        auto &ranges = scan->ranges;
        auto &intens = scan->intensities;

        for (int i = 0; i < scan_size; ++i) {
            if(ranges[i]<check_rack_circle_[i]) {//判断当前range是否在要进行判断的范围之内
                if (intens[i] < low_inten_thresh) {
                    scan->ranges[i] = 0;
                }
            }
        }
    }

    //显式实例化
    template class perception_module::laser_detect_pallet<sensor_msgs::LaserScanPtr>;


}
