//
// Created by zxj on 2024/4/16.
//
#include "../include/laser_detect_pallet.h"
#include <cmath>
#define DEBUG_MODE

namespace perception_module{
    template <class ScanType>
    laser_detect_pallet<ScanType>::laser_detect_pallet(InstallPara &install_para,
                           Eigen::Vector3d &pallet_pose_in_world,Eigen::Vector3d &car_pose_in_world)
        :install_para_(install_para)
        ,pallet_pose_in_world_(pallet_pose_in_world)
        ,car_pose_in_world_(car_pose_in_world){

        server_=node_.advertiseService("detect_status",&laser_detect_pallet::doReq,this);


        scan_sub_ = node_.subscribe("/oradar_node/scan", 5, &laser_detect_pallet::scanCallback, this);
        pallet_pose_pub_ = node_.advertise<geometry_msgs::PoseArray>("poses", 5);
        clusters_pub_=node_.advertise<sensor_msgs::PointCloud>("cluster_points",5);
        cluster_mean_pub_=node_.advertise<sensor_msgs::PointCloud>("cluster_mean",5);
        circle_pub_=node_.advertise<sensor_msgs::PointCloud>("circle_filter",5);

        cur_detect_status_= false;
        is_detect_= false;
        cur_count_=0;
    }

    template <class ScanType>
    bool laser_detect_pallet<ScanType>::doReq(laser_perception::status::Request& req,
                                              laser_perception::status::Response& resp) {
        cur_detect_status_ = req.request_status;

//        if (is_detect_) {
            resp.do_request_status = true;
            return true;
//        } else {
//            return false;
//        }
    }

    template <class ScanType>
    void laser_detect_pallet<ScanType>::scanCallback(ScanType scan) {
        if(!cur_detect_status_) {
            LOG_EVERY_N(INFO,50) << "stop to detect!!! ";
            return;
        }
        LOG_EVERY_N(INFO,50) << " begin to detect!!!";

        if(check_rack_circle_.empty()) {
            computeCircleInfo(scan);
        }
        std::vector<bool> points_state(scan->ranges.size(),true);

        filterOutBoarderPoint(scan, install_para_);

//        filterLowIntenPoint(scan);
        filterNearRangePoints(scan);

        filterScanTrailingPoint(scan,points_state,3,0.01);

        std::vector<ClusterPoint_Ptr> clusters;

        extractClusters(scan, points_state, clusters);

        dilateClusters(scan, clusters);

        for(auto& cluster:clusters){
            cluster->computeCenterPoint();  //计算货架腿的聚点平均值
        }
        combineClusters(clusters);  //将靠得较近的点云进一步组合，求聚点平均值

        std::vector<PalletInfo_Ptr> pallets;

        detectPose(clusters, pallets); //找出所有托盘位姿及中心，存放在triangles中

#ifdef DEBUG_MODE
        pubClusterPoints(scan,pallets);
        PubCircle(check_rack_circle_,scan);

#endif

//        std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> rack_point_pairs;
//
//        getRackPointPair(clusters,rack_point_pairs); //校验聚点距离，通过将均值点对放入容器

#ifdef DEBUG_MODE
        pubClusterMean(scan,pallets);

        std::vector<Eigen::Vector3d> detect_poses;
        for(const auto& pallet:pallets) {
            detect_poses.emplace_back();
            detect_poses.back().head(2)=pallet->mean;
            detect_poses.back().z()=std::atan2(pallet->direction.y(),pallet->direction.x());
        }

        if(detect_poses.empty()){
            LOG_EVERY_N(INFO,50) << "can not detect!!!";
        }
        is_detect_=true;
        pubPoses(scan,detect_poses);

#else
        if(cur_count_ < detect_count_){
            ++cur_count_;
            Eigen::Vector3d detect_pose=getDetectPose(rack_point_pairs);
            if(detect_pose.norm()!=0){
                detect_pallet_poses_.emplace_back(detect_pose);
            }
        }else{
            int size=detect_pallet_poses_.size();
            if(size > min_detect_point_num_){
                Eigen::Vector3d true_pose_in_world;
                for(const auto pose:detect_pallet_poses_){
                    true_pose_in_world+=pose;
                }
                true_pose_in_world=true_pose_in_world/size;
                Eigen::Vector3d detect_pose_in_lidar=world_to_laser(true_pose_in_world);

                pubPose(scan,detect_pose_in_lidar);

                cur_count_=0;
                detect_pallet_poses_.clear();
            }else{
                LOG(INFO)  << "can not detect enough time!!! " << "size: " << size;
            }

        }

#endif
    }

    template <class ScanType>
    void laser_detect_pallet<ScanType>::filterNearRangePoints(ScanType scan){
        std::vector<float> &ranges=scan->ranges;
        for(auto &range:ranges){
            if(range < near_filter_range_){
                range=0;
            }
        }
    }

    template <class ScanType>
    void laser_detect_pallet<ScanType>::PubCircle(std::vector<double> &check_rack_circle,ScanType scan){
        sensor_msgs::PointCloud cloud;
        cloud.header=scan->header;
        int size=check_rack_circle.size();
        double angle_min=scan->angle_min;
        double angle_incre=scan->angle_increment;
        for (int j=0;j< size;j++){
            double angle=angle_min+j*angle_incre;
            auto range=check_rack_circle[j];
            // LOG(INFO) << "range111111: " << range;
            Eigen::Vector2d point(range*cos(angle),range*sin(angle));
            cloud.points.emplace_back();
            cloud.points.back().x=point.x();
            cloud.points.back().y=point.y();
        }
        circle_pub_.publish(cloud);
    }


    template <class TypeScan>
    Eigen::Vector3d laser_detect_pallet<TypeScan>::world_to_laser(const Eigen::Vector3d &pose_in_world){
        Eigen::Vector3d detect_pose;
        ///todo:此时需要重新订阅一次car_pose_in_world,实时定位
        Eigen::Affine2d car2world_tf(
                Eigen::Translation2d(car_pose_in_world_.x(), car_pose_in_world_.y())
                * Eigen::Rotation2Dd(car_pose_in_world_.z()));

        //世界坐标系到车体中心
        detect_pose.head(2) = car2world_tf.inverse() * Eigen::Vector2d(detect_pose[0],detect_pose[1]);
        detect_pose[2] = detect_pose[2] - car_pose_in_world_.z(); //todo 角度转换待确定

        Eigen::Affine2d laser2Car_tf(
                Eigen::Translation2d(install_para_.laser_coord_x, install_para_.laser_coord_y)
                * Eigen::Rotation2Dd(install_para_.laser_coord_yaw));

        //车体中心到世界坐标系
        detect_pose.head(2) = laser2Car_tf.inverse() * Eigen::Vector2d(detect_pose[0],detect_pose[1]); //防止混淆
        detect_pose[2] = detect_pose[2] -install_para_.laser_coord_yaw;//todo 角度转换待确定

        return detect_pose;
    }


    template <class ScanType>
    void laser_detect_pallet<ScanType>::pubPose(ScanType &scan,Eigen::Vector3d &true_pose){
        geometry_msgs::PoseArray pose_array;
        pose_array.header = scan->header;

        pose_array.poses.emplace_back();
        pose_array.poses.back().position.x = true_pose[0];
        pose_array.poses.back().position.y = true_pose[1];
        EulerToQuaternion(true_pose[2], 0, 0,pose_array.poses.back().orientation);

        pallet_pose_pub_.publish(pose_array);

    }

    template <class ScanType>
    void laser_detect_pallet<ScanType>::pubPoses(ScanType &scan,std::vector<Eigen::Vector3d> &true_poses){
        geometry_msgs::PoseArray pose_array;
        pose_array.header = scan->header;
        for(auto true_pose:true_poses){
            pose_array.poses.emplace_back();
            pose_array.poses.back().position.x = true_pose[0];
            pose_array.poses.back().position.y = true_pose[1];
            EulerToQuaternion(true_pose[2], 0, 0,pose_array.poses.back().orientation);
        }

        pallet_pose_pub_.publish(pose_array);

    }

    template<class ScanType>
    void laser_detect_pallet<ScanType>::pubClusterPoints(ScanType &scan,const std::vector<PalletInfo_Ptr> pallets){
        sensor_msgs::PointCloud cluster_points;
        cluster_points.header=scan->header;
        for(const auto pallet:pallets){
            for(const auto cluster:pallet->points){
                for(const auto &point:cluster->infos){
                    cluster_points.points.emplace_back();
                    cluster_points.points.back().x=point.point.x();
                    cluster_points.points.back().y=point.point.y();
                }
            }
        }
        clusters_pub_.publish(cluster_points);
    }

    template <class ScanType>
    void laser_detect_pallet<ScanType>::pubClusterMean(ScanType &scan,const std::vector<PalletInfo_Ptr> pallets){
        sensor_msgs::PointCloud points;
        points.header=scan->header;
        for(const auto pallet:pallets){
            points.points.emplace_back();
            points.points.back().x=pallet->points[0]->mean.x();
            points.points.back().y=pallet->points[0]->mean.y();

            points.points.emplace_back();
            points.points.back().x=pallet->points[1]->mean.x();
            points.points.back().y=pallet->points[1]->mean.y();

            points.points.emplace_back();
            points.points.back().x=pallet->points[2]->mean.x();
            points.points.back().y=pallet->points[2]->mean.y();

        }

        cluster_mean_pub_.publish(points);
    }


    template <class ScanType>
    template <class Quaternion>
    void laser_detect_pallet<ScanType>::EulerToQuaternion(double yaw, double roll, double pitch, Quaternion &quaternion_) {
        double cos_yaw_2 = cos(yaw / 2.0);
        double sin_yaw_2 = sin(yaw / 2.0);
        double cos_roll_2 = cos(roll / 2.0);
        double sin_roll_2 = sin(roll / 2.0);
        double cos_pitch_2 = cos(pitch / 2.0);
        double sin_pitch_2 = sin(pitch / 2.0);
        quaternion_.w = cos_yaw_2 * cos_roll_2 * cos_pitch_2 + sin_yaw_2 * sin_roll_2 * sin_pitch_2;
        quaternion_.x = cos_yaw_2 * sin_roll_2 * cos_pitch_2 + sin_yaw_2 * cos_roll_2 * sin_pitch_2;
        quaternion_.y = cos_yaw_2 * cos_roll_2 * sin_pitch_2 - sin_yaw_2 * sin_roll_2 * cos_pitch_2;
        quaternion_.z = sin_yaw_2 * cos_roll_2 * cos_pitch_2 - cos_yaw_2 * sin_roll_2 * sin_pitch_2;
    }


    template <class ScanType>
    std::vector<Eigen::Vector3d> laser_detect_pallet<ScanType>::getDectectPoses(std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> &rack_point_pairs) {
        const int segment_num = 4;
        const double segment_step=1.0/segment_num;
        std::vector<Eigen::Vector3d> detect_poses;
        for (const auto &rack_point_pair:rack_point_pairs) {
            for (int i = 1; i < segment_num; i++) {
                Eigen::Vector3d detect_pose;
                double step_sum=i*segment_step;
                detect_pose[0]=(1-step_sum)*rack_point_pair.first.x()+step_sum*rack_point_pair.second.x();
                detect_pose[1]=(1-step_sum)*rack_point_pair.first.y()+step_sum*rack_point_pair.second.y();
                detect_pose[2] = std::atan2(rack_point_pair.second.x() - rack_point_pair.first.x(),
                                            rack_point_pair.first.y() -
                                            rack_point_pair.second.y());   //atan(x2-x1,y1-y2)
//                LOG(INFO) << "detect pose in lidar: " << detect_pose.transpose();
                Eigen::Vector3d dir(std::cos(detect_pose[2]), std::sin(detect_pose[2]), 0);
                Eigen::Vector3d zero_dir(std::cos(0), std::sin(0), 0);
                if (dir.cross(zero_dir).norm() < angle_detect_thresh_) {
                    detect_poses.emplace_back(detect_pose);
                }
            }
        }
        return detect_poses;
    }

    template <class ScanType>
    Eigen::Vector3d laser_detect_pallet<ScanType>::getDetectPose(std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> &rack_point_pairs){
        const int segment_num = 4;
        const double segment_step=1.0/segment_num;
        std::vector<Eigen::Vector3d> detect_poses;
        for (const auto &rack_point_pair:rack_point_pairs) {
            for (int i = 1; i < segment_num; i++) {
                Eigen::Vector3d detect_pose;
                double step_sum=i*segment_step;
                detect_pose[0]=(1-step_sum)*rack_point_pair.first.x()+step_sum*rack_point_pair.second.x();
                detect_pose[1]=(1-step_sum)*rack_point_pair.first.y()+step_sum*rack_point_pair.second.y();
                detect_pose[2] = std::atan2(rack_point_pair.second.x() - rack_point_pair.first.x(),
                                            rack_point_pair.first.y() -
                                            rack_point_pair.second.y());   //atan(x2-x1,y1-y2)
//                LOG(INFO) << "detect pose in lidar: " << detect_pose.transpose();

                Eigen::Affine2d laser2Car_tf(
                        Eigen::Translation2d(install_para_.laser_coord_x, install_para_.laser_coord_y)
                        * Eigen::Rotation2Dd(install_para_.laser_coord_yaw));

                //将激光点云转化到车体中心坐标系
                detect_pose.head(2) = laser2Car_tf * Eigen::Vector2d(detect_pose[0],detect_pose[1]); //防止混淆
                detect_pose[2] = detect_pose[2] + install_para_.laser_coord_yaw;//todo 角度转换待确定

                Eigen::Affine2d car2world_tf(
                        Eigen::Translation2d(car_pose_in_world_.x(), car_pose_in_world_.y())
                        * Eigen::Rotation2Dd(car_pose_in_world_.z()));

                //车体中心到世界坐标系
                detect_pose.head(2) = car2world_tf * Eigen::Vector2d(detect_pose[0],detect_pose[1]);
                detect_pose[2] = detect_pose[2] + car_pose_in_world_.z(); //todo 角度转换待确定

                Eigen::Vector3d detect_direct(std::cos(detect_pose[2]), std::sin(detect_pose[2]),0);
                Eigen::Vector3d locate_direct(std::cos(car_pose_in_world_.z()), std::sin(car_pose_in_world_.z()),0);

                if ((detect_pose.head(2) - pallet_pose_in_world_.head(2)).norm() <= range_detect_thresh_ &&
                        detect_direct.cross(locate_direct).norm() <= angle_detect_thresh_) { //cross叉乘仅支持三维
                    return detect_pose;
                }
            }
        }
        LOG(INFO) << "1111111111111 " << " can not detect !!!";
        return Eigen::Vector3d(0,0,0);

    }

    template <class ScanType>
    void laser_detect_pallet<ScanType>::getRackPointPair(std::vector<ClusterPoint_Ptr> &clusters,
                     std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> &rack_points){
        int size=clusters.size();
        for(int i=0;i<size;i++){
            Eigen::Vector2d first=clusters[i]->mean;
            for(int j=i+1;j<size;j++){
                Eigen::Vector2d second=clusters[j]->mean;
                float length=(first-second).norm();
                if(std::fabs(length-rack_length_) <= rack_length_thresh_){
                    rack_points.emplace_back(std::make_pair(first,second));
                }
            }
        }
    }


    template <class ScanType>
    void laser_detect_pallet<ScanType>::filterScanTrailingPoint(ScanType& scan,std::vector<bool> &points_state,int step,double min_thresh) {//注意，这里如果拖尾部分有一个点被滤掉了，那其他点则无法识别出为拖尾点了，因为，这里有距离判断
        auto& ranges = scan->ranges;

        double cos_increment = cos(scan->angle_increment * (double) step * 2.0);
        double theta_thresh = sin((double) scan->angle_increment * (double) step * 2.0) / sin(0.34);//临界值,用于识别断点

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
                info.point = Eigen::Vector2d (range * cos(curr_angle), range * sin(curr_angle));
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
        Eigen::Affine2d laser_tf(
                Eigen::Translation2d(install_para_.laser_coord_x, install_para_.laser_coord_y)
                    * Eigen::Rotation2Dd(install_para_.laser_coord_yaw));
        Eigen::Vector2d center_pose = laser_tf.inverse() * Eigen::Vector2d(0, 0);   //车体中心在雷达坐标系下的坐标
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

    template <class ScanType>
    void laser_detect_pallet<ScanType>::combineClusters(std::vector<ClusterPoint_Ptr> &clusters){
        if (clusters.size()) {
            std::vector<ClusterPoint_Ptr> bk_clusters;
            ClusterPoint_Ptr curr_cluster = clusters[0];
            bk_clusters.push_back(curr_cluster);
            int cluster_size = clusters.size();
            for (int i = 1; i < cluster_size; ++i) {
                if((curr_cluster->mean - clusters[i]->mean).norm()<0.1f){
                    for (auto &info : clusters[i]->infos) {
                        curr_cluster->infos.push_back(info);
                    }
                    curr_cluster->computeCenterPoint();
                }else{
                    curr_cluster = clusters[i];
                    bk_clusters.push_back(curr_cluster);
                }
            }
            clusters.swap(bk_clusters);
        }
    }

    template <class ScanType>
    void laser_detect_pallet<ScanType>::detectPose(const std::vector<ClusterPoint_Ptr> &points,std::vector<PalletInfo_Ptr>& pallets){
        for(auto& point:points) {
            std::vector<EdgeInfo_Ptr> edges;
            for(auto& other_point:points) {
                if (point != other_point) {
                    double length = (other_point->mean - point->mean).norm();
                    if (std::fabs(length-rack_length_) < rack_length_thresh_) {
                        edges.emplace_back(new EdgeInfo);
                        edges.back()->computerEdge(point, other_point);
                    }
                }
            }
            findPallet(edges, pallets);
        }
    }



    //找到雷达5米内的所有货架信息，将货架腿位姿以直角三角形的信息存储在triangles中
    template <class ScanType>
    void laser_detect_pallet<ScanType>::findPallet(const std::vector<EdgeInfo_Ptr> &edges,std::vector<PalletInfo_Ptr>& pallets){
        int size = edges.size();
        for (int i = 0; i < size; ++i) {
            auto& edge = edges[i];
            for (int j = i + 1; j < size; ++j) {
                auto& other_edge = edges[j];
                if(edge->norm.dot(other_edge->norm) < pallet_detect_angle_thresh_) {    //dot 向量点乘
                    pallets.emplace_back(new PalletInfo);
                    pallets.back()->computePallet(edge, other_edge);
                }
            }
        }
    }



    //显式实例化
    template class perception_module::laser_detect_pallet<sensor_msgs::LaserScanPtr>;


}
