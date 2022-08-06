//
// Created by ubuntu on 2022/7/30.
//

#include "voxel_random_gauss.h"

namespace voxel_grid {

    bool VoxelGridGauss::Run() {
        getAABB();
        matchRule();
        getIndex();
        getVoxelNum();
        computeVoxelCenter();
        resetP();
        return true;
    }

    void VoxelGridGauss::getAABB() {
        pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
        feature_extractor.setInputCloud(cloud_);
        feature_extractor.compute();
        feature_extractor.getAABB(min_point_AABB_, max_point_AABB_);
        // 获取最大值和最小值 计算voxel尺寸

        pcl::getMinMax3D(*cloud_, minPt_, maxPt_);  //获取点云最大最小值
        x_ = std::abs(maxPt_.x - minPt_.x)/grid_ +1;
        y_ = std::abs(maxPt_.y - minPt_.y)/grid_ +1;
        z_ = std::abs(maxPt_.z - minPt_.z)/grid_ +1;
        if (debug_){
            std::cout << x_ << std::endl;
            std::cout << y_ << std::endl;
            std::cout << z_ << std::endl;
        }
        space_index_.resize(x_, std::vector<std::vector<int> >(y_,std::vector<int>(z_, 0)));
    }

    void VoxelGridGauss::matchRule() {
        //point and vexel match rule
        index_rule_.x = 1;
        index_rule_.y = x_;
        index_rule_.z =  x_* y_;
        //initialize the space index array x, y, z
        for (int s_x = 0; s_x < x_; s_x++) {
            for (int s_y = 0; s_y < x_; s_y++) {
                for (int s_z = 0; s_z < y_; s_z++) {
                    space_index_[s_x][s_y][s_z] = 0;
                }
            }
        }
        if (debug_){
            std::cout << "matchRule" << std::endl;
        }
    }

    void VoxelGridGauss::getIndex() {
        p_v_match_.reserve(cloud_->size());

        for (int i = 0; i < cloud_->size(); ++i) {

            pcl::PointXYZ p = cloud_->points[i];
            int ijk0 = static_cast<int> (std::abs(p.x - minPt_.x)/grid_);
            int ijk1 = static_cast<int> (std::abs(p.y - minPt_.y)/grid_);
            int ijk2 = static_cast<int> (std::abs(p.z - minPt_.z)/grid_);
            //compute the voxel index
            int idx = ijk0 * index_rule_.x + ijk1 * index_rule_.y + ijk2 * index_rule_.z;
            p_v_match_.push_back(point_voxel_Match(i, idx));
            if (false){
                std::cout << "-------------" << std::endl;
                std::cout << "ijk0: " << ijk0 << std::endl;
                std::cout << "ijk0: " << ijk0 << std::endl;
                std::cout << "ijk0: " << ijk0 << std::endl;
            }

            //save the occupied voxel grid into space index array
            //space_Index[ijk0-1][ijk1-1][ijk2-1] = 1;
            space_index_[ijk0][ijk1][ijk2] = 1;
        }
        std::sort(p_v_match_.begin(), p_v_match_.end(), std::less<point_voxel_Match>());
        std::cout << "getIndex" << std::endl;
    }

    void VoxelGridGauss::getVoxelNum() {
        int voxel_total = 0;
        int v_index = 0;
        voxel_interval_.reserve(p_v_match_.size());
        while (v_index < p_v_match_.size()) {

            int i = v_index + 1;
            while (i < p_v_match_.size() && p_v_match_[i].voxel_Index == p_v_match_[v_index].voxel_Index) {

                ++i;
            }

            if ((i - v_index) >= 0) {

                ++voxel_total;
                voxel_interval_.push_back(std::pair<int, int>(v_index, i));
            }
            v_index = i;
        }
        std::cout << "getVoxelNum" << std::endl;
    }

    void VoxelGridGauss::computeVoxelCenter() {
        int ini_index;
        int last_index;

        double x_sum;
        double y_sum;
        double z_sum;

        pclXYZ center_point;

        // TODO 计算每个体素的中心点
        for (int j = 0; j < voxel_interval_.size(); ++j) {

            ini_index = voxel_interval_[j].first;
            last_index = voxel_interval_[j].second;

            x_sum = 0;
            y_sum = 0;
            z_sum = 0;

            pcl::PointXYZ point_temp;
            pcl::PointCloud<pcl::PointXYZ> pointCloud_temp;
            for (int k = ini_index; k < last_index; ++k) {
                x_sum += cloud_->points[p_v_match_[k].point_Index].x;
                y_sum += cloud_->points[p_v_match_[k].point_Index].y;
                z_sum += cloud_->points[p_v_match_[k].point_Index].z;
                // 获取每个体素中的点云
                point_temp.x = cloud_->points[p_v_match_[k].point_Index].x;
                point_temp.y = cloud_->points[p_v_match_[k].point_Index].y;
                point_temp.z = cloud_->points[p_v_match_[k].point_Index].z;
                pointCloud_temp.push_back(point_temp);
            }
            // TODO 随机增加噪声  这块不一定是你所需要的随机噪声添加方式，可以进行更改
            pcl::PointCloud<pcl::PointXYZ> gauss_cloud;
            float xmean = 0, ymean = 0, zmean = 0;
            float xstddev = 0.002, ystddev = 0.002, zstddev = 0.002;
            uint32_t seed = static_cast<uint32_t>(time(NULL));
            pcl::common::CloudGenerator<pcl::PointXYZ, pcl::common::NormalGenerator<float>> generator;
            pcl::common::NormalGenerator<float>::Parameters x_params(xmean, xstddev, seed++);
            pcl::common::NormalGenerator<float>::Parameters y_params(ymean, ystddev, seed++);
            pcl::common::NormalGenerator<float>::Parameters z_params(zmean, zstddev, seed++);
            generator.setParametersForX(x_params);
            generator.setParametersForY(y_params);
            generator.setParametersForZ(z_params);
            generator.fill(pointCloud_temp.width, pointCloud_temp.height, gauss_cloud);
            for (int i = 0; i < pointCloud_temp.size(); ++i) {
                gauss_cloud.points[i].x += pointCloud_temp.points[i].x;
                gauss_cloud.points[i].y += pointCloud_temp.points[i].y;
                gauss_cloud.points[i].z += pointCloud_temp.points[i].z;
            }
            if (debug_){
                std::cout << "子栅格增加噪声完成" << std::endl;
            }
            center_point.x = x_sum / (last_index - ini_index);
            center_point.y = y_sum / (last_index - ini_index);
            center_point.z = z_sum / (last_index - ini_index);

            center_cloud_.push_back(center_point);
            all_point_cloud_with_gauss_.push_back(gauss_cloud);
        }

    }

}