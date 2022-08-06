//
// Created by ubuntu on 2022/7/30.
//

/*代码功能：
 * 对点云进行体素化，并且对每个山歌独立增加高斯噪声
*/

#ifndef POINT_CLOUD_CUBE_VOXEL_RANDOM_GAUSS_H
#define POINT_CLOUD_CUBE_VOXEL_RANDOM_GAUSS_H

#include <thread>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/generate.h>  // 生成高斯噪声
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <math.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

namespace voxel_grid {
    typedef pcl::PointXYZ pclXYZ;

    typedef struct point_voxel_Match {

        int point_Index;
        int voxel_Index;

        point_voxel_Match(int point_Index_, int voxel_Index_) : point_Index(point_Index_), voxel_Index(voxel_Index_) {}

        bool operator<(const point_voxel_Match &p) const {

            return (voxel_Index < p.voxel_Index);
        }

    } Match;

    class VoxelGridGauss {
    public:
        VoxelGridGauss(const std::string &pcd_file, float grid) : pcd_file_(pcd_file),
                                                                  grid_(grid) {
            cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());

            if (loadPcd()) {
                std::cout << "读取点云成功 " << std::endl;
            } else {
                std::cout << "Could not open the file: " << pcd_file_ << "..." << std::endl;
            }
            if (!pointExist()) {
                std::cout << "The file: " << pcd_file_ << "has no point..." << std::endl;
            }
        }

        VoxelGridGauss(pcl::PointCloud<pclXYZ> input, float grid) : cloud_(new pcl::PointCloud<pcl::PointXYZ>(input)),

                                                                    grid_(grid) {
            if (!pointExist()) {
                std::cout << "The file: " << pcd_file_ << "has no point..." << std::endl;
            }
        }

        VoxelGridGauss(float grid) : grid_(grid) {
            cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        }

        bool Run();

        bool Run(const std::string &bag_file, const std::string &in_dir, const std::string &out_dir) {
            //😀
            out_dir_ = out_dir;
            rosbag::Bag rosbag;
            rosbag.open(bag_file, rosbag::bagmode::Read);
            std::vector<std::string> topics;
            topics.emplace_back(lidar_name);
            rosbag::View view(rosbag, rosbag::TopicQuery(topics));
            long long n = 0;
            pcl::PointCloud<pcl::PointXYZ> pcl_point;
            for (auto it = view.begin(); it != view.end(); it++) {
                auto m = *it;
                std::string topic = m.getTopic();
                if (topic == lidar_name) {
                    sensor_msgs::PointCloud2 point_cloud_msg;
                    point_cloud_msg = *(m.instantiate<sensor_msgs::PointCloud2>());
                    pcl::fromROSMsg(point_cloud_msg, pcl_point);
                    generate(pcl_point, n);
                    std::cout << "111" << std::endl;
                }
                std::cout << "222" << std::endl;
                n++;
            }
        }


//
    private:

        void generate(const pcl::PointCloud<pcl::PointXYZ> &input, const long long &n) {
            pcl::PointCloud<pcl::PointXYZ> full_cloud;
            if (input.empty()) {
                return;
            }
            *cloud_ = input;
            if (!Run()) {
                return;
            }
            std::vector<pcl::PointCloud<pcl::PointXYZ>> point_with_gauss;
            point_with_gauss = getPointWithGauss();

            for (int i = 0; i < point_with_gauss.size(); ++i) {
                full_cloud += point_with_gauss[i];
            }
            std::string file_name = out_dir_ + std::to_string(n) + ".pcd";
            pcl::io::savePCDFileBinary(file_name, full_cloud);
            std::cout << "000" << std::endl;
        }


        // 读取点云
        inline bool loadPcd() {
            return !(pcl::io::loadPCDFile(pcd_file_, *cloud_) == -1);
        }

        // 判断点云数据是否存在
        inline bool pointExist() {
            return !(cloud_->points.size() == 0);
        }

        // 获取包围框
        void getAABB();

        void matchRule();

        void getIndex();

        void getVoxelNum();

        void computeVoxelCenter();

    public:
        inline pclXYZ getMinAABB() {
            pclXYZ p(min_point_AABB_);
            return p;
        }

        inline pclXYZ getMaxAABB() {
            pclXYZ p(max_point_AABB_);
            return p;
        }

        inline pcl::PointCloud<pclXYZ> getSourseCloud() {
            pcl::PointCloud<pclXYZ> p{*cloud_};

            return p;
        }

        inline std::vector<pcl::PointCloud<pclXYZ>> getPointWithGauss() {
            std::vector<pcl::PointCloud<pclXYZ>> p{all_point_cloud_with_gauss_};
            return p;
        }

        inline std::vector<pclXYZ> getCenterPoint() {
            std::vector<pclXYZ> p{center_cloud_};
            return p;
        }

        ~VoxelGridGauss() {}

        void resetP() {
            cloud_->clear();
        };
    private:
        std::string pcd_file_;
        pcl::PointCloud<pclXYZ>::Ptr cloud_ = nullptr;

        pclXYZ min_point_AABB_;
        pclXYZ max_point_AABB_;
        float grid_;
        int x_, y_, z_; // voxel 个数
        pcl::PointXYZ minPt_, maxPt_;

        pclXYZ index_rule_;  // 点和voxel匹配的规则
        std::vector<std::vector<std::vector<int>>> space_index_{
                0, std::vector<std::vector<int>>(0, std::vector<int>(0, 0))
        };
        std::vector<point_voxel_Match> p_v_match_;
        std::vector<std::pair<int, int>> voxel_interval_;  // 体素间隔
        std::vector<pclXYZ> center_cloud_;
        std::vector<pcl::PointCloud<pcl::PointXYZ>> all_point_cloud_with_gauss_;

        const std::string lidar_name{"/velodyne_points"};

    private:
        bool debug_ = false;
        std::string out_dir_;
    };
}


#endif