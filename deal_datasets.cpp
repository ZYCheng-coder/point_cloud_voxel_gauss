//
// Created by ubuntu on 2022/7/31.
//

/*代码功能：
 * 读取rosbag，使用voxel random gauss进行处理，并且保存为pcd文件，路径默认当前路径的pcd_dir
*/
#include "voxel_random_gauss.h"

const std::string bag_file{"/home/ubuntu/datasets/IndoorLioMapping/fast1.bag"};
const std::string lidar_name{"/velodyne_points"};
const std::string pcd_file{"/home/ubuntu/partTimeJob/point_cloud_cube/pcd_dir/"};

void generate(const pcl::PointCloud<pcl::PointXYZ>& input, const long long & n){
    pcl::PointCloud<pcl::PointXYZ> pcl_point = input;
    pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl_point.empty()){
        return;
    }
    voxel_grid::VoxelGridGauss voxelGridGauss(pcl_point,1);
    voxelGridGauss.Run();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<pcl::PointCloud<pcl::PointXYZ>> point_with_gauss;
    *cloud_ = voxelGridGauss.getSourseCloud();
    point_with_gauss = voxelGridGauss.getPointWithGauss();

    for (int i = 0; i < point_with_gauss.size(); ++i) {
        *full_cloud += point_with_gauss[i];
    }
    std::string file_name = pcd_file + std::to_string(n) + ".pcd";
    pcl::io::savePCDFileBinary(file_name, *full_cloud);
    voxelGridGauss.resetP();
    std::cout << "000" << std::endl;
}

int main() {
//    rosbag::Bag rosbag;
//    rosbag.open(bag_file, rosbag::bagmode::Read);
//    std::vector<std::string> topics;
//    topics.emplace_back(lidar_name);
//    rosbag::View view(rosbag, rosbag::TopicQuery(topics));
//    rosbag::View::iterator it = view.begin();
//    long long n = 0;
//    pcl::PointCloud<pcl::PointXYZ> pcl_point;
//    for (; it != view.end(); it++) {
//        auto m = *it;
//        std::string topic = m.getTopic();
//        if (topic == lidar_name) {
//            sensor_msgs::PointCloud2::ConstPtr point_cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
//            pcl::fromROSMsg(*point_cloud_msg, pcl_point);
//            generate( pcl_point, n);
//            std::cout << "111" << std::endl;
//        }
//        std::cout << "222" << std::endl;
//        n++;
//    }
    voxel_grid::VoxelGridGauss voxelGridGauss(1.0);
    voxelGridGauss.Run(bag_file,pcd_file,pcd_file);

    return 0;
}
