/*代码功能：
 * 使用ROS播放rosbag文件，保存点云pcd文件，供后续使用
 * */
#include <iostream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

const std::string bag_file{"/home/ubuntu/datasets/IndoorLioMapping/fast1.bag"};
const std::string lidar_name{"/velodyne_points"};
const std::string pcd_file{"/home/ubuntu/partTimeJob/point_cloud_cube/lidar.pcd"};
int main() {
    std::cout << "Hello, World!" << std::endl;
    rosbag::Bag rosbag;
    rosbag.open(bag_file, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.emplace_back(lidar_name);
    rosbag::View view (rosbag, rosbag::TopicQuery(topics));
    rosbag::View::iterator it = view.begin();
    unsigned int n = 0;  // 取第100帧数据
    pcl::PointCloud<pcl::PointXYZ> pcl_point;
    for (; it!= view.end(); it++) {
        auto m = *it;
        std::string topic = m.getTopic();
        if (topic == lidar_name && n == 100)
        {
            sensor_msgs::PointCloud2 ::ConstPtr point_cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
            pcl::fromROSMsg(*point_cloud_msg, pcl_point);
            pcl::io::savePCDFile(pcd_file, pcl_point);
        }
        n++;
    }


    return 0;
}
