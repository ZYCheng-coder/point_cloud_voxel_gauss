//
// Created by ubuntu on 2022/7/30.
//
#include "voxel_random_gauss.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
using namespace std;


int main(int argc, char **argv) {

    //estimate whether inputs are correct or not
//    if (argc < 3 || argc > 5) {
//
//        return 0;
//    }
//
//    if (argc == 4) {
//
//        return 0;
//    }
    // TODO 检查参数合法化
    std::string file = argv[1];
    float grid = std::atof(argv[2]);
    std::string flag = argv[3];
    // TODO 验证后缀
    voxel_grid::VoxelGridGauss voxelGridGauss(file, grid);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<pcl::PointCloud<pcl::PointXYZ>> point_with_gauss;
    std::vector<pcl::PointXYZ> center_cloud;
    *cloud_ = voxelGridGauss.getSourseCloud();

    voxelGridGauss.Run();

    point_with_gauss = voxelGridGauss.getPointWithGauss();
    center_cloud = voxelGridGauss.getCenterPoint();
    pcl::PointXYZ minAABB, maxAABB;
    minAABB = voxelGridGauss.getMinAABB();
    maxAABB = voxelGridGauss.getMaxAABB();

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Voxelization"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCube(minAABB.x, maxAABB.x, minAABB.y, maxAABB.y,
                    minAABB.z, maxAABB.z, 1.0, 1.0, 1.0, "AABB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");

    if (flag == "with_voxel") {
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> v_color(cloud_, "z");
        viewer->addPointCloud<pcl::PointXYZ>(cloud_, v_color, "vertices color");
        for (int i = 0; i < center_cloud.size(); i++) {

            double x = center_cloud[i].x;
            double y = center_cloud[i].y;
            double z = center_cloud[i].z;

            Eigen::Vector3f center(floor(x / grid) * grid + grid / 2,
                                   floor(y / grid) * grid + grid / 2,
                                   floor(z / grid) * grid + grid / 2);

            Eigen::Quaternionf rotation(1, 0, 0, 0);
            string cube = "AABB" + to_string(i);
            viewer->addCube(center, rotation, grid, grid, grid, cube);

            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                                pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                                cube);
        }
    } else if (flag == "without_voxel") {
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> v_color(cloud_, "z");
        viewer->addPointCloud<pcl::PointXYZ>(cloud_, v_color, "vertices color");
    } else if (flag == "gauss_without_voxel") {
        pcl::PointCloud<pcl::PointXYZ>::Ptr p{new pcl::PointCloud<pcl::PointXYZ>()};
        for (int i = 0; i < point_with_gauss.size(); ++i) {
            *p += point_with_gauss[i];
        }
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> v_color(cloud_, "z");
        viewer->addPointCloud<pcl::PointXYZ>(p, v_color, "vertices color");
    } else if (flag == "gauss_with_voxel") {
        pcl::PointCloud<pcl::PointXYZ>::Ptr p{new pcl::PointCloud<pcl::PointXYZ>()};
        for (int i = 0; i < point_with_gauss.size(); ++i) {
            *p += point_with_gauss[i];
        }
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> v_color(cloud_, "z");
        viewer->addPointCloud<pcl::PointXYZ>(p, v_color, "vertices color");
        for (int i = 0; i < center_cloud.size(); i++) {

            double x = center_cloud[i].x;
            double y = center_cloud[i].y;
            double z = center_cloud[i].z;

            Eigen::Vector3f center(floor(x / grid) * grid + grid / 2,
                                   floor(y / grid) * grid + grid / 2,
                                   floor(z / grid) * grid + grid / 2);

            Eigen::Quaternionf rotation(1, 0, 0, 0);
            string cube = "AABB" + to_string(i);
            viewer->addCube(center, rotation, grid, grid, grid, cube);

            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                                pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                                cube);
        }
    }else{
        std::cout << "error " << std::endl;
    }


    while (!viewer->wasStopped()) {

        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }

    return 0;
}