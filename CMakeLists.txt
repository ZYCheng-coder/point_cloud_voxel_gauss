cmake_minimum_required(VERSION 3.10)
project(point_cloud_cube)

set(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED COMPONENTS
        pcl_conversions
        pcl_ros
        roscpp
        sensor_msgs)
#catkin_package()
find_package(PCL 1.8 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

add_executable(get_pcd get_pcd.cpp)
target_link_libraries(get_pcd ${catkin_LIBRARIES})

add_executable(main main.cpp
        voxel_random_gauss.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(deal_datasets deal_datasets.cpp
        voxel_random_gauss.cpp)
target_link_libraries(deal_datasets ${catkin_LIBRARIES} ${PCL_LIBRARIES})