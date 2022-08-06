#1 main 处理单帧点云并且进行可视化
###1 可视化单独的点云,未增加噪声
./main hair1.pcd 0.015 without_voxel
###2 可视化点云和voxel公视
./main hair1.pcd 0.015 with_voxel
###3 可视化点云增加噪声，不显示voxel
./main hair1.pcd 0.015 gauss_without_voxel
###4 可视化点云增加噪声，显示voxel
./main hair1.pcd 0.015 gauss_with_voxel

#2 deal_datasets 处理rosbag并且保存为pcd文件