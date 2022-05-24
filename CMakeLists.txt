cmake_minimum_required(VERSION 3.22)
project(PCL_filter)

set(CMAKE_CXX_STANDARD 14)

# 设置输出根目录为build/Debug
set(OUTPUT_DIRECTORY_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/build/${CMAKE_BUILD_TYPE})
# 设置可执行程序输出到build/Debug/bin目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
# 设置库文件输出到build/Debug/lib目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib" CACHE PATH "Library directory" FORCE)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib" CACHE PATH "Archive directory" FORCE)


find_package(PCL REQUIRED)
# 包含头文件目录
include_directories(${PCL_INCLUDE_DIRS})
# 设置依赖库链接目录
link_directories(${PCL_LIBRARY_DIRS})
# 添加预处理器和编译器标记
add_definitions(${PCL_DEFINITIONS})


##1.输入输出展示
#add_executable(PCL_RS read_and_save.cpp )
#target_link_libraries(PCL_RS ${PCL_LIBRARIES})
#
##2.直通滤波
#add_executable(pass_throgh_filter pass_throgh_filter.cpp)
#target_link_libraries(pass_throgh_filter ${PCL_LIBRARIES})

#3.统计去噪滤波
#add_executable(PCL_SR statistical_removal.cpp)
#target_link_libraries(PCL_SR ${PCL_LIBRARIES})
##4.kdTREE
#add_executable(KDtree KDtree.cpp )
#target_link_libraries(KDtree ${PCL_LIBRARIES})

#5.octree 搜索
#add_executable(Octree octree.cpp)
#target_link_libraries(Octree ${PCL_LIBRARIES})
#6.体素滤波
#add_executable(voxel_grid_filter voxel_grid_filter.cpp )
#target_link_libraries(voxel_grid_filter ${PCL_LIBRARIES})
##7.统计滤波
#add_executable(statustical_removal  statustical_removal.cpp )
#target_link_libraries(statustical_removal ${PCL_LIBRARIES})
##8.提取点云索引
#add_executable(extr_indices  extr_indices_filter.cpp )
#target_link_libraries(extr_indices ${PCL_LIBRARIES})
#9.range-image深度图像
#add_executable(range_image range_image.cpp )
#target_link_libraries(range_image ${PCL_LIBRARIES})
#10,法向量估计
#add_executable(feature feature.cpp )
#target_link_libraries(feature ${PCL_LIBRARIES})
#11,基于多项式的表面重构
#add_executable(surface_MLS  surface_MLS.cpp )
#target_link_libraries(surface_MLS ${PCL_LIBRARIES})
##12,贪婪投影三角剖分
#add_executable(greedy_projection  greedy_projection.cpp )
#target_link_libraries(greedy_projection ${PCL_LIBRARIES})
#13,随机一致采样——直线
#add_executable(RANSAC_line RANSAC_line.cpp)
#target_link_libraries(RANSAC_line ${PCL_LIBRARIES})
#14，随机一直采样——平面
#add_executable(RANSAC_plane RANSAC_plane.cpp)
#target_link_libraries(RANSAC_plane ${PCL_LIBRARIES})
#14，随机一直采样——平面
#add_executable(seg_plane  seg_plane.cpp)
#target_link_libraries(seg_plane ${PCL_LIBRARIES})
#14，随机一直采样——圆柱
#add_executable(seg_plane seg_cylinder.cpp seg_Euclidean_cluster.cpp)
#target_link_libraries(seg_plane ${PCL_LIBRARIES})
#14,欧式据类分割
#add_executable(seg_Euclidean_cluster seg_Euclidean_cluster.cpp)
#target_link_libraries(seg_Euclidean_cluster ${PCL_LIBRARIES})
#add_executable(seg_Euclidean_cluster2 seg_Euclidean_cluster2.cpp )
#target_link_libraries(seg_Euclidean_cluster2 ${PCL_LIBRARIES})
#15,区域生长
#add_executable(seg_region_growing seg_region_growing.cpp)
#target_link_libraries(seg_region_growing ${PCL_LIBRARIES})
#15,区域生长
#add_executable(seg_region_growing_rgb seg_region_growing_rgb.cpp)
#target_link_libraries(seg_region_growing_rgb ${PCL_LIBRARIES})
#16,pfh
#add_executable(PFH PFH.cpp)
#target_link_libraries(PFH ${PCL_LIBRARIES})
#17,pfh
#add_executable(FPFH FPFH.cpp )
#target_link_libraries(FPFH ${PCL_LIBRARIES})
#18，SAC-IA
#add_executable(SAC-IA SAC-IA.cpp ICP.cpp)
#target_link_libraries(SAC-IA ${PCL_LIBRARIES})
#19 ICP
#add_executable(ICP ICP.cpp )
#target_link_libraries(ICP ${PCL_LIBRARIES})
#19 SAC + ICP
#add_executable(SAC-IA+ICP SAC-IA+ICP.cpp)
#target_link_libraries(SAC-IA+ICP ${PCL_LIBRARIES})
#19 ISS
#add_executable(ISS ISS.cpp)
#target_link_libraries(ISS ${PCL_LIBRARIES})
#19 Harris3D
#add_executable(Harris3D Harris3D.cpp sift.cpp)
#target_link_libraries(Harris3D ${PCL_LIBRARIES})
#20 sift
add_executable(sift  sift.cpp)
target_link_libraries(sift ${PCL_LIBRARIES})