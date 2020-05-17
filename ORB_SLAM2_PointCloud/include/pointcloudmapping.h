/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"
#include <condition_variable>
 

 
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/features/normal_3d.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/compression_profiles.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include<pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include<pcl/filters/passthrough.h>
 #include<pcl/filters/voxel_grid.h>
 
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
namespace ORB_SLAM2
{

class PointCloudMapping
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    
    PointCloudMapping( double resolution_ );
    ~PointCloudMapping();
    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame( KeyFrame* kf, cv::Mat& color, cv::Mat& depth );
    void shutdown();
    void viewer();
    void getGlobalCloudMap(pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr &outputMap);
    void reset();

protected:
    PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);
    PointCloud::Ptr generatePointCloud(cv::Mat& color, cv::Mat& depth);
     
    PointCloud::Ptr globalMap;
    PointCloud::Ptr Part_tem;//局部地图,用于大尺度场景下的重建
    shared_ptr<thread>  viewerThread;   
    
    bool    shutDownFlag    =false; // 程序退出标志位
    mutex   shutDownMutex;  
    
    bool    dataupdate    =false;       //数据更新标志位
    condition_variable  keyFrameUpdated;
    mutex               keyFrameUpdateMutex;
    
    // data to generate point clouds
    vector<KeyFrame*>       keyframes;
    vector<cv::Mat>         colorImgs,depthImgs;
    cv::Mat   depthImg,colorImg,mpose;
 
  
    vector<PointCloud::Ptr>   mvPointClouds; //存储点云序列
    vector<PointCloud::Ptr>   mvPointCloudsForMatch;
    vector<cv::Mat>   mvPosePointClouds;
    unsigned long int  mpointcloudID=0;
    
    mutex                   keyframeMutex;
    uint16_t                lastKeyframeSize =0;
    
    double resolution = 0.04;
    pcl::VoxelGrid<PointT>  voxel; //显示精度
    pcl::VoxelGrid<PointT>  voxelForMatch;//用于滤波得到ＩＣＰ匹配的点云
    float cx=0,cy=0,fx=0,fy=0;
    
   //pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> * PointCloudEncoder; //点云
  // pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudDecoder;
    
   g2o::SparseOptimizer globaloptimizer;
   
void compressPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,std::stringstream& compressedData);
void depressPointCloud(std::stringstream& compressedData,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloudOut);
   
  void readResultOptimizer(void);
  void addVertexToOptimizer(long unsigned int ID, Eigen::Isometry3d T);
  void addEdgeToOptimizer(long unsigned int ID1,long unsigned int ID2, Eigen::Isometry3d T );
  void computeTranForTwoPiontCloud(pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr &P1,
				                                                   pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr &P2,   Eigen::Isometry3d &T );
void computeTranForTwoPiontCloud2(pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr &P1,
						                                     pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr &P2, Eigen::Isometry3d&  T );
  Eigen::Matrix4f cvMat2Eigen(const cv::Mat &cvT);
};
}
#endif // POINTCLOUDMAPPING_H
