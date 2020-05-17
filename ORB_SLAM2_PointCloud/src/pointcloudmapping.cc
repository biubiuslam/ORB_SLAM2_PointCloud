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
#include <chrono>
#include <ctime>
#include <climits>

#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include "Converter.h"

#include "pointcloudmapping.h"
 
namespace ORB_SLAM2
{
	
/*
 * 
 * @ 设置点云分辨率
 */
PointCloudMapping::PointCloudMapping(double resolution_)
{
   this->resolution = resolution_;
   this->resolution =0.005;
    voxel.setLeafSize( resolution, resolution, resolution);
    voxelForMatch.setLeafSize( 0.1,0.1, 0.1);
    globalMap = boost::make_shared< PointCloud >( );
    Part_tem = boost::make_shared< PointCloud >( );
 
 
   // 初始化全局优化器
//     g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
//     linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
//     g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
//     g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
//     globaloptimizer.setAlgorithm(solver);
//     globaloptimizer.setVerbose(false);// 不打印调试信息
    
    lastKeyframeSize=0;
    
    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
}
PointCloudMapping::~PointCloudMapping()
{
     viewerThread->join();
}

// 复位点云显示模块
void PointCloudMapping::reset()
{
      mvPosePointClouds.clear();
      mvPointClouds.clear();
      mpointcloudID=0;
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
    }
 
  string save_path = "/home/biubiu/ORB_SLAM2_PointCloud/resultPointCloudFile.pcd";
  pcl::io::savePCDFile(save_path,*globalMap);
  cout<<"save pcd files to :  "<<save_path<<endl;
  
  //if(mpointcloudID == mvPosePointClouds.size())
 // {
 //     stringstream ss1;
 //     for(int i=0;i<mpointcloudID;i++)
 //     {
// 	cv::Mat pose_tem;
// 	pose_tem = mvPosePointClouds[i];
//	ss1<<i<<",";
// 	ss1<<pose_tem.at<float>(0,0)<<","<<pose_tem.at<float>(0,1)<<","<<pose_tem.at<float>(0,2)<<","<<pose_tem.at<float>(0,3)<<",";
// 	ss1<<pose_tem.at<float>(1,0)<<","<<pose_tem.at<float>(1,1)<<","<<pose_tem.at<float>(1,2)<<","<<pose_tem.at<float>(1,3)<<",";
//  	ss1<<pose_tem.at<float>(2,0)<<","<<pose_tem.at<float>(2,1)<<","<<pose_tem.at<float>(2,2)<<","<<pose_tem.at<float>(2,3)<<",";
// 	ss1<<pose_tem.at<float>(3,0)<<","<<pose_tem.at<float>(3,1)<<","<<pose_tem.at<float>(3,2)<<","<<pose_tem.at<float>(3,3)<<",";
    //  }
      //cout<<ss1.str()<<endl;
  //}
 
}
// 由外部函数调用，每生成一个关键帧调用一次该函数
void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
     unique_lock<mutex> lck(keyframeMutex);
   cv::Mat T =kf->GetPose();
    mvPosePointClouds.push_back(T.clone());    // 每个点云的pose
    colorImgs.push_back( color.clone() );
    depthImgs.push_back( depth.clone() );
    
//     colorImg =color.clone();
//     depthImg =  depth.clone();
//     mpose = kf->GetPose();
    if(cx ==0 || cy ==0||fx==0||fy==0)
    {
	  cx = kf->cx;
	  cy = kf->cy;
	  fx = kf->fx;
	  fy = kf->fy;
    }
    keyFrameUpdated.notify_one();
    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
}
/*
 * @function 根据获取的关键帧位置、姿态和图像数据生产点云
 * 
 * 
 * 
 */
pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp( new PointCloud() );
    // point cloud is null ptr
    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>10)
                continue;
            PointT p;
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;  
            
            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];
                
            tmp->points.push_back(p);
        }
    }
  
    // rotation the pointcloud and stiching 
    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;
    voxel.setInputCloud( cloud );
    voxel.filter( *tmp );
    cloud->swap( *tmp );
    
       // save pointcloud to disk 
    //mvPosePointClouds.push_back(kf->GetPose());
    //mvPointClouds.push_back(*tmp);
    //mpointcloudID++;
    //stringstream ss;
    //ss << "/home/crp/dataset/rgbd_dataset_freiburg1_room/pointcloud/"<< mpointcloudID<<".pcd";
    //pcl::io::savePCDFile(ss.str(),*tmp);

     
    cout<<"generate point cloud from  kf-ID:"<<kf->mnId<<", size="<<cloud->points.size()<<endl;
     
    return cloud;
}
/*
 * @function 更加关键帧生成点云、并对点云进行滤波处理
 * 
 * 备注：点云生成函数在　台式机上调用时间在0.1ｓ 左右
 */
pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud( cv::Mat& color, cv::Mat& depth)
{
     chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    PointCloud::Ptr tmp( new PointCloud() );// point cloud is null ptr
    
    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>10)
                continue;
            PointT p;
            p.z = d;
            p.x = ( n - cx) * p.z / fx;
            p.y = ( m -cy) * p.z / fy;  
            
            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];
                
            tmp->points.push_back(p);
        }
    }
// rotation the pointcloud and stiching 
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity() ;
    PointCloud::Ptr cloud1(new PointCloud);
    PointCloud::Ptr cloud2(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud1, T.inverse().matrix());
     pcl::transformPointCloud( *tmp, *cloud2, T.inverse().matrix()); //专用于点云匹配
     
    cloud1->is_dense = false;
    voxel.setInputCloud( cloud1 );
    voxel.filter( *tmp );
    cloud1->swap( *tmp );
    
  
     cloud2->is_dense = false;
    voxelForMatch.setInputCloud( cloud2 );
    voxelForMatch.filter( *tmp );
    cloud2->swap( *tmp );
   // addVertexToOptimizer(mpointcloudID,mpose);
   
    mvPointClouds.push_back(cloud1) ;    //点云数据 
     mvPointCloudsForMatch.push_back(cloud2) ;    
    mpointcloudID++;
     cout<<"generate point cloud from  kf-ID:"<<mpointcloudID<<", size="<<cloud1->points.size();
     
    //stringstream ss;
    //ss << "/home/crp/dataset/rgbd_dataset_freiburg1_room/pointcloud/"<< mpointcloudID<<".pcd";
    //pcl::io::savePCDFile(ss.str(),*tmp);
  
     chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
     chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
     cout<<"   cost time: "<<time_used.count()*1000<<" ms ."<<endl;
     
    return cloud1;
}

/*
 * 
 * 原来版本的显示函数
 * 由于随着尺寸的增加以后,显示函数会异常退出
 */
  void PointCloudMapping::viewer()
{
 
    pcl::visualization::CloudViewer viewer("viewer");
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }
        
        // keyframe is updated 
        // 这种方式下每次都需要吧队列里面所有的关键帧都去生成点云，效率不高 
        size_t N=0,i=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N =mvPosePointClouds.size();
        }
	for (  i=lastKeyframeSize; i<N ; i++ )
	{
		//PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
		//*globalMap += *p;
	      if((mvPosePointClouds.size() != colorImgs.size() )|| (mvPosePointClouds.size()!= depthImgs.size() ) || (depthImgs.size() != colorImgs.size() ))
	      {
		                 cout<<" depthImgs.size != colorImgs.size()  "<<endl;
				 continue;
	      }
		cout<<"i: "<<i<<"  mvPosePointClouds.size(): "<<mvPosePointClouds.size()<<endl;
		//cout<<"depthImgs.size(): "<<depthImgs.size()<<endl;
		//cout<<"colorImgs.size(): "<<colorImgs.size()<<endl;
		
		PointCloud::Ptr tem_cloud1( new PointCloud() );
		PointCloud::Ptr tem_cloud2(new PointCloud);
		tem_cloud1 = generatePointCloud(colorImgs[i], depthImgs[i]); //生成一幅点云大约在０．１s左右
  		//*tem_cloud1 = mvPointClouds[i] ;    //点云数据 
   		
		Eigen::Isometry3d T_c2w =ORB_SLAM2::Converter::toSE3Quat( mvPosePointClouds[i]);
		
		Eigen::Isometry3d T_cw= Eigen::Isometry3d::Identity();
		if(mvPointClouds.size()>1)
		{
				Eigen::Isometry3d T_c1w =ORB_SLAM2::Converter::toSE3Quat( mvPosePointClouds[i-1]);
				Eigen::Isometry3d T_c1c2 = T_c1w*T_c2w.inverse();// T_c1_c2
			 				
				PointCloud::Ptr tem_match_cloud1 =mvPointCloudsForMatch[i-1];
				PointCloud::Ptr tem_match_cloud2 =mvPointCloudsForMatch[i];
	 
				//PointCloud::Ptr cloud(new PointCloud);
				
				//pcl::transformPointCloud( *tem_match_cloud2, *cloud, T_c1c2.matrix());
				computeTranForTwoPiontCloud2(tem_match_cloud1,tem_match_cloud2,T_c1c2);// 计算cloud1 cloud2 之间的相对旋转变换
				 
		               T_cw = T_c1c2*T_c1w;
			      // T_tem = T_tem.inverse()*T_c1w;
			     // T_cw=T_tem.inverse().matrix();
		}
             //  std::stringstream compressedData;     
	     //  compressPointCloud(tem_cloud1,compressedData);
	     //  depressPointCloud(compressedData,tem_cloud1);
 
 		pcl::transformPointCloud( *tem_cloud1, *tem_cloud2, T_c2w.inverse().matrix());
 		//pcl::transformPointCloud( *tem_cloud1, *tem_cloud2,T_cw.inverse().matrix());
		
 		*globalMap += *tem_cloud2;
	}
	lastKeyframeSize = i;
	 viewer.showCloud( globalMap );
        cout<<"show global map, size="<<globalMap->points.size()<<endl;	
	    // 删除已经存储的点云数据
//     for(vector<int>::iterator iter=colorImgs.begin(); iter<=colorImgs.begin()+5;)
// 	  iter = colorImgs.erase(iter);
//     for(vector<int>::iterator iter=depthImgs.begin(); iter<=depthImgs.begin()+5;)
// 	  iter = depthImgs.erase(iter);
	
	}
   
}
// 压缩点云需要４０-50ms　解压缩需要２０-30ｍｓ
// 参考：　http://www.voidcn.com/article/p-zbhwxhfh-bpr.html
void PointCloudMapping::compressPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,std::stringstream& compressedData)
{
  // 压缩选项详见 /io/include/pcl/compression/compression_profiles.h
//  - LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR：分辨率1立方厘米，压缩完之后无颜色，快速在线编码 
// - LOW_RES_ONLINE_COMPRESSION_WITH_COLOR：分辨率1立方厘米，压缩完之后有颜色，快速在线编码 
// - MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR：分辨率5立方毫米，压缩完之后无颜色，快速在线编码 
// - MED_RES_ONLINE_COMPRESSION_WITH_COLOR：分辨率5立方毫米，压缩完之后有颜色，快速在线编码 
// - HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR：分辨率1立方毫米，压缩完之后无颜色，快速在线编码 
// - HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR：分辨率1立方毫米，压缩完之后有颜色，快速在线编 
// - LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR：分辨率1立方厘米，压缩完之后无颜色，高效离线编码 
// - LOW_RES_OFFLINE_COMPRESSION_WITH_COLOR：分辨率1立方厘米，压缩完之后有颜色，高效离线编码 
// - MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR：分辨率5立方毫米，压缩完之后无颜色，高效离线编码 
// - MED_RES_OFFLINE_COMPRESSION_WITH_COLOR：分辨率5立方毫米，压缩完之后有颜色，高效离线编码 
// - HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR：分辨率1立方毫米，压缩完之后无颜色，高效离线编码 
// - HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR：分辨率1立方毫米，压缩完之后有颜色，高效离线编码
    chrono::steady_clock::time_point t1= chrono::steady_clock::now();
    bool showStatistics=true;                       //设置在标准设备上输出打印出压缩结果信息
    pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> * PointCloudEncoder;
    PointCloudEncoder=new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> (compressionProfile, showStatistics);
    //std::stringstream compressedData;// 存储压缩点云的字节流对象
    PointCloudEncoder->encodePointCloud(cloud, compressedData);// 压缩点云
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"\t encode point cloud  cost time: "<<time_used.count()*1000<<"ms  seconds."<<endl;
}
void PointCloudMapping::depressPointCloud(std::stringstream& compressedData,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloudOut)
{
   chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudDecoder;
  PointCloudDecoder=new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> ();
  PointCloudDecoder->decodePointCloud (compressedData, cloudOut);// 解压缩点云
  //pcl::visualization::CloudViewer viewer("viewer");
  //viewer.showCloud (cloudOut);//可视化解压缩点云
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
  cout<<"\t decode point cloud  cost time: "<<time_used.count()*1000<<"ms  seconds."<<endl;
}

//--------------------------------------------------优化器--------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------------------------------//
  // 读取优化结果
void PointCloudMapping::readResultOptimizer(void)
 {
//       g2o::VertexSE3*v = dynamic_cast<g2o::VertexSE3*>(globaloptimizer.vertex(1));
//       Eigen::Isometry3d pose = v->estimate();
//       cout<<"pose matrix :"<<endl<<pose.matrix()<<endl;
   
}

 
 // 添加顶点
void PointCloudMapping::addVertexToOptimizer(long unsigned int ID, Eigen::Isometry3d T)
{
//       g2o::VertexSE3 * vSE3 = new g2o::VertexSE3();
//       vSE3->setEstimate(T); //初始位姿
//       vSE3->setId(ID);
//       vSE3->setFixed(ID == 0);
//       globaloptimizer.addVertex(vSE3);
}
   // 添加边
void PointCloudMapping::addEdgeToOptimizer(long unsigned int ID1,long unsigned int ID2, Eigen::Isometry3d T )
{
//       Eigen::Matrix<double,6,6> info =  Eigen::Matrix <double,6,6> ::Identity();
//       //g2o::RobustKernel*robustKernel = g2o::RobustKernelFactory::instance()->construct("Cauchy");
// 
//        g2o::EdgeSE3* edge = new g2o::EdgeSE3();//边信息
//       edge->setMeasurement(T.inverse());
//       // edge->setInformation(info);
//       //edge->setRobustKernel(robustKernel);
//       edge->vertices()[0] = globaloptimizer.vertex(ID1);
//        edge->vertices()[1] = globaloptimizer.vertex(ID2);
//       globaloptimizer.addEdge(edge);
  
}
// 使用ＩＣＰ计算两帧点云的相对变换
void PointCloudMapping::computeTranForTwoPiontCloud(pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr &P1,
						                                                                              pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr &P2,
															      Eigen::Isometry3d& T )
{
  	  Eigen::Matrix<float,4,4> Tcw;
	  Eigen::Matrix<double,3,3> R;
  	  Eigen::Matrix<double, 3, 1> t; 
	  T = Eigen::Isometry3d::Identity();
	  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
	  
	  pcl::PointCloud<pcl::PointXYZRGBA>   finalpointcloud;
	  pcl::IterativeClosestPoint<pcl::PointXYZRGBA,pcl::PointXYZRGBA> icp;
	  icp.setInputCloud(P1);
	  icp.setInputTarget(P2);
	  icp.setTransformationEpsilon(1e-6); // 设置迭代精度
	   // icp.setMaximumIterations(5);
	  icp.align(finalpointcloud);
	  
	  Tcw = icp.getFinalTransformation();
	  R(0,0)=Tcw(0,0);R(0,1)=Tcw(0,1);R(0,2)=Tcw(0,2);
	  R(1,0)=Tcw(1,0);  R(1,1)=Tcw(1,1);  R(1,2)=Tcw(1,2);
	  R(2,0)=Tcw(2,0);  R(2,1)=Tcw(2,1);  R(2,2)=Tcw(2,2);
	  
	 t(0,0)=Tcw(0,3);t(1,0)=Tcw(1,3); t(2,0)=Tcw(2,3);
	  
	  T.rotate(R);
	  T.pretranslate(t);
	  std::cout <<"T: "<<T.matrix()<< std::endl;
	  
	  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
	  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
	  cout<<"\t ICP algin  cost time: "<<time_used.count()*1000<<"ms  seconds."<<endl;
}
// 使用ＩＣＰ计算两帧点云的相对变换
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//以< x, y, z, curvature >形式定义一个新的点
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
    MyPointRepresentation ()
    {
        //定义尺寸值
        nr_dimensions_ = 4;
    }
    //覆盖copyToFloatArray方法来定义我们的特征矢量
    virtual void copyToFloatArray (const PointNormalT &p, float * out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};	

void PointCloudMapping::computeTranForTwoPiontCloud2(pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr &P1,
						                                                                              pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr &P2,
															     Eigen::Isometry3d&  T )
{
    	  Eigen::Matrix<float,4,4> Tcw;
	  Eigen::Matrix<double,3,3> R;
  	  Eigen::Matrix<double, 3, 1> t; 
	  
      //计算曲面法线和曲率
    PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);
    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    //norm_est.setSearchMethod (tree);
    norm_est.setKSearch (30);
    norm_est.setInputCloud (P1);
    norm_est.compute (*points_with_normals_src);
    pcl::copyPointCloud (*P1, *points_with_normals_src);
    norm_est.setInputCloud (P2);
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*P2, *points_with_normals_tgt);
//     
//     //举例说明我们自定义点的表示（以上定义）
//     MyPointRepresentation point_representation;
//     //调整'curvature'尺寸权重以便使它和x, y, z平衡
//     float alpha[4] = {1.0, 1.0, 1.0, 1.0};
//     point_representation.setRescaleValues (alpha);
//     //
    // 配准
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
    reg.setTransformationEpsilon (1e-6);
    //将两个对应关系之间的(src<->tgt)最大距离设置为10厘米
    //注意：根据你的数据集大小来调整
    reg.setMaxCorrespondenceDistance (0.1);  
//     //设置点表示
//    // reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));
     reg.setInputCloud (points_with_normals_src);
     reg.setInputTarget (points_with_normals_tgt);

//     //在一个循环中运行相同的最优化并且使结果可视化
//     Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
     PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations (2);
     reg.align (*reg_result);
     
    Tcw = reg.getFinalTransformation();
    R(0,0)=Tcw(0,0);R(0,1)=Tcw(0,1);R(0,2)=Tcw(0,2);
    R(1,0)=Tcw(1,0);  R(1,1)=Tcw(1,1);  R(1,2)=Tcw(1,2);
    R(2,0)=Tcw(2,0);  R(2,1)=Tcw(2,1);  R(2,2)=Tcw(2,2);

    t(0,0)=Tcw(0,3);t(1,0)=Tcw(1,3); t(2,0)=Tcw(2,3);

    T.rotate(R);
    T.pretranslate(t);
    std::cout <<"T: "<<T.matrix()<< std::endl;
 
//     
}



  void PointCloudMapping::getGlobalCloudMap(pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr &outputMap)
 {
	   unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );   
	   outputMap= globalMap;
}

  Eigen::Matrix4f  PointCloudMapping::cvMat2Eigen(const cv::Mat &cvT)
{
    Eigen::Matrix<float,4,4> T;
    T<< cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),cvT.at<float>(0,3),
         cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),cvT.at<float>(1,3),
         cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2),cvT.at<float>(2,3),
         0,0,0,1;
 
    return  T;
}

// -----end of namespace
}
