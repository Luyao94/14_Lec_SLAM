/*************************************************************************
 File Name: src/jointPointCloud.cpp
 Author: Xiang gao/ Mao Li
 Mail: gaoxiang12@mails.tsinghua.edu.cn 
 Created Time: 2015年07月22日 星期三 20时46分08秒
 ************************************************************************/


#include<iostream>

using namespace std;

#include "slamBase.h"

#include <opencv2/core/eigen.hpp>

#include <pcl/common/transforms.h>

#include <pcl/visualization/cloud_viewer.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

int main()
{
	// 本節要拼接data中的兩對圖像
	ParameterReader pd;
	// 聲明兩個幀
	FRAME frame1, frame2;

	// 讀取圖像
	frame1.rgb = cv::imread("../data/rgb1.png");
	frame1.depth = cv::imread("../data/depth1.png",-1);
	frame2.rgb = cv::imread("../data/rgb2.png");
	frame2.depth = cv::imread("../data/depth2.png",-1);
	
	if(frame1.rgb.empty()) cout<<"NOT FIND image1!"<<endl;
	if(frame2.rgb.empty()) cout<<"NOT FIND image2!"<<endl;
	if(frame1.depth.empty()) cout<<"NOT FIND depth1!"<<endl;
	if(frame2.depth.empty()) cout<<"NOT FIND depth2!"<<endl;


	// 提取特徵幷計算描述子
	cout<<"extracting features"<<endl;
	string detecter = pd.getData("detector");
	string descriptor = pd.getData("descriptor");

	computeKeyPointsAndDesp(frame1,detecter,descriptor);
	computeKeyPointsAndDesp(frame2,detecter,descriptor);

	// 相機內參 讀取相機內參
	CAMERA_INTRINSIC_PARAMETERS camera;
	camera.fx = atof(pd.getData("camera.fx").c_str());
	camera.fy = atof(pd.getData("camera.fy").c_str());
	camera.cx = atof(pd.getData("camera.cx").c_str());
	camera.cy = atof(pd.getData("camera.cy").c_str());
	camera.scale = atof(pd.getData("camera.scale").c_str());

	cout<<"solving pnp"<<endl;
	// 求解pnp
	RESULT_OF_PNP result = estimateMotion(frame1,frame2,camera);

	cout<<result.rvec<<endl<<result.tvec<<endl;

	// 處理result
	// 將旋轉向量轉化爲旋轉矩陣
	cv::Mat R;
	cv::Rodrigues(result.rvec,R);
	Eigen::Matrix3d r;
	cv::cv2eigen(R,r);// onlinedoc 沒有這個函數但是opencv源代碼有

	// 將平移向量和旋轉矩陣轉換成變換矩陣
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

	Eigen::AngleAxisd angle(r);

	cout<<"translation"<<endl;
	Eigen::Translation<double,3> trans(result.tvec.at<double>(0,0), result.tvec.at<double>(0,1), result.tvec.at<double>(0,2));
	T = angle;
	T(0,3) = result.tvec.at<double>(0,0);
	T(1,3) = result.tvec.at<double>(0,1);
	T(2,3) = result.tvec.at<double>(0,2);
	// 轉換點雲
	cout<<"converting image to clouds"<<endl;
	PointCloud::Ptr cloud1 = image2PointCloud(frame1.rgb, frame1.depth, camera);
	PointCloud::Ptr cloud2 = image2PointCloud(frame2.rgb, frame2.depth, camera);

	// 合併點雲
	cout<<"Break point"<<endl;
	//pcl::visualization::CloudViewer viewer_cloud1("cloud1");
	//viewer_cloud1.showCloud(cloud1);
	//pcl::visualization::CloudViewer viewer_cloud2("cloud2");
	//viewer_cloud2.showCloud(cloud2);
	pcl::io::savePCDFile("../data/cloud1.pcd", *cloud1);
	pcl::io::savePCDFile("../data/cloud2.pcd", *cloud2);

	PointCloud::Ptr output(new PointCloud());
	pcl::transformPointCloud(*cloud1, *output, T.matrix());
	*output += *cloud2;
	pcl::io::savePCDFile("../data/result.pcd", *output);
	cout<<"Final result saved."<<endl;

	pcl::visualization::CloudViewer viewer("viewer");
	viewer.showCloud(output);
	while(!viewer.wasStopped())
	{
	
	}
	return 0;
	
}











