/*************************************************************************
File Name: /VisualOdometry.cpp
Author: xiang gao/Mao
Mail: gaoxiang12@mails.tsinghua.edu.cn
Created Time: 2015年08月01日 星期六 15时35分42秒          
************************************************************************/


#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#include "slamBase.h"

int main()
{
	ParameterReader pd;
	int startIndex = atoi(pd.getData("start_index").c_str());
	int endIndex   = atoi(pd.getData("end_index").c_str());

	// initialize

	cout<<"Initializing ..."<<endl;
	int currIndex = startIndex; //當前索引爲currIndex
	FRAME lastFrame = readFrame(currIndex, pd);// 上一幀數據
	// 我們總是在比較currFrame和lastFRAME
	string detector = pd.getData("detector");
	string descriptor = pd.getData("descriptor");
	CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
	computeKeyPointsAndDesp(lastFrame, detector, descriptor);
	PointCloud::Ptr cloud = image2PointCloud(lastFrame.rgb, lastFrame.depth, camera);

	pcl::visualization::CloudViewer viewer("viewer");

	// 是否顯示點雲
	bool visualize = pd.getData("visualize_pointcloud")==string("yes");

	int min_inliers = atoi(pd.getData("min_inliers").c_str());
	double max_norm = atof(pd.getData("max_norm").c_str());

	for(currIndex=startIndex+1; currIndex<endIndex; currIndex++)
	{
		cout<<"Readinf files"<<currIndex<<endl;
		FRAME currFrame = readFrame(currIndex,pd);// 讀取currFrame
		computeKeyPointsAndDesp(currFrame, detector, descriptor);
		// 比較currFrame 和 lastFrame
		RESULT_OF_PNP result = estimateMotion(lastFrame, currFrame, camera);
		if(result.inliers<min_inliers)// inliers不夠，放棄該幀
			continue;
		// 計算運動範圍是否太大
		double norm = normofTransform(result.rvec, result.tvec);
		cout<<"norm = "<<norm<<endl;
		if(norm>=max_norm)
			continue;
		Eigen::Isometry3d T = cvMat2Eigen(result.rvec, result.tvec);
		cout<<"T= "<<T.matrix()<<endl;

		// cloud = joinPointCloud(cloud, currFrame, T.inverse(), camera);
		cloud = joinPointCloud(cloud, currFrame, T, camera);

		if(visualize == true)
			viewer.showCloud(cloud);
		lastFrame = currFrame;

	}

	pcl::io::savePCDFile("../data/result.pcd", *cloud);
	return 0;



}
