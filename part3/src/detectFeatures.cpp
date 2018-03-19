/*************************************************************************
> File Name: detectFeatures.cpp
> Author: xiang gao/Mao Li
> Mail: gaoxiang12@mails.tsinghua.edu.cn
> 特征提取与匹配
> Created Time: 2015年07月18日 星期六 16时00分21秒
************************************************************************/



#include <iostream>
#include "slamBase.h"
using namespace std;

// Opencv 特徵檢測模塊
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>

int main()
{
	// 聲明幷從data文件夾裏讀取兩個rgb與深度圖
	cv::Mat rgb1 = cv::imread("../data/rgb1.png");
	cv::Mat rgb2 = cv::imread("../data/rgb2.png");
	cv::Mat depth1 = cv::imread("../data/depth1.png",-1);
	cv::Mat depth2 = cv::imread("../data/depth2.png",-1);
	
	// 聲明特徵提取器與描述子提取器
	cv::Ptr<cv::FeatureDetector> _detector;
	cv::Ptr<cv::DescriptorExtractor> _descriptor;

	// 構建提取器，默認兩者都爲sift
	// 構建sift, surf之前要初始化nonfree模塊
	cv::initModule_nonfree();
	_detector = cv::FeatureDetector::create("GridSIFT");
	_descriptor = cv::DescriptorExtractor::create("SIFT");

	vector<cv::KeyPoint> kp1, kp2;
	_detector->detect(rgb1, kp1);
	_detector->detect(rgb2, kp2);

	cout<<"Key points of two images:"<<kp1.size()<<", "<<kp2.size()<<endl;
	// 可視化， 顯示關鍵點
	cv::Mat imgShow;
	cv::drawKeypoints(rgb1,kp1,imgShow,cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	cv::imshow("keypoints", imgShow);
	//try{
	cv::imwrite("/home/maoli/RGBD_tutorial/code/part3/data/keypoints.jpg",imgShow);//}
	//catch(runtime_error& ex)
	//{
		fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
	//}
	//cv::waitKey(0); // 暫停等待一個按鍵

	// 計算描述子
	cv::Mat desp1, desp2;
	_descriptor->compute(rgb1, kp1, desp1);
	_descriptor->compute(rgb2, kp2, desp2);

	// 匹配描述子
	vector<cv::DMatch> matches;
	cv::FlannBasedMatcher matcher;
	matcher.match(desp1, desp2, matches);
	cout<<"Find Total "<<matches.size()<<" matches."<<endl;

	// 可視化：顯示匹配的特徵
	cv::Mat imgMatches;
	cv::drawMatches(rgb1, kp1, rgb2, kp2, matches, imgMatches);
	cv::imshow("matches", imgMatches);
	cv::imwrite("/home/maoli/RGBD_tutorial/code/part3/data/matches.jpg", imgMatches);
	
	//cv::waitKey(0);

	// 篩選匹配，把距離大的去掉
	// 這裏使用的準則是去掉大於4倍最小距離的匹配
	vector<cv::DMatch> goodMatches;
	double minDis = 9999;
	for (size_t i=0; i<matches.size(); i++) //size_t 是啥？
	{
		if (matches[i].distance<minDis)
			minDis = matches[i].distance;
	}

	for (size_t i = 0; i<matches.size();i++)
	{
		if(matches[i].distance<4*minDis)
			goodMatches.push_back(matches[i]);

	}
	
	// 顯示 good matches
	cout<<"good matches= "<<goodMatches.size()<<endl;
	cv::drawMatches(rgb1,kp1,rgb2,kp2,goodMatches,imgMatches);
	//cv::waitKey(0);

	// 計算圖像間的運動關係
	// 關鍵函數：cv::solvePnPRansac()
	// 爲調用此函數準備必要的參數

	// 第一個幀的三維點
	vector<cv::Point3f> pts_obj;
	// 第二個幀的圖像點
	vector<cv::Point2f> pts_img;

	// 相機內參
	CAMERA_INTRINSIC_PARAMETERS C;

	C.cx = 325.5;
	C.cy = 253.5;
	C.fx = 518.0;
	C.fy = 519.0;
	C.scale = 1000.0;

	for (size_t i = 0; i<goodMatches.size(); i++)
	{
		// query is the first one, train is the second
		cv::Point2f p = kp1[goodMatches[i].queryIdx].pt;
		// 獲取d是要小心！x是向右的,y是向下的,所以y是行，x是列！
		ushort d = depth1.ptr<ushort>(int(p.y))[int(p.x)];
		if(d==0)
			continue;
		pts_img.push_back(cv::Point2f(kp2[goodMatches[i].trainIdx].pt));
		// 將(u,v,d)轉成（x，y，z）
		cv::Point3f pt (p.x,p.y,d);
		cv::Point3f pd = point2dTo3d(pt,C);
		pts_obj.push_back(pd);
	}

	double camera_matrix_data[3][3] = 
	{
		{C.fx, 0 , C.cx},
		{0, C.fy, C.cy},
		{0, 0,  1}
	};

	// 構建相機矩陣
	cv::Mat cameraMatrix(3,3,CV_64F, camera_matrix_data);
	cv::Mat rvec, tvec, inliers;
	// solve PnP
	cv::solvePnPRansac(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers);
	cout<<"inliers: "<<inliers.rows<<endl;
	cout<<"R= "<<rvec<<endl;
	cout<<"t= "<<tvec<<endl;

	// 畫出inliers匹配
	vector<cv::DMatch> matchesShow;
	for (size_t i = 0; i<inliers.rows; i++)
	{
		matchesShow.push_back(goodMatches[inliers.ptr<int>(i)[0]]);
	}
	cv::drawMatches(rgb1, kp1, rgb2, kp2, matchesShow, imgMatches);
	cv::imshow("inlier matches ", imgMatches);
	cv::imwrite("/home/maoli/RGBD_tutorial/code/part3/data/inliers.jpg", imgMatches);
	//cv::waitKey(0);

	return 0;



}
