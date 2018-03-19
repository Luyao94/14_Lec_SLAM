/*************************************************************************
> File Name: src/slamBase.cpp
> Author: xiang gao/Mao Li
> Mail: gaoxiang12@mails.tsinghua.edu.cn
> Implementation of slamBase.h
> Created Time: 2015年07月18日 星期六 15时31分49秒
************************************************************************/
#include "slamBase.h"

PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera)
{
	PointCloud::Ptr cloud(new PointCloud);
	
	for(int m = 0;m<depth.rows;m++)
		for (int n=0;n<depth.cols;n++)
		{
			// 獲取深度圖中(m,n)處的值
			ushort d = depth.ptr<ushort>(m)[n];
			// d 可能沒有值，如此則跳過
			if (d = 0)
				continue;
			PointT p;

			// 計算這個點的空間座標
			p.z = double(d)/camera.scale;
			p.x = (n - camera.cx)*p.z/camera.fx;
			p.y = (m - camera.cy)*p.z/camera.fy;

			// 從rgb圖像中獲取他的顏色
			// rgb是三通道的BGR格式圖，所以按下面的順序獲取顏色
			p.b = rgb.ptr<uchar>(m)[n*3];
			p.g = rgb.ptr<uchar>(m)[n*3+1];
			p.r = rgb.ptr<uchar>(m)[n*3+2];

			// 把p加入到點雲中
			cloud->points.push_back(p);

		}
	// 設置幷保存點雲
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cloud->is_dense = false;

	return cloud;
}

cv::Point3f point2dTo3d(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera)
{
	cv::Point3f p;//3D 點
	p.z = double(point.z) / camera.scale;
	p.x = (point.x - camera.cx)*p.z / camera.fx;
	p.y = (point.y - camera.cy)*p.z / camera.fy;
	return p;
}
