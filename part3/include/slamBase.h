/*************************************************************************
> File Name: rgbd-slam-tutorial-gx/part III/code/include/slamBase.h
> Author: xiang gao/Mao Li
> Mail: gaoxiang12@mails.tsinghua.edu.cn
> Created Time: 2015年07月18日 星期六 15时14分22秒
> 说明：rgbd-slam教程所用到的基本函数（C风格）
******************************************************************/

// 各種頭文件
// C++標準庫
#include <fstream>
#include <vector>

using namespace std;

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// 類型定義
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相機內參
struct CAMERA_INTRINSIC_PARAMETERS
{
	double cx,cy,fx,fy,scale;
};

// 函數接口
// image2PointCloud 將rgb圖轉換爲點雲

PointCloud::Ptr image2PointCloud(cv::Mat&rgb,cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera);

// point2dTo3d 將單個點從圖像座標轉換爲空間座標
// input: 3維點Point3f(u,v,d)


cv::Point3f point2dTo3d(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera);





