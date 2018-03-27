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
#include <map>

using namespace std;

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/nonfree/features2d.hpp>
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
// 封裝FRAME
struct FRAME 
{
	cv::Mat rgb, depth;
	cv::Mat desp;// define descriptor
	vector<cv::KeyPoint> kp;

};

// PnP 結果

struct RESULT_OF_PNP
{
	cv::Mat rvec, tvec;
	int inliers;
};


// 函數接口
// image2PointCloud 將rgb圖轉換爲點雲

PointCloud::Ptr image2PointCloud(cv::Mat& rgb,cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera);

// point2dTo3d 將單個點從圖像座標轉換爲空間座標
// input: 3維點Point3f(u,v,d)


cv::Point3f point2dTo3d(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera);




// 參數讀取類
class ParameterReader
{
public:
	ParameterReader(string filename="../src/parameters.txt")
	{
		ifstream fin(filename.c_str());
		if (!fin)
		{
			cerr<<"parameter file does not exist."<<endl;
			return ;
		}
		while(!fin.eof())
		{
			string str;
			getline(fin,str);
			if (str[0] == '#')
			{
				// 以‘#’ 開頭爲註釋
				continue;
			}
			int pos = str.find("=");
			if (pos == -1)
				continue;
			string key = str.substr(0, pos);
			string value = str.substr(pos+1,str.length());
			data[key] = value;
			
			if(!fin.good())
				break;
		}
	}

	string getData(string key)
	{
		map<string, string>::iterator iter = data.find(key);
		if (iter == data.end())
		{
			cerr<<"Parameter name "<<key<<" not found"<<endl;
			return string("NOT_FOUND");
		}
		return iter->second;
	}
public:
	map<string, string> data;
};




// 封裝PnP Frame computerKeyPointsAndDesp


// computeKeyPointsAndDesp
void computeKeyPointsAndDesp(FRAME& frame,string detector, string descriptor);
// estimateMotion
// 輸入幀1，幀2，相機內參
RESULT_OF_PNP estimateMotion(FRAME& frame1,FRAME& frame2,CAMERA_INTRINSIC_PARAMETERS& camera);




