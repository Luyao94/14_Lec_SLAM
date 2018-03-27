//C++ 標準庫
#include <iostream>
#include <string>

//opencv 庫
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/opencv.hpp>


// PCL 庫
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
using namespace std;


// 定義點雲類型
typedef pcl::PointXYZRGBA PoinT;
typedef pcl::PointCloud<PoinT> PointCloud;


// 相機內參
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;


// 主函數

int main()
{
	// 讀取./data/rgb.png 和 ./data/depth.png, 幷轉化成點雲

	// 圖像矩陣
	cv::Mat rgb, depth;
	// 使用cv::imread()來讀取圖像
	// API:
	
	rgb = cv::imread("/home/maoli/RGBD_tutorial/code/src/data/RGB.png");
	depth = cv::imread("/home/maoli/RGBD_tutorial/code/src/data/depth.png",-1);
	

	// 智能指針，創建一個空點雲。這種指針用完自動釋放
	PointCloud::Ptr cloud (new PointCloud);
	
	// 遍歷深度圖
	for (int m = 0;m<depth.rows;m++)
	{
		for (int n = 0;n<depth.cols;n++)
		{
			//獲取深度圖中(m,n)處的值
			ushort d = depth.ptr<ushort>(m)[n];
			//d可能沒有值，若如此，跳過此點
			if (d==0)
				continue;
			// d存在值，則向點雲增加一個點
			PoinT p;

			// 計算這個點的空間座標
			p.z = double(d)/camera_factor;
			p.x = (n-camera_cx)*p.z/camera_fx;
			p.y = (m-camera_cy)*p.z / camera_fy;

			// 從rgb圖像中獲取該點顏色
			// rgb是三通道的rgb格式圖，所以按下面的順序獲取顏色
			p.b = rgb.ptr<uchar>(m)[n*3];
			p.g = rgb.ptr<uchar>(m)[n*3+1];
			p.r = rgb.ptr<uchar>(m)[n*3+2];
			// 把p加入到點雲中
			cloud->points.push_back(p);


		}
		
	}
	// 設置幷保存點雲
		cloud->height = 1;
		cloud->width = cloud->points.size();
		cout<<"point cloud size= "<<cloud->points.size()<<endl;
		cloud->is_dense = false;
		pcl::io::savePCDFile("/home/maoli/RGBD_tutorial/code/src/data/pointcloud.pcd", *cloud);
		// 清除數據幷退出
		cloud->points.clear();
		cout<<"point cloud saved."<<endl;

	return 0;
}


