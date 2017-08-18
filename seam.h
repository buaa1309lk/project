#ifndef SEAM_H_
#define SEAM_H_
#include "alignment.h"
#include "MinCut\graph.h"

//找到重叠区域的位置坐标
Point2i findRegion(const Mat &a, const Mat &b);

//得到重叠区域的差值图像
Mat absDiff(const Mat &region1, const Mat &region2);

//得到重叠区域的梯度图像
void gradient(const Mat &region, Mat &grad);


typedef Graph<double, double, double> GraphType;

//马尔科夫域能量最小化求解最佳缝合线
vector<Point2f> findBestSeamMRF(
		const Mat &a, const Mat &b,			//两张同一坐标系下预配准的图像，注意左右顺序
		Point2i &edge,						//重叠区域的两个横向坐标	
		double alpha,						//能量函数第一项的系数
		double beta,						//能量函数第二项中的系数
		vector<vector<int>> &lable);		//重叠区域标签
	

//初始化t-links和n-links，由于要便利所有的像素点，所以放在一起初始化
void initLinks(
		GraphType *g,						//流量图
		double alpha,						//能量函数第一项的系数
		double beta,						//能量函数第二项中的系数
		const Mat &a, const Mat &b,			//重叠区域的两张图像
		Mat &diff,							//重叠区域差值图像
		Mat &grad1, Mat &grad2);			//重叠区域梯度图像

//从已经得到的最大流中，求出相应的缝合线和相应的标签
vector<Point2f> getSeamFromFlow(
		GraphType *g,						//最大流
		int rows, int cols,					//重叠区域行列数
		vector<vector<int>> &lable);		//重叠区域标签


#endif SEAM_H_





