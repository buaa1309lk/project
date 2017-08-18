#ifndef ALIGNMENT_H_
#define ALIGNMENT_H_
#include "highgui/highgui.hpp"    
#include "opencv2/nonfree/nonfree.hpp"    
#include "opencv2/legacy/legacy.hpp"   
#include <math.h>
using namespace cv;
using namespace std;

//初步配准
void preAlign(	
		Mat &img1, Mat &img2,				//两张输入图像
		Mat &img1T, Mat &img2T,				//两张配准后的图像
		Mat &adjustHomo, Mat &transform,	//单应性矩阵和平移矩阵
		vector<KeyPoint> &feature1,			//图一的特征点
		vector<KeyPoint> &feature2);		//图二的特征点

//初步配准，3张图，以中间为主
void preAlign3(
	Mat &img1, Mat &img2, Mat &img3,
	Mat &imgA, Mat &imgB, Mat &imgC,
	Mat &homoAB, Mat &homoCB,
	Mat &transAB, Mat transCB);




//进一步配准
void refineAlign();


//特征点聚类
void featureCluster(
		const Mat &img,						//输入的图像			
		const vector<Point2f> &position,	//图像特征点位置
		int	clusterNum,						//聚类的个数
		vector<int> &clusterLable);			//每个特征点对应的类

//特征点检测，匹配，过滤
void featureProcess(
		const Mat &img1,
		const Mat &img2,
		const double Max,
		const double Min,
		vector<KeyPoint> &feature1,
		vector<KeyPoint> &feature2,
		vector<DMatch> &inlierMatch);

//单应性矩阵变换
void homoTransform(
		const Mat &img1,
		const Mat &img2,
		const vector<KeyPoint> &feature1,
		const vector<KeyPoint> &feature2,
		const vector<DMatch> &inlierMatch,
		Mat &adjustHomo,
		Mat &transform,
		Mat &img1T,
		Mat &img2T);


#endif // !ALIGNMENT_H_
