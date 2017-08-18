#ifndef ALIGNMENT_H_
#define ALIGNMENT_H_
#include "highgui/highgui.hpp"    
#include "opencv2/nonfree/nonfree.hpp"    
#include "opencv2/legacy/legacy.hpp"   
#include <math.h>
using namespace cv;
using namespace std;

//������׼
void preAlign(	
		Mat &img1, Mat &img2,				//��������ͼ��
		Mat &img1T, Mat &img2T,				//������׼���ͼ��
		Mat &adjustHomo, Mat &transform,	//��Ӧ�Ծ����ƽ�ƾ���
		vector<KeyPoint> &feature1,			//ͼһ��������
		vector<KeyPoint> &feature2);		//ͼ����������

//������׼��3��ͼ�����м�Ϊ��
void preAlign3(
	Mat &img1, Mat &img2, Mat &img3,
	Mat &imgA, Mat &imgB, Mat &imgC,
	Mat &homoAB, Mat &homoCB,
	Mat &transAB, Mat transCB);




//��һ����׼
void refineAlign();


//���������
void featureCluster(
		const Mat &img,						//�����ͼ��			
		const vector<Point2f> &position,	//ͼ��������λ��
		int	clusterNum,						//����ĸ���
		vector<int> &clusterLable);			//ÿ���������Ӧ����

//�������⣬ƥ�䣬����
void featureProcess(
		const Mat &img1,
		const Mat &img2,
		const double Max,
		const double Min,
		vector<KeyPoint> &feature1,
		vector<KeyPoint> &feature2,
		vector<DMatch> &inlierMatch);

//��Ӧ�Ծ���任
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
