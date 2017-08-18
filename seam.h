#ifndef SEAM_H_
#define SEAM_H_
#include "alignment.h"
#include "MinCut\graph.h"

//�ҵ��ص������λ������
Point2i findRegion(const Mat &a, const Mat &b);

//�õ��ص�����Ĳ�ֵͼ��
Mat absDiff(const Mat &region1, const Mat &region2);

//�õ��ص�������ݶ�ͼ��
void gradient(const Mat &region, Mat &grad);


typedef Graph<double, double, double> GraphType;

//����Ʒ���������С�������ѷ����
vector<Point2f> findBestSeamMRF(
		const Mat &a, const Mat &b,			//����ͬһ����ϵ��Ԥ��׼��ͼ��ע������˳��
		Point2i &edge,						//�ص������������������	
		double alpha,						//����������һ���ϵ��
		double beta,						//���������ڶ����е�ϵ��
		vector<vector<int>> &lable);		//�ص������ǩ
	

//��ʼ��t-links��n-links������Ҫ�������е����ص㣬���Է���һ���ʼ��
void initLinks(
		GraphType *g,						//����ͼ
		double alpha,						//����������һ���ϵ��
		double beta,						//���������ڶ����е�ϵ��
		const Mat &a, const Mat &b,			//�ص����������ͼ��
		Mat &diff,							//�ص������ֵͼ��
		Mat &grad1, Mat &grad2);			//�ص������ݶ�ͼ��

//���Ѿ��õ���������У������Ӧ�ķ���ߺ���Ӧ�ı�ǩ
vector<Point2f> getSeamFromFlow(
		GraphType *g,						//�����
		int rows, int cols,					//�ص�����������
		vector<vector<int>> &lable);		//�ص������ǩ


#endif SEAM_H_





