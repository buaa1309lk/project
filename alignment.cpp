#include "alignment.h"
int input(Mat img1, Mat img2, vector<Point2f> &kp1, vector<Point2f> &kp2, ifstream &a, ifstream &b)
{
	int m, n;
	a >> m;
	b >> n;
	vector<KeyPoint> k;
	vector<KeyPoint> p;
	vector<DMatch> mp;
	Mat c;
	drawMatches(img1, k, img2, p, mp, c);

	if (m != n)
	{
		cout << "input error" << endl;
		return 0;
	}
	kp1.resize(m);
	kp2.resize(m);
	for (int i = 0; i < m; i++)
	{
		a >> kp1[i].x >> kp1[i].y;
		b >> kp2[i].x >> kp2[i].y;
		circle(c, kp1[i], 4, Scalar(0), 2, 8, 0);
		circle(c, Point2f(kp2[i].x + img1.cols, kp2[i].y), 4, Scalar(0), 2, 8, 0);
		if (i % 10 == 0)
			line(c, kp1[i], Point2f(kp2[i].x + img1.cols, kp2[i].y), Scalar::all(-1));
	}

	imshow("c", c);
	return 1;
}

//������׼
void preAlign(Mat &img1, Mat &img2, Mat &img1T, Mat &img2T, Mat &adjustHomo, Mat &transform, vector<KeyPoint> &feature1, vector<KeyPoint> &feature2)
{
	//�������⡢ƥ�䡢����
	vector<DMatch> inlierMatch;
	featureProcess(img1, img2, 200.0, 10.0, feature1, feature2, inlierMatch);
	Mat tmp;
	drawMatches(img1, feature1, img2, feature2, inlierMatch, tmp);
	imshow("RANSAC ���˺��ƥ���������", tmp);
	homoTransform(img1, img2, feature1, feature2, inlierMatch, adjustHomo, transform, img1T, img2T);
	//homoTransform(img2, img1, feature2, feature1, inlierMatch, adjustHomo, transform, img2T, img1T);

	imshow("ͼһͶӰ�任", img1T);
	imshow("ͼ��ͶӰ�任1", img2T);
}

//������׼��3��ͼ�����м�Ϊ��
void preAlign3(
	Mat &img1, Mat &img2, Mat &img3,
	Mat &imgA, Mat &imgB, Mat &imgC,
	Mat &homoAB, Mat &homoCB,
	Mat &transAB, Mat transCB)
{
	vector<DMatch> inlierMatchAB;
	vector<KeyPoint> featureA;
	vector<KeyPoint> featureB1;
	featureProcess(img1, img2, 250.0, 10.0, featureA, featureB1, inlierMatchAB);
	Mat tmp;
	drawMatches(img1, featureA, img2, featureB1, inlierMatchAB, tmp);
	imshow("RANSAC AB���˺��ƥ���������", tmp);

	homoTransform(img1, img2, featureA, featureB1, inlierMatchAB, homoAB, transAB, imgA, imgB);
	imshow("ͼһͶӰ�任", imgA);
	imshow("ͼ��ͶӰ�任1", imgB);


	vector<DMatch> inlierMatchCB;
	vector<KeyPoint> featureB2;
	vector<KeyPoint> featureC;
	Mat imgB2;
	featureProcess(img3, img2, 250.0, 10.0, featureC, featureB2, inlierMatchCB);
	Mat tmp1;
	drawMatches(img3, featureC, img2, featureB2, inlierMatchCB, tmp1);
	imshow("RANSAC CB���˺��ƥ���������", tmp1);
	

	homoTransform(img3, img2, featureC, featureB2, inlierMatchCB, homoCB, transCB, imgC, imgB2);
	imshow("ͼ��ͶӰ�任", imgC);
	imshow("ͼ��ͶӰ�任2", imgB2);

}

//���������
void featureCluster(
	const Mat &img,						//�����ͼ��			
	const vector<Point2f> &position,	//ͼ��������λ��
	int clusterNum,						//�������
	vector<int> &clusterLable)			//ÿ���������Ӧ����
{
	//���������������ϸ���������������
	Mat points(position.size(), 1, CV_32FC2);
	for (int i = 0; i < position.size(); i++)	
		points.at<Point2f>(i) = position[i];

	//�洢�������ľ���
	Mat centers(clusterNum, 1, CV_32FC2);

	//ÿ���������Ӧ�ı�ǩ
	Mat pointLable;

	kmeans(
		points,		//��Ҫ�������ԭʼ���ݼ���
		clusterNum,	//��Ҫ������ĸ���
		pointLable,	//ÿһ�����������ǩ
					//�㷨�ĵ�����ֹ����
		TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0),
		1,			//���࣬ȡ�����õ��Ǵ�
					//�����ʼ����������PP�ض�������㷨		
		KMEANS_RANDOM_CENTERS,
		centers);	//�������ľ���

	//���������ͼ��
	Mat imgCluster;
	imgCluster = img.clone();

	Scalar colorTab[] =     //��Ϊ���ֻ��5�࣬�������Ҳ�͸�5����ɫ
	{
		Scalar(0, 0, 255),
		Scalar(0, 255, 0),
		Scalar(255, 100, 100),
		Scalar(255, 0, 255),
		Scalar(0, 255, 255)
	};

	for (int i = 0; i < position.size(); i++)
	{
		int clusterIdx = pointLable.at<int>(i);
		clusterLable.push_back(clusterIdx);
		Point tempPoint = points.at<Point2f>(i);
		circle(imgCluster, tempPoint, 2, colorTab[clusterIdx],2,8,0);
	}

	imshow("imgCluster", imgCluster);

}

//�������⣬ƥ�䣬����
void featureProcess(
		const Mat &img1,
		const Mat &img2,
		const double Max,
		const double Min,
		vector<KeyPoint> &feature1,
		vector<KeyPoint> &feature2,
		vector<DMatch> &inlierMatch)
{
	//ת��Ϊ�Ҷ�ͼ��
	Mat img1G, img2G;
	cvtColor(img1, img1G, CV_RGB2GRAY);
	cvtColor(img2, img2G, CV_RGB2GRAY);

	///��ȡ�ؼ����������
	SIFT sift(2000, 3, 0.04, 10, 1.6);
	vector<KeyPoint> keypoint1, keypoint2;	//�ؼ���
	Mat img1Desc, img2Desc;					//������
	sift.detect(img1G, keypoint1);
	sift.detect(img2G, keypoint2);
	sift.compute(img1G, keypoint1, img1Desc);
	sift.compute(img2G, keypoint2, img2Desc);

	///�ؼ���ƥ��
	FlannBasedMatcher matcher;
	//BruteForceMatcher<L2<float>> matcher;//��һ�ֹؼ���ƥ�䷽��
	vector<DMatch> matchPointTmp;
	matcher.match(img1Desc, img2Desc, matchPointTmp);
	//sort(matchPoint.begin(), matchPoint.end());



	//�ҵ�dist�������Сֵ������dist����ķ�Χ��һ�����ƥ��
	double maxDist = 0;
	double minDist = 100;
	for (int i = 0; i < img1Desc.rows; i++)
	{
		double dist = matchPointTmp[i].distance;
		if (dist < minDist) minDist = dist;
		if (dist > maxDist) maxDist = dist;
	}
	cout << "--Max dist:" << maxDist << endl;
	cout << "--Min dist:" << minDist << endl;

	vector<DMatch> matchPoint;
	for (int i = 0; i < img1Desc.rows; i++)
	{
		if (matchPointTmp[i].distance <= max(minDist, Max)
			&& matchPointTmp[i].distance >= Min)
			matchPoint.push_back(matchPointTmp[i]);
	}

	//�������������ֵ�ĵ�Եı�ǣ��ڶ�����ˣ�RANSAC
	vector<KeyPoint> keypoint11, keypoint22;
	for (int i = 0; i < matchPoint.size(); i++)
	{
		keypoint11.push_back(keypoint1[matchPoint[i].queryIdx]);
		keypoint22.push_back(keypoint2[matchPoint[i].trainIdx]);
	}
	vector<Point2f> leftPoint, rightPoint;
	for (int i = 0; i < matchPoint.size(); i++)
	{
		leftPoint.push_back(keypoint11[i].pt);
		rightPoint.push_back(keypoint22[i].pt);
	}

	vector<uchar> mask;
	findFundamentalMat(leftPoint, rightPoint, mask, FM_RANSAC);
	int index = 0;
	for (int i = 0; i < matchPoint.size(); i++)
	{
		if (mask[i] != 0)
		{
			feature1.push_back(keypoint11[i]);
			feature2.push_back(keypoint22[i]);
			matchPoint[i].queryIdx = index;
			matchPoint[i].trainIdx = index;
			inlierMatch.push_back(matchPoint[i]);
			index++;
		}
	}
}

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
		Mat &img2T)
{

	//��ȡ��������λ��
	vector<Point2f> leftPoint1, rightPoint1;
	for (int i = 0; i < feature1.size(); i++)
	{
		leftPoint1.push_back(feature1[i].pt);
		rightPoint1.push_back(feature2[i].pt);
	}
	//��Ӧ�Ա任�����ƽ�ƾ���
	Mat homo = findHomography(leftPoint1, rightPoint1, CV_RANSAC, 3);
	transform = (Mat_<double>(3, 3) << 1.0, 0, img1.cols-300, 0, 1.0, 100, 0, 0, 1.0);
	adjustHomo = transform*homo;

	warpPerspective(img1, img1T, adjustHomo, Size(img1.cols + img2.cols+200, img2.rows+200));
	warpPerspective(img2, img2T, transform, Size(img1.cols + img2.cols+200, img2.rows+200));
}
