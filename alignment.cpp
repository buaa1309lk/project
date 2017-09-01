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

//初步配准
void preAlign(Mat &img1, Mat &img2, Mat &img1T, Mat &img2T, Mat &adjustHomo, Mat &transform, vector<KeyPoint> &feature1, vector<KeyPoint> &feature2)
{
	//特征点检测、匹配、过滤
	vector<DMatch> inlierMatch;
	featureProcess(img1, img2, 200.0, 10.0, feature1, feature2, inlierMatch);
	Mat tmp;
	drawMatches(img1, feature1, img2, feature2, inlierMatch, tmp);
	imshow("RANSAC 过滤后的匹配特征点对", tmp);
	homoTransform(img1, img2, feature1, feature2, inlierMatch, adjustHomo, transform, img1T, img2T);
	//homoTransform(img2, img1, feature2, feature1, inlierMatch, adjustHomo, transform, img2T, img1T);

	imshow("图一投影变换", img1T);
	imshow("图二投影变换1", img2T);
}

//初步配准，3张图，以中间为主
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
	imshow("RANSAC AB过滤后的匹配特征点对", tmp);

	homoTransform(img1, img2, featureA, featureB1, inlierMatchAB, homoAB, transAB, imgA, imgB);
	imshow("图一投影变换", imgA);
	imshow("图二投影变换1", imgB);


	vector<DMatch> inlierMatchCB;
	vector<KeyPoint> featureB2;
	vector<KeyPoint> featureC;
	Mat imgB2;
	featureProcess(img3, img2, 250.0, 10.0, featureC, featureB2, inlierMatchCB);
	Mat tmp1;
	drawMatches(img3, featureC, img2, featureB2, inlierMatchCB, tmp1);
	imshow("RANSAC CB过滤后的匹配特征点对", tmp1);
	

	homoTransform(img3, img2, featureC, featureB2, inlierMatchCB, homoCB, transCB, imgC, imgB2);
	imshow("图三投影变换", imgC);
	imshow("图二投影变换2", imgB2);

}

//特征点聚类
void featureCluster(
	const Mat &img,						//输入的图像			
	const vector<Point2f> &position,	//图像特征点位置
	int clusterNum,						//聚类个数
	vector<int> &clusterLable)			//每个特征点对应的类
{
	//特征点样本矩阵（严格意义上是向量）
	Mat points(position.size(), 1, CV_32FC2);
	for (int i = 0; i < position.size(); i++)	
		points.at<Point2f>(i) = position[i];

	//存储聚类中心矩阵
	Mat centers(clusterNum, 1, CV_32FC2);

	//每个特征点对应的标签
	Mat pointLable;

	kmeans(
		points,		//需要被聚类的原始数据集合
		clusterNum,	//需要被聚类的个数
		pointLable,	//每一个样本的类标签
					//算法的迭代终止条件
		TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0),
		1,			//聚类，取结果最好的那次
					//聚类初始化条件采用PP特定的随机算法		
		KMEANS_RANDOM_CENTERS,
		centers);	//聚类中心矩阵

	//画出聚类的图像
	Mat imgCluster;
	imgCluster = img.clone();

	Scalar colorTab[] =     //因为最多只有5类，所以最多也就给5个颜色
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

//特征点检测，匹配，过滤
void featureProcess(
		const Mat &img1,
		const Mat &img2,
		const double Max,
		const double Min,
		vector<KeyPoint> &feature1,
		vector<KeyPoint> &feature2,
		vector<DMatch> &inlierMatch)
{
	//转换为灰度图像
	Mat img1G, img2G;
	cvtColor(img1, img1G, CV_RGB2GRAY);
	cvtColor(img2, img2G, CV_RGB2GRAY);

	///提取关键点和描述子
	SIFT sift(2000, 3, 0.04, 10, 1.6);
	vector<KeyPoint> keypoint1, keypoint2;	//关键点
	Mat img1Desc, img2Desc;					//描述子
	sift.detect(img1G, keypoint1);
	sift.detect(img2G, keypoint2);
	sift.compute(img1G, keypoint1, img1Desc);
	sift.compute(img2G, keypoint2, img2Desc);

	///关键点匹配
	FlannBasedMatcher matcher;
	//BruteForceMatcher<L2<float>> matcher;//另一种关键点匹配方法
	vector<DMatch> matchPointTmp;
	matcher.match(img1Desc, img2Desc, matchPointTmp);
	//sort(matchPoint.begin(), matchPoint.end());



	//找到dist的最大最小值，根据dist满足的范围第一层过滤匹配
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

	//满足基础矩阵阈值的点对的标记，第二层过滤，RANSAC
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
		Mat &img2T)
{

	//提取出特征点位置
	vector<Point2f> leftPoint1, rightPoint1;
	for (int i = 0; i < feature1.size(); i++)
	{
		leftPoint1.push_back(feature1[i].pt);
		rightPoint1.push_back(feature2[i].pt);
	}
	//单应性变换矩阵和平移矩阵
	Mat homo = findHomography(leftPoint1, rightPoint1, CV_RANSAC, 3);
	transform = (Mat_<double>(3, 3) << 1.0, 0, img1.cols-300, 0, 1.0, 100, 0, 0, 1.0);
	adjustHomo = transform*homo;

	warpPerspective(img1, img1T, adjustHomo, Size(img1.cols + img2.cols+200, img2.rows+200));
	warpPerspective(img2, img2T, transform, Size(img1.cols + img2.cols+200, img2.rows+200));
}
