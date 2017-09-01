#include"seam.h"

//�ҵ��ص������λ������
Point2i findRegion(const Mat &a, const Mat &b)
{
	uchar p, q;
	Point2i position;
	position.y = 0;
	position.x = a.cols;

	Mat region1G, region2G;
	cvtColor(a, region1G, CV_RGB2GRAY);
	cvtColor(b, region2G, CV_RGB2GRAY);

	for(int i=0;i<a.rows;i++)
	{
		for (int j = 0; j < a.cols; j++)
		{
			p = region1G.at<uchar>(i, j);
			q = region2G.at<uchar>(i, j);
			if (p != 0 && j > position.y)
				position.y = j;

			if (q != 0 && j < position.x)
				position.x = j;

		}
	}
	position.x = position.x + 5;
	return position;
}

Point2i findRegion(
	const vector<KeyPoint> &feature1,
	const vector<KeyPoint> &feature2,
	const Mat &adjusthomo, const Mat &T)
{
	KeyPoint p1 = feature1[0];//ͼһ�任������ߵ�������
	KeyPoint p2 = feature2[0];//ͼ���任�����ұߵ�������
	for (int i = 1; i < feature1.size(); i++)
	{
		p1 = feature1[i].pt.x < p1.pt.x ? feature1[i] : p1;
		p2 = feature2[i].pt.x > p2.pt.x ? feature2[i] : p2;
	}

	Mat p3 = (Mat_<double>(3, 1) << p1.pt.x, p1.pt.y, 1.0);
	Mat p4 = (Mat_<double>(3, 1) << p2.pt.x, p2.pt.y, 1.0);
	p3 = adjusthomo*p3;
	p4 = T*p4;

	float x1 = p3.at<double>(0, 0) / p3.at<double>(2, 0);
	float x2 = p4.at<double>(0, 0) / p4.at<double>(2, 0);

	return Point2i(x1, x2);//�����������x���꣬ע��ڶ�����ĺ����껹Ҫ�����任��

}




//ͼ���ֵ
Mat absDiff(const Mat &region1, const Mat &region2)
{
	Mat region1G, region2G;

	uchar pos = 80;
	cvtColor(region1, region1G, CV_RGB2GRAY);
	cvtColor(region2, region2G, CV_RGB2GRAY);
	Mat diff;
	diff.create(region1G.size(), region1G.type());
	uchar p, q;

	for (int i = 0; i < region1G.rows; i++) {
		for (int j = 0; j < region1G.cols; j++)
		{
			p = region1G.at<uchar>(i,j);
			q = region2G.at<uchar>(i,j);

			

			if (fabs(p - q) < pos)
				diff.at<uchar>(i, j) = 0;
			else
				diff.at<uchar>(i,j) =(uchar) fabs(p - q);
			if (p == 0 && q == 0)
				diff.at<uchar>(i, j) = 255;
			if ((p == 0 && q != 0) || (p != 0 && q == 0))
				diff.at<uchar>(i, j) = 100;
		}
	}
	return diff;
}

//�ݶ�ͼ��
void gradient(const Mat &region, Mat &grad)
{
	Mat regionG;
	cvtColor(region, regionG, CV_RGB2GRAY);

	Mat sobelx, sobely;
	Sobel(regionG, sobelx, CV_32F, 1, 0, 3);
	Sobel(regionG, sobely, CV_32F, 0, 1, 3);

	Mat gradValue;
	Mat direction;
	cartToPolar(sobelx, sobely, gradValue, direction);

	double gradValueMax;
	minMaxLoc(gradValue, NULL, &gradValueMax);
	gradValue.convertTo(grad, CV_8UC1, 255.0 / gradValueMax, 0);


}


//����Ʒ���������С�������ѷ����
vector<Point2f> findBestSeamMRF(
		const Mat &img1T, const Mat &img2T,	//����ͬһ����ϵ��Ԥ��׼��ͼ��ע������˳��
		Point2i &edge,						//�ص������������������	
		double alpha,						//����������һ���ϵ��
		double beta,						//���������ڶ����е�ϵ��
		vector<vector<int>> &lable)			//�ص������ǩ
{
	vector<Point2f> seamPoint;
	GraphType *g = new GraphType(2,1);
	int flow;
	
	////1.�ص���������
	edge = findRegion(img1T,img2T);
	//edge.y = edge.y - 40;
	//2.�ص������ֵͼ��
	Mat region1 = img1T(Rect(Point(edge.x, 0), Point(edge.y, img1T.rows)));
	Mat region2 = img2T(Rect(Point(edge.x, 0), Point(edge.y, img2T.rows)));
	imshow("region1", region1);
	imshow("region2", region2);
	Mat diff = absDiff(region1, region2);
	imshow("diff", diff);

	//3.�ҵ��ص�������ݶ�ͼ��
	Mat grad1, grad2;
	gradient(region1, grad1);
	gradient(region2, grad2);
	//Mat diffgrad = absDiff(grad1, grad2);
	//imshow("diffgrad",diffgrad);
	imshow("grad1", grad1);
	imshow("grad2", grad2);
	//4.��ʼ��t-links��n-links
	initLinks(g, alpha, beta, region1, region2, diff, grad1, grad2);
	//5.MaxFlow/MinCut���
	flow = g->maxflow();
	//6.�������/��С���������еõ���ѵķ���ߺ���Ӧ�ı�ǩ
	seamPoint = getSeamFromFlow(g, region1.rows, region1.cols, lable);

	return seamPoint;
}



//��ʼ��t-links��n-links������Ҫ�������е����ص㣬���Է���һ���ʼ��
void initLinks(
		GraphType *g,						//����ͼ
		double alpha,						//����������һ���ϵ��
		double beta,						//���������ڶ����е�ϵ��
		const Mat &a, const Mat &b,			//�ص���������Ŵ�С��ͬ��ͼ��
		Mat &diff,							//�ص������ֵͼ��
		Mat &grad1, Mat &grad2) 			//�ص������ݶ�ͼ��
{
	//1.��ʼ���ڵ����
	int rows, cols;														//ͼ����к���
	if ((a.rows == b.rows) && (a.cols == b.cols))
		rows = a.rows, cols = a.cols;
	else
	{
		cout << "Error:�����ص������ͼ���С��һ��" << endl;
		return;
	}

	int nodeNum = rows*cols;
	g->add_node(nodeNum);

	for (int i = 0; i < rows; i++)
	{		
		double s2p=  alpha*grad1.at<uchar>(i, 0);
		double p2t=  alpha*grad2.at<uchar>(i, cols-1);

		/*double k = 1000;
		s2p = k;
		p2t = k;*/
		g->add_tweights(i*cols, s2p, 0);
		g->add_tweights(i*cols + cols - 1, 0, p2t);
	}
	



	//2.ѭ����ʼ��t-links��n-links,ÿ�����ص�����Χ�ĸ��ڵ㡢�����ն˽ڵ���������������������ӵ���Ŀ��
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < cols; j++)			
		{
			////��ʼ��t-links
			//float s2p = -alpha*grad1.at<uchar>(i, j);
			//float p2t = -alpha*grad2.at<uchar>(i, j);
			//g->add_tweights(i*cols + j, s2p, p2t);
			//
			//��ʼ��n-links
			double p,p1, p2;												//p���ұߺ����������ߵĳ�ʼ����
			p = pow(diff.at<uchar>(i, j), 2.0) + beta*pow(grad1.at<uchar>(i, j) - grad2.at<uchar>(i, j), 2.0);

			if (i != rows - 1 && j != cols - 1)
			{
				p1 = pow(diff.at<uchar>(i, j + 1), 2.0) + beta*pow(grad1.at<uchar>(i, j + 1) - grad2.at<uchar>(i, j + 1), 2.0);
				p2 = pow(diff.at<uchar>(i + 1, j), 2.0) + beta*pow(grad1.at<uchar>(i + 1, j) - grad2.at<uchar>(i + 1, j), 2.0);
				g->add_edge(i*cols + j, i*cols + (j + 1), p+p1, 0);
				g->add_edge(i*cols + j, (i + 1)*cols + j, p+p2, 0);
			}
			else if (i == rows - 1 && j != cols - 1)					//���ұߵ�Ȩ��
			{
				p1 = pow(diff.at<uchar>(i, j + 1), 2.0) + beta*pow(grad1.at<uchar>(i, j + 1) - grad2.at<uchar>(i, j + 1), 2.0);
				g->add_edge(i*cols + j, i*cols + (j + 1), p+p1, 0);
			}
			else if (i != rows - 1 && j == cols - 1)					//���±ߵ�Ȩ��
			{
				p2 = pow(diff.at<uchar>(i + 1, j), 2.0) + beta*pow(grad1.at<uchar>(i + 1, j) - grad2.at<uchar>(i + 1, j), 2.0);
				g->add_edge(i*cols + j, (i + 1)*cols + j, p+p2, 0);
			}
		}
}

//�������/��С���������еõ���ѵķ���ߺ���Ӧ�ı�ǩ
vector<Point2f> getSeamFromFlow(
		GraphType *g,						//�����
		int rows, int cols,					//�ص�����������
		vector<vector<int>> &lable) 		//�ص������ǩ
{
	vector<Point2f> seamPoint;

	//��ά��ǩ�����ʼ��
	lable.resize(rows);
	for (int i = 0; i < rows; i++)
		lable[i].resize(cols);

	
	//��ÿ���ڵ���б��
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < cols; j++)
		{
			if (g->what_segment(i*cols + j) == GraphType::SOURCE)
				lable[i][j] = 1;
			else if (g->what_segment(i*cols + j) == GraphType::SINK)
				lable[i][j] = 0;
		}

	//�ҵ��ָ�����Ϊ���ŷ����
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < cols; j++)
		{
			if (
				(lable[i][j] == 1) && (
				((i - 1 > 0) && lable[i - 1][j] == 0) ||
					((i + 1 < rows) && lable[i + 1][j] == 0) ||
					((j - 1 > 0) && lable[i][j - 1] == 0) ||
					((j + 1 < cols) && lable[i][j + 1] == 0)))
				seamPoint.push_back(Point2f(i, j));
		}

	return seamPoint;
}