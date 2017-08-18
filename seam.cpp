#include"seam.h"

//找到重叠区域的位置坐标
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
//图像差值
Mat absDiff(const Mat &region1, const Mat &region2)
{
	Mat region1G, region2G;

	uchar pos = 30;
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
		}
	}
	return diff;
}

//梯度图像
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


//马尔科夫域能量最小化求解最佳缝合线
vector<Point2f> findBestSeamMRF(
		const Mat &img1T, const Mat &img2T,	//两张同一坐标系下预配准的图像，注意左右顺序
		Point2i &edge,						//重叠区域的两个横向坐标	
		double alpha,						//能量函数第一项的系数
		double beta,						//能量函数第二项中的系数
		vector<vector<int>> &lable)			//重叠区域标签
{
	vector<Point2f> seamPoint;
	GraphType *g = new GraphType(2,1);
	int flow;
	
	////1.重叠区域坐标
	edge = findRegion(img1T,img2T);
	//2.重叠区域差值图像
	Mat region1 = img1T(Rect(Point(edge.x, 0), Point(edge.y, img1T.rows)));
	Mat region2 = img2T(Rect(Point(edge.x, 0), Point(edge.y, img2T.rows)));
	Mat diff = absDiff(region1, region2);
	//3.找到重叠区域的梯度图像
	Mat grad1, grad2;
	gradient(region1, grad1);
	gradient(region2, grad2);
	//4.初始化t-links和n-links
	initLinks(g, alpha, beta, region1, region2, diff, grad1, grad2);
	//5.MaxFlow/MinCut求解
	flow = g->maxflow();
	//6.从最大流/最小割的求解结果中得到最佳的缝合线和相应的标签
	seamPoint = getSeamFromFlow(g, region1.rows, region1.cols, lable);

	return seamPoint;
}



//初始化t-links和n-links，由于要便利所有的像素点，所以放在一起初始化
void initLinks(
		GraphType *g,						//流量图
		double alpha,						//能量函数第一项的系数
		double beta,						//能量函数第二项中的系数
		const Mat &a, const Mat &b,			//重叠区域的两张大小相同的图像
		Mat &diff,							//重叠区域差值图像
		Mat &grad1, Mat &grad2) 			//重叠区域梯度图像
{
	//1.初始化节点个数
	int rows, cols;														//图像的行和列
	if ((a.rows == b.rows) && (a.cols == b.cols))
		rows = a.rows, cols = a.cols;
	else
	{
		cout << "Error:两张重叠区域的图像大小不一致" << endl;
		return;
	}

	int nodeNum = rows*cols;
	g->add_node(nodeNum);

	for (int i = 0; i < rows; i++)
	{
		double s2p=  alpha*grad1.at<uchar>(i, 0);
		double p2t=  alpha*grad2.at<uchar>(i, cols-1);
		g->add_tweights(i*cols, s2p, 0);
		g->add_tweights(i*cols + cols - 1, 0, p2t);
	}
	



	//2.循环初始化t-links和n-links,每个像素点与周围四个节点、两个终端节点相连（后面可以增加连接的数目）
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < cols; j++)			
		{
			//初始化t-links
			//float s2p = alpha*grad1.at<uchar>(i, j);
			//float p2t = alpha*grad2.at<uchar>(i, j);
			//g->add_tweights(i*cols + j, s2p, p2t);
			
			//初始化n-links
			double p,p1, p2;												//p点右边和下面两条边的初始流。
			p = pow(diff.at<uchar>(i, j), 2.0) + beta*pow(grad1.at<uchar>(i, j) - grad2.at<uchar>(i, j), 2.0);

			if (i != rows - 1 && j != cols - 1)
			{
				p1 = pow(diff.at<uchar>(i, j + 1), 2.0) + beta*pow(grad1.at<uchar>(i, j + 1) - grad2.at<uchar>(i, j + 1), 2.0);
				p2 = pow(diff.at<uchar>(i + 1, j), 2.0) + beta*pow(grad1.at<uchar>(i + 1, j) - grad2.at<uchar>(i + 1, j), 2.0);
				g->add_edge(i*cols + j, i*cols + (j + 1), p+p1, 0);
				g->add_edge(i*cols + j, (i + 1)*cols + j, p+p2, 0);
			}
			else if (i == rows - 1 && j != cols - 1)					//最右边的权重
			{
				p1 = pow(diff.at<uchar>(i, j + 1), 2.0) + beta*pow(grad1.at<uchar>(i, j + 1) - grad2.at<uchar>(i, j + 1), 2.0);
				g->add_edge(i*cols + j, i*cols + (j + 1), p+p1, 0);
			}
			else if (i != rows - 1 && j == cols - 1)					//最下边的权重
			{
				p2 = pow(diff.at<uchar>(i + 1, j), 2.0) + beta*pow(grad1.at<uchar>(i + 1, j) - grad2.at<uchar>(i + 1, j), 2.0);
				g->add_edge(i*cols + j, (i + 1)*cols + j, p+p2, 0);
			}
		}
}

//从最大流/最小割的求解结果中得到最佳的缝合线和相应的标签
vector<Point2f> getSeamFromFlow(
		GraphType *g,						//最大流
		int rows, int cols,					//重叠区域行列数
		vector<vector<int>> &lable) 		//重叠区域标签
{
	vector<Point2f> seamPoint;

	//二维标签数组初始化
	lable.resize(rows);
	for (int i = 0; i < rows; i++)
		lable[i].resize(cols);


	//对每个节点进行标记
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < cols; j++)
		{
			if (g->what_segment(i*cols + j) == GraphType::SOURCE)
				lable[i][j] = 1;
			else if (g->what_segment(i*cols + j) == GraphType::SINK)
				lable[i][j] = 0;
		}

	//找到分割线作为最优缝合线
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < cols; j++)
		{
			if (
				(lable[i][j] == 1) && (
				((i - 1 > 0) && lable[i - 1][j] == 0) ||
					((i + 1 < rows) && lable[i + 1][j] == 0) ||
					((j - 1 > 0) && lable[i][j - 1] == 0) ||
					((j + 1 < cols) && lable[i][j + 1] == 0))

				)
				seamPoint.push_back(Point2f(i, j));
		}

	return seamPoint;
}