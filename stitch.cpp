#include "stitch.h"

//���ݱ�ǩ���м򵥵�ͼ��ƴ��
Mat lableStitch(Mat &img1T, Mat &img2T, Point2i pos, vector<Point2f> seamPoint, vector<vector<int>> &lable)
{
	Mat result;
	result.create(img1T.size(), img1T.type());

	//�ص���������
	for (int j = 0; j < pos.x; j++)
	{
		for (int i = 0; i < result.rows; i++)
		{
			result.at<Vec3b>(i, j)[0] = img1T.at<Vec3b>(i, j)[0];
			result.at<Vec3b>(i, j)[1] = img1T.at<Vec3b>(i, j)[1];
			result.at<Vec3b>(i, j)[2] = img1T.at<Vec3b>(i, j)[2];
		}
	}

	//�ص�������ұ�
	for (int j = pos.y; j < result.cols; j++)
	{
		for (int i = 0; i < result.rows; i++)
		{
			result.at<Vec3b>(i, j)[0] = img2T.at<Vec3b>(i, j)[0];
			result.at<Vec3b>(i, j)[1] = img2T.at<Vec3b>(i, j)[1];
			result.at<Vec3b>(i, j)[2] = img2T.at<Vec3b>(i, j)[2];
		}
	}

	//�ص�������ݱ�ǩ������ƴ��
	for (int j = pos.x; j < pos.y; j++)
	{
		for (int i = 0; i < result.rows; i++)
		{
			if (lable[i][j - pos.x] == 1)
			{
				result.at<Vec3b>(i, j)[0] = img1T.at<Vec3b>(i, j)[0];
				result.at<Vec3b>(i, j)[1] = img1T.at<Vec3b>(i, j)[1];
				result.at<Vec3b>(i, j)[2] = img1T.at<Vec3b>(i, j)[2];
			}
			else if (lable[i][j - pos.x] == 0)
			{
				result.at<Vec3b>(i, j)[0] = img2T.at<Vec3b>(i, j)[0];
				result.at<Vec3b>(i, j)[1] = img2T.at<Vec3b>(i, j)[1];
				result.at<Vec3b>(i, j)[2] = img2T.at<Vec3b>(i, j)[2];
			}
		}
	}

	imshow("result", result);

	Mat tmp = result.clone();




	for (int i = 0; i < seamPoint.size(); i++)
		circle(result, Point2f(seamPoint[i].y + pos.x, seamPoint[i].x), 0.5, Scalar(100), 2, 8, 0);
	imshow("stitch result with seam", result);
	imwrite("Data/seamtmp.jpg", result);
	return result;
}