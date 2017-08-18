#include "alignment.h"
#include "seam.h"
#include "stitch.h"
int main(int argc, char *argv[])
{
//默认三张图的大小是一样的
	Mat img1 = imread("Data/4.jpg");
	Mat img2 = imread("Data/2.jpg");
	Mat img3 = imread("Data/3.jpg");

	//////图像校准
	//Mat img1T, img2T, adjustHomo, transform;
	//vector<KeyPoint> feature1, feature2;
	//preAlign(img1, img2, img1T, img2T, adjustHomo, transform, feature1, feature2);

	////找缝合线
	//Point2i edge;
	//vector<vector<int>> lable;
	//vector<Point2f> seam = findBestSeamMRF(img1T, img2T, edge, 2, 0.001, lable);

	//////图像融合
	//Mat result = lableStitch(img1T, img2T, edge, seam, lable);

	Mat img1T, img2T, img3T;
	Mat homoAB, homoCB;
	Mat transAB, transCB;
	preAlign3(img1, img2, img3, img1T, img2T, img3T, homoAB, homoCB, transAB, transCB);

	Point2i edge1, edge2;
	vector<vector<int>> lable1, lable2;
	vector<Point2f> seam1 = findBestSeamMRF(img1T, img2T, edge1, 5, 0.01, lable1);

	Mat result = lableStitch(img1T, img2T, edge1, seam1, lable1);
	
	vector<Point2f> seam2 = findBestSeamMRF(result, img3T, edge2, 5, 0.01, lable2);
	Mat finalResult = lableStitch(result, img3T, edge2, seam2, lable2);
	
	
	
	waitKey(0);
	return 0;
}