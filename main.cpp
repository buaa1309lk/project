#include "alignment.h"
#include "seam.h"
#include "stitch.h"
#include <fstream>
ifstream filein1("out1.txt");
ifstream filein2("out2.txt");
int main(int argc, char *argv[])
{
//默认图的大小是一样的
	Mat img1 = imread("left1.png");
	Mat img2 = imread("right2.png");
	//Mat img11, img22;
	//resize(img1, img11, Size(800, 300));
	//resize(img2, img22, Size(800, 300));
	//imwrite("left1.png", img11);
	//imwrite("right2.png", img22);
	//imshow("img1", img1);
	//imshow("img2", img2);


//////////////////////////////////////////////////////////////////////////////////////////
	////图像校准
	Mat img1T, img2T, adjustHomo, transform;
	vector<KeyPoint> feature1, feature2;
	//preAlign(img1, img2, img1T, img2T, adjustHomo, transform, feature1, feature2);

	vector<Point2f> kp1, kp2;
	input(img1, img2, kp1, kp2, filein1, filein2);
	Mat homo = findHomography(kp1, kp2, CV_RANSAC, 3);
	transform = (Mat_<double>(3, 3) << 1.0, 0, img1.cols - 300, 0, 1.0, 100, 0, 0, 1.0);
	adjustHomo = transform*homo;
	warpPerspective(img1, img1T, adjustHomo, Size(img1.cols + img2.cols + 200, img2.rows + 200));
	warpPerspective(img2, img2T, transform, Size(img1.cols + img2.cols + 200, img2.rows + 200));

	imshow("img1T", img1T);
	imshow("img2T", img2T);


	//找缝合线
	Point2i edge;
	edge = findRegion(img1T,img2T);
	vector<vector<int>> lable;
	vector<Point2f> seam = findBestSeamMRF(img1T, img2T, edge, 3, 0.001, lable);

	//图像融合
	Mat result = lableStitch(img1T, img2T, edge, seam, lable);

////////////////////////////////////////////////////////////////////////////////////////////
	//Mat img1T, img2T, img3T;
	//Mat homoAB, homoCB;
	//Mat transAB, transCB;
	//preAlign3(img1, img2, img3, img1T, img2T, img3T, homoAB, homoCB, transAB, transCB);

	//Point2i edge1, edge2;
	//vector<vector<int>> lable1, lable2;
	//vector<Point2f> seam1 = findBestSeamMRF(img1T, img2T, edge1, 5, 0.001, lable1);

	//Mat result = lableStitch(img1T, img2T, edge1, seam1, lable1);
	//
	//vector<Point2f> seam2 = findBestSeamMRF(result, img3T, edge2, 5, 0.001, lable2);
	//Mat finalResult = lableStitch(result, img3T, edge2, seam2, lable2);


	

	
	waitKey(0);
	return 0;
}