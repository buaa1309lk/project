#ifndef STITCH_H_
#define STITCH_H_
#include "seam.h"


//���ݱ�ǩ���м򵥵�ͼ��ƴ��
Mat lableStitch(
		Mat &imgT, Mat &img2T,
		Point2i pos,
		vector<Point2f> seamPoint,
		vector<vector<int>> &lable);
#endif
