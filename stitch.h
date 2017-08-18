#ifndef STITCH_H_
#define STITCH_H_
#include "seam.h"


//根据标签进行简单的图像拼接
Mat lableStitch(
		Mat &imgT, Mat &img2T,
		Point2i pos,
		vector<Point2f> seamPoint,
		vector<vector<int>> &lable);
#endif
