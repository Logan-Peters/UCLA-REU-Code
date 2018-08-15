#include "main2.hpp"

#include <opencv2/core/base.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/mat.inl.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/operations.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv2/videoio.hpp>
#include <stddef.h>
#include <algorithm>
#include <iterator>
#include <vector>

#include "handGesture.hpp"
#include "myImage.hpp"
#include "roi.hpp"

using namespace cv;
using namespace std;

/* Global Variables  */
int fontFace2 = FONT_HERSHEY_PLAIN;
int square_len2;
int avgColor2[NSAMPLES][3];
int c_lower2[NSAMPLES][3];
int c_upper2[NSAMPLES][3];
int avgBGR2[3];
int nrOfDefects2;
int iSinceKFInit2;
HandGesture hg2;
struct dim2 {
	int w;
	int h;
} boundingDim2;
VideoWriter out2;
Mat edges2;
My_ROI roi12, roi22, roi32, roi42, roi52, roi62;
vector<My_ROI> roi222;
vector<KalmanFilter> kf2;
vector<Mat_<float> > measurement2;

/* end global variables */

void init2(MyImage *m) {
	square_len2 = 20;
	iSinceKFInit2 = 0;
}

// change a color from one space to another
void col2origCol2(int hsv[3], int bgr[3], Mat src) {
	Mat avgBGRMat = src.clone();
	for (int i = 0; i < 3; i++) {
		avgBGRMat.data[i] = hsv[i];
	}
	cvtColor(avgBGRMat, avgBGRMat, COL2ORIGCOL);
	for (int i = 0; i < 3; i++) {
		bgr[i] = avgBGRMat.data[i];
	}
}

void printText2(Mat src, string text) {
	int fontFace = FONT_HERSHEY_PLAIN;
	putText(src, text, Point(src.cols / 2, src.rows / 10), fontFace, 1.2f,
			Scalar(200, 0, 0), 2);
}

void waitForPalmCover2(MyImage* m) {
	m->cap >> m->src;
	flip(m->src, m->src, 1);
	roi222.push_back(
			My_ROI(Point(m->src.cols / 3, m->src.rows / 6),
					Point(m->src.cols / 3 + square_len2,
							m->src.rows / 6 + square_len2), m->src));
	roi222.push_back(
			My_ROI(Point(m->src.cols / 4, m->src.rows / 2),
					Point(m->src.cols / 4 + square_len2,
							m->src.rows / 2 + square_len2), m->src));
	roi222.push_back(
			My_ROI(Point(m->src.cols / 3, m->src.rows / 1.5),
					Point(m->src.cols / 3 + square_len2,
							m->src.rows / 1.5 + square_len2), m->src));
	roi222.push_back(
			My_ROI(Point(m->src.cols / 2, m->src.rows / 2),
					Point(m->src.cols / 2 + square_len2,
							m->src.rows / 2 + square_len2), m->src));
	roi222.push_back(
			My_ROI(Point(m->src.cols / 2.5, m->src.rows / 2.5),
					Point(m->src.cols / 2.5 + square_len2,
							m->src.rows / 2.5 + square_len2), m->src));
	roi222.push_back(
			My_ROI(Point(m->src.cols / 2, m->src.rows / 1.5),
					Point(m->src.cols / 2 + square_len2,
							m->src.rows / 1.5 + square_len2), m->src));
	roi222.push_back(
			My_ROI(Point(m->src.cols / 2.5, m->src.rows / 1.8),
					Point(m->src.cols / 2.5 + square_len2,
							m->src.rows / 1.8 + square_len2), m->src));

	for (int i = 0; i < 50; i++) {
		m->cap >> m->src;
		flip(m->src, m->src, 1);
		for (int j = 0; j < NSAMPLES; j++) {
			roi222[j].draw_rectangle(m->src);
		}
		string imgText = string("Cover rectangles with palm");
		printText2(m->src, imgText);

		if (i == 30) {
			//	imwrite("./images/waitforpalm1.jpg",m->src);
		}

		imshow("img1", m->src);
		out2 << m->src;
		if (cv::waitKey(30) >= 0)
			break;
	}
}

int getMedian2(vector<int> val) {
	int median;
	size_t size = val.size();
	sort(val.begin(), val.end());
	if (size % 2 == 0) {
		median = val[size / 2 - 1];
	} else {
		median = val[size / 2];
	}
	return median;
}

void getAvgColor2(MyImage *m, My_ROI roi, int avg[3]) {
	Mat r;
	roi.roi_ptr.copyTo(r);
	vector<int> hm;
	vector<int> sm;
	vector<int> lm;
	// generate vectors
	for (int i = 2; i < r.rows - 2; i++) {
		for (int j = 2; j < r.cols - 2; j++) {
			hm.push_back(r.data[r.channels() * (r.cols * i + j) + 0]);
			sm.push_back(r.data[r.channels() * (r.cols * i + j) + 1]);
			lm.push_back(r.data[r.channels() * (r.cols * i + j) + 2]);
		}
	}
	avg[0] = getMedian2(hm);
	avg[1] = getMedian2(sm);
	avg[2] = getMedian2(lm);
}

void average2(MyImage *m) {
	m->cap >> m->src;
	flip(m->src, m->src, 1);
	for (int i = 0; i < 30; i++) {
		m->cap >> m->src;
		flip(m->src, m->src, 1);
		cvtColor(m->src, m->src, ORIGCOL2COL);
		for (int j = 0; j < NSAMPLES; j++) {
			getAvgColor2(m, roi222[j], avgColor2[j]);
			roi222[j].draw_rectangle(m->src);
		}
		cvtColor(m->src, m->src, COL2ORIGCOL);
		string imgText = string("Finding average color of hand");
		printText2(m->src, imgText);
		imshow("img1", m->src);
		if (cv::waitKey(30) >= 0)
			break;
	}
}

void initTrackbars2() {
	for (int i = 0; i < NSAMPLES; i++) {
		c_lower2[i][0] = 12;
		c_upper2[i][0] = 7;
		c_lower2[i][1] = 30;
		c_upper2[i][1] = 40;
		c_lower2[i][2] = 80;
		c_upper2[i][2] = 80;
	}
	createTrackbar("lower1", "trackbars", &c_lower2[0][0], 255);
	createTrackbar("lower2", "trackbars", &c_lower2[0][1], 255);
	createTrackbar("lower3", "trackbars", &c_lower2[0][2], 255);
	createTrackbar("upper1", "trackbars", &c_upper2[0][0], 255);
	createTrackbar("upper2", "trackbars", &c_upper2[0][1], 255);
	createTrackbar("upper3", "trackbars", &c_upper2[0][2], 255);
}

void normalizeColors2(MyImage * myImage) {
	// copy all boundries read from trackbar
	// to all of the different boundries
	for (int i = 1; i < NSAMPLES; i++) {
		for (int j = 0; j < 3; j++) {
			c_lower2[i][j] = c_lower2[0][j];
			c_upper2[i][j] = c_upper2[0][j];
		}
	}
	// normalize all boundries so that
	// threshold is whithin 0-255
	for (int i = 0; i < NSAMPLES; i++) {
		if ((avgColor2[i][0] - c_lower2[i][0]) < 0) {
			c_lower2[i][0] = avgColor2[i][0];
		}
		if ((avgColor2[i][1] - c_lower2[i][1]) < 0) {
			c_lower2[i][1] = avgColor2[i][1];
		}
		if ((avgColor2[i][2] - c_lower2[i][2]) < 0) {
			c_lower2[i][2] = avgColor2[i][2];
		}
		if ((avgColor2[i][0] + c_upper2[i][0]) > 255) {
			c_upper2[i][0] = 255 - avgColor2[i][0];
		}
		if ((avgColor2[i][1] + c_upper2[i][1]) > 255) {
			c_upper2[i][1] = 255 - avgColor2[i][1];
		}
		if ((avgColor2[i][2] + c_upper2[i][2]) > 255) {
			c_upper2[i][2] = 255 - avgColor2[i][2];
		}
	}
}

HandGesture getHG2(){
	return hg2;
}

void produceBinaries2(MyImage *m) {
	Mat foo;
		m->bwList.push_back(Mat(m->srcLR.rows, m->srcLR.cols, CV_8U));
		inRange(m->srcLR, Scalar(80, 20, 20), Scalar(220, 260, 260), m->bwList[0]);


	m->bwList[0].copyTo(m->bw);
	m->bw += m->bwList[0];
	medianBlur(m->bw, m->bw, 7);
}

void initWindows2(MyImage m) {
	namedWindow("trackbars", CV_WINDOW_KEEPRATIO);
	namedWindow("img1", CV_WINDOW_FULLSCREEN);
}

void showWindows2(MyImage m) {
	pyrDown(m.bw, m.bw);
	pyrDown(m.bw, m.bw);
	Rect roi(Point(3 * m.src.cols / 4, 0), m.bw.size());
	vector<Mat> channels;
	Mat result;
	for (int i = 0; i < 3; i++)
		channels.push_back(m.bw);
	merge(channels, result);
	result.copyTo(m.src(roi));
	imshow("img2", m.src);
}

int findBiggestContour2(vector<vector<Point> > contours) {
	int indexOfBiggestContour = -1;
	int sizeOfBiggestContour = 0;
	for (int i = 0; i < contours.size(); i++) {
		if (contours[i].size() > sizeOfBiggestContour) {
			sizeOfBiggestContour = contours[i].size();
			indexOfBiggestContour = i;
		}
	}
	return indexOfBiggestContour;
}

void myDrawContours2(MyImage *m, HandGesture *hg) {
	drawContours(m->src, hg->hullP, hg->cIdx, cv::Scalar(200, 0, 0), 2, 8,
			vector<Vec4i>(), 0, Point());

	rectangle(m->src, hg->bRect.tl(), hg->bRect.br(), Scalar(0, 0, 200));
	vector<Vec4i>::iterator d = hg->defects[hg->cIdx].begin();
	int fontFace = FONT_HERSHEY_PLAIN;

	vector<Mat> channels;
	Mat result;
	for (int i = 0; i < 3; i++)
		channels.push_back(m->bw);
	merge(channels, result);
	//	drawContours(result,hg->contours,hg->cIdx,cv::Scalar(0,200,0),6, 8, vector<Vec4i>(), 0, Point());
	drawContours(result, hg->hullP, hg->cIdx, cv::Scalar(0, 0, 250), 10, 8,
			vector<Vec4i>(), 0, Point());

	while (d != hg->defects[hg->cIdx].end()) {
		Vec4i& v = (*d);
		int startidx = v[0];
		Point ptStart(hg->contours[hg->cIdx][startidx]);
		int endidx = v[1];
		Point ptEnd(hg->contours[hg->cIdx][endidx]);
		int faridx = v[2];
		Point ptFar(hg->contours[hg->cIdx][faridx]);
		float depth = v[3] / 256;
		/*
		 line( m->src, ptStart, ptFar, Scalar(0,255,0), 1 );
		 line( m->src, ptEnd, ptFar, Scalar(0,255,0), 1 );
		 circle( m->src, ptFar,   4, Scalar(0,255,0), 2 );
		 circle( m->src, ptEnd,   4, Scalar(0,0,255), 2 );
		 circle( m->src, ptStart,   4, Scalar(255,0,0), 2 );
		 */
		circle(result, ptFar, 9, Scalar(0, 205, 0), 5);

		d++;

	}
//	imwrite("./images/contour_defects_before_eliminate.jpg",result);

}

void makeContours2(MyImage *m, HandGesture* hg) {
	Mat aBw;
	pyrUp(m->bw, m->bw);
	m->bw.copyTo(aBw);
	findContours(aBw, hg->contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	hg->initVectors();
	hg->cIdx = findBiggestContour2(hg->contours);
	if (hg->cIdx != -1) {
//		approxPolyDP( Mat(hg->contours[hg->cIdx]), hg->contours[hg->cIdx], 11, true );
		hg->bRect = boundingRect(Mat(hg->contours[hg->cIdx]));
		convexHull(Mat(hg->contours[hg->cIdx]), hg->hullP[hg->cIdx], false,
				true);
		convexHull(Mat(hg->contours[hg->cIdx]), hg->hullI[hg->cIdx], false,
				false);
		approxPolyDP(Mat(hg->hullP[hg->cIdx]), hg->hullP[hg->cIdx], 18, true);
		if (hg->contours[hg->cIdx].size() > 3) {
			convexityDefects(hg->contours[hg->cIdx], hg->hullI[hg->cIdx],
					hg->defects[hg->cIdx]);
			hg->eleminateDefects(m);
		}
		bool isHand = hg->detectIfHand();
		hg->printGestureInfo(m->src);
		if (isHand) {
			hg->getFingerTips(m);
			hg->drawFingerTips(m);
			myDrawContours2(m, hg);
		}
	}
	else{
		hg->bRect = Rect(0,0,0,0);
	}
}

void setupFindHand2(MyImage *m) {
	init2(m);
}

void findHand2(MyImage *m){
	hg2.frameNumber++;
	flip(m->src, m->src, 1);
	m->cap >> m->src;
	pyrDown(m->src, m->srcLR);
	blur(m->srcLR, m->srcLR, Size(3, 3));
	cvtColor(m->srcLR, m->srcLR, ORIGCOL2COL);
	produceBinaries2(m);
	cvtColor(m->srcLR, m->srcLR, COL2ORIGCOL);
	makeContours2(m, &hg2);
	hg2.getFingerNumber(m);
	showWindows2(*m);
	out2 << m->src;

}
