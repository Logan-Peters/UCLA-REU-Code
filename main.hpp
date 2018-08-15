#ifndef _MAIN_HEADER_ 
#define _MAIN_HEADER_ 

#include <opencv2/core/mat.hpp>
#include <opencv2/core/mat.inl.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <sys/_types/_size_t.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <fstream>
#include "handGesture.hpp"
#include "myImage.hpp"

using namespace cv;


#define ORIGCOL2COL CV_BGR2HLS
#define COL2ORIGCOL CV_HLS2BGR
#define NSAMPLES 7
#define PI 3.14159

HandGesture getHG();
void findHand(MyImage *m);
void setupFindHand(MyImage *m);


#endif
