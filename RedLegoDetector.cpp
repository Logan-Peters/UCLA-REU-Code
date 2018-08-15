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
#include "main.hpp"
#include <chrono>
#include "myImage.hpp"
#include "handGesture.hpp"
#include "main2.hpp"
#include "PointPolygonInteresector.hpp"
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <iostream>
#include <cstdlib>
#include <sys/stat.h>
#include <errno.h>
#include <dirent.h>
#include <cstdio>
#include <string>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>

using namespace cv;
using namespace std;
using namespace chrono;

int fast = 0;
int fine = 0;
int still = 0;
double memorizeTime;
vector<double> height;
vector<double> height2;
int numOfLegos = 0;
int iterationOfExperiment = 0;
bool foundLegosYet = false;
auto start =
		duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

//A structure containing data about the hand used to manipulate the legos
struct Hand {
	bool found;
	Point prevHand;
	HandGesture hand;
};

//A structures containing all the statistics for lego's once they've appeared
struct foundLego {
	double startTime;
	double variationDistance;
	double variationAngle;
	double closestDistance;
	double closestAngle;
	double height;
	RotatedRect currentPos;
};

enum {
	STILL, FINE, SPEEDY
};

//Given x,y coordinates from each camera of an ROI returns estimated depth in inches
float* get3DCoords(float* l, float* r) {
	double b = 100.0 / 25.4 * 2;
	double f = (8.1259257941370424 + 8.1351812555998572) / 4 * 100 / 25.4 / .022
			/ 1.3; //Constants for our webcam
	int xl = l[0], xr = r[0], yi;
	yi = (l[1] + r[1]) / 2;
	int d = abs(xl - xr);
	double z = ((b * f / d));
	double x = z * xl / f;
	double y = z * yi / f;
	float* p = new float[3];
	p[0] = x;
	p[1] = y;
	p[2] = z;
	return p;
}
bool tempor = true;

int n = 0;
int sockfd = 0;

//Establishes the connection to the rasberry pi
void setUpRasberry() {
	struct sockaddr_in serv_addr;

	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		cout << "Error : Could not create socket" << endl;
		return;
	}
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(65432);
	serv_addr.sin_addr.s_addr = inet_addr("172.20.10.6");

	if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr))
			< 0) {
		cout << "Error : Connect Failed" << endl;
		return;
	}
	return;
}

//Sends whether the hand is in frame to the PI and receives whether or not the PI thinks a lego might
//have been placed
bool sendHandToRasberryPI(bool found) {
	char * buffer = "no";
	if (found) {
		buffer = "yes";
	}

	n = write(sockfd, buffer, strlen(buffer));

	string output(1024, 0);
	read(sockfd, &output[0], 1024);
	if (strcmp(output.c_str(), "placed") == 0) {
		cout << "placed" << endl;
		return true;
	}
	cout << "not placed " << endl;
	return false;
}

//Processes the image to find the edges of the legos
Mat legoImageProcessing(Mat baseImage) {
	Mat grid = Mat::zeros(baseImage.size(), CV_32F); //Creates an image to copy onto
	imwrite("Base.jpg", baseImage); //Saves base image
	Mat temp = Mat::zeros(baseImage.size(), CV_32F); //a blank image to threshold onto
	GaussianBlur(baseImage, baseImage, Size(3, 3), 0, 0); //blurs the image to get rid of irregularities
	cvtColor(baseImage, baseImage, CV_BGR2HSV); //Swaps color space from BGR to HSV
	inRange(baseImage, Scalar(0, 50, 60), Scalar(10, 300, 300), temp); //Threshold the image
	inRange(baseImage, Scalar(170, 50, 60), Scalar(190, 300, 300), baseImage); //Threshold the image
	addWeighted(baseImage, 1.0, temp, 1.0, 0.0, grid); //Combines the two thresholds since red is on both ends of the hue values

	imwrite("thresh.jpg", grid); // Saves threshholded image

	//Dilates and then erodes the thresholded image consolidate parts
	dilate(grid, grid, getStructuringElement(MORPH_ELLIPSE, Size(35, 35)));
	erode(grid, grid, getStructuringElement(MORPH_ELLIPSE, Size(35, 35)));

	imwrite("de.jpg", grid);

	//Erodes then dilates the thresholded image
	erode(grid, grid, getStructuringElement(MORPH_ELLIPSE, Size(20, 20)));
	dilate(grid, grid, getStructuringElement(MORPH_ELLIPSE, Size(20, 20)));

	//Displays the thresholded image
	string s = "";
	if (tempor) {
		s = "thresh1";
	} else {
		s = "thresh2";
	}
	tempor = !tempor;
	imshow(s, grid);
	imwrite("ed.jpg", grid);

	//Detects the edges of the thresholded image
	Canny(grid, grid, 100, 200, 3);
	return grid;
}

//Gets the contours of the legos
vector<vector<Point>> getContours(Mat processedImage) {
	vector<Vec4i> hierarchy;
	vector<vector<Point>> contours;
	findContours(processedImage, contours, hierarchy, RETR_EXTERNAL,
			CV_CHAIN_APPROX_SIMPLE, Point(0, 0)); //Calculates the contours and ignores hierarchy
	return contours;
}

//Draws a gray rectangle to represent the hand
void drawHandRectangle(Hand *h, Mat *imgToDraw) {
	if (h->hand.bRect.area() < 80000) { //Only draws hand if bigger than a certain size
		h->found = false;
		return;
	}
	rectangle(*imgToDraw, h->hand.bRect, Scalar(100, 100, 100), 2, 8, 0); //Draws a gray rectangle
	h->found = true;
}

//updates current hand based on new frame
void updateHand(Hand *h, HandGesture newHand) {
	h->prevHand = Point(h->hand.bRect.x + h->hand.bRect.width / 2,
			h->hand.bRect.y + h->hand.bRect.height / 2); //Updates prevHand
	h->hand = newHand; //saves the new hand
}

//Calculates the hand movement type: STILL = hand off screen, FAST = hand on screen in rapid motion, FINE = slow hand movement
int handMovementTracking(Hand h) {
	if (!h.found) { //If the hand isn't found
		return STILL;
	}
	if (norm(
			Point(h.hand.bRect.x + h.hand.bRect.width / 2,
					h.hand.bRect.y + h.hand.bRect.height / 2) - h.prevHand)
			< 75) { //If the hand moved less than 75 pixels
		return FINE;
	}
	return SPEEDY;
}

//Transforms legos so that it is ordered so that the closest legos are at the same index as in firstLegos
void maximizeCloseness(vector<RotatedRect> *legos,
		vector<RotatedRect> *firstLegos) {
	vector<int> pos;
	vector<int> distance;
	int newPos;
	int minDist = 1000000;
	int replaced = 0;

	//loops through legos and first legos
	for (int i = 0; i < legos->size(); i++) {
		newPos = i - replaced; //so that legos is bigger than firstLegos it'll just be put it at the end
		for (int j = 0; j < firstLegos->size(); j++) {
			if (norm(legos->at(i).center - firstLegos->at(j).center)
					< minDist) { //if firstLegos->at(j) is the closest lego yet
				if (find(pos.begin(), pos.end(), j) == pos.end()) { //if no other lego has this as it's closest
					minDist = norm(
							legos->at(i).center - firstLegos->at(j).center);
					newPos = j;
				} else if (norm(legos->at(i).center - firstLegos->at(j).center)
						< distance[((find(pos.begin(), pos.end(), j)
								- pos.begin()))]) { //if this lego is closer than the other lego that has it as it's closest
					ptrdiff_t a =
							(find(pos.begin(), pos.end(), j) - pos.begin());
					legos->push_back(legos->at(a)); //re-adds the lego that this lego replaced
					pos[find(pos.begin(), pos.end(), j) - pos.begin()] = -1; //sets the pos position to -1 so it never get's found
					minDist = norm(
							legos->at(i).center - firstLegos->at(j).center);
					newPos = j;
					replaced++; //keeps track of how many -1's there are
				}
			}
		}
		pos.push_back(newPos);
		distance.push_back(minDist);
		minDist = 1000000;
	}
	vector<RotatedRect> temp;

	//Reorders legos based on results from previous loop
	for (int i = 0; i < legos->size() - replaced; i++) {
		temp.push_back(
				legos->at(
						(int) (find(pos.begin(), pos.end(), i) - pos.begin())));
	}
	*legos = temp;
}

//draws best fit rotated rectangle for the legos
void drawLegos(RotatedRect cur, Mat toDraw, Scalar colorToDraw) {
	cv::Point2f pts2[4];
	cur.points(pts2);
	line(toDraw, pts2[0], pts2[1], colorToDraw, 2, 8, 0);
	line(toDraw, pts2[1], pts2[2], colorToDraw, 2, 8, 0);
	line(toDraw, pts2[2], pts2[3], colorToDraw, 2, 8, 0);
	line(toDraw, pts2[3], pts2[0], colorToDraw, 2, 8, 0);
}

//updates statistics for each lego
void updateFoundLegosStatistics(vector<RotatedRect> *prevLego,
		vector<RotatedRect> *firstLegos, vector<foundLego> *foundLegos,
		bool legoPlaced) {
	if (firstLegos->size() < prevLego->size()) {
		cout << "Seems like there are more legos placed than you started with"
				<< endl;
		return;
	}
	for (int i = 0; i < prevLego->size(); i++) {
		if (!foundLegosYet || foundLegos->size() <= i) { //if it's the first time through or foundLegos size is less than i
			foundLego l;
			foundLegosYet = true;
			int minLoc = 0;
			int minDistance = 10000000;
			for (int j = 0; j < firstLegos->size(); j++) { //finds closest lego
				if (norm(firstLegos->at(j).center - prevLego->at(i).center)
						< minDistance) {
					minLoc = j;
					minDistance = norm(
							firstLegos->at(j).center - prevLego->at(i).center);
				}
			}

			numOfLegos++;
			cout << "New" << endl;
			//updates the statistics
			l.closestAngle = abs(
					firstLegos->at(minLoc).angle - prevLego->at(i).angle);
			l.closestDistance = minDistance;
			l.startTime = duration_cast<milliseconds>(
					system_clock::now().time_since_epoch()).count();
			l.variationAngle = 0;
			l.variationDistance = 0;
			l.currentPos = prevLego->at(i);
			foundLegos->push_back(l);
		} else {
			//Way to keep track of how many legos there are(not perfect)
			if ((foundLegos->at(i).height - height[i] > 1.2
					|| prevLego->at(i).size.area()
							- foundLegos->at(i).currentPos.size.area() > 1250)) {
				cout << "new lego" << endl;
				numOfLegos++;
			}
			if (foundLegos->at(i).height - height[i] > .2) {
				cout << "why...";
			}
			if (prevLego->at(i).size.area()
					- foundLegos->at(i).currentPos.size.area() > 1000) {
				cout << "...";
			}
			cout << numOfLegos << endl;

			if (foundLegos->at(i).closestAngle
					> fmod(
							abs(
									foundLegos->at(i).currentPos.angle
											- firstLegos->at(i).angle), 45)) {
				foundLegos->at(i).closestAngle = fmod(
						abs(
								foundLegos->at(i).currentPos.angle
										- firstLegos->at(i).angle), 45);
			}
			if (foundLegos->at(i).closestDistance
					> norm(
							foundLegos->at(i).currentPos.center
									- firstLegos->at(i).center)) {
				foundLegos->at(i).closestDistance = norm(
						foundLegos->at(i).currentPos.center
								- firstLegos->at(i).center);
			}
			if (norm(
					foundLegos->at(i).currentPos.center
							- prevLego->at(i).center) > 5) {
//				cout
//				<< norm(
//						foundLegos->at(i).currentPos.center
//						- prevLego->at(i).center) << endl;

				foundLegos->at(i).variationDistance += norm(
						foundLegos->at(i).currentPos.center
								- prevLego->at(i).center);
			}
			if (fmod(
					abs(
							foundLegos->at(i).currentPos.angle
									- prevLego->at(i).angle), 45) > 3) {
//				cout
//						<< fmod(
//								abs(
//										foundLegos->at(i).currentPos.angle
//												- prevLego->at(i).angle), 45)
//						<< endl;
				foundLegos->at(i).variationAngle += fmod(
						abs(
								foundLegos->at(i).currentPos.angle
										- prevLego->at(i).angle), 45);
			}
			foundLegos->at(i).currentPos = prevLego->at(i);

		}
	}

}

//Finds corresponding legos and calculate distance based on stereovision techniques.  Prints out guess.
void stereoVision(vector<RotatedRect> legos, vector<RotatedRect> legos2,
		vector<Scalar> *color, vector<Scalar> *color2, Mat imageToDraw) {
	int a = 0;
	int b = 0;
	for (int i = 0; i < min(legos.size() - a, legos2.size() + b); i++) {
		if (i + a >= legos.size()) {
			break;
		} else if (i + b >= legos2.size()) {
			break;
		}

		//checks if the y difference is greater than 50
		if (abs(legos[i + a].center.y - legos2[i + b].center.y) > 50) {
			if (legos[i + a].center.y < legos2[i + b].center.y) { //depending on which one was on the bottom it removes it and then continues loop
				legos.erase(legos.begin() + i + a);
				a++;
			} else {
				legos2.erase(legos2.begin() + i + b);
				b++;
			}
			i--;
			continue;
		}

		//checks that if the x values are too far away from each other
		if (abs(legos[i + a].center.x - legos[i + b].center.x) > 1000) {
			if (legos[i + a].center.y < legos2[i + b].center.y) { //depending on which one was on the bottom it removes it and then continues loop
				legos.erase(legos.begin() + i + a);
				a++;
			} else {
				legos2.erase(legos2.begin() + i + b);
				b++;
			}
			i--;
			continue;
		}

		//passed previous two conditions so must be the same lego

		color2->at(i + b) = color->at(i + a); //sets them to be the same color
		line(imageToDraw, legos[i + a].center,
				Point(legos2[i + b].center.x + imageToDraw.cols / 2,
						legos2[i + b].center.y), Scalar(255, 255, 255), 2, 8,
				0); //draws a line between them
		int q = legos2[i + b].center.x - legos[i + a].center.x;
		string s = "Offset: " + to_string(q);
		putText(imageToDraw, s,
				cvPoint(
						legos[i + a].center.x / 2 + imageToDraw.cols / 4
								+ legos2[i + b].center.x / 2,
						legos[i + a].center.y / 2 - 15
								+ legos2[i + b].center.y / 2),
				FONT_HERSHEY_COMPLEX_SMALL, 1.4, cvScalar(255, 255, 255), 1,
				CV_AA); //writes offset
		float left[2] = { legos[i + a].center.x, legos[i + a].center.y };
		float right[2] = { legos2[i + a].center.x, legos2[i + a].center.y };
		float* coord = get3DCoords(left, right);
		height.push_back(coord[2]);
		putText(imageToDraw, to_string(coord[2]),
				cvPoint(legos2[i + b].center.x - 10 + imageToDraw.cols / 2,
						legos2[i + b].center.y + 40),
				FONT_HERSHEY_COMPLEX_SMALL, 1.4, cvScalar(255, 255, 255), 1,
				CV_AA); //writes distance
		putText(imageToDraw, to_string(coord[2]),
				cvPoint(legos[i + a].center.x - 10, legos[i + a].center.y + 40),
				FONT_HERSHEY_COMPLEX_SMALL, 1.4, cvScalar(255, 255, 255), 1,
				CV_AA); //writes distance
	}
}

//Displays the original placement of legos in green and the placement of the legos in
//red.  Also prints out and saves a number of metrics to be used as input for HMM.
void showCorrected(vector<RotatedRect> firstLegos2, vector<RotatedRect> legos2,
		vector<RotatedRect> firstLegos, vector<RotatedRect> legos,
		Mat leftCorrection, Mat rightCorrection, long start2,
		vector<foundLego> foundLegos, vector<foundLego> foundLegos2) {
	double dist = 0;
	double angle = 0;
	int legoNumber = 0;
	ofstream file;
	file.open("Data.txt", std::ios::app);
	file << fixed;
	cout << iterationOfExperiment << endl;
	file << iterationOfExperiment << ", ";

	maximizeCloseness(&legos, &firstLegos);

	//finds each legos distance and angle difference and draws them
	for (int q = 0; q < legos.size(); q++) {
		Point2f pts2[4];
		legos[q].points(pts2);
		line(leftCorrection, pts2[0], pts2[1], Scalar(0, 255, 0), 2, 8, 0);
		line(leftCorrection, pts2[1], pts2[2], Scalar(0, 255, 0), 2, 8, 0);
		line(leftCorrection, pts2[2], pts2[3], Scalar(0, 255, 0), 2, 8, 0);
		line(leftCorrection, pts2[3], pts2[0], Scalar(0, 255, 0), 2, 8, 0);

		firstLegos[q].points(pts2);
		line(leftCorrection, pts2[0], pts2[1], Scalar(0, 0, 255), 2, 8, 0);
		line(leftCorrection, pts2[1], pts2[2], Scalar(0, 0, 255), 2, 8, 0);
		line(leftCorrection, pts2[2], pts2[3], Scalar(0, 0, 255), 2, 8, 0);
		line(leftCorrection, pts2[3], pts2[0], Scalar(0, 0, 255), 2, 8, 0);
		file << norm(legos[q].center - firstLegos[q].center) << ", ";
		file << fmod(abs(legos[q].angle - firstLegos[q].angle), 45) << ", ";
		cout << "Lego Number " << legoNumber << " distance "
				<< norm(legos[q].center - firstLegos[q].center) << endl;
		cout << "Lego Number " << legoNumber << " angle "
				<< fmod(abs(legos[q].angle - firstLegos[q].angle), 45) << endl;
		dist += norm(legos[q].center - firstLegos[q].center);
		angle += fmod(abs(legos[q].angle - firstLegos[q].angle), 45);
		legoNumber++;
	}

	double dist2 = 0;
	double angle2 = 0;

	//finds each legos distance and angle difference and draws them
	maximizeCloseness(&legos2, &firstLegos2);
	legoNumber = 0;
	for (int q = 0; q < legos2.size(); q++) {
		Point2f pts2[4];
		legos2[q].points(pts2);
		line(rightCorrection, pts2[0], pts2[1], Scalar(0, 255, 0), 2, 8, 0);
		line(rightCorrection, pts2[1], pts2[2], Scalar(0, 255, 0), 2, 8, 0);
		line(rightCorrection, pts2[2], pts2[3], Scalar(0, 255, 0), 2, 8, 0);
		line(rightCorrection, pts2[3], pts2[0], Scalar(0, 255, 0), 2, 8, 0);

		firstLegos2[q].points(pts2);
		line(rightCorrection, pts2[0], pts2[1], Scalar(0, 0, 255), 2, 8, 0);
		line(rightCorrection, pts2[1], pts2[2], Scalar(0, 0, 255), 2, 8, 0);
		line(rightCorrection, pts2[2], pts2[3], Scalar(0, 0, 255), 2, 8, 0);
		line(rightCorrection, pts2[3], pts2[0], Scalar(0, 0, 255), 2, 8, 0);
		file << norm(legos2[q].center - firstLegos2[q].center) << ", ";
		file << fmod(abs(legos2[q].angle - firstLegos2[q].angle), 45) << ", ";
		cout << "Lego Number " << legoNumber << " distance "
				<< norm(legos2[q].center - firstLegos2[q].center) << endl;
		cout << "Lego Number " << legoNumber << " angle "
				<< fmod(abs(legos2[q].angle - firstLegos2[q].angle), 45)
				<< endl;
		legoNumber++;

		dist2 += norm(legos2[q].center - firstLegos2[q].center);
		angle2 += fmod(abs(legos2[q].angle - firstLegos2[q].angle), 45);
	}

	//writes all the statistics
	file << (dist + dist2) / 2 << ", ";
	file << (angle + 2) / 2 << ", ";
	file << still << ", ";
	file << fine << ", ";
	file << fast << ", ";
	cout << "Distance Difference: " << (dist + dist2) / 2 << endl;
	cout << "Angle Difference: " << (angle + angle2) / 2 << endl;
	file
			<< (duration_cast<milliseconds>(
					system_clock::now().time_since_epoch()).count() - start2)
					/ 1000000000.0 << ", ";
	file << memorizeTime << ", ";
	file << start2 - memorizeTime << ", ";
	cout << start2 - memorizeTime << endl;
	;
	file
			<< duration_cast<milliseconds>(
					system_clock::now().time_since_epoch()).count() << ", ";
	cout
			<< duration_cast<milliseconds>(
					system_clock::now().time_since_epoch()).count() << endl;
	cout << "Time to Memorize: " << memorizeTime << endl;
	cout << "Time to Place: "
			<< (duration_cast<milliseconds>(
					system_clock::now().time_since_epoch()).count() - start2)
			<< endl;
	cout << "Still: " << still << endl;
	cout << "Fast: " << fast << endl;
	cout << "Fine: " << fine << endl;
	legoNumber = 0;
	for (int i = 0; i < min(foundLegos.size(), foundLegos2.size()); i++) {
		cout << "Lego Number " << legoNumber << " Variation Angle: "
				<< (foundLegos[i].variationAngle + foundLegos2[i].variationAngle)
						/ 2 << endl;
		cout << "Lego Number " << legoNumber << " Variation Distance: "
				<< (foundLegos[i].variationDistance
						+ foundLegos2[i].variationDistance) / 2 << endl;
		cout << "Lego Number " << legoNumber << " Closest Angle: "
				<< (foundLegos[i].closestAngle + foundLegos2[i].closestAngle)
						/ 2 << endl;
		cout << "Lego Number " << legoNumber << " Closest Distance: "
				<< (foundLegos[i].closestDistance
						+ foundLegos2[i].closestDistance) / 2 << endl;

		cout << "Lego Number " << legoNumber << " Initial Placement Time: "
				<< (foundLegos[i].startTime - start2) << endl;

		file
				<< (foundLegos[i].variationAngle + foundLegos2[i].variationAngle)
						/ 2 << ", ";
		file
				<< (foundLegos[i].variationDistance
						+ foundLegos2[i].variationDistance) / 2 << ", ";
		file << (foundLegos[i].closestAngle + foundLegos2[i].closestAngle) / 2
				<< ", ";
		file
				<< (foundLegos[i].closestDistance
						+ foundLegos2[i].closestDistance) / 2 << ", ";

		file << (foundLegos[i].startTime - start2) << ", ";
		legoNumber++;
	}
	file << endl;
	imshow("Left Correction", leftCorrection);
	imshow("Right Correction", rightCorrection);
	waitKey(0);
}

int main(int argc, char** argv) {
	cout << fixed;
	//setUpRasberry();
	vector<RotatedRect> firstlegos;
	vector<RotatedRect> firstlegos2;
	while (true) {
		VideoCapture cap(0);
		VideoCapture cap2(1);
		iterationOfExperiment++;
		vector<foundLego> foundLegos;
		vector<foundLego> foundLegos2;
		int count = 0;
		MyImage m(0);
		MyImage m2(1);
		Hand hand;
		Hand hand2;
		Mat imgTmp;
		Mat imgTmp2;
		setupFindHand(&m);
		setupFindHand2(&m2);
		cap2.read(imgTmp2);
		cap.read(imgTmp);
		bool newLegoPossible = false;
		vector<RotatedRect> prevLegos;
		vector<RotatedRect> prevLegos2;
		vector<RotatedRect> legos;
		vector<RotatedRect> legos2;
		vector<Scalar> color;
		vector<Scalar> prevColor;
		vector<Scalar> color2;
		vector<Scalar> prevColor2;
		bool legoPlaced;
		while (true) {
			Mat lines = Mat::zeros(imgTmp.size(), CV_8U);
			Mat lines2 = Mat::zeros(imgTmp.size(), CV_8U);
			cvtColor(lines, lines, CV_GRAY2BGR);
			cvtColor(lines2, lines2, CV_GRAY2BGR);

			Mat imgTmp;
			Mat imgTmp2;
			cap.read(imgTmp);
			cap2.read(imgTmp2);
			Mat grid = legoImageProcessing(imgTmp);
			Mat grid2 = legoImageProcessing(imgTmp2);

			imwrite("canny.jpg", grid2);

			vector<vector<Point>> contours = getContours(grid);
			vector<vector<Point>> contours2 = getContours(grid2);

			vector<Scalar> color;
			vector<Scalar> color2;

			//Finds hand and draws
			findHand(&m);
			findHand2(&m2);
			updateHand(&hand, getHG());
			updateHand(&hand2, getHG2());
			drawHandRectangle(&hand, &lines);
			drawHandRectangle(&hand2, &lines2);

			//records movement types
			int movement = handMovementTracking(hand);
			if (movement == FINE) {
				fine++;
			} else if (movement == STILL) {
				still++;
			} else if (movement == SPEEDY) {
				fast++;
			}
			bool temp = movement;
			movement = handMovementTracking(hand2);
			if (movement == FINE) {
				fine++;
			} else if (movement == STILL) {
				still++;
			} else if (movement == SPEEDY) {
				fast++;
			}

			//sends whether hand is in frame and receives whether the PI thinks a lego has been placed
			//legoPlaced = sendHandToRasberryPI(
			//		movement != STILL && temp != STILL);
			//if the hand isn't found then finds legos and calculates distance
			if (movement == STILL && temp == STILL) {
				if (!newLegoPossible) {
					newLegoPossible = true;
				}

				for (int i = 0; i < contours.size(); i++) {
					RotatedRect cur = minAreaRect(Mat(contours[i]));
					legos.push_back(cur);
				}
				maximizeCloseness(&legos, &prevLegos);

				for (int i = 0; i < legos.size(); i++) {
					if (i >= prevLegos.size()) {
						Scalar colorToDraw = Scalar(rand() % 205 + 50,
								rand() % 205 + 50, rand() % 205 + 50);
						drawLegos(legos[i], lines, colorToDraw);
						color.push_back(colorToDraw);
					} else {
						drawLegos(legos[i], lines, prevColor[i]);
						color.push_back(prevColor[i]);
					}
				}

				for (int i = 0; i < contours2.size(); i++) {
					RotatedRect cur = minAreaRect(Mat(contours2[i]));
					legos2.push_back(cur);
				}

				maximizeCloseness(&legos2, &prevLegos2);

				for (int i = 0; i < legos2.size(); i++) {
					if (i >= prevLegos2.size()) {
						Scalar colorToDraw = Scalar(rand() % 205 + 50,
								rand() % 205 + 50, rand() % 205 + 50);
						drawLegos(legos2[i], lines2, colorToDraw);
						color2.push_back(colorToDraw);
					} else {
						drawLegos(legos2[i], lines2, prevColor2[i]);
						color2.push_back(prevColor2[i]);
					}
				}
			}
//			putText(lines2,
//					"guess: "
//							+ to_string(
//									0.04676397
//											+ (177802.6 - 0.04676397)
//													/ (1
//															+ pow(
//																	((cur.size.area())
//																			/ 0.00005175283),
//																	0.5001077))),
//					cvPoint(legos2[i].center.x - 10, legos2[i].center.y - 10),
//					FONT_HERSHEY_COMPLEX_SMALL, 1.4, cvScalar(255, 255, 255), 1,
//					CV_AA);

			imwrite("rects.jpg", lines2);
			Mat im3(lines.rows, lines.cols * 2, CV_8UC3);
			Mat left(im3, Rect(0, 0, lines.cols, lines.rows));
			lines.copyTo(left);
			Mat right(im3, Rect(lines.cols, 0, lines.cols, lines.rows));
			lines2.copyTo(right);
			stereoVision(legos, legos2, &color, &color2, im3);

			//updates vectors of legos
			if (movement == STILL && temp == STILL) {
				prevLegos = legos;
				prevColor = color;
				prevLegos2 = legos2;
				prevColor2 = color2;
			}
			//updates statistics
			if (count > 1 && newLegoPossible) {
				updateFoundLegosStatistics(&prevLegos, &firstlegos, &foundLegos,
						legoPlaced);
				updateFoundLegosStatistics(&prevLegos2, &firstlegos2,
						&foundLegos2, legoPlaced);
				newLegoPossible = false;
			}
			Mat im4(im3.rows / 2, im3.cols / 2, CV_8UC3);
			resize(im3, im4, im4.size(), 0, 0, INTER_LINEAR);

			Mat eraser(lines.rows, lines.cols, CV_8UC3);

			Mat eraser2(lines.rows, lines.cols, CV_8UC3);
			Mat rightCorrection(lines.rows, lines.cols, CV_8UC3);
			Mat leftCorrection(lines.rows, lines.cols, CV_8UC3);

			imwrite("cloud.jpg", im4);
			imshow("Cloud", im4);
			if (waitKey(50) == char('q')) { //says user is finished placing legos
				showCorrected(firstlegos, legos, firstlegos2, legos2,
						leftCorrection, rightCorrection, start, foundLegos,
						foundLegos2);

				cout
						<< duration_cast<milliseconds>(
								system_clock::now().time_since_epoch()).count()
						<< endl;
				fast = 0;
				fine = 0;
				still = 0;
				foundLegosYet = false;

				break;
			}
			if (count == 0) { //first time through loop waits for start
				auto memorizeTimeStart = duration_cast<milliseconds>(
						system_clock::now().time_since_epoch()).count();
				prevLegos.clear();
				prevLegos2.clear();
				prevColor.clear();
				prevColor2.clear();
				if (iterationOfExperiment != 1) { //if it's not the first time through then wait 10 seconds

					cout
							<< duration_cast<milliseconds>(
									system_clock::now().time_since_epoch()).count()
							<< endl;
					start = duration_cast<milliseconds>(
							system_clock::now().time_since_epoch()).count();
					waitKey(10000);
					cout << "Begin" << endl;
				} else { //calculates time to memorize
					waitKey(0);
					memorizeTime = duration_cast<milliseconds>(
							system_clock::now().time_since_epoch()).count()
							- memorizeTimeStart;
					;
				}
			}

			//only on the first round records initial setup
			if (count == 1 && iterationOfExperiment == 1) {
				for (int s = 0; s < legos.size(); s++) {
					firstlegos.push_back(legos[s]);
				}
				for (int s = 0; s < legos2.size(); s++) {
					firstlegos2.push_back(legos2[s]);
				}
				waitKey(15000);

				start = duration_cast<milliseconds>(
						system_clock::now().time_since_epoch()).count();
				cout << "Begin" << endl;
				prevLegos.clear();
				prevLegos2.clear();
				prevColor.clear();
				prevColor2.clear();
			}
			count++;
			color.clear();
			legos.clear();
			color2.clear();
			legos2.clear();
			height.clear();
//		cout << "Time: "
//				<< chrono::milliseconds(end2 - start2).count() / 1000000000.0
//				<< endl;
		}
		cvDestroyAllWindows();
	}
	return 0;
}
