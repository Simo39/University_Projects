#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace cv;

void inputError(char* argv[]);

void applyCanny();
void onChangeFirstThreshold(int pos, void* userdata);
void onChangeRatio(int pos, void* userdata);

void applyHoughLines();
void onChangeRho(int pos, void* userdata);
void onChangeTheta(int pos, void* userdata);

void applyHoughCircles();
void onChangeAccThreshold(int pos, void* userdata);
void onChangeMaxRadius(int pos, void* userdata);

/*
* Structures implemented for an easier and quicker handling of the
* Canny's and Hough's functions parameters and to allow an easier showing 
* of the result in a window through the trackbar use
*/
struct CannyData {
	Mat input;
	Mat output;
	int lower_threshold=431;
	int ratio = 2;
	const String windName{ "Canny" };
} canny;
struct HoughData {
	Mat& input = canny.output;
	std::vector<Vec3f> lines, circles;
	Mat output;
	//lines parameters
	int distRHO_acc = 8;
	int distTHETA_acc = 22;
	//circle parameters
	int canny_threshold = 655;
	int acc_threshold = 15;
	int max_radius = 20;
	const String windName{ "Hough" };
} hough;
struct ShapesData {
	double m1, m2, c1, c2;		//line equations coefficients
	int x, y, radius;		//circle parameters
} shapes;



int main(int argc, char* argv[]) {
	if (argc != 2) {
		inputError(argv);
		return -1;
	}
	String img_path = static_cast<String>(argv[1]);
	Mat input = imread(img_path);
	if (input.empty()) {
		inputError(argv);
		return -1;
	}


	//edge detection
	canny.input = input.clone();
	namedWindow(canny.windName);
	createTrackbar("Lower Threshold", canny.windName, &(canny.lower_threshold), 500, onChangeFirstThreshold);
	createTrackbar("Ratio", canny.windName, &(canny.ratio), 3, onChangeRatio);
	applyCanny();
	waitKey(0);
	destroyAllWindows();
	
	//line detection
	namedWindow(hough.windName);
	hough.output = input.clone();
	createTrackbar("Rho", hough.windName, &(hough.distRHO_acc), 50, onChangeRho);
	createTrackbar("Theta", hough.windName, &(hough.distTHETA_acc), 50, onChangeTheta);
	applyHoughLines();
	waitKey(0);
	destroyAllWindows();

	if (shapes.m1 == shapes.m2) {
		std::cout << "You have detected 2 parallel lines" << std::endl;
		return -1;
	}		

	//edge detecion
	canny.input = input.clone();
	canny.lower_threshold = hough.canny_threshold;
	canny.ratio = 2;
	namedWindow(canny.windName);
	createTrackbar("Lower Threshold", canny.windName, &(canny.lower_threshold), 1000, onChangeFirstThreshold);
	applyCanny();
	waitKey(0);
	destroyAllWindows();

	//circle detection
	cvtColor(input, hough.input, COLOR_BGR2GRAY);
	hough.output = input.clone();
	hough.canny_threshold = canny.lower_threshold;
	namedWindow(hough.windName);
	createTrackbar("Acc Threshold", hough.windName, &(hough.acc_threshold), 50, onChangeAccThreshold);
	createTrackbar("Max Radius", hough.windName, &(hough.max_radius), 50, onChangeMaxRadius);
	applyHoughCircles();
	waitKey(0);
	destroyAllWindows();

	if (hough.circles.size() != 1) {
		std::cout << "ERROR: You have detected multiple circles." << std::endl;
		return -1;
	}

	//drawing the shapes on the input image
	Mat result = input.clone();
	//triangle
	Point intersection, P1, P2;
	intersection.x = cvRound( (shapes.c2 - shapes.c1) / (shapes.m1 - shapes.m2));
	intersection.y = cvRound((shapes.m2 * intersection.x + shapes.c2));
	P1.y = result.rows - 1;
	P1.x = cvRound((P1.y - shapes.c1) / shapes.m1);
	P2.y = result.rows - 1;
	P2.x = cvRound((P2.y - shapes.c2)/shapes.m2);
	std::vector<Point> triangle{ intersection, P1, P2 };
	fillPoly(result, { triangle }, Scalar(0, 0, 255));

	//circle
	Point center{ cvRound(shapes.x),cvRound(shapes.y) };
	circle(result, center, static_cast<int>(shapes.radius), Scalar(0, 255, 0), FILLED);

	imshow("Result", result);
	waitKey(0);
	destroyAllWindows();

	return 0;
}


//function called in case of error
void inputError(char* argv[]) {
	std::string program = static_cast<std::string>(argv[0]);
	int pos = program.rfind("\\");
	if (pos == -1)
		pos = program.rfind("/");
	program = program.substr(pos + 1);
	std::cout << "ERROR on using: " << program << std::endl;
	std::cout << "The program requires one image file as argument." << std::endl;
	std::cout << "Correct use:\n\n\t" << program << " PATH_TO_AN_IMAGE_FILE" << std::endl;
}

/*
* This function is called every time the Canny's filter parameters are changed.
* It applies the OpenCV Canny function and it show the result in a window.
*/
void applyCanny() {
	Canny(canny.input, canny.output, canny.lower_threshold, canny.ratio * canny.lower_threshold);
	imshow(canny.windName, canny.output);
}

void onChangeFirstThreshold(int pos, void* userdata) {
	canny.lower_threshold = pos;
	applyCanny();
}

void onChangeRatio(int pos, void* userdata) {
	if (pos < 2)
		pos = 2;
	canny.ratio = pos;
	applyCanny();
}

/*
* This function is called every time the Hough's filter parameters are changed.
* It applies the OpenCV Hough function and it show the result in a window.
* Here we are looking for lines
*/
void applyHoughLines() {
	Mat result = hough.output.clone();
	HoughLines(hough.input, hough.lines, hough.distRHO_acc, hough.distTHETA_acc * CV_PI / 180, 0);
	for (int i = 0; i < 2 && i < hough.lines.size(); i++)
	{
		float rho = hough.lines[i][0], theta = hough.lines[i][1], votes = hough.lines[i][2];
		Point pt1, pt2;
		double cos_theta = cos(theta), sin_theta = sin(theta);
		double m = -cos_theta / sin_theta, c = rho / sin_theta;
		pt1.x = 0;
		pt1.y = cvRound(c);
		pt2.x = cvRound(result.cols - 1);
		pt2.y = cvRound(m * pt2.x + c);
		line(result, pt1, pt2, Scalar(0, 0, 255), 2);
		//saving the lines coefficients to draw the triangle later
		if (i == 0) {
			shapes.m1 = m;
			shapes.c1 = c;
		}
		else {
			shapes.m2 = m;
			shapes.c2 = c;
		}
	}
	imshow(hough.windName, result);
}

void onChangeRho(int pos, void* userdata) {
	if (pos == 0)
		pos = 1;
	hough.distRHO_acc = pos;
	applyHoughLines();
}

void onChangeTheta(int pos, void* userdata) {
	if (pos == 0)
		pos = 1;
	hough.distTHETA_acc = pos;
	applyHoughLines();
}

/*
* This function is called every time the Hough's filter parameters are changed.
* It applies the OpenCV Hough function and it show the result in a window.
* Here we are looking for circles
*/
void applyHoughCircles() {
	Mat result = hough.output.clone();
	HoughCircles(hough.input, hough.circles, HOUGH_GRADIENT, 1,
		hough.input.rows / 16.0, hough.canny_threshold, hough.acc_threshold, 0, hough.max_radius);
	for (int i = 0; i < hough.circles.size(); i++)
	{
		float x = hough.circles[i][0], y = hough.circles[i][1], radius = hough.circles[i][2];
		circle(result, Point(x, y), radius, Scalar(0, 255, 0), 2);
	}
	if (hough.circles.size() == 1) {
		shapes.x = hough.circles[0][0];
		shapes.y = hough.circles[0][1];
		shapes.radius = hough.circles[0][2];
	}
	imshow(hough.windName, result);
}

void onChangeAccThreshold(int pos, void* userdata) {
	hough.acc_threshold = pos;
	applyHoughCircles();
}

void onChangeMaxRadius(int pos, void* userdata) {
	hough.max_radius = pos;
	applyHoughCircles();
}
