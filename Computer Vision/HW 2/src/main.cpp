#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "filter.h"

using namespace cv;

void inputError(char* argv[]) {
	std::string program = static_cast<std::string>(argv[0]);
	int pos = program.rfind("\\");
	if (pos == -1)
		pos = program.rfind("/");
	program = program.substr(pos + 1);
	std::cout << "ERROR on using: " << program << std::endl;
	std::cout << "The program requires one image file as argument." << std::endl;
	std::cout <<"Correct use:\n\n\t"<< program << " PATH_TO_AN_IMAGE_FILE" << std::endl;
}

// hists = vector of 3 cv::mat of size nbins=256 with the 3 histograms
// e.g.: hists[0] = cv:mat of size 256 with the red histogram
//       hists[1] = cv:mat of size 256 with the green histogram
//       hists[2] = cv:mat of size 256 with the blue histogram
void showHistogram(std::vector<cv::Mat>& hists)
{
	// Min/Max computation
	double hmax[3] = { 0,0,0 };
	double min;
	cv::minMaxLoc(hists[0], &min, &hmax[0]);
	cv::minMaxLoc(hists[1], &min, &hmax[1]);
	cv::minMaxLoc(hists[2], &min, &hmax[2]);

	std::string wname[3] = { "blue", "green", "red" };
	cv::Scalar colors[3] = { cv::Scalar(255,0,0), cv::Scalar(0,255,0),
							 cv::Scalar(0,0,255) };

	std::vector<cv::Mat> canvas(hists.size());

	// Display each histogram in a canvas
	for (int i = 0, end = static_cast<int>(hists.size()); i < end; i++)
	{
		canvas[i] = cv::Mat::ones(125, hists[0].rows, CV_8UC3);

		for (int j = 0, rows = canvas[i].rows; j < hists[0].rows - 1; j++)
		{
			cv::line(
				canvas[i],
				cv::Point(j, rows),
				cv::Point(j, rows - (hists[i].at<float>(j) * rows / hmax[i])),
				hists.size() == 1 ? cv::Scalar(200, 200, 200) : colors[i],
				1, 8, 0
			);
		}

		cv::imshow(hists.size() == 1 ? "value" : wname[i], canvas[i]);
	}
}

void computeHistograms(Mat& channel, Mat& histogram, int bins, int upperRange) {
	int histSize[]{ bins };
	float range[]{ 0, upperRange };
	const float* ranges[]{ range };
	calcHist(&channel, 1, { 0 }, Mat(), histogram, 1, histSize, ranges, true, false);
}

void showResizedImg(String windowName, Mat& img, double scale) {
	Size new_size(static_cast<int>(img.cols / scale), static_cast<int>(img.rows / scale));
	Mat output;
	resize(img, output, new_size, 0, 0, INTER_AREA);
	imshow(windowName, output);
}

void equalizeSingleChannel(Mat& img, int channel) {
	Mat channels[3];
	split(img, channels);
	equalizeHist(channels[channel], channels[channel]);
	merge(channels, 3, img);
}

void RGB_histogramEqualization(Mat& img) {
	Mat bgr = img.clone();
	Mat bgr_channels[3];
	split(bgr, bgr_channels);
	std::vector<Mat> histograms(3);
	//histogram computation of each channel
	computeHistograms(bgr_channels[0], histograms[0], 256, 256);
	computeHistograms(bgr_channels[1], histograms[1], 256, 256);
	computeHistograms(bgr_channels[2], histograms[2], 256, 256);
	showHistogram(histograms);
	waitKey(0);
	destroyAllWindows();
	//histograms equalization
	equalizeHist(bgr_channels[0], bgr_channels[0]);
	equalizeHist(bgr_channels[1], bgr_channels[1]);
	equalizeHist(bgr_channels[2], bgr_channels[2]);
	//computing the new image (with better contrast)
	merge(bgr_channels, 3, bgr);
	//showing new equalized histograms and new computed image
	computeHistograms(bgr_channels[0], histograms[0], 256, 256);
	computeHistograms(bgr_channels[1], histograms[1], 256, 256);
	computeHistograms(bgr_channels[2], histograms[2], 256, 256);
	showHistogram(histograms);
	Mat result;
	hconcat(img, bgr, result);
	showResizedImg("equalized", result, 3);
	waitKey(0);
	destroyAllWindows();
}

void HSV_histogramEqualization(Mat& img) {
	Mat hsv;
	cvtColor(img, hsv, COLOR_BGR2HSV);
	std::vector<Mat> histograms(3);

	std::vector<Mat> equalized_HSV;
	std::vector<std::vector<Mat>> eqHSV_channels;
	std::vector<std::string> windowName{ "hue equalization", "saturation equalization", "value equalization" };
	//histogram equalization and showing the results one channel at time
	for (int i = 0; i < 3; i++) {
		equalized_HSV.push_back(hsv.clone());
		equalizeSingleChannel(equalized_HSV[i], i);
		cvtColor(equalized_HSV[i], equalized_HSV[i], COLOR_HSV2BGR);
		std::vector<Mat> temp;
		split(equalized_HSV[i], temp);
		eqHSV_channels.push_back(temp);
		computeHistograms(eqHSV_channels[i][0], histograms[0], 256, 256);
		computeHistograms(eqHSV_channels[i][1], histograms[1], 256, 256);
		computeHistograms(eqHSV_channels[i][2], histograms[2], 256, 256);
		showHistogram(histograms);
		Mat result;
		hconcat(img, equalized_HSV[i], result);
		showResizedImg(windowName[i], result, 3);
		waitKey(0);
		destroyAllWindows();
	}
}

//necessary global variables to allow changes on the images throught trackbars
const String GAUSSIAN_WINDOW_NAME{ "Gaussian Filter" };
const String MEDIAN_WINDOW_NAME{ "Median Filter" };
const String BILATERAL_WINDOW_NAME{ "Bilateral Filter" };

/*
* callback to trackbar for gaussian filtering
*/

//callback to kernel size
void onSetKernelGF(int pos, void* userdata) {
	GaussianFilter* img = static_cast<GaussianFilter*>(userdata);
	img->setSize(pos);
	img->doFilter();
	imshow(GAUSSIAN_WINDOW_NAME, img->getResult());
}

//callback to sigma size
void onSetSigmaGF(int pos, void* userdata) {
	GaussianFilter* img = static_cast<GaussianFilter*>(userdata);
	img->setSigma(pos);
	img->doFilter();
	imshow(GAUSSIAN_WINDOW_NAME, img->getResult());
}

/*
* callback to trackbar for median filtering
*/

//callback to kernel size
void onSetKernelMF(int pos, void* userdata) {
	MedianFilter* img = static_cast<MedianFilter*>(userdata);
	img->setSize(pos);
	img->doFilter();
	imshow(MEDIAN_WINDOW_NAME, img->getResult());
}

/*
* callback to trackbar for bilateral filtering
*/

//callback to sigma range size
void onSetSigmaRangeBF(int pos, void* userdata) {
	BilateralFilter* img = static_cast<BilateralFilter*>(userdata);
	img->setSigmaRange(pos);
	img->doFilter();
	imshow(BILATERAL_WINDOW_NAME, img->getResult());
}

//callback to sigma space size
void onSetSigmaSpaceBF(int pos, void* userdata) {
	BilateralFilter* img = static_cast<BilateralFilter*>(userdata);
	img->setSigmaSpace(pos);
	img->doFilter();
	imshow(BILATERAL_WINDOW_NAME, img->getResult());
}



int main(int argc, char* argv[]){
	if (argc != 2) {
		inputError(argv);
		return -1;
	}
	String img_path = static_cast<String>(argv[1]);
	Mat img = imread(img_path);
	if (img.empty()) {
		inputError(argv);
		return -1;
	}

	//First part: HISTOGRAM EQUALIZATION
	RGB_histogramEqualization(img);
	HSV_histogramEqualization(img);

	//Second part: IMAGE FILTERING;

	//resizing only for view purpose
	double scale = 2.0;
	Size new_size(static_cast<int>(img.cols / scale), static_cast<int>(img.rows / scale));
	Mat output;
	resize(img, output, new_size, 0, 0, INTER_AREA);

	//Gaussian Filtering
	GaussianFilter gaussian_filter{ output };
	namedWindow(GAUSSIAN_WINDOW_NAME);
	createTrackbar("Kernel", GAUSSIAN_WINDOW_NAME, 0, 100, onSetKernelGF, static_cast<void*>(&gaussian_filter));
	createTrackbar("Sigma", GAUSSIAN_WINDOW_NAME, 0, 100, onSetSigmaGF, static_cast<void*>(&gaussian_filter));
	imshow(GAUSSIAN_WINDOW_NAME, gaussian_filter.getResult());

	//Median Filtering
	MedianFilter median_filter{ output };
	namedWindow(MEDIAN_WINDOW_NAME);
	createTrackbar("Kernel", MEDIAN_WINDOW_NAME, 0, 100, onSetKernelMF, static_cast<void*>(&median_filter));
	imshow(MEDIAN_WINDOW_NAME, median_filter.getResult());

	
	//Bilateral Filtering
	BilateralFilter bilateral_filter{ output };
	namedWindow(BILATERAL_WINDOW_NAME);
	createTrackbar("S_Range", BILATERAL_WINDOW_NAME, 0, 200, onSetSigmaRangeBF, static_cast<void*>(&bilateral_filter));
	createTrackbar("S_Space", BILATERAL_WINDOW_NAME, 0, 10, onSetSigmaSpaceBF, static_cast<void*>(&bilateral_filter));
	imshow(BILATERAL_WINDOW_NAME, bilateral_filter.getResult());
	
	waitKey(0);
	return 0;
}