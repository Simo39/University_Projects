#pragma once
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
// Generic class implementing a filter with the input and output image data and the parameters

class Filter {
public:
	Filter(Mat input_img, int filter_size);
	void doFilter();
	Mat getResult();
	void setSize(int size);
	int getSize();

protected:
	Mat input_image;
	Mat result_image;
	int filter_size;
};

// Gaussian Filter
class GaussianFilter : public Filter {

public:
	GaussianFilter(Mat input_img, int kernel_size = 0, double sigma = 0);
	void doFilter();
	void setSigma(double sigma);
	double getSigma();

protected:
	double sigma;

};

class MedianFilter : public Filter {

public:
	MedianFilter(Mat input_img, int kernel_size = 1);
	void doFilter();

};

class BilateralFilter : public Filter {

public:
	BilateralFilter(Mat input_img, double simga_range = 0, double sigma_space = 0);
	void doFilter();
	void setSigmaRange(double sigma);
	double getSigmaRange();

	void setSigmaSpace(double sigma);
	double getSigmaSpace();

protected:
	double sigma_range;
	double sigma_space;

};

class CannyFilter : public Filter{

public:
	CannyFilter(Mat input_img, double ratio = 0, double lower_threshold = 0);
	void doFilter();
	void setLowerThreshold(double lower_threshold);
	double getLowerThreshold();
	void setRatio(double ratio);
	double getRatio();

protected:
	double ratio;
	double lower_threshold;
};

class MorphologicOperator : public Filter {
public:
	MorphologicOperator(Mat input_img, int morph_operator, int element, int kernel_size, int iterations);
	void doFilter();
	void setMorphOperator(int morp_operator);
	int getMorphOperator();
	void setElement(int element);
	int getElement();
	void setIterations(int element);
	int getIterations();

protected:
	int morph_operator;
	int element;
	int iterations;
};

class HistogramEqualizer {
public:
	HistogramEqualizer(Mat input, std::vector<int> thresholds);
	void equalizeChannel(int index);
	Mat getResult();
	void setThreshold(int index, int threshold);
	int getThreshold(int index);
	double getMean(int index);
protected:
	std::vector<double> means{ 0.0, 0.0, 0.0 };
	std::vector<Mat> channels{ Mat(), Mat(), Mat() };
	std::vector<Mat> eq_channels{ Mat(), Mat(), Mat() };
	std::vector<int> thresholds{ 0, 0, 0 };
};