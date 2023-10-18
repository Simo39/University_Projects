#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Generic class implementing a filter with the input and output image data and the parameters
class Filter {

	// Methods

public:

	// constructor 
	// input_img: image to be filtered
	// filter_size : size of the kernel/window of the filter
	Filter(cv::Mat input_img, int filter_size);

	// perform filtering (in base class do nothing, to be reimplemented in the derived filters)
	void doFilter();

	// get the output of the filter
	cv::Mat getResult();

	//set the window size (square window of dimensions size x size)
	void setSize(int size);

	//get the Window Size
	int getSize();

	// Data

protected:

	// input image
	cv::Mat input_image;

	// output image (filter result)
	cv::Mat result_image;

	// window size
	int filter_size;



};

// Gaussian Filter
class GaussianFilter : public Filter {

public:
	//constructor
	GaussianFilter(cv::Mat input_img, int kernel_size = 0, double sigma = 0);
	//doFilter() re-implemantation
	void doFilter();
	//set and get methods for sigma value
	void setSigma(double sigma);
	double getSigma();

protected:
	//additional parameter
	double sigma;

};

class MedianFilter : public Filter {

public:
	//constructor
	MedianFilter(cv::Mat input_img, int kernel_size = 1);
	//doFilter() re-implemantation
	void doFilter();

};

class BilateralFilter : public Filter {

public:
	//constructor
	BilateralFilter(cv::Mat input_img, double simga_range = 0, double sigma_space = 0);
	//doFilter() re-implemantation
	void doFilter();
	//set and get methods for sigma values
	void setSigmaRange(double sigma);
	double getSigmaRange();

	void setSigmaSpace(double sigma);
	double getSigmaSpace();

protected:
	//additional parameters
	double sigma_range;
	double sigma_space;

};