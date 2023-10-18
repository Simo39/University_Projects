#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include "filter.h"

//using namespace cv;

	// constructor
Filter::Filter(cv::Mat input_img, int size) {

	input_image = input_img;
	if (size % 2 == 0)
		size++;
	filter_size = size;
}

// for base class do nothing (in derived classes it performs the corresponding filter)
void Filter::doFilter() {

	// it just returns a copy of the input image
	result_image = input_image.clone();

}

// get output of the filter
cv::Mat Filter::getResult() {

	return result_image;
}

//set window size (it needs to be odd)
void Filter::setSize(int size) {

	if (size % 2 == 0)
		size++;
	filter_size = size;
}

//get window size 
int Filter::getSize() {

	return filter_size;
}



// Write your code to implement the Gaussian, median and bilateral filters

/*
* Gaussian Filter
*/
GaussianFilter::GaussianFilter(cv::Mat input_img, int kernel_size, double sigma) :
	Filter(input_img, kernel_size) {
	this->sigma = sigma;
	doFilter();
}

//doFilter re-implemantation
void GaussianFilter::doFilter() {
	cv::GaussianBlur(input_image, result_image, cv::Size(filter_size,filter_size), sigma);
}

//set and get of sigma

void GaussianFilter::setSigma(double sigma) {
	this->sigma = sigma;
}

double GaussianFilter::getSigma() {
	return sigma;
}

/*
* Median Filter
*/
MedianFilter::MedianFilter(cv::Mat input_img, int kernel_size) :
	Filter(input_img, kernel_size){
	doFilter();
}

//doFilter re-implemantation
void MedianFilter::doFilter() {
	cv::medianBlur(input_image, result_image, filter_size);
}

/*
* Bilateral Filter
*/
BilateralFilter::BilateralFilter(cv::Mat input_img, double sigma_range, double sigma_space) :
	Filter(input_img, static_cast<int>(6*sigma_space)) {
	doFilter();
}

//doFilter re-implemantation
void BilateralFilter::doFilter() {
	cv::bilateralFilter(input_image, result_image, filter_size, sigma_range, sigma_space);
}

//set and get of sigma range and space

void BilateralFilter::setSigmaRange(double sigma) {
	sigma_range = sigma;
}

double BilateralFilter::getSigmaRange() {
	return sigma_range;
}

void BilateralFilter::setSigmaSpace(double sigma) {
	sigma_space = sigma;
	filter_size = static_cast<int>(6 * sigma_space);
}

double BilateralFilter::getSigmaSpace() {
	return sigma_space;
}