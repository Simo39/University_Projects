#include "Filter.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <iostream>

using namespace cv;

Filter::Filter(Mat input_img, int size) {
	this->input_image = input_img;
	if (size % 2 == 0)
		size++;
	filter_size = size;
}
void Filter::doFilter() {
	result_image = input_image.clone();
}
Mat Filter::getResult() {
	return result_image;
}
void Filter::setSize(int size) {
	if (size % 2 == 0)
		size++;
	filter_size = size;
}
int Filter::getSize() {
	return filter_size;
}

/*
* Gaussian Filter
*/
GaussianFilter::GaussianFilter(Mat input_img, int kernel_size, double sigma) :
	Filter(input_img, kernel_size) {
	this->sigma = sigma;
	doFilter();
}
void GaussianFilter::doFilter() {
	GaussianBlur(input_image, result_image, Size(filter_size, filter_size), sigma);
}
void GaussianFilter::setSigma(double sigma) {
	this->sigma = sigma;
}
double GaussianFilter::getSigma() {
	return sigma;
}

/*
* Median Filter
*/
MedianFilter::MedianFilter(Mat input_img, int kernel_size) :
	Filter(input_img, kernel_size) {
	doFilter();
}
void MedianFilter::doFilter() {
	medianBlur(input_image, result_image, filter_size);
}

/*
* Bilateral Filter
*/
BilateralFilter::BilateralFilter(Mat input_img, double sigma_range, double sigma_space) :
	Filter(input_img, static_cast<int>(6 * sigma_space)) {
	doFilter();
}
void BilateralFilter::doFilter() {
	bilateralFilter(input_image, result_image, filter_size, sigma_range, sigma_space);
}
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

CannyFilter::CannyFilter(Mat input_img, double ratio, double lower_threshold) :
	Filter(input_img, 3){
	this->ratio = ratio;
	this->lower_threshold = lower_threshold;
	doFilter();
}
void CannyFilter::doFilter(){
	Canny(input_image, result_image, lower_threshold, ratio * lower_threshold, filter_size);
}
void CannyFilter::setLowerThreshold(double lower_threshold){
	this->lower_threshold = lower_threshold;
}
double CannyFilter::getLowerThreshold(){
	return lower_threshold;
}
void CannyFilter::setRatio(double ratio){
	this->ratio = ratio;
}
double CannyFilter::getRatio(){
	return ratio;
}

MorphologicOperator::MorphologicOperator(Mat input_img, int morph_operator, int element, int kernel_size, int iterations) :
	Filter(input_img, kernel_size) {
	if (morph_operator < 0 || morph_operator > 6)
		morph_operator = 2;
	this->morph_operator = morph_operator;
	if (element < 0 || element > 2)
		element = 0;
	this->element = element;
	if (iterations < 1)
		iterations = 1;
	this->iterations = iterations;
	result_image = input_img.clone();
	doFilter();
}
void MorphologicOperator::doFilter() {
	Mat element = getStructuringElement(this->element, Size(filter_size, filter_size));
	morphologyEx(input_image, result_image, morph_operator, element, Point(-1,-1), iterations);
}
void MorphologicOperator::setMorphOperator(int morph_operator) {
	if (morph_operator < 0 || morph_operator > 6)
		return;
	this->morph_operator = morph_operator;
}
int MorphologicOperator::getMorphOperator() {
	return morph_operator;
}
void MorphologicOperator::setElement(int element) {
	if (element < 0 || element > 2)
		return;
	this->element = element;
}
int MorphologicOperator::getElement() {
	return element;
}
void MorphologicOperator::setIterations(int iterations) {
	if (iterations < 1)
		return;
	this->iterations = iterations;
}
int MorphologicOperator::getIterations() {
	return iterations;
}


HistogramEqualizer::HistogramEqualizer(Mat input, std::vector<int> thresholds) {
	cvtColor(input, input, COLOR_BGR2HLS);
	split(input, channels);
	eq_channels = channels;
	std::vector<double> temp{ 0,0,0 };
	meanStdDev(input, means, temp);
	if (!thresholds.empty())
		this->thresholds = thresholds;
	for (int i = 0; i < 3; i++)
		equalizeChannel(i);
}
void HistogramEqualizer::equalizeChannel(int index){
	if (thresholds[index] < means[index]) {
		eq_channels[index] = channels[index].clone();
	}
	else{
		Mat temp = channels[index];
		equalizeHist(temp, eq_channels[index]);
	}
}
Mat HistogramEqualizer::getResult() {
	Mat result;
	merge(eq_channels, result);
	cvtColor(result, result, COLOR_HLS2BGR);
	return result;
}
void HistogramEqualizer::setThreshold(int index, int threshold) {
	thresholds[index] = threshold;
}
int HistogramEqualizer::getThreshold(int index) {
	return thresholds[index];
}
double HistogramEqualizer::getMean(int index) {
	return means[index];
}