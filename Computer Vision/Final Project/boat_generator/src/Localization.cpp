
#include "debug_utils.h"
#include "Localization.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace cv;

Localization::Localization(Mat image) {
	this->image = image;
}
void Localization::doLocalization(){
	Mat bw = image.clone();
	if (bw.channels() > 1)
		cvtColor(bw, bw, COLOR_BGR2GRAY);
	Mat mask = createMask(bw);
	createRegions(mask);
	clusterRegions();
}
std::vector<Rect> Localization::getRegions(){
	return regions;
}
Mat Localization::getInputImage() {
	return image.clone();
}

Mat Localization::createMask(Mat image){
	BilateralFilter bilateral(image, 72, 5);
	CannyFilter canny(bilateral.getResult(), 2, 134);
	MorphologicOperator morph_1(canny.getResult(), 5, 2, 9, 3);  
	MorphologicOperator morph_2(morph_1.getResult(), 3, 1, 3, 2);
	return morph_2.getResult();
}
void Localization::createRegions(cv::Mat mask){
	std::vector<std::vector<Point> > contours;
	findContours(mask, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
	std::vector<std::vector<Point> > contours_poly(contours.size());
	std::vector<Rect> boundRect(contours.size());
	for (int i = 0; i < contours.size(); i++) {
		Scalar color(255, 0, 0);
		approxPolyDP(contours[i], contours_poly[i], 3, true);
		boundRect[i] = boundingRect(contours_poly[i]);
	}
	regions = boundRect;
}
void Localization::clusterRegions() {
	std::vector<std::vector<Rect>> clusters;
	clusters = cluster_rects(regions, 0);
	regions.clear();
	for (std::vector<Rect> cluster : clusters) {
		Rect region = union_of_rects(cluster);
		regions.push_back(region);
	}
}
std::vector<std::vector<Rect>> Localization::cluster_rects(const std::vector<Rect>& rects, const double th)
{
	std::vector<int> labels;
	int n_labels = partition(rects, labels,
		[th](const Rect& lhs, const Rect& rhs) {
			double i = static_cast<double>((lhs & rhs).area());
			double ratio_intersection_over_lhs_area = i / static_cast<double>(lhs.area());
			double ratio_intersection_over_rhs_area = i / static_cast<double>(rhs.area());
			return (ratio_intersection_over_lhs_area > th) || (ratio_intersection_over_rhs_area > th);
		});

	std::vector<std::vector<Rect>> clusters(n_labels);
	for (size_t i = 0; i < rects.size(); ++i) {
		clusters[labels[i]].push_back(rects[i]);
	}

	return clusters;
}
Rect Localization::union_of_rects(const std::vector<Rect>& cluster)
{
	Rect one;
	if (!cluster.empty())
	{
		one = cluster[0];
		for (const auto& r : cluster) { one |= r; }
	}
	return one;
}
