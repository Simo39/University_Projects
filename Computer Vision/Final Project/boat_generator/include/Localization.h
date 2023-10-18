#pragma once

#include "Filter.h"
#include <opencv2/core.hpp>

class Localization {
public:
	Localization(cv::Mat img);
	void doLocalization();
	std::vector<cv::Rect> getRegions();
	cv::Mat getInputImage();

protected:
	cv::Mat image;
	std::vector<cv::Rect> regions;

	cv::Mat createMask(cv::Mat image);
	void createRegions(cv::Mat mask);
	void clusterRegions();
	std::vector<std::vector<Rect>> cluster_rects(const std::vector<Rect>& rects, const double th);
	Rect union_of_rects(const std::vector<Rect>& cluster);

private:
	Mat adaptiveHistogramEqualizer(Mat input, HistogramEqualizer* data = NULL);
	Mat useCanny(Mat input, CannyFilter* data = NULL);
	Mat useGaussian(Mat input, GaussianFilter* data = NULL);
	Mat useBilateral(Mat input, BilateralFilter* data = NULL);
	Mat useMorphOperator(Mat input, MorphologicOperator* data = NULL);
	Mat useSelectiveSearch(Mat input, bool fastSearch = true);
	Mat useStaticSaliencyDetection(Mat input, bool spectralResidual = true);
	Mat useWatershed(Mat input, Mat seeds);
};