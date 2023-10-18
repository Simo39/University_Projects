#pragma once
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

using namespace cv;

class Dictionary {
public:
	Dictionary(std::string dictionary_path);
	Dictionary(const Dictionary& vocabulary);
	std::vector<KeyPoint> extractKeypoints(Mat img, float ratio_threshold = 1.0);
	float getAccuracy();

protected:
	Mat dictionary;
	Ptr<FeatureDetector> detector = SIFT::create();
	Ptr<DescriptorExtractor> extractor = SIFT::create();
	Ptr<BFMatcher> matcher = BFMatcher::create();
	BOWImgDescriptorExtractor bowDE = BOWImgDescriptorExtractor(extractor, matcher);

	float last_accuracy;

	Mat getListOfDescriptors(std::vector<int> indices, Mat descriptors);
	std::vector<int> thresholdPoints(std::vector<int> indices, Mat descriptors, Mat cluster_descriptor, float ratio_threshold);
	void computeAccuracy(std::vector<std::vector<int>> histogram, int total_points);
};

