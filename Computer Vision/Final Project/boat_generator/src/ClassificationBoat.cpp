
#include "ClassificationBoat.h"
#include "Dictionary.h"
#include <opencv2/core.hpp>

ClassificationBoat::ClassificationBoat(Dictionary* positive, Dictionary* negative, float threshold) {
	this->positive = positive;
	this->negative = negative;
	this->threshold = threshold;
}

bool ClassificationBoat::isBoat(cv::Mat image, cv::Rect region) {
	float positive_score = getScore(image, region, positive);
	float negative_score = getScore(image, region, negative);
	last_positive = positive_score;
	last_negative = negative_score;
	return positive_score >= negative_score;
}
float ClassificationBoat::getLastPositive() {
	return last_positive;
}
float ClassificationBoat::getLastNegative() {
	return last_negative;
}

float ClassificationBoat::getScore(cv::Mat image, cv::Rect region, Dictionary* BoW) {
	Mat sample(image, region);
	BoW->extractKeypoints(sample, threshold);
	return BoW->getAccuracy();
}