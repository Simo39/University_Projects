#pragma once

#include "Dictionary.h"
#include <opencv2/core.hpp>
#include <math.h>

class ClassificationBoat {
public:
	ClassificationBoat(Dictionary* positive, Dictionary* negative, float threshold = 1.0);
	bool isBoat(cv::Mat image, cv::Rect region);
	float getLastPositive();
	float getLastNegative();
protected:
	Dictionary* positive;
	Dictionary* negative;
	float threshold;
	float last_positive;
	float last_negative;

	float getScore(cv::Mat image, cv::Rect region, Dictionary* BoW);
};