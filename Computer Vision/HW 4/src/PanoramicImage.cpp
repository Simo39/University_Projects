#include "PanoramicImage.h"
#include "panoramic_utils.h"
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <algorithm>
#include <exception>
#include <limits>


using namespace cv;

PanoramicImage::PanoramicImage(std::vector<String> file_list, double field_of_view) {
	for (String file : file_list) {
		Mat img = imread(file);
		img = PanoramicUtils::cylindricalProj_BGR(img, field_of_view / 2);
		projectedImages.push_back(img);
	}
}

Mat PanoramicImage::composeWithSIFT(float ratio_thresold) {
	reset();
	extractAndMatch_SIFT();
	computeTranslations(ratio_thresold);
	compose();
	return panoramic_image;
}
Mat PanoramicImage::composeWithORB(float ratio_thresold) {
	reset();
	extractAndMatch_ORB();
	computeTranslations(ratio_thresold);
	compose();
	return panoramic_image;
}

Mat PanoramicImage::getImage() {
	return panoramic_image;
}

bool PanoramicImage::isEmpty() {
	return panoramic_image.empty();
}

void PanoramicImage::extractAndMatch_SIFT(){
	Ptr<SIFT> detector = SIFT::create();
	std::vector<Mat> descriptors;
	detect(detector, descriptors);
	Ptr<BFMatcher> matcher = BFMatcher::create();
	match(matcher, descriptors);
}

void PanoramicImage::extractAndMatch_ORB() {
	Ptr<ORB> detector = ORB::create(2000);
	std::vector<Mat> descriptors;
	detect(detector, descriptors);
	Ptr<BFMatcher> matcher = BFMatcher::create(NORM_HAMMING);
	match(matcher, descriptors);
}

template <typename T> void PanoramicImage::detect(Ptr<T>& detector, std::vector<Mat>& descriptors) {
	for (Mat img : projectedImages) {
		std::vector<KeyPoint> temp_kp;
		Mat temp_ds;
		detector->detectAndCompute(img, Mat(), temp_kp, temp_ds);
		keypoints.push_back(temp_kp);
		descriptors.push_back(temp_ds);
	}
}

template <typename T> void PanoramicImage::match(Ptr<T>& matcher, std::vector<Mat>& descriptors) {
	for (int i = 1; i < projectedImages.size(); i++) {
		std::vector<DMatch> temp_matches;
		matcher->match(descriptors[i - 1], descriptors[i], temp_matches);
		matches.push_back(temp_matches);
	}
}

/*
* Compute the average translation for each pair of image.
* 
* For each pair of image (or element of matches) it's:
*	- computed the minimum distance among the pair of keypoints
*	- discarded the matches with distance > minimum distance * ratio
*	- looked for the inliers through Ransac
*	- computed the average translation among the inliers
*
* NOTE THAT the average translation among y is negleted
*/
void PanoramicImage::computeTranslations(float ratio_thresold) {
	for (int i = 0; i < matches.size(); i++) {

		//minumum distance
		float min_distance = std::numeric_limits<float>::infinity();
		for (DMatch match : matches[i]) {
			if (match.distance < min_distance)
				min_distance = match.distance;
		}

		//discarding matches
		min_distance = ratio_thresold * min_distance;
		auto predicate{
			[min_distance](DMatch match) -> bool {
			return (match.distance > min_distance);
			}
		};
		matches[i].erase(std::remove_if(matches[i].begin(), matches[i].end(), predicate), matches[i].end());
		if (matches[i].size() < 4)
			throw std::invalid_argument("The chosen ratio threshold it's too low");
		//looking for inliers through Ransac
		std::vector<Point2f> srcPoints;
		std::vector<Point2f> dstPoints;
		for (DMatch match : matches[i]) {
			srcPoints.push_back(keypoints[i][match.queryIdx].pt);
			dstPoints.push_back(keypoints[i + 1][match.trainIdx].pt);
		}
		std::vector<uchar> mask;
		findHomography(srcPoints, dstPoints, RANSAC, 4.0, mask);

		//computing average translation among inliers
		float dx = 0;
		int inliers = 0;
		for (int j = 0; j < mask.size(); j++) {
			if (mask[j]) {
				dx += dstPoints[j].x - srcPoints[j].x;
				inliers++;
			}
		}
		dx /= inliers;
		x_translations.push_back(dx);
	}
}

void PanoramicImage::compose() {
	int total_dx = 0;
	int i = 0;
	for (float dx : x_translations) {
		total_dx += cvRound(dx);
	}
	Mat result;
	//if the panoramic image is taken counterclockwise
	if (total_dx < 0) {
		total_dx = abs(total_dx);
		int cols = projectedImages[0].cols;
		int rows = projectedImages[0].rows;
		result = Mat::zeros(rows, cols + total_dx, projectedImages[0].type());
		projectedImages[0].copyTo(result.colRange(0, projectedImages[0].cols));
		total_dx = 0;
		for (int i = 0; i < x_translations.size(); i++) {
			int dx = cvRound(x_translations[i]);
			dx = abs(dx);
			Mat temp = projectedImages[i + 1].colRange(cols - dx, cols);
			Range colRange(cols + total_dx, cols + total_dx + dx);
			temp.copyTo(result(Range::all(), colRange));
			total_dx += dx;
		}
	}
	//if the panoramic image is taken counterclockwise
	else {
		int cols = projectedImages[0].cols;
		int rows = projectedImages[0].rows;
		result = Mat::zeros(rows, cols + total_dx, projectedImages[0].type());
		projectedImages[0].copyTo(result.colRange(result.cols - cols, result.cols));
		cols = result.cols - cols;
		total_dx = 0;
		for (int i = 0; i < x_translations.size(); i++) {
			int dx = cvRound(x_translations[i]);
			Mat temp = projectedImages[i + 1].colRange(0, dx + 1);
			Range colRange(cols - total_dx - dx, cols - total_dx + 1);
			temp.copyTo(result(Range::all(), colRange));
			total_dx += dx;
		}
	}
	panoramic_image = result.clone();
}

void PanoramicImage::reset() {
	keypoints.clear();
	matches.clear();
	x_translations.clear();
}